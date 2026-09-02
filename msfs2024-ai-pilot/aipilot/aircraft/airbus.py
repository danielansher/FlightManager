"""Airbus adapter: selected-versus-managed autoflight.

An Airbus FCU does not simply hold a value. Every knob has two states -- the
value is *selected* when the knob is pulled and *managed* (flown by the FMGC)
when it is pushed -- and setting a number without pulling the knob changes
nothing at all about where the aeroplane goes.

The AI Pilot is the FMGC here, so it wants everything selected: it computes the
heading, altitude and speed itself and needs the aeroplane to fly exactly those.
That means every value change is followed by a pull.

Pulls are HTML gauge events, which SimConnect cannot transmit. They go through
the WASM bridge as calculator code instead. When no bridge is available the
adapter falls back to plain SimConnect events and says so once, rather than
sending pulls into a void and leaving the aeroplane wandering off on managed
guidance -- which is exactly the failure that makes third-party autopilot tools
feel broken.
"""

from __future__ import annotations

import json
import os
from typing import Optional

from ..perf.profiles import AircraftProfile
from ..sim.base import SimBackend, SimState
from .base import AdapterCapabilities, AircraftAdapter, Logger

CONVENTIONS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "profiles", "fcu_conventions.json")

_CONVENTIONS: Optional[dict] = None


def load_conventions() -> dict:
    global _CONVENTIONS
    if _CONVENTIONS is None:
        try:
            with open(CONVENTIONS_PATH, "r", encoding="utf-8") as handle:
                _CONVENTIONS = json.load(handle)
        except (OSError, ValueError):
            _CONVENTIONS = {}
    return _CONVENTIONS


def convention(name: str) -> dict:
    """One FCU naming convention, with the documentation keys stripped out."""
    data = load_conventions().get(name, {})
    return {k: v for k, v in data.items() if k != "description" and isinstance(v, str)}


class AirbusAdapter(AircraftAdapter):
    """Drives an Airbus FCU, pulling each knob so guidance stays selected."""

    key = "airbus"

    def __init__(self, sim: SimBackend, profile: AircraftProfile,
                 log: Optional[Logger] = None, fcu_convention: str = "") -> None:
        super().__init__(sim, profile, log)
        self.convention_name = fcu_convention
        self.fcu = convention(fcu_convention) if fcu_convention else {}
        self._warned_no_bridge = False
        self._pulled: set[str] = set()

    # -- Capability reporting ------------------------------------------------
    def capabilities(self) -> AdapterCapabilities:
        caps = self.sim.capabilities()
        return AdapterCapabilities(
            autoland=self.profile.autoland_capable,
            managed_modes=True,
            needs_lvars=bool(self.fcu),
            lvars_available=caps.calculator_code,
        )

    def describe(self) -> str:
        if not self.fcu:
            return (f"{self.profile.name} via standard SimConnect events "
                    "(no FCU convention configured)")
        route = "WASM bridge" if self.sim.capabilities().calculator_code else \
            "standard events -- WASM bridge unavailable"
        return f"{self.profile.name} via {self.convention_name} FCU over {route}"

    # -- Knob mechanics ------------------------------------------------------
    def _press(self, key: str) -> bool:
        """Fire one FCU push or pull. Returns whether it actually went out."""
        event = self.fcu.get(key)
        if not event:
            return False
        if not self.sim.capabilities().calculator_code:
            if not self._warned_no_bridge:
                self._warned_no_bridge = True
                self.log(
                    "No WASM bridge, so the FCU knobs cannot be pulled. Falling back to "
                    "standard autopilot events -- watch that the aeroplane follows the "
                    "selected values rather than its own managed profile."
                )
            return False
        return self.sim.exec_calculator_code(f"1 (>H:{event})")

    def _pull_once(self, key: str) -> None:
        """Pull a knob the first time we set that channel, not every cycle."""
        if key in self._pulled:
            return
        if self._press(key):
            self._pulled.add(key)

    # -- Overrides -----------------------------------------------------------
    def set_heading_magnetic(self, magnetic_deg: float) -> None:
        before = self._last_heading
        super().set_heading_magnetic(magnetic_deg)
        if self._last_heading != before:
            self._pull_once("heading_pull")

    def set_altitude(self, altitude_ft: float) -> None:
        before = self._last_altitude
        super().set_altitude(altitude_ft)
        if self._last_altitude != before:
            # The altitude knob is pulled every time on an Airbus: a pull means
            # "open climb/descent to the selected level now".
            self._press("altitude_pull")

    def set_speed_kt(self, kt: float) -> None:
        before = self._last_speed
        super().set_speed_kt(kt)
        if self._last_speed != before:
            self._pull_once("speed_pull")

    def set_mach(self, mach: float) -> None:
        before = self._last_mach
        super().set_mach(mach)
        if self._last_mach != before:
            self._pull_once("speed_pull")

    def set_vertical_speed(self, fpm: float) -> None:
        before = self._last_vs
        super().set_vertical_speed(fpm)
        if self._last_vs != before:
            self._press("vs_pull")

    def arm_approach(self, state: SimState) -> None:
        if self._approach_armed or state.ap_approach_hold:
            return
        # Prefer the aeroplane's own APPR button; fall back to the stock event.
        if not self._press("approach_push"):
            self.sim.send_event("AP_APR_HOLD_ON")
        self._approach_armed = True
        self.log("APPR armed")

    def approach_is_captured(self, state: SimState) -> bool:
        if state.ap_approach_hold and state.ap_glideslope_hold:
            return True
        # Some add-ons do not publish the stock glideslope flag, so fall back
        # to the localizer and glideslope deviations being small and live.
        return bool(
            state.nav1_has_localizer
            and state.nav1_has_glideslope
            and abs(state.nav1_localizer_error_deg) < 1.0
            and abs(state.nav1_glideslope_error_deg) < 0.6
        )


class BoeingAdapter(AircraftAdapter):
    """Boeing MCP adapter.

    A Boeing MCP holds what it is given, which is exactly the standard-event
    model, so this adds only one thing: a level change is commanded explicitly
    when a new altitude is selected, which is what a crew does on the MCP.

    That command is sent *once per new altitude*, and this is the whole point
    of the class. An earlier version sent it unconditionally on every control
    cycle -- four times a second for the length of a flight, some ten thousand
    times -- which continually re-commands a mode change the aeroplane is in
    the middle of executing. On the default 787 that is a plausible cause of
    the well-known complaint that its autopilot engages and then quietly stops
    flying: the aeroplane is being interrupted faster than it can capture.
    """

    key = "boeing"

    def describe(self) -> str:
        return f"{self.profile.name} via Boeing MCP (standard SimConnect events)"

    def set_altitude(self, altitude_ft: float) -> None:
        before = self._last_altitude
        super().set_altitude(altitude_ft)
        if self._last_altitude != before:
            self._level_change_pending = True

    def select_altitude_mode(self, state: SimState) -> None:
        # Delegate, so the vertical-speed mode handling in the base class
        # applies here too. Overriding it outright is what previously left a
        # Boeing unable to return to altitude capture after any commanded
        # rate: a go-around then sat at five hundred feet with full thrust and
        # a three thousand foot target it could not climb to.
        super().select_altitude_mode(state)
        if self._level_change_pending:
            self._level_change_pending = False
            self.sim.send_event("FLIGHT_LEVEL_CHANGE_ON", 1)
