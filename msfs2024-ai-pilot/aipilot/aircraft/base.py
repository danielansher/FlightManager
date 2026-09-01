"""Aircraft adapters: the translation from intent to simulator commands.

The guidance says *fly heading 271, climb to 37,000, slow to 250*. What that
means in button presses differs between a Boeing and an Airbus, and between one
add-on Airbus and another. Adapters absorb that difference so the autopilot
logic never has to care which aeroplane it is in.

The base adapter uses only standard SimConnect key events. That is not a
lowest-common-denominator fallback -- it is the *right* implementation for a
large part of the fleet, the default 787 included, because Asobo wire the stock
events straight into the aircraft's autoflight system. Where an add-on needs
more (Airbus FCU push/pull semantics, say) a subclass adds it.

Two design choices are worth calling out:

*Closed-loop configuration changes.* Flaps are moved one detent at a time
against the reported handle index rather than commanded absolutely, because
``FLAPS_SET`` scaling differs between aircraft with different numbers of
detents. Asking for "one more" and checking is correct everywhere.

*Magnetic versus true.* All guidance is computed in true degrees; the heading
bug is magnetic. The conversion happens here, once, using the magnetic
variation the simulator reports for the aeroplane's present position.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from ..geo import normalize_deg
from ..perf.profiles import AircraftProfile
from ..sim.base import SimBackend, SimState

Logger = Callable[[str], None]


@dataclass
class AdapterCapabilities:
    """What this adapter can actually do in the aeroplane it is driving."""

    autopilot: bool = True
    autothrottle: bool = True
    vertical_speed: bool = True
    approach_mode: bool = True
    autoland: bool = True
    autobrake: bool = False
    managed_modes: bool = False       # Airbus-style push/pull
    needs_lvars: bool = False
    lvars_available: bool = True

    @property
    def degraded(self) -> bool:
        return self.needs_lvars and not self.lvars_available


class AircraftAdapter:
    """Standard-event adapter. Correct for the default 787 and most others."""

    key = "generic"

    #: How much the heading bug must be out before it is worth resending. Every
    #: event is a network round trip and the sim's own bug has finite
    #: resolution, so re-sending an unchanged value every frame is pure noise.
    heading_deadband_deg = 0.5
    altitude_deadband_ft = 40.0
    speed_deadband_kt = 1.0
    vs_deadband_fpm = 90.0

    def __init__(self, sim: SimBackend, profile: AircraftProfile,
                 log: Optional[Logger] = None) -> None:
        self.sim = sim
        self.profile = profile
        self.log = log or (lambda _message: None)
        self._last_heading: Optional[float] = None
        self._last_altitude: Optional[float] = None
        self._last_speed: Optional[float] = None
        self._last_mach: Optional[float] = None
        self._last_vs: Optional[float] = None
        self._flap_command_index: Optional[int] = None
        self._gear_down: Optional[bool] = None
        self._spoilers_armed = False
        self._approach_armed = False
        self._autothrottle_on = False
        self._toga = False
        self._vs_mode = False
        self._switches: dict[str, bool] = {}

    # -- Introspection -------------------------------------------------------
    def capabilities(self) -> AdapterCapabilities:
        return AdapterCapabilities(autoland=self.profile.autoland_capable)

    def describe(self) -> str:
        return f"{self.profile.name} via standard SimConnect events"

    def prepare(self) -> None:
        """Register anything the adapter needs before the first command."""

    # -- Autoflight ----------------------------------------------------------
    def engage_autopilot(self, state: SimState) -> None:
        if not state.ap_master:
            self.sim.send_event("AP_MASTER")
            self.log("Autopilot engaged")

    def disengage_autopilot(self, state: SimState) -> None:
        if state.ap_master:
            self.sim.send_event("AP_MASTER")
            self.log("Autopilot disengaged")

    def set_heading_true(self, true_deg: float, state: SimState) -> None:
        """Command a true heading; converts to the magnetic bug the sim wants."""
        magnetic = normalize_deg(true_deg - state.magvar_deg)
        self.set_heading_magnetic(magnetic)

    def set_heading_magnetic(self, magnetic_deg: float) -> None:
        target = normalize_deg(magnetic_deg)
        if self._last_heading is not None and \
                abs(_angle_gap(target, self._last_heading)) < self.heading_deadband_deg:
            return
        self._last_heading = target
        self.sim.send_event("HEADING_BUG_SET", int(round(target)) % 360)

    def select_heading_mode(self, state: SimState) -> None:
        if not state.ap_heading_lock:
            self.sim.send_event("AP_HDG_HOLD_ON")

    def set_altitude(self, altitude_ft: float) -> None:
        target = round(altitude_ft / 100.0) * 100.0
        if self._last_altitude is not None and \
                abs(target - self._last_altitude) < self.altitude_deadband_ft:
            return
        self._last_altitude = target
        self.sim.send_event("AP_ALT_VAR_SET_ENGLISH", int(round(target)))

    def select_altitude_mode(self, state: SimState) -> None:
        """Put the vertical channel back under altitude capture.

        The condition has to include our own vertical-speed flag, not just the
        aeroplane's altitude-lock flag. Selecting a vertical speed *also* sets
        altitude lock -- both are "the autopilot is controlling the vertical
        channel" -- so testing the flag alone means that once a vertical speed
        has been commanded, altitude capture can never be selected again. The
        aeroplane then holds the last rate it was given, and a go-around that
        commands a climb to three thousand feet sits at five hundred instead.
        """
        if not state.ap_altitude_lock or self._vs_mode:
            self.sim.send_event("AP_ALT_HOLD_ON")
            self._vs_mode = False
            self._last_vs = None

    def set_vertical_speed(self, fpm: float) -> None:
        target = round(fpm / 50.0) * 50.0
        if self._vs_mode and self._last_vs is not None and \
                abs(target - self._last_vs) < self.vs_deadband_fpm:
            return
        self._last_vs = target
        self._vs_mode = True
        self.sim.send_event("AP_VS_VAR_SET_ENGLISH", int(round(target)))
        self.sim.send_event("AP_PANEL_VS_HOLD", 1)

    def clear_vertical_speed(self) -> None:
        """Leave vertical-speed mode, so the next rate command is not filtered.

        Only clears the cache. Leaving the mode itself is
        :meth:`select_altitude_mode`, because that is a command to the
        aeroplane and this is not.
        """
        self._last_vs = None

    def set_speed_kt(self, kt: float) -> None:
        target = round(kt)
        if self._last_speed is not None and abs(target - self._last_speed) < self.speed_deadband_kt:
            return
        self._last_speed = target
        self._last_mach = None
        self.sim.send_event("AP_SPD_VAR_SET", int(target))

    def set_mach(self, mach: float) -> None:
        target = round(mach, 3)
        if self._last_mach is not None and abs(target - self._last_mach) < 0.004:
            return
        self._last_mach = target
        self._last_speed = None
        # The event takes Mach scaled by 100, so 0.85 goes across as 85.
        self.sim.send_event("AP_MACH_VAR_SET", int(round(target * 100)))

    def set_autothrottle(self, on: bool) -> None:
        if on and not self._autothrottle_on:
            self.sim.send_event("AUTO_THROTTLE_ARM")
            self._autothrottle_on = True
            self.log("Autothrottle armed")
        elif not on and self._autothrottle_on:
            self.sim.send_event("AUTO_THROTTLE_ARM")
            self._autothrottle_on = False

    def takeoff_thrust(self) -> None:
        """Set takeoff or go-around thrust. Safe to call every cycle."""
        self.sim.send_event("THROTTLE_SET", 16383)
        self.sim.send_event("AUTO_THROTTLE_TO_GA")
        if not self._toga:
            self._toga = True
            self.log("Takeoff thrust set")

    def clear_takeoff_thrust(self) -> None:
        """Forget that takeoff thrust was set, so a go-around announces itself."""
        self._toga = False

    def idle_thrust(self) -> None:
        self._toga = False
        self.sim.send_event("THROTTLE_SET", 0)

    # -- Approach ------------------------------------------------------------
    def tune_nav1(self, frequency_mhz: float, course_true_deg: Optional[float],
                  state: SimState) -> None:
        hz = int(round(frequency_mhz * 1_000_000))
        self.sim.send_event("NAV1_RADIO_SET_HZ", hz)
        if course_true_deg is not None:
            magnetic = normalize_deg(course_true_deg - state.magvar_deg)
            self.sim.send_event("VOR1_SET", int(round(magnetic)) % 360)
        self.log(f"NAV1 tuned {frequency_mhz:.2f}")

    def arm_approach(self, state: SimState) -> None:
        if self._approach_armed or state.ap_approach_hold:
            return
        self.sim.send_event("AP_APR_HOLD_ON")
        self._approach_armed = True
        self.log("Approach mode armed")

    def approach_is_captured(self, state: SimState) -> bool:
        """Whether the aeroplane's own ILS guidance has taken over."""
        return bool(state.ap_approach_hold and state.ap_glideslope_hold)

    # -- Configuration -------------------------------------------------------
    def set_flaps(self, index: int, state: SimState) -> bool:
        """Move the flap handle one detent towards ``index``.

        Returns ``True`` once the handle is where it was asked to be. Called
        every cycle by the configuration logic, so it walks the handle out one
        notch at a time as speed allows.
        """
        wanted = max(0, min(index, self._max_flap_index()))
        self._flap_command_index = wanted
        current = state.flaps_index
        if current == wanted:
            return True
        self.sim.send_event("FLAPS_INCR" if current < wanted else "FLAPS_DECR")
        return False

    def _max_flap_index(self) -> int:
        return max((f.index for f in self.profile.flaps), default=0)

    def set_gear(self, down: bool, state: SimState) -> None:
        if self._gear_down is down:
            return
        self._gear_down = down
        self.sim.send_event("GEAR_DOWN" if down else "GEAR_UP")
        self.log("Gear down" if down else "Gear up")

    def arm_spoilers(self) -> None:
        if self._spoilers_armed:
            return
        self.sim.send_event("SPOILERS_ARM_ON")
        self._spoilers_armed = True
        self.log("Speedbrake armed")

    def deploy_spoilers(self) -> None:
        self.sim.send_event("SPOILERS_ON")

    def retract_spoilers(self) -> None:
        self.sim.send_event("SPOILERS_OFF")

    def set_autobrake(self, level: int) -> None:
        """Best effort: the stock events only step the selector up and down."""
        for _ in range(max(0, level)):
            self.sim.send_event("INCREASE_AUTOBRAKE_CONTROL")

    def set_parking_brake(self, on: bool, state: SimState) -> None:
        if state.parking_brake != on:
            self.sim.send_event("PARKING_BRAKES")

    def apply_brakes(self) -> None:
        self.sim.send_event("BRAKES")

    # -- Lights, for the sake of looking like an aeroplane --------------------
    # Each is remembered, because these are called every cycle and the events
    # would otherwise be re-sent several times a second for a whole flight.
    def _switch(self, key: str, on: bool, on_event: str, off_event: str) -> None:
        if self._switches.get(key) is on:
            return
        self._switches[key] = on
        self.sim.send_event(on_event if on else off_event)

    def set_landing_lights(self, on: bool) -> None:
        self._switch("landing", on, "LANDING_LIGHTS_ON", "LANDING_LIGHTS_OFF")

    def set_strobes(self, on: bool) -> None:
        self._switch("strobe", on, "STROBES_ON", "STROBES_OFF")

    def set_taxi_lights(self, on: bool) -> None:
        self._switch("taxi", on, "TAXI_LIGHTS_ON", "TAXI_LIGHTS_OFF")


def _angle_gap(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0
