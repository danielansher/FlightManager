"""The boundary between the AI Pilot and Microsoft Flight Simulator.

Everything above this module works in the canonical units from
:mod:`aipilot.units` and never sees a SimConnect type. Two backends implement
:class:`SimBackend`: the real one over SimConnect (Windows, sim running) and a
point-mass :class:`~aipilot.sim.mock.MockSim` used by the test suite and by
``--sim mock`` so the whole flight can be flown without the game.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
from typing import Optional

from ..geo import LatLon, normalize_deg


@dataclass
class SimState:
    """One snapshot of the aeroplane, in canonical units."""

    # Position and attitude
    lat: float = 0.0
    lon: float = 0.0
    altitude_ft: float = 0.0          # indicated altitude, MSL
    altitude_agl_ft: float = 0.0
    ground_elevation_ft: float = 0.0
    pitch_deg: float = 0.0
    bank_deg: float = 0.0
    heading_true_deg: float = 0.0
    heading_mag_deg: float = 0.0
    track_true_deg: float = 0.0
    magvar_deg: float = 0.0

    # Speeds
    ias_kt: float = 0.0
    tas_kt: float = 0.0
    ground_speed_kt: float = 0.0
    mach: float = 0.0
    vertical_speed_fpm: float = 0.0

    # Environment
    wind_from_deg: float = 0.0
    wind_kt: float = 0.0
    sea_level_pressure_inhg: float = 29.92
    ambient_temp_c: float = 15.0

    # Configuration
    on_ground: bool = True
    gear_down_pct: float = 100.0
    flaps_index: int = 0
    flaps_pct: float = 0.0
    spoilers_pct: float = 0.0
    parking_brake: bool = True
    total_weight_lb: float = 0.0
    fuel_lb: float = 0.0
    engine_count: int = 2
    engines_running: bool = False

    # Autopilot feedback -- what the aeroplane says it is actually doing
    ap_master: bool = False
    ap_heading_lock: bool = False
    ap_altitude_lock: bool = False
    ap_nav_lock: bool = False
    ap_approach_hold: bool = False
    ap_glideslope_hold: bool = False
    ap_backcourse_hold: bool = False
    ap_autothrottle: bool = False
    ap_heading_bug_deg: float = 0.0
    ap_altitude_target_ft: float = 0.0
    ap_vs_target_fpm: float = 0.0
    ap_airspeed_target_kt: float = 0.0

    # Radio navigation
    nav1_freq_mhz: float = 0.0
    nav1_has_localizer: bool = False
    nav1_localizer_error_deg: float = 0.0
    nav1_has_glideslope: bool = False
    nav1_glideslope_error_deg: float = 0.0
    nav1_obs_deg: float = 0.0

    # Housekeeping
    sim_rate: float = 1.0
    sim_time_s: float = 0.0
    connected: bool = False

    @property
    def position(self) -> LatLon:
        return LatLon(self.lat, self.lon)

    @property
    def flight_level(self) -> int:
        return int(round(self.altitude_ft / 100.0))

    def with_position(self, pos: LatLon) -> "SimState":
        return replace(self, lat=pos.lat, lon=pos.lon)


@dataclass
class SimCapabilities:
    """What a backend can actually do.

    ``lvars`` is the interesting one: plain SimConnect cannot read or write an
    aircraft's local variables, so add-on aeroplanes whose autoflight panel is
    driven by L:Vars need the MobiFlight WASM module bridge. Adapters query
    this and degrade gracefully rather than silently sending commands into a
    void.
    """

    simvars: bool = True
    events: bool = True
    lvars: bool = False
    calculator_code: bool = False
    input_events: bool = False


class SimBackendError(RuntimeError):
    """Raised when the simulator connection fails or a request is rejected."""


class SimBackend(ABC):
    """Minimal surface the rest of the AI Pilot depends on."""

    name = "abstract"

    @abstractmethod
    def connect(self) -> None:
        """Open the connection. Raises :class:`SimBackendError` on failure."""

    @abstractmethod
    def close(self) -> None:
        """Release the connection. Must be safe to call more than once."""

    @abstractmethod
    def poll(self, dt: float) -> SimState:
        """Pump the connection and return the latest state.

        ``dt`` is the wall-clock interval since the previous poll, which the
        mock backend integrates over and the real backend ignores.
        """

    @abstractmethod
    def send_event(self, event: str, value: int = 0) -> None:
        """Transmit a simulator key event (a ``K:`` event such as ``AP_MASTER``)."""

    @abstractmethod
    def set_var(self, name: str, value: float, unit: str = "number") -> None:
        """Write a settable simulation variable (an ``A:`` var)."""

    def capabilities(self) -> SimCapabilities:
        return SimCapabilities()

    # --- Optional, WASM-module-backed operations ----------------------------
    def get_lvar(self, name: str) -> Optional[float]:
        """Read an aircraft local variable, or ``None`` if unsupported."""
        return None

    def set_lvar(self, name: str, value: float) -> bool:
        """Write an aircraft local variable. Returns whether it was sent."""
        return False

    def exec_calculator_code(self, code: str) -> bool:
        """Run a gauge RPN expression. Returns whether it was sent."""
        return False

    def list_lvars(self) -> list[str]:
        """Every local variable the module knows about, for the discovery tool."""
        return []

    # --- Conveniences -------------------------------------------------------
    def __enter__(self) -> "SimBackend":
        self.connect()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def normalize_heading(deg: float) -> float:
    return normalize_deg(deg)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def approach_value(current: float, target: float, max_step: float) -> float:
    """Move ``current`` towards ``target`` by at most ``max_step``."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)
