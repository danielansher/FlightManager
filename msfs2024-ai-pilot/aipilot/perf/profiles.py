"""Performance profiles for the aeroplanes the AI Pilot knows how to fly.

These are operational numbers -- climb and descent speed schedules, flap
placard speeds, typical approach speeds, sensible cruise levels -- at the
fidelity a real crew works to, not certification data. They are what the
guidance uses to decide when to start down, how fast to fly, and when to put
the flaps out.

Everything here can be overridden from JSON without touching code, because
these vary with weight and with the particular add-on's flight model, and
someone who flies one aeroplane a lot will know better than this table does.
See :func:`load_profile_overrides`.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, replace
from typing import Optional

from ..units import crossover_altitude_ft


@dataclass(frozen=True)
class FlapSetting:
    """One flap detent: the handle index, its placard speed, and its name."""

    index: int
    max_speed_kt: float
    label: str = ""

    def __str__(self) -> str:  # pragma: no cover - display only
        return self.label or str(self.index)


@dataclass(frozen=True)
class AircraftProfile:
    """Everything the guidance needs to know about how a type flies."""

    key: str
    name: str
    icao_type: str

    # Speed schedules
    climb_speed_kt: float = 300.0
    climb_mach: float = 0.84
    cruise_mach: float = 0.85
    descent_mach: float = 0.84
    descent_speed_kt: float = 300.0
    speed_below_10k_kt: float = 250.0
    terminal_speed_kt: float = 210.0      # after the approach transition
    final_approach_speed_kt: float = 145.0
    v2_kt: float = 165.0
    initial_climb_speed_kt: float = 180.0

    # Envelope
    max_altitude_ft: float = 43000.0
    typical_cruise_ft: float = 37000.0
    max_climb_rate_fpm: float = 2500.0
    max_descent_rate_fpm: float = 3000.0
    max_bank_deg: float = 25.0
    approach_bank_deg: float = 20.0

    # Configuration
    takeoff_flaps_index: int = 1
    landing_flaps_index: int = 5
    flaps: tuple[FlapSetting, ...] = ()
    gear_extend_speed_kt: float = 250.0
    descent_angle_deg: float = 3.0

    # Behaviour
    autoland_capable: bool = True
    autothrottle: bool = True
    notes: str = ""

    # -- Derived -------------------------------------------------------------
    @property
    def climb_crossover_ft(self) -> float:
        """Where the climb schedule changes from a CAS to a Mach target."""
        return crossover_altitude_ft(self.climb_speed_kt, self.climb_mach)

    @property
    def descent_crossover_ft(self) -> float:
        return crossover_altitude_ft(self.descent_speed_kt, self.descent_mach)

    def flap_for_speed(self, speed_kt: float) -> Optional[FlapSetting]:
        """The most extended flap setting legal at this speed."""
        legal = [f for f in self.flaps if f.index > 0 and speed_kt <= f.max_speed_kt]
        return max(legal, key=lambda f: f.index) if legal else None

    def flap(self, index: int) -> Optional[FlapSetting]:
        for setting in self.flaps:
            if setting.index == index:
                return setting
        return None

    @property
    def landing_flaps(self) -> Optional[FlapSetting]:
        return self.flap(self.landing_flaps_index)

    def with_overrides(self, data: dict) -> "AircraftProfile":
        """Apply a dict of field overrides, ignoring unknown keys."""
        fields = {f for f in asdict(self)}
        clean = {k: v for k, v in data.items() if k in fields and k != "flaps"}
        profile = replace(self, **clean)
        if "flaps" in data:
            profile = replace(profile, flaps=tuple(
                FlapSetting(int(f["index"]), float(f["max_speed_kt"]), f.get("label", ""))
                for f in data["flaps"]
            ))
        return profile


def _boeing_flaps() -> tuple[FlapSetting, ...]:
    """787 flap placard speeds, in handle-index order."""
    return (
        FlapSetting(0, 999.0, "UP"),
        FlapSetting(1, 255.0, "1"),
        FlapSetting(2, 235.0, "5"),
        FlapSetting(3, 215.0, "15"),
        FlapSetting(4, 205.0, "20"),
        FlapSetting(5, 180.0, "30"),
    )


def _airbus_flaps(vfe: tuple[float, float, float, float]) -> tuple[FlapSetting, ...]:
    """Airbus CONF 1 / 2 / 3 / FULL against the handle indices the sim uses."""
    return (
        FlapSetting(0, 999.0, "UP"),
        FlapSetting(1, vfe[0], "1"),
        FlapSetting(2, vfe[1], "2"),
        FlapSetting(3, vfe[2], "3"),
        FlapSetting(4, vfe[3], "FULL"),
    )


PROFILES: dict[str, AircraftProfile] = {}


def _register(profile: AircraftProfile) -> AircraftProfile:
    PROFILES[profile.key] = profile
    return profile


B787_10 = _register(AircraftProfile(
    key="b787-10",
    name="Boeing 787-10 Dreamliner",
    icao_type="B78X",
    climb_speed_kt=300.0, climb_mach=0.84,
    cruise_mach=0.85, descent_mach=0.84, descent_speed_kt=300.0,
    final_approach_speed_kt=150.0, v2_kt=170.0, initial_climb_speed_kt=190.0,
    max_altitude_ft=43100.0, typical_cruise_ft=37000.0,
    max_climb_rate_fpm=2400.0, max_descent_rate_fpm=3000.0,
    takeoff_flaps_index=2, landing_flaps_index=5,
    flaps=_boeing_flaps(), gear_extend_speed_kt=250.0,
    notes="Default MSFS 2024 aeroplane. Flies entirely on standard autopilot "
          "events, so it needs no WASM module.",
))

B787_9 = _register(replace(
    B787_10, key="b787-9", name="Boeing 787-9 Dreamliner", icao_type="B789",
    typical_cruise_ft=39000.0, final_approach_speed_kt=145.0, v2_kt=165.0,
    max_climb_rate_fpm=2600.0,
))

A350_900 = _register(AircraftProfile(
    key="a350-900",
    name="Airbus A350-900 (iniBuilds)",
    icao_type="A359",
    climb_speed_kt=300.0, climb_mach=0.84,
    cruise_mach=0.85, descent_mach=0.84, descent_speed_kt=300.0,
    final_approach_speed_kt=142.0, v2_kt=160.0, initial_climb_speed_kt=185.0,
    max_altitude_ft=43100.0, typical_cruise_ft=39000.0,
    max_climb_rate_fpm=2400.0, max_descent_rate_fpm=3000.0,
    takeoff_flaps_index=1, landing_flaps_index=4,
    flaps=_airbus_flaps((263.0, 222.0, 204.0, 196.0)),
    gear_extend_speed_kt=250.0,
    notes="Airbus autoflight lives in local variables, so the FCU is driven "
          "through the WASM bridge where one is available and through standard "
          "events otherwise.",
))

A350_1000 = _register(replace(
    A350_900, key="a350-1000", name="Airbus A350-1000 (iniBuilds)", icao_type="A35K",
    final_approach_speed_kt=146.0, v2_kt=165.0, typical_cruise_ft=38000.0,
))

A380_800 = _register(AircraftProfile(
    key="a380-800",
    name="Airbus A380-800",
    icao_type="A388",
    climb_speed_kt=300.0, climb_mach=0.84,
    cruise_mach=0.85, descent_mach=0.84, descent_speed_kt=300.0,
    final_approach_speed_kt=140.0, v2_kt=160.0, initial_climb_speed_kt=185.0,
    max_altitude_ft=43000.0, typical_cruise_ft=37000.0,
    # The heaviest thing in the fleet: shallower climb, gentler manoeuvring.
    max_climb_rate_fpm=1900.0, max_descent_rate_fpm=2800.0,
    max_bank_deg=22.0, approach_bank_deg=18.0,
    takeoff_flaps_index=2, landing_flaps_index=4,
    flaps=_airbus_flaps((263.0, 222.0, 196.0, 182.0)),
    gear_extend_speed_kt=250.0,
    notes="Very heavy: step climbs matter, and the descent is planned shallower "
          "than the rest of the fleet.",
))

A330_900 = _register(AircraftProfile(
    key="a330-900",
    name="Airbus A330-900neo (Headwind)",
    icao_type="A339",
    climb_speed_kt=300.0, climb_mach=0.82,
    cruise_mach=0.82, descent_mach=0.82, descent_speed_kt=290.0,
    final_approach_speed_kt=138.0, v2_kt=155.0, initial_climb_speed_kt=180.0,
    max_altitude_ft=41100.0, typical_cruise_ft=37000.0,
    max_climb_rate_fpm=2200.0, max_descent_rate_fpm=2800.0,
    takeoff_flaps_index=1, landing_flaps_index=4,
    flaps=_airbus_flaps((240.0, 196.0, 186.0, 180.0)),
    gear_extend_speed_kt=250.0,
    notes="Community A330neo. Built on the FlyByWire codebase, so its local "
          "variables mostly follow the A32NX naming convention.",
))

A320_NEO = _register(AircraftProfile(
    key="a320neo",
    name="Airbus A320neo",
    icao_type="A20N",
    climb_speed_kt=290.0, climb_mach=0.78,
    cruise_mach=0.78, descent_mach=0.78, descent_speed_kt=280.0,
    final_approach_speed_kt=135.0, v2_kt=145.0, initial_climb_speed_kt=170.0,
    max_altitude_ft=39100.0, typical_cruise_ft=36000.0,
    max_climb_rate_fpm=2500.0, max_descent_rate_fpm=2800.0,
    takeoff_flaps_index=1, landing_flaps_index=4,
    flaps=_airbus_flaps((230.0, 200.0, 185.0, 177.0)),
    gear_extend_speed_kt=250.0,
    notes="Included because it is the aeroplane the original MSFS 2020 AI Pilot "
          "was built around, which makes it the natural reference for checking "
          "that this one behaves sensibly.",
))

GENERIC_JET = _register(AircraftProfile(
    key="generic",
    name="Generic airliner",
    icao_type="ZZZZ",
    flaps=_boeing_flaps(),
    notes="Fallback for a type with no profile. Conservative throughout.",
))


def get_profile(key: str) -> Optional[AircraftProfile]:
    return PROFILES.get(key.strip().lower())


def profile_for_icao_type(icao_type: str) -> Optional[AircraftProfile]:
    wanted = icao_type.strip().upper()
    for profile in PROFILES.values():
        if profile.icao_type == wanted:
            return profile
    return None


def load_profile_overrides(path: str) -> list[str]:
    """Apply a JSON file of per-type overrides. Returns the keys it changed.

    The file maps a profile key to the fields to change::

        {"a380-800": {"cruise_mach": 0.84, "typical_cruise_ft": 35000}}
    """
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    changed = []
    for key, overrides in data.items():
        profile = PROFILES.get(key.lower())
        if profile is None:
            continue
        PROFILES[key.lower()] = profile.with_overrides(overrides)
        changed.append(key.lower())
    return changed


def select_cruise_altitude(distance_nm: float, course_deg: float,
                           profile: AircraftProfile) -> float:
    """A sensible initial cruise level for the trip.

    Short sectors do not have time to climb high, and long ones want the
    efficiency, so the level scales with distance and is then snapped to the
    correct semicircular direction: odd thousands eastbound, even westbound.
    """
    if distance_nm < 150:
        target = 20000.0
    elif distance_nm < 300:
        target = 26000.0
    elif distance_nm < 600:
        target = 31000.0
    elif distance_nm < 1200:
        target = min(35000.0, profile.typical_cruise_ft)
    else:
        target = profile.typical_cruise_ft
    target = min(target, profile.max_altitude_ft - 1000.0)

    eastbound = 0.0 <= (course_deg % 360.0) < 180.0
    # Above FL410 the semicircular rule reverts to 2000 ft steps; we stay below.
    thousands = int(round(target / 1000.0))
    if eastbound and thousands % 2 == 0:
        thousands -= 1
    elif not eastbound and thousands % 2 == 1:
        thousands -= 1
    return max(10000.0, float(thousands) * 1000.0)
