"""Vertical and speed guidance.

The vertical channel is where an autopilot most obviously either does or does
not look like a pilot. Two rules shape what is here:

*Set the selector to where you are going, fly the path with rate.* In the
descent the altitude selector is put on the next hard constraint below and the
vertical speed is used to follow the computed path. Feeding the instantaneous
path altitude into the altitude selector every second instead produces a
staircase, and the aeroplane spends the descent hunting.

*Never command a rate the aeroplane cannot fly.* Commands are clamped to the
type's climb and descent limits, so being high on the path produces a maximum-
rate descent and a note in the log, rather than a nonsense number and an
autopilot that gives up.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..perf.profiles import AircraftProfile
from ..route.plan import FlightPlan, RouteLeg
from ..route.profile import (
    VerticalProfile,
    climb_speed_target,
    cruise_speed_target,
    descent_speed_target,
)
from .phases import Phase

#: Height above the field to level off at if the approach is not yet stable.
MIN_APPROACH_ALTITUDE_AGL_FT = 1500.0

#: How close to the target level counts as level.
LEVEL_TOLERANCE_FT = 250.0


@dataclass
class VerticalCommand:
    """What the vertical channel wants."""

    altitude_ft: float
    vertical_speed_fpm: Optional[float]   # None: let the aeroplane manage it
    speed: float
    speed_is_mach: bool
    reason: str = ""
    off_path_ft: float = 0.0


class VerticalGuidance:
    """Computes altitude, vertical speed and speed targets for each phase."""

    def __init__(self, plan: FlightPlan, profile: AircraftProfile,
                 vertical_profile: VerticalProfile) -> None:
        self.plan = plan
        self.profile = profile
        self.vertical = vertical_profile

    # -- Constraints ---------------------------------------------------------
    def next_constraint(self, active_index: int) -> Optional[RouteLeg]:
        """The next fix ahead with an altitude constraint on it."""
        for leg in self.plan.legs[active_index:]:
            if leg.altitude_ft is not None and leg.phase != "takeoff":
                return leg
        return None

    def descent_floor_ft(self, active_index: int, altitude_ft: Optional[float] = None) -> float:
        """Altitude to put in the selector during the descent.

        Never above the aeroplane. On a short sector the cruise level is
        capped by how much room there is to climb and get down again, while
        the approach fixes are still built on an unclipped three degree slope
        -- so the next constraint could sit thousands of feet above an
        aeroplane that was being told to descend at the same moment. On a real
        MCP that is a mode conflict: a vertical speed away from the selected
        altitude is either refused or captured upwards, and the aeroplane
        climbs, or sits at its level, exactly when it needs to start down.
        """
        constraint = self.next_constraint(active_index)
        floor = self.vertical.faf_altitude_ft
        if constraint is not None and constraint.altitude_ft:
            floor = constraint.altitude_ft
        if altitude_ft is not None:
            floor = min(floor, altitude_ft)
        return floor

    # -- The command ---------------------------------------------------------
    def update(self, phase: Phase, altitude_ft: float, distance_to_go_nm: float,
               ground_speed_kt: float, active_index: int,
               agl_ft: float = 0.0) -> VerticalCommand:
        if phase in (Phase.TAKEOFF,):
            return VerticalCommand(
                altitude_ft=self.plan.cruise_altitude_ft,
                vertical_speed_fpm=None,
                speed=self.profile.initial_climb_speed_kt,
                speed_is_mach=False,
                reason="initial climb",
            )

        if phase is Phase.CLIMB:
            speed, is_mach = climb_speed_target(altitude_ft, self.profile)
            return VerticalCommand(
                altitude_ft=self.plan.cruise_altitude_ft,
                vertical_speed_fpm=None,          # the aeroplane climbs at its own rate
                speed=speed,
                speed_is_mach=is_mach,
                reason=f"climb to FL{self.plan.cruise_altitude_ft / 100:.0f}",
            )

        if phase is Phase.CRUISE:
            speed, is_mach = cruise_speed_target(altitude_ft, self.profile)
            return VerticalCommand(
                altitude_ft=self.plan.cruise_altitude_ft,
                vertical_speed_fpm=None,
                speed=speed,
                speed_is_mach=is_mach,
                reason="cruise",
            )

        if phase is Phase.DESCENT:
            return self._descent(altitude_ft, distance_to_go_nm, ground_speed_kt, active_index)

        if phase in (Phase.APPROACH, Phase.LANDING):
            return self._approach(altitude_ft, distance_to_go_nm, ground_speed_kt,
                                  active_index, agl_ft)

        # Ground phases: hold what we have.
        return VerticalCommand(altitude_ft=altitude_ft, vertical_speed_fpm=None,
                               speed=0.0, speed_is_mach=False, reason="on the ground")

    def _descent(self, altitude_ft: float, distance_to_go_nm: float,
                 ground_speed_kt: float, active_index: int) -> VerticalCommand:
        path_altitude = self.vertical.target_altitude_at(distance_to_go_nm)
        off_path = altitude_ft - path_altitude
        required = self.vertical.required_vertical_speed_fpm(
            distance_to_go_nm, altitude_ft, ground_speed_kt
        )
        commanded = max(-self.profile.max_descent_rate_fpm, min(0.0, required))

        speed, is_mach = descent_speed_target(altitude_ft, self.profile)
        reason = "on the descent path"
        if off_path > 400.0:
            reason = f"{off_path:.0f} ft high on the path"
        elif off_path < -400.0:
            reason = f"{-off_path:.0f} ft low on the path"
            # Below the path: stop descending and let the path come to us.
            commanded = max(commanded, -500.0)

        return VerticalCommand(
            altitude_ft=self.descent_floor_ft(active_index, altitude_ft),
            vertical_speed_fpm=commanded,
            speed=speed,
            speed_is_mach=is_mach,
            reason=reason,
            off_path_ft=off_path,
        )

    def _approach(self, altitude_ft: float, distance_to_go_nm: float,
                  ground_speed_kt: float, active_index: int,
                  agl_ft: float) -> VerticalCommand:
        path_altitude = self.vertical.target_altitude_at(distance_to_go_nm)
        floor = max(
            self.vertical.field_elevation_ft + 50.0,
            min(path_altitude, self.descent_floor_ft(active_index)),
        )
        required = self.vertical.required_vertical_speed_fpm(
            distance_to_go_nm, altitude_ft, ground_speed_kt, lookahead_nm=0.6
        )
        # An approach is flown at a stable rate; a 2000 fpm correction on final
        # is a go-around, not a correction.
        commanded = max(-1800.0, min(500.0, required))

        speed = self._approach_speed(distance_to_go_nm)
        return VerticalCommand(
            altitude_ft=floor,
            vertical_speed_fpm=commanded,
            speed=speed,
            speed_is_mach=False,
            reason=f"approach, {distance_to_go_nm:.1f} nm to run",
            off_path_ft=altitude_ft - path_altitude,
        )

    def _approach_speed(self, distance_to_go_nm: float) -> float:
        """Speed schedule down final: terminal speed, then Vapp by the gate."""
        vapp = self.profile.final_approach_speed_kt
        if distance_to_go_nm > 20.0:
            return self.profile.terminal_speed_kt
        if distance_to_go_nm > 12.0:
            return max(vapp, 210.0)
        if distance_to_go_nm > 8.0:
            return max(vapp, 180.0)
        if distance_to_go_nm > 5.0:
            return max(vapp, 160.0)
        return vapp


def should_start_descent(distance_to_go_nm: float, vertical: VerticalProfile,
                         altitude_ft: float) -> bool:
    """Whether it is time to leave the cruise level.

    Purely a distance test. An earlier version also returned true whenever the
    aeroplane was already near the final approach altitude, which is correct
    over the destination and catastrophic on the runway at the other end: on
    departure the aeroplane is by definition near field elevation, and it
    happily declared top of descent forty seconds after takeoff.
    """
    return distance_to_go_nm <= vertical.top_of_descent_nm
