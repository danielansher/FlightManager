"""The vertical profile: where to climb, where to level, where to start down.

Top of descent is the one number an AI pilot has to get right. Start down late
and the aeroplane arrives high and fast and cannot get rid of the energy; start
down early and it spends twenty minutes droning along at low level. Real FMS
computes it from an idle-thrust path prediction; here it is computed from
geometry, which is what a crew does in their head anyway:

    distance to lose the height at a 3 degree gradient
      + the distance the approach itself occupies
      + an allowance for slowing down

with the whole thing anchored on the final approach fix rather than on the
airport, so the path arrives at a sensible height and speed to be configured
for landing rather than pointing at the runway from cruise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..perf.profiles import AircraftProfile
from ..units import FT_PER_NM

#: How far out the final approach fix sits. At 3 degrees this puts it at about
#: 1600 ft above the field, which is the standard platform altitude.
FAF_DISTANCE_NM = 5.0

#: Extra track distance to allow for decelerating from cruise Mach to approach
#: speed. Airliners need a surprising amount of it: you cannot slow down and go
#: down at the same time. It is spent by flying the whole descent slightly
#: shallower than the nominal angle rather than by levelling off somewhere.
DECELERATION_ALLOWANCE_NM = 8.0

#: The 250 kt restriction below 10,000 ft costs a little extra track distance.
LOW_LEVEL_DECEL_NM = 3.0


def gradient_ft_per_nm(angle_deg: float) -> float:
    """Height lost per nautical mile on a given descent angle."""
    return FT_PER_NM * math.tan(math.radians(angle_deg))


@dataclass(frozen=True)
class VerticalProfile:
    """Altitude targets as a function of distance still to fly.

    All distances are measured to the *destination threshold* along the route,
    which is what the guidance has readily to hand.
    """

    cruise_altitude_ft: float
    field_elevation_ft: float
    descent_angle_deg: float = 3.0
    faf_distance_nm: float = FAF_DISTANCE_NM
    faf_altitude_ft: float = 0.0
    top_of_descent_nm: float = 0.0        # distance to run at which to start down
    threshold_crossing_ft: float = 50.0

    @property
    def gradient(self) -> float:
        return gradient_ft_per_nm(self.descent_angle_deg)

    @property
    def effective_gradient(self) -> float:
        """Height lost per mile actually flown between TOD and the FAF.

        Slightly shallower than :attr:`gradient`, because the deceleration
        allowance is spent by flying the whole descent a little flatter rather
        than by levelling off part way down. That is both what a real econ
        descent looks like and what keeps the aeroplane able to slow down: a
        jet at idle on a 3 degree path does not decelerate, it just goes down.
        """
        run = self.top_of_descent_nm - self.faf_distance_nm
        if run <= 0.1:
            return self.gradient
        return (self.cruise_altitude_ft - self.faf_altitude_ft) / run

    @property
    def effective_angle_deg(self) -> float:
        return math.degrees(math.atan(self.effective_gradient / FT_PER_NM))

    def target_altitude_at(self, distance_to_go_nm: float) -> float:
        """The altitude the descent path wants at this distance to run."""
        if distance_to_go_nm >= self.top_of_descent_nm:
            return self.cruise_altitude_ft
        if distance_to_go_nm <= self.faf_distance_nm:
            # On the glidepath proper, anchored at the threshold at the
            # published angle -- this part must match what the ILS will fly.
            height = self.threshold_crossing_ft + max(0.0, distance_to_go_nm) * self.gradient
            return self.field_elevation_ft + height
        # Between top of descent and the final approach fix: a straight line
        # joining the cruise level to the FAF altitude, so the descent starts
        # the moment we pass top of descent.
        above_faf = (distance_to_go_nm - self.faf_distance_nm) * self.effective_gradient
        return min(self.cruise_altitude_ft, self.faf_altitude_ft + above_faf)

    def required_vertical_speed_fpm(self, distance_to_go_nm: float, current_altitude_ft: float,
                                    ground_speed_kt: float, lookahead_nm: float = 1.0) -> float:
        """Vertical speed to be on the path a short distance further along.

        Using a lookahead point rather than the present one means the commanded
        rate converges onto the path instead of chasing it, which is what stops
        the classic sawtooth of a naive path follower.
        """
        if ground_speed_kt <= 10.0:
            return 0.0
        ahead = max(0.0, distance_to_go_nm - lookahead_nm)
        target = self.target_altitude_at(ahead)
        minutes = (lookahead_nm / ground_speed_kt) * 60.0
        if minutes <= 0.0:
            return 0.0
        return (target - current_altitude_ft) / minutes

def build_vertical_profile(cruise_altitude_ft: float, field_elevation_ft: float,
                           profile: AircraftProfile,
                           faf_distance_nm: float = FAF_DISTANCE_NM) -> VerticalProfile:
    """Work out the descent geometry, including where top of descent falls."""
    angle = profile.descent_angle_deg
    gradient = gradient_ft_per_nm(angle)
    threshold_crossing = 50.0
    # Take the final approach fix altitude from the glidepath itself rather
    # than from a nominal platform height. If the two disagree the profile has
    # a step in it at the FAF, and the aeroplane arrives at the fix a hundred
    # feet off the slope it is about to intercept.
    faf_altitude = field_elevation_ft + threshold_crossing + faf_distance_nm * gradient

    height_to_lose = max(0.0, cruise_altitude_ft - faf_altitude)
    descent_distance = height_to_lose / gradient if gradient > 0 else 0.0

    allowance = DECELERATION_ALLOWANCE_NM
    if cruise_altitude_ft > 10000.0:
        allowance += LOW_LEVEL_DECEL_NM
    # A heavier, higher-Mach aeroplane needs a little more room to slow down.
    allowance *= 1.0 + max(0.0, profile.cruise_mach - 0.78) * 2.0

    return VerticalProfile(
        cruise_altitude_ft=cruise_altitude_ft,
        field_elevation_ft=field_elevation_ft,
        descent_angle_deg=angle,
        faf_distance_nm=faf_distance_nm,
        faf_altitude_ft=faf_altitude,
        threshold_crossing_ft=threshold_crossing,
        top_of_descent_nm=descent_distance + faf_distance_nm + allowance,
    )


def climb_speed_target(altitude_ft: float, profile: AircraftProfile,
                       transition_altitude_ft: float = 10000.0) -> tuple[float, bool]:
    """Climb speed target. Returns ``(value, is_mach)``."""
    if altitude_ft < transition_altitude_ft:
        return (profile.speed_below_10k_kt, False)
    if altitude_ft < profile.climb_crossover_ft:
        return (profile.climb_speed_kt, False)
    return (profile.climb_mach, True)


def cruise_speed_target(altitude_ft: float, profile: AircraftProfile,
                        transition_altitude_ft: float = 10000.0):
    """Cruise speed target. Returns ``(value, is_mach)``.

    Cruise is not automatically a Mach number. On a short sector the cruise
    level can be three thousand feet, where the type's cruise Mach works out at
    something like five hundred and sixty knots indicated -- comfortably beyond
    Vmo, and beyond anything the airframe would survive. Commanding it produced
    exactly the low-level overspeed it sounds like it would.
    """
    if altitude_ft < transition_altitude_ft:
        return (profile.speed_below_10k_kt, False)
    if altitude_ft < profile.climb_crossover_ft:
        return (profile.climb_speed_kt, False)
    return (profile.cruise_mach, True)


def descent_speed_target(altitude_ft: float, profile: AircraftProfile,
                         transition_altitude_ft: float = 10000.0) -> tuple[float, bool]:
    """Descent speed target. Returns ``(value, is_mach)``."""
    if altitude_ft >= profile.descent_crossover_ft:
        return (profile.descent_mach, True)
    if altitude_ft >= transition_altitude_ft:
        return (profile.descent_speed_kt, False)
    return (profile.speed_below_10k_kt, False)
