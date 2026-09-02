"""Lateral guidance -- the LNAV the stock autopilot does not give you.

The autopilot in the simulator can hold a heading. It cannot fly a route. This
module is the part that turns a list of fixes into a heading to hold, and then
into the decision of when one fix has been passed and the next becomes active.

Two things it gets right that a naive implementation does not:

*It tracks the centreline, not the fix.* Steering straight at the active
waypoint converges on the waypoint but not on the path -- blown ten miles off
track by a jet stream, an aeroplane doing that flies a long diagonal and
arrives at the fix from the wrong side. Here the commanded track is the leg's
own course plus a correction proportional to cross-track error, which returns
to the centreline and then stays on it. On final approach that difference is
the difference between landing and going around.

*It turns early.* A fix is sequenced when the aeroplane reaches the point where
a normal-rate turn will roll out on the next leg, not when it flies over it.
Waiting until overhead means every turn overshoots and then S-turns back.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from ..geo import (
    LatLon,
    along_track_nm,
    cross_track_nm,
    destination_point,
    distance_nm,
    initial_bearing_deg,
    normalize_deg,
    turn_anticipation_nm,
    turn_radius_nm,
    wind_correction_angle_deg,
)
from ..route.plan import FlightPlan

#: Degrees of correction per nautical mile off track. 3 deg/nm gives a 30
#: degree intercept at 10 nm out, which is the angle a controller would give
#: you and which rolls out without overshooting.
XTK_GAIN_DEG_PER_NM = 3.0

#: Never intercept at more than this. Beyond about 45 degrees the geometry
#: stops converging usefully and the aeroplane just flies sideways.
MAX_INTERCEPT_DEG = 45.0

#: Bank limit on final. A steep turn close to the ground is neither
#: comfortable nor stable.
FINAL_BANK_LIMIT_DEG = 15.0

#: On final the limit is lower: a large intercept angle close in cannot be
#: flown out before the threshold.
FINAL_INTERCEPT_DEG = 25.0

#: On final the correction is worked out by pure pursuit -- aim at a point on
#: the centreline some way ahead -- rather than by a gain per mile.
#:
#: A linear gain converges asymptotically, and asymptotically is not good
#: enough when the track runs out at the threshold. At ten degrees per mile a
#: hundred yards of offset earns less than a degree of correction, which over
#: the mile that is left closes almost nothing: the aeroplane arrived at the
#: stabilisation gate five hundred feet to one side and touched down in the
#: grass beside a two-hundred-foot runway, having reported itself lined up the
#: whole way down. Pure pursuit gives a correction that grows with the offset
#: relative to how far ahead it is looking, so it actually arrives.
#:
#: The lookahead is derived from the turning circle, not fixed. A lookahead
#: point *inside* the turning circle cannot be reached -- the aeroplane turns
#: towards it, misses, and weaves across the centreline -- and that is
#: geometry rather than tuning, which is what a hard-coded distance gets wrong
#: the moment the speed changes. The same reasoning, and the same factor, as
#: the taxi guidance.
FINAL_LOOKAHEAD_FACTOR = 1.3
FINAL_LOOKAHEAD_FLOOR_NM = 0.5

#: Sequence a fix when this close even if the turn geometry says otherwise --
#: covers a fix flown at very low speed, and the last fix of the route.
MIN_SEQUENCE_NM = 0.4


@dataclass
class LateralCommand:
    """What the lateral channel wants, and why."""

    heading_true_deg: float
    desired_track_deg: float
    cross_track_nm: float
    distance_to_fix_nm: float
    bank_limit_deg: float
    sequenced: bool = False
    active_index: int = 0
    reason: str = ""


class LateralGuidance:
    """Follows a :class:`~aipilot.route.plan.FlightPlan`."""

    def __init__(self, plan: FlightPlan, max_bank_deg: float = 25.0) -> None:
        self.plan = plan
        self.max_bank_deg = max_bank_deg
        #: Index of the fix being flown *to*. Leg 0 is the runway, so the first
        #: fix we actually track towards is 1.
        self.active_index = 1 if len(plan) > 1 else 0
        #: Origin of the current leg when flying direct to a fix, replacing the
        #: preceding route fix. Cleared as soon as the leg sequences.
        self._direct_origin: Optional[LatLon] = None

    # -- Geometry helpers ----------------------------------------------------
    @property
    def active_leg(self):
        return self.plan[min(self.active_index, len(self.plan) - 1)]

    @property
    def previous_position(self) -> LatLon:
        """Where the active leg starts.

        Normally the preceding route fix, but after a direct-to it is the
        position the aeroplane was in when the instruction was given. Without
        that substitution a direct-to leaves the aeroplane tracking a leg whose
        origin is hundreds of miles behind, and if it happens to be past the
        far end of that leg it flies the leg's course outbound for ever --
        cross-track error obediently near zero the whole way, because it is
        precisely on the extended centreline, going the wrong way.
        """
        if self._direct_origin is not None:
            return self._direct_origin
        index = max(0, self.active_index - 1)
        return self.plan[index].position

    @property
    def finished(self) -> bool:
        return self.active_index >= len(self.plan) - 1

    def distance_to_end_nm(self, position: LatLon) -> float:
        return self.plan.distance_to_end_nm(position, self.active_index)

    def leg_course_deg(self, position: LatLon) -> float:
        """Course of the leg centreline abeam the aeroplane.

        Taken at the point on the leg the aeroplane is abeam of rather than at
        the leg start, because on an ocean crossing those differ by tens of
        degrees.
        """
        start = self.previous_position
        end = self.active_leg.position
        leg_length = distance_nm(start, end)
        if leg_length < 0.1:
            return initial_bearing_deg(start, end) if leg_length > 0 else 0.0
        travelled = min(max(along_track_nm(position, start, end), 0.0), leg_length - 0.05)
        abeam = destination_point(start, initial_bearing_deg(start, end), travelled)
        return initial_bearing_deg(abeam, end)

    # -- The command ---------------------------------------------------------
    def update(self, position: LatLon, tas_kt: float, wind_from_deg: float,
               wind_kt: float, approach_mode: bool = False) -> LateralCommand:
        """Compute the heading to fly, sequencing the route as required."""
        sequenced = self._maybe_sequence(position, tas_kt)

        leg = self.active_leg
        start = self.previous_position
        end = leg.position
        is_final = approach_mode or leg.phase in ("final", "landing")

        leg_course = self.leg_course_deg(position)
        xtk = cross_track_nm(position, start, end) if distance_nm(start, end) > 0.1 else 0.0

        bank_limit = self.max_bank_deg
        if is_final:
            bank_limit = min(bank_limit, FINAL_BANK_LIMIT_DEG)

        if is_final:
            limit = FINAL_INTERCEPT_DEG
            lookahead = max(FINAL_LOOKAHEAD_FLOOR_NM,
                            turn_radius_nm(tas_kt, bank_limit)
                            * FINAL_LOOKAHEAD_FACTOR)
            correction = math.degrees(math.atan2(-xtk, lookahead))
        else:
            limit = MAX_INTERCEPT_DEG
            correction = -xtk * XTK_GAIN_DEG_PER_NM
        correction = max(-limit, min(limit, correction))
        desired_track = normalize_deg(leg_course + correction)

        wca = wind_correction_angle_deg(desired_track, tas_kt, wind_from_deg, wind_kt)
        heading = normalize_deg(desired_track + wca)

        return LateralCommand(
            heading_true_deg=heading,
            desired_track_deg=desired_track,
            cross_track_nm=xtk,
            distance_to_fix_nm=distance_nm(position, end),
            bank_limit_deg=bank_limit,
            sequenced=sequenced,
            active_index=self.active_index,
            reason=f"track {desired_track:.0f} to {leg.ident}",
        )

    def _maybe_sequence(self, position: LatLon, tas_kt: float) -> bool:
        """Advance to the next fix if this one is effectively behind us."""
        if self.finished:
            return False
        leg = self.plan[self.active_index]
        distance = distance_nm(position, leg.position)

        if leg.flyover:
            # A flyover fix is passed when it is genuinely behind, judged by the
            # along-track projection rather than by distance -- otherwise a fix
            # missed slightly wide never sequences at all.
            start = self.previous_position
            leg_length = distance_nm(start, leg.position)
            if leg_length > 0.1:
                passed = along_track_nm(position, start, leg.position) >= leg_length
            else:
                passed = distance <= MIN_SEQUENCE_NM
            if not (passed or distance <= MIN_SEQUENCE_NM):
                return False
        else:
            course_change = self.plan.course_change_at_deg(self.active_index)
            anticipation = turn_anticipation_nm(tas_kt, course_change, self.max_bank_deg)
            if distance > max(anticipation, MIN_SEQUENCE_NM) and \
                    not self._is_behind(position, leg):
                return False

        self.active_index = min(self.active_index + 1, len(self.plan) - 1)
        self._direct_origin = None
        return True

    def _is_behind(self, position: LatLon, leg) -> bool:
        """Whether the aeroplane has flown past the fix.

        A fly-by fix normally sequences on turn anticipation, which is a
        distance test -- and a distance test can only ever say "not yet" once
        the fix is behind and getting further away. Any manoeuvre that leaves
        the aeroplane past its active fix (a go-around, a direct-to, engaging
        in mid-air) would strand it there permanently.
        """
        start = self.previous_position
        leg_length = distance_nm(start, leg.position)
        if leg_length < 0.1:
            return True
        return along_track_nm(position, start, leg.position) > leg_length

    def direct_to(self, index: int, from_position: Optional[LatLon] = None) -> None:
        """Fly straight to a fix, as a crew would when given 'direct'.

        Pass ``from_position`` -- the aeroplane's present position -- to get
        the real thing: a leg running from where it is now to the fix. Without
        it the leg keeps its original origin, which is only right when the
        aeroplane is already somewhere along it.
        """
        self.active_index = min(max(index, 0), len(self.plan) - 1)
        self._direct_origin = from_position
