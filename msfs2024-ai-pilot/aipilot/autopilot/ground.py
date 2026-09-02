"""Pushback and taxi: getting the aeroplane from the stand to the runway.

Two things are worth saying plainly about what this can and cannot do.

It does not see obstacles. Nothing in SimConnect will say what scenery is
where, so there is no sensing anything and steering around it. What it does
instead is stay on the taxiway centrelines the scenery itself defines, which is
where there is by construction nothing parked -- and it will not move at all
unless it has those centrelines, rather than guessing a path across an apron.

Pushback is best effort. The simulator's tug is driven by a pair of events
whose behaviour varies between aircraft, so the pushback here is deliberately
simple -- straight back until the aeroplane can reach the taxiways, then stop --
and it says what it is doing at each step so a wrong turn is obvious rather
than mysterious.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from ..geo import (
    LatLon,
    cross_track_nm,
    distance_nm,
    initial_bearing_deg,
    normalize_deg,
    signed_diff_deg,
)
from ..sim.base import SimState

#: Taxi speeds. Straight-line, through a turn, and the speed below which the
#: aeroplane counts as stopped.
TAXI_SPEED_KT = 15.0
TAXI_TURN_SPEED_KT = 6.0
TAXI_FINAL_SPEED_KT = 6.0
STOPPED_KT = 0.6

#: Course change ahead that counts as a turn to slow down for, and how far
#: ahead to start slowing.
TURN_THRESHOLD_DEG = 25.0
TURN_LOOKAHEAD_NM = 0.10

#: Pure-pursuit lookahead. Both this and the capture radius are derived from
#: the turning circle rather than fixed, because a lookahead point *inside* the
#: turning circle cannot be reached: the aeroplane turns towards it, misses,
#: and orbits it for ever. That is not a tuning problem, it is geometry, and it
#: is what a hard-coded distance gets wrong the moment the speed changes.
LOOKAHEAD_BASE_NM = 0.012
LOOKAHEAD_TURN_FACTOR = 1.3

#: Degrees per second the nosewheel can turn the aeroplane at taxi speed,
#: which sets the turning circle.
NOSEWHEEL_RATE_DEG_S = 6.0

#: A waypoint more than this far off the nose is behind the aeroplane and is
#: never going to be reached by continuing to turn towards it.
BEHIND_ME_DEG = 105.0

#: Degrees of nosewheel per degree of heading error, and the limit.
STEER_GAIN = 0.045
STEER_LIMIT = 1.0

#: Nosewheel per nautical mile off the leg centreline. The number looks large
#: only because a nautical mile is an enormous distance on a taxiway: it works
#: out at full deflection for about a hundred feet off, which is what it takes
#: to hold a centreline in a turn rather than cutting the corner.
XTK_GAIN_PER_NM = 60.0

#: A point is passed once the aeroplane is within this of it.
WAYPOINT_REACHED_NM = 0.010

#: How far to push back before trying to taxi, and the most that will ever be
#: attempted before giving up and asking for help.
PUSHBACK_MIN_NM = 0.03
PUSHBACK_MAX_NM = 0.12


@dataclass
class GroundCommand:
    """What the ground channel wants: where to point, and how fast to go."""

    steering: float = 0.0          # -1 full left .. +1 full right
    target_speed_kt: float = 0.0
    brake: bool = False
    finished: bool = False
    reason: str = ""
    distance_remaining_nm: float = 0.0
    next_point: Optional[LatLon] = None


class TaxiGuidance:
    """Follows a taxi route by pure pursuit, at a sensible speed.

    Pure pursuit rather than tracking the centreline directly: an aeroplane
    steers with its nosewheel from a point well behind the nose, so chasing a
    point some distance ahead is both what the geometry wants and what stops it
    weaving between the lights.
    """

    def __init__(self, route: list[LatLon]) -> None:
        self.route = list(route)
        self.index = 0

    @property
    def finished(self) -> bool:
        return self.index >= len(self.route)

    @property
    def target(self) -> Optional[LatLon]:
        return self.route[self.index] if not self.finished else None

    def distance_remaining_nm(self, position: LatLon) -> float:
        if self.finished:
            return 0.0
        total = distance_nm(position, self.route[self.index])
        total += sum(distance_nm(a, b) for a, b
                     in zip(self.route[self.index:], self.route[self.index + 1:]))
        return total

    @staticmethod
    def turn_radius_nm(speed_kt: float) -> float:
        """The tightest circle the aeroplane can taxi at this speed."""
        if speed_kt <= 0.5:
            return LOOKAHEAD_BASE_NM
        feet_per_second = speed_kt * 1.68781
        radius_ft = feet_per_second / math.radians(NOSEWHEEL_RATE_DEG_S)
        return radius_ft / 6076.11548556

    def capture_radius_nm(self, speed_kt: float) -> float:
        return max(WAYPOINT_REACHED_NM, self.turn_radius_nm(speed_kt) * 0.9)

    def _lookahead_point(self, position: LatLon, speed_kt: float) -> LatLon:
        """A point on the route, far enough ahead to be reachable."""
        wanted = max(LOOKAHEAD_BASE_NM,
                     self.turn_radius_nm(speed_kt) * LOOKAHEAD_TURN_FACTOR)
        remaining = wanted
        cursor = position
        for point in self.route[self.index:]:
            leg = distance_nm(cursor, point)
            if leg >= remaining:
                from ..geo import destination_point

                return destination_point(cursor, initial_bearing_deg(cursor, point),
                                         remaining)
            remaining -= leg
            cursor = point
        return cursor

    def _turn_ahead_deg(self) -> float:
        """The sharpest course change within the lookahead, to slow down for."""
        sharpest = 0.0
        travelled = 0.0
        points = self.route[max(0, self.index - 1):]
        for previous, point, following in zip(points, points[1:], points[2:]):
            travelled += distance_nm(previous, point)
            if travelled > TURN_LOOKAHEAD_NM:
                break
            change = abs(signed_diff_deg(initial_bearing_deg(point, following),
                                         initial_bearing_deg(previous, point)))
            sharpest = max(sharpest, change)
        return sharpest

    def update(self, state: SimState) -> GroundCommand:
        position = state.position
        capture = self.capture_radius_nm(state.ground_speed_kt)
        while not self.finished:
            target = self.route[self.index]
            distance = distance_nm(position, target)
            if distance <= capture:
                self.index += 1
                continue
            # A point behind the aeroplane cannot be reached by turning towards
            # it -- the turn just carries it round in a circle. Once it is off
            # the nose by more than a right angle and inside the turning
            # circle, it has been passed.
            if distance <= self.turn_radius_nm(state.ground_speed_kt) * 2.5 and \
                    abs(signed_diff_deg(initial_bearing_deg(position, target),
                                        state.heading_true_deg)) > BEHIND_ME_DEG:
                self.index += 1
                continue
            break
        if self.finished:
            return GroundCommand(steering=0.0, target_speed_kt=0.0, brake=True,
                                 finished=True, reason="at the holding point")

        aim = self._lookahead_point(position, state.ground_speed_kt)
        wanted_heading = initial_bearing_deg(position, aim)
        error = signed_diff_deg(wanted_heading, state.heading_true_deg)

        # Pure pursuit alone cuts every corner: it steers at a point ahead and
        # is perfectly happy to arrive there having missed the whole leg in
        # between. Adding the distance from the leg itself pulls the aeroplane
        # back onto the centreline rather than merely towards the next point,
        # which is the difference between following a taxiway and crossing the
        # grass between two of them.
        offset = 0.0
        if self.index >= 1:
            leg_start = self.route[self.index - 1]
            leg_end = self.route[self.index]
            if distance_nm(leg_start, leg_end) > 1e-6:
                offset = cross_track_nm(position, leg_start, leg_end)
        steering = error * STEER_GAIN - offset * XTK_GAIN_PER_NM
        steering = max(-STEER_LIMIT, min(STEER_LIMIT, steering))

        remaining = self.distance_remaining_nm(position)
        turn = self._turn_ahead_deg()
        if remaining < 0.08:
            speed = TAXI_FINAL_SPEED_KT
        elif turn >= TURN_THRESHOLD_DEG or abs(error) > 25.0:
            speed = TAXI_TURN_SPEED_KT
        else:
            speed = TAXI_SPEED_KT

        return GroundCommand(
            steering=steering,
            target_speed_kt=speed,
            finished=False,
            reason=f"taxiing, {remaining:.2f} nm to the holding point",
            distance_remaining_nm=remaining,
            next_point=self.route[self.index],
        )


class PushbackGuidance:
    """Pushes straight back until the aeroplane can reach the taxiways."""

    def __init__(self, start: LatLon, heading_true_deg: float,
                 target_distance_nm: float = PUSHBACK_MIN_NM,
                 final_heading_deg: Optional[float] = None) -> None:
        self.start = start
        self.heading = normalize_deg(heading_true_deg)
        #: The heading to leave the aeroplane on. A real pushback does not just
        #: move an aeroplane backwards, it turns it to face the way it is about
        #: to taxi -- and without that the aeroplane comes off the stand still
        #: pointing at the terminal and has to swing ninety degrees on the
        #: apron to get going, which takes it off the pavement.
        self.final_heading = (normalize_deg(final_heading_deg)
                              if final_heading_deg is not None else self.heading)
        self.target_distance_nm = max(PUSHBACK_MIN_NM,
                                      min(PUSHBACK_MAX_NM, target_distance_nm))
        self._done = False
        self._travelled = 0.0
        self._last: Optional[LatLon] = None

    @property
    def push_direction_deg(self) -> float:
        """The direction the aeroplane travels: backwards along its heading."""
        return normalize_deg(self.heading + 180.0)

    @property
    def travelled_nm(self) -> float:
        """Distance actually covered, accumulated along the path.

        Neither straight-line distance from the start nor a projection onto the
        initial push direction survives the turn: a real pushback swings the
        aeroplane through ninety degrees or more, after which it is no longer
        travelling the way it set off, and both measures stop increasing. The
        pushback then never completes and the aeroplane is pushed across the
        apron for the rest of the day.
        """
        return self._travelled

    def advance(self, position: LatLon) -> None:
        """Record where the aeroplane has got to. Called once per cycle."""
        if self._last is not None:
            self._travelled += distance_nm(self._last, position)
        self._last = position
        if self._travelled >= self.target_distance_nm:
            self._done = True

    def finished(self, position: LatLon) -> bool:
        self.advance(position)
        return self._done


#: Turn beyond which the aeroplane cannot simply drive out of the stand.
PUSHBACK_TURN_LIMIT_DEG = 100.0


def pushback_needed(position: LatLon, network, heading_true_deg: float = 0.0,
                    first_leg: Optional[LatLon] = None) -> tuple[bool, float]:
    """Whether a pushback is needed, and roughly how far.

    Two reasons for one. The obvious one is that the aeroplane cannot reach the
    taxi network from where it stands. The one that matters more in practice is
    that it *can* reach it but is pointing the wrong way: parked nose-in, the
    route starts behind the aeroplane, and an aeroplane asked to drive to a
    point behind it turns a hundred and eighty degrees on the spot -- across
    whatever the stand is next to. That is what a tug is for.
    """
    from ..route.taxi import MAX_JOIN_DISTANCE_NM

    if network is None or not network.usable:
        return (False, 0.0)

    nearest = network.nearest_node(position, limit_nm=MAX_JOIN_DISTANCE_NM)
    if nearest is None:
        node = network.nearest_node(position, limit_nm=PUSHBACK_MAX_NM * 4)
        if node is None:
            return (False, 0.0)
        return (True, distance_nm(position, network.nodes[node].position))

    if first_leg is not None:
        turn = abs(signed_diff_deg(initial_bearing_deg(position, first_leg),
                                   heading_true_deg))
        if turn > PUSHBACK_TURN_LIMIT_DEG:
            return (True, PUSHBACK_MIN_NM)
    return (False, 0.0)
