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
    destination_point,
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
#: Enough for the straight leg plus the longest turn a tug is ever asked for:
#: 152 ft clear of the stand and 612 ft to come through 180 degrees, which is
#: 764 ft. Not a round number picked for room -- widening this also widens what
#: counts as a stand the taxiways can reach at all.
PUSHBACK_MAX_NM = 0.15

#: How far the aeroplane goes straight back before the tug begins to turn it.
#: A tug does not start swinging the moment it takes the weight: it pulls the
#: aeroplane clear of the stand and the jetway first, and only then turns it to
#: face the taxiway. Turning from the stand drags a wingtip through whatever is
#: parked alongside.
PUSHBACK_STRAIGHT_NM = 0.025

#: Travel needed for each degree the tug turns the aeroplane, in nautical
#: miles. Measured on a Horizon 787-9 in MSFS 2020: the tug turned at about
#: 1.3 degrees a second while pushing at 2.6 kt, which is 3.4 ft per degree.
#: Without this the distance and the heading were chosen independently, and a
#: push of 182 ft was asked to deliver a 180 degree turn -- it managed 56, and
#: left the aeroplane across the apron pointing nowhere useful.
PUSHBACK_NM_PER_TURN_DEG = 3.4 / 6076.12


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
                 final_heading_deg: Optional[float] = None,
                 straight_distance_nm: float = PUSHBACK_STRAIGHT_NM) -> None:
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
        #: How far to go straight back before the tug starts turning. Never
        #: more than the push itself, or the turn would never begin.
        self.straight_distance_nm = min(straight_distance_nm,
                                        self.target_distance_nm)
        self._done = False
        self._travelled = 0.0
        self._last: Optional[LatLon] = None

    @property
    def push_direction_deg(self) -> float:
        """The direction the aeroplane travels: backwards along its heading."""
        return normalize_deg(self.heading + 180.0)

    @property
    def tug_heading(self) -> float:
        """The heading to give the tug right now.

        Straight back until the aeroplane is clear of the stand, and only then
        the heading it is to end up on. Handing the tug the final heading at
        the outset makes it turn from the moment it takes the weight, which
        swings the tail across the neighbouring stand and puts a wingtip where
        the jetway is.
        """
        if self._travelled < self.straight_distance_nm:
            return self.heading
        return self.final_heading

    @property
    def turning(self) -> bool:
        """Whether the straight leg is behind us and the turn has begun."""
        return self._travelled >= self.straight_distance_nm

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

    @property
    def done(self) -> bool:
        """Whether the push is over, without advancing the accumulator.

        ``finished`` has a side effect and must be called exactly once a
        cycle. Anything else that needs to know reads this.
        """
        return self._done

    def finished(self, position: LatLon) -> bool:
        self.advance(position)
        return self._done


#: Turn beyond which the aeroplane cannot simply drive out of the stand.
PUSHBACK_TURN_LIMIT_DEG = 100.0


#: Radius of the arc the tug swings the aeroplane round, in nautical miles.
#: Travel per degree and radius are the same measurement seen two ways.
PUSHBACK_TURN_RADIUS_NM = PUSHBACK_NM_PER_TURN_DEG * 180.0 / math.pi


def pushback_end_point(start: LatLon, heading_true_deg: float, turn_deg: float,
                       straight_nm: float = PUSHBACK_STRAIGHT_NM,
                       total_nm: Optional[float] = None) -> LatLon:
    """Where the aeroplane finishes: straight back, then round an arc.

    Not "straight back for the whole push". Once the tug starts turning, the
    aeroplane is travelling a curve, and for a large turn it comes back on
    itself: a 764 ft push through 174 degrees moved a 787 just 418 ft from the
    stand, on a bearing 68 degrees off the one it set off on. Taking the end as
    a straight run put it nowhere near, so the onward route was read from a
    point the aeroplane never went, and the taxi opened with a 124 degree turn
    to undo what the tug had just done.

    Checked against that flight: this predicts 425 ft on a bearing of 133, and
    the aeroplane recorded 418 ft on 135.
    """
    back = normalize_deg(heading_true_deg + 180.0)
    end = destination_point(start, back, straight_nm)
    if abs(turn_deg) >= 0.5:
        # The chord of the arc, which bisects the turn.
        chord = 2.0 * PUSHBACK_TURN_RADIUS_NM * math.sin(
            math.radians(abs(turn_deg)) / 2.0)
        end = destination_point(end, normalize_deg(back + turn_deg / 2.0), chord)
    # Any push left over once the turn is done carries on straight, backwards
    # along the new heading. Left out, this was worth 20 degrees of error in
    # where the aeroplane was predicted to be: a 763 ft push that only needed
    # 580 ft to turn spent the last 183 ft going straight somewhere the model
    # said it would not be, and the taxi opened with a 155 degree turn.
    if total_nm is not None:
        tail = total_nm - straight_nm - abs(turn_deg) * PUSHBACK_NM_PER_TURN_DEG
        if tail > 0.0:
            end = destination_point(
                end, normalize_deg(heading_true_deg + turn_deg + 180.0), tail)
    return end


def pushback_distance_for(turn_deg: float,
                          straight_nm: float = PUSHBACK_STRAIGHT_NM) -> float:
    """How far to push: clear of the stand, then far enough to turn.

    The two used to be picked independently -- a fixed distance, and whatever
    heading the taxi wanted -- so nothing noticed when the distance could not
    deliver the heading. The worst case is also the one that got the shortest
    push: a turn past the limit meant "nose-in, needs a tug", and was answered
    with the minimum distance, when it is exactly the turn needing the most
    room.
    """
    return straight_nm + abs(turn_deg) * PUSHBACK_NM_PER_TURN_DEG


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
            # A floor, not the answer. How far to push depends on the turn
            # actually chosen, which is decided against the taxiways and is
            # not known here -- this angle is only "is the route behind us".
            # Returning a distance sized from *this* angle looks helpful and
            # is not: it stood at 763 ft while the turn chosen needed 580, and
            # the aeroplane spent the difference going straight past where it
            # was meant to stop.
            return (True, PUSHBACK_MIN_NM)
    return (False, 0.0)
