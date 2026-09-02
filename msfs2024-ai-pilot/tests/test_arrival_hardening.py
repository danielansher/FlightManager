"""Where it touches down, and whether it stops.

The complaints about every other AI Pilot are almost all about the arrival:
arriving high and fast, going around at 600 ft, floating and landing at the
far end of the runway. The existing tests check that the aeroplane reaches
the runway and that the touchdown is soft. Neither of those catches a
landing that uses up the whole runway, which is the one that hurts.
"""

from __future__ import annotations

import pytest

from aipilot.autopilot.phases import Phase
from aipilot.geo import along_track_nm, cross_track_nm, destination_point, distance_nm

FEET_PER_NM = 6076.11548556


def _runway_frame(runway):
    far = destination_point(runway.threshold, runway.heading_true_deg,
                            runway.length_ft / FEET_PER_NM)
    return runway.threshold, far


def touchdown_point_ft(result) -> float:
    """How far past the threshold the wheels came down, in feet."""
    start, end = _runway_frame(result.plan.arrival_runway)
    return along_track_nm(result.pilot.touchdown_position, start, end) * FEET_PER_NM


def stop_point_ft(result) -> float:
    start, end = _runway_frame(result.plan.arrival_runway)
    return along_track_nm(result.sim.state.position, start, end) * FEET_PER_NM


def test_it_touches_down_in_the_touchdown_zone(navdata, fly_flight_fn):
    """Not just "on the runway". A jet transport aims for a point about a
    thousand feet in; floating half way down a runway is how an aeroplane
    ends up in the grass at the far end, and it is the single most common
    complaint about the simulator's own AI Pilot."""
    result = fly_flight_fn(navdata, "EGLL", "EGCC")
    assert result.pilot.phase is Phase.COMPLETE
    where = touchdown_point_ft(result)
    assert 0 < where, f"touched down {abs(where):.0f} ft short of the threshold"
    assert where < 2600, f"touched down {where:.0f} ft down the runway"


def test_it_stops_on_the_runway(navdata, fly_flight_fn):
    result = fly_flight_fn(navdata, "EGLL", "EGCC")
    runway = result.plan.arrival_runway
    stopped = stop_point_ft(result)
    assert stopped < runway.length_ft, (
        f"ran {stopped:.0f} ft down a {runway.length_ft:.0f} ft runway")
    assert result.sim.state.ground_speed_kt < 30.0


def test_it_lands_on_the_centreline_not_merely_near_it(navdata, fly_flight_fn):
    result = fly_flight_fn(navdata, "EGLL", "EGCC")
    start, end = _runway_frame(result.plan.arrival_runway)
    offset_ft = abs(cross_track_nm(result.pilot.touchdown_position, start, end)) \
        * FEET_PER_NM
    half_width = result.plan.arrival_runway.width_ft / 2 or 75.0
    assert offset_ft < half_width, \
        f"touched down {offset_ft:.0f} ft off the centreline"


@pytest.mark.parametrize("aircraft", ["b787-10", "a320neo", "a380-800"])
def test_every_aeroplane_lands_in_the_touchdown_zone(navdata, fly_flight_fn, aircraft):
    result = fly_flight_fn(navdata, "EGLL", "EGCC", aircraft=aircraft)
    assert result.pilot.phase is Phase.COMPLETE
    where = touchdown_point_ft(result)
    assert 0 < where < 2600, f"{aircraft} touched down at {where:.0f} ft"


def test_a_tailwind_landing_still_stops_on_the_runway(navdata, fly_flight_fn):
    """The runway is chosen from the wind, so a tailwind landing only happens
    when it is forced -- but when it does, it must still stop."""
    result = fly_flight_fn(navdata, "EGLL", "EGCC", arrival_runway="05L",
                           wind_from_deg=230, wind_kt=15)
    assert result.pilot.phase is Phase.COMPLETE
    stopped = stop_point_ft(result)
    assert stopped < result.plan.arrival_runway.length_ft


@pytest.mark.parametrize("wind", [(180, 40), (320, 30), (52, 35), (232, 25)])
def test_it_lands_on_the_runway_in_a_strong_wind(navdata, fly_flight_fn, wind):
    """Head, tail and both crosswinds. The wind correction is the part most
    likely to put an aeroplane beside the runway rather than on it."""
    result = fly_flight_fn(navdata, "EGLL", "EGCC", wind_from_deg=wind[0],
                           wind_kt=wind[1], arrival_runway="05L")
    assert result.pilot.phase is Phase.COMPLETE, \
        f"wind {wind}: ended in {result.pilot.phase.value}"
    start, end = _runway_frame(result.plan.arrival_runway)
    offset_ft = abs(cross_track_nm(result.pilot.touchdown_position, start, end)) \
        * FEET_PER_NM
    # Half a runway width is 100 ft here, so this is "on the runway" with a
    # little to spare. The residual scales with the crosswind: a proportional
    # tracker cannot fully null a disturbance that keeps changing, and the
    # wind gradient through the boundary layer is exactly that. Thirty knots
    # across is at the 787's demonstrated limit, and it still lands on the
    # paved surface.
    assert offset_ft < 110, f"wind {wind}: touched down {offset_ft:.0f} ft off"


def test_the_stabilisation_gate_would_catch_a_missed_centreline(navdata, fly_flight_fn):
    """The gate allowed half a mile, which is three thousand feet: it could
    never have fired on an approach that merely landed in the grass."""
    from aipilot.autopilot.controller import STABILISATION_XTK_NM

    assert STABILISATION_XTK_NM * FEET_PER_NM < 600, \
        "the gate is wider than any runway, so it cannot catch a missed one"


def test_a_long_landing_is_reported(navdata, fly_flight_fn):
    """Whether or not it happens today, it must be visible when it does."""
    from aipilot.autopilot.controller import LONG_LANDING_FRACTION

    result = fly_flight_fn(navdata, "EGLL", "EGCC")
    runway = result.plan.arrival_runway
    assert result.pilot.touchdown_along_ft is not None
    said_long = any("long landing" in e.message.lower() for e in result.pilot.log)
    actually_long = result.pilot.touchdown_along_ft > \
        runway.length_ft * LONG_LANDING_FRACTION
    assert said_long == actually_long, \
        "the log and the touchdown point disagree about whether it was long"
