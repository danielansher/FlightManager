"""Lateral and vertical guidance, and the adapter's command discipline."""

import pytest

from aipilot.aircraft.registry import build_adapter, resolve_key
from aipilot.autopilot.lateral import LateralGuidance
from aipilot.autopilot.phases import Phase, phase_rank
from aipilot.autopilot.vertical import VerticalGuidance, should_start_descent
from aipilot.geo import (
    LatLon,
    destination_point,
    distance_nm,
    initial_bearing_deg,
    signed_diff_deg,
)
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.route.profile import build_vertical_profile
from aipilot.sim.mock import MockSim


@pytest.fixture
def plan(navdata):
    return plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata,
                      departure_runway="27R", arrival_runway="05L")


def _on_centreline(plan, index, distance_from_start):
    start = plan[index - 1].position
    end = plan[index].position
    return destination_point(start, initial_bearing_deg(start, end), distance_from_start)


def test_on_centreline_needs_no_correction(plan):
    guidance = LateralGuidance(plan)
    guidance.direct_to(2)
    point = _on_centreline(plan, 2, 5.0)
    command = guidance.update(point, 300, 0, 0)
    assert command.cross_track_nm == pytest.approx(0, abs=0.01)
    expected = initial_bearing_deg(point, plan[2].position)
    assert signed_diff_deg(command.desired_track_deg, expected) == pytest.approx(0, abs=0.5)


def test_correction_is_towards_the_centreline_and_proportional(plan):
    guidance = LateralGuidance(plan)
    guidance.direct_to(2)
    point = _on_centreline(plan, 2, 5.0)
    course = initial_bearing_deg(plan[1].position, plan[2].position)

    right = destination_point(point, initial_bearing_deg(point, plan[2].position) + 90, 4.0)
    command = guidance.update(right, 300, 0, 0)
    assert command.cross_track_nm > 0
    # Right of track means the commanded track is left of the leg course.
    assert signed_diff_deg(command.desired_track_deg, course) < -5

    guidance.direct_to(2)
    left = destination_point(point, initial_bearing_deg(point, plan[2].position) - 90, 4.0)
    command = guidance.update(left, 300, 0, 0)
    assert command.cross_track_nm < 0
    assert signed_diff_deg(command.desired_track_deg, course) > 5


def test_intercept_angle_is_clamped(plan):
    guidance = LateralGuidance(plan)
    guidance.direct_to(2)
    point = _on_centreline(plan, 2, 5.0)
    course = initial_bearing_deg(plan[1].position, plan[2].position)
    far = destination_point(point, initial_bearing_deg(point, plan[2].position) + 90, 60.0)
    command = guidance.update(far, 300, 0, 0)
    assert abs(signed_diff_deg(command.desired_track_deg, course)) <= 46


def test_heading_crabs_into_the_wind(plan):
    guidance = LateralGuidance(plan)
    guidance.direct_to(2)
    point = _on_centreline(plan, 2, 5.0)
    still = guidance.update(point, 300, 0, 0)
    guidance.direct_to(2)
    # Wind from the left of the desired track pushes us right, so we crab left.
    from_left = (still.desired_track_deg - 90) % 360
    windy = guidance.update(point, 300, from_left, 40)
    assert signed_diff_deg(windy.heading_true_deg, windy.desired_track_deg) < -3


def _dogleg_plan(navdata, turn_deg=60.0):
    """A three-fix route with a deliberate turn at the middle fix."""
    from aipilot.navdata.base import Waypoint
    from aipilot.route.plan import FlightPlan, RouteLeg

    start = LatLon(50.0, 0.0)
    middle = destination_point(start, 90.0, 100.0)
    end = destination_point(middle, 90.0 + turn_deg, 100.0)
    legs = [
        RouteLeg(Waypoint("START", start), phase="enroute"),
        RouteLeg(Waypoint("TURN", middle), phase="enroute"),
        RouteLeg(Waypoint("END", end), phase="enroute"),
    ]
    return FlightPlan(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      None, None, 35000.0, legs)


def test_fly_by_fix_sequences_before_the_waypoint(navdata):
    """A turn must start early enough to roll out on the next leg."""
    dogleg = _dogleg_plan(navdata, turn_deg=60.0)
    guidance = LateralGuidance(dogleg)
    assert guidance.active_index == 1
    assert abs(dogleg.course_change_at_deg(1)) > 20

    target = dogleg[1].position
    inbound = dogleg.leg_course_deg(1)
    # Twenty miles out: nowhere near the turn.
    far = destination_point(target, inbound + 180, 20.0)
    assert not guidance.update(far, 450, 0, 0).sequenced
    assert guidance.active_index == 1

    # Inside the turn anticipation distance, still short of the fix: sequenced.
    from aipilot.geo import turn_anticipation_nm

    anticipation = turn_anticipation_nm(450, dogleg.course_change_at_deg(1), 25.0)
    assert anticipation > 1.0, "a 60 degree turn at 450 kt needs real room"
    close = destination_point(target, inbound + 180, anticipation * 0.8)
    command = guidance.update(close, 450, 0, 0)
    assert command.sequenced
    assert guidance.active_index == 2


def test_a_sharper_turn_is_started_earlier(navdata):
    from aipilot.geo import turn_anticipation_nm

    gentle = turn_anticipation_nm(450, 30, 25.0)
    sharp = turn_anticipation_nm(450, 100, 25.0)
    assert sharp > gentle > 0


def test_flyover_fix_is_not_cut(plan):
    """Approach fixes must be overflown, or the aeroplane cuts the corner
    off final approach."""
    guidance = LateralGuidance(plan)
    index = plan.threshold_index
    assert plan[index].flyover
    guidance.direct_to(index)
    course = plan.leg_course_deg(index)
    # Half a mile short of the threshold, which is inside a turn radius.
    short = destination_point(plan[index].position, course + 180, 0.5)
    assert not guidance.update(short, 150, 0, 0).sequenced
    # Past it.
    beyond = destination_point(plan[index].position, course, 0.2)
    assert guidance.update(beyond, 150, 0, 0).sequenced


def test_final_approach_uses_a_tighter_intercept(plan):
    """Tighter than enroute, and shallower in bank -- but still decisive.

    A mile off the centreline three miles out has to be flown out before the
    threshold, so this is a proper intercept, not a nudge.
    """
    from aipilot.autopilot.lateral import FINAL_INTERCEPT_DEG, MAX_INTERCEPT_DEG

    guidance = LateralGuidance(plan)
    index = plan.threshold_index
    guidance.direct_to(index)
    course = plan.leg_course_deg(index)
    point = destination_point(plan[index].position, course + 180, 3.0)
    offset = destination_point(point, course + 90, 1.0)
    command = guidance.update(offset, 150, 0, 0, approach_mode=True)

    intercept = abs(signed_diff_deg(command.desired_track_deg, course))
    assert intercept <= FINAL_INTERCEPT_DEG < MAX_INTERCEPT_DEG
    assert intercept > 15, \
        f"only {intercept:.0f} degrees to close a mile in three: too feeble"
    assert command.bank_limit_deg <= 15


def test_a_small_offset_on_final_earns_a_correction_that_can_close_it(plan):
    """The defect this replaced: a hundred yards off earned less than a
    degree, which over the mile that was left closed nothing, and the
    aeroplane landed beside the runway while reporting itself lined up."""
    guidance = LateralGuidance(plan)
    index = plan.threshold_index
    guidance.direct_to(index)
    course = plan.leg_course_deg(index)
    point = destination_point(plan[index].position, course + 180, 1.5)
    offset = destination_point(point, course + 90, 0.1)      # ~600 ft
    command = guidance.update(offset, 150, 0, 0, approach_mode=True)

    correction = abs(signed_diff_deg(command.desired_track_deg, course))
    assert correction > 3.0, \
        f"{correction:.1f} degrees for 600 ft off is not going to close it"
    # It has to actually close, over the final approach rather than over the
    # last few seconds: at 150 kt this is the sideways speed available, and a
    # three mile final is about seventy seconds of it. (Closing 600 ft inside
    # the last mile is not the standard -- no aeroplane can, and a crew that
    # far out of line at a mile goes around instead. That is what the
    # stabilisation gate is for.)
    import math
    closure_fps = 150 * math.sin(math.radians(correction)) * 1.68781
    assert closure_fps * 70 > 600, \
        f"closes only {closure_fps * 70:.0f} ft over a three mile final"


def test_the_correction_on_final_eases_off_as_it_arrives(plan):
    """It must converge, not weave across the centreline."""
    guidance = LateralGuidance(plan)
    index = plan.threshold_index
    guidance.direct_to(index)
    course = plan.leg_course_deg(index)
    point = destination_point(plan[index].position, course + 180, 1.5)

    corrections = []
    for offset_nm in (0.3, 0.1, 0.03, 0.005):
        where = destination_point(point, course + 90, offset_nm)
        command = guidance.update(where, 150, 0, 0, approach_mode=True)
        corrections.append(abs(signed_diff_deg(command.desired_track_deg, course)))
    assert corrections == sorted(corrections, reverse=True)
    assert corrections[-1] < 1.0, "still fighting when it is already there"


def test_guidance_terminates_at_the_last_fix(plan):
    guidance = LateralGuidance(plan)
    guidance.direct_to(len(plan) - 1)
    assert guidance.finished
    command = guidance.update(plan[-1].position, 140, 0, 0)
    assert not command.sequenced
    assert guidance.active_index == len(plan) - 1


# -- Vertical -----------------------------------------------------------------
@pytest.fixture
def vertical(plan):
    profile = get_profile("b787-10")
    vp = build_vertical_profile(plan.cruise_altitude_ft,
                                plan.arrival_runway.elevation_ft, profile)
    return VerticalGuidance(plan, profile, vp), vp


def test_climb_commands_the_cruise_level_and_lets_the_aeroplane_climb(plan, vertical):
    guidance, _ = vertical
    command = guidance.update(Phase.CLIMB, 12000, 500, 400, 2)
    assert command.altitude_ft == plan.cruise_altitude_ft
    assert command.vertical_speed_fpm is None
    assert command.speed == 300.0 and not command.speed_is_mach


def test_speed_below_ten_thousand_respects_the_restriction(plan, vertical):
    guidance, _ = vertical
    command = guidance.update(Phase.CLIMB, 8000, 500, 400, 2)
    assert command.speed == 250.0


def test_descent_commands_a_descent_and_a_floor_below_the_cruise(plan, vertical):
    guidance, vp = vertical
    command = guidance.update(Phase.DESCENT, plan.cruise_altitude_ft,
                              vp.top_of_descent_nm - 5, 450, 2)
    assert command.vertical_speed_fpm is not None and command.vertical_speed_fpm < 0
    assert command.altitude_ft < plan.cruise_altitude_ft


def test_descent_rate_is_clamped_to_the_type_limit(plan, vertical):
    guidance, vp = vertical
    profile = get_profile("b787-10")
    # Ten thousand feet above the path is a big correction, not an impossible one.
    high = vp.target_altitude_at(40) + 10000
    command = guidance.update(Phase.DESCENT, high, 40, 450, 2)
    assert command.vertical_speed_fpm >= -profile.max_descent_rate_fpm
    assert "high" in command.reason


def test_below_the_path_the_descent_is_arrested(plan, vertical):
    guidance, vp = vertical
    low = vp.target_altitude_at(40) - 3000
    command = guidance.update(Phase.DESCENT, low, 40, 450, 2)
    assert command.vertical_speed_fpm >= -600
    assert "low" in command.reason


def test_approach_speed_reduces_towards_the_threshold(plan, vertical):
    guidance, _ = vertical
    speeds = [guidance.update(Phase.APPROACH, 3000, d, 200, 3).speed
              for d in (25, 15, 10, 6, 3)]
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[-1] == pytest.approx(get_profile("b787-10").final_approach_speed_kt)


def test_approach_never_commands_a_dive(plan, vertical):
    guidance, vp = vertical
    command = guidance.update(Phase.APPROACH, vp.target_altitude_at(8) + 4000, 8, 200, 3)
    assert command.vertical_speed_fpm >= -1800


def test_top_of_descent_test_does_not_fire_on_departure(plan, vertical):
    """The regression that declared top of descent forty seconds after takeoff."""
    _, vp = vertical
    field = plan.origin.elevation_ft
    assert not should_start_descent(plan.total_distance_nm, vp, field + 500)
    assert not should_start_descent(vp.top_of_descent_nm + 50, vp, field)
    assert should_start_descent(vp.top_of_descent_nm - 1, vp, plan.cruise_altitude_ft)


def test_phase_order_is_monotonic():
    assert phase_rank(Phase.CLIMB) < phase_rank(Phase.CRUISE) < phase_rank(Phase.DESCENT)
    assert phase_rank(Phase.APPROACH) < phase_rank(Phase.LANDING) < phase_rank(Phase.ROLLOUT)


# -- Adapters -----------------------------------------------------------------
def test_aircraft_aliases_resolve():
    assert resolve_key("787") == "b787-10"
    assert resolve_key("B78X") == "b787-10"
    assert resolve_key("a350") == "a350-900"
    assert resolve_key("headwind") == "a330-900"
    assert resolve_key("nonsense") is None


def test_unknown_aircraft_falls_back_to_generic():
    sim = MockSim(LatLon(0, 0))
    adapter, profile = build_adapter("something-made-up", sim)
    assert profile.key == "generic"


def test_heading_command_converts_true_to_magnetic():
    from aipilot.sim.base import SimState

    sim = MockSim(LatLon(0, 0))
    adapter, _ = build_adapter("b787-10", sim)
    state = SimState(magvar_deg=-5.0)      # 5 degrees west variation
    adapter.set_heading_true(90.0, state)
    event = [e for e in sim.events_sent if e[0] == "HEADING_BUG_SET"][-1]
    assert event[1] == 95


def test_repeated_identical_commands_are_not_resent():
    sim = MockSim(LatLon(0, 0))
    adapter, _ = build_adapter("b787-10", sim)
    for _ in range(20):
        adapter.set_altitude(35000)
        adapter.set_speed_kt(280)
    assert len([e for e in sim.events_sent if e[0] == "AP_ALT_VAR_SET_ENGLISH"]) == 1
    assert len([e for e in sim.events_sent if e[0] == "AP_SPD_VAR_SET"]) == 1


def test_mach_is_transmitted_scaled_by_one_hundred():
    sim = MockSim(LatLon(0, 0))
    adapter, _ = build_adapter("b787-10", sim)
    adapter.set_mach(0.85)
    assert ("AP_MACH_VAR_SET", 85) in sim.events_sent


def test_negative_vertical_speed_survives_the_wire():
    """The event parameter is an unsigned DWORD; a descent must still descend."""
    sim = MockSim(LatLon(0, 0))
    adapter, _ = build_adapter("b787-10", sim)
    adapter.set_vertical_speed(-2000)
    assert sim.target_vs_fpm == pytest.approx(-2000)


def test_flaps_walk_one_detent_at_a_time():
    sim = MockSim(LatLon(0, 0), start_airborne_at_ft=5000)
    adapter, _ = build_adapter("b787-10", sim)
    reached = False
    for _ in range(60):
        state = sim.step(1.0)
        reached = adapter.set_flaps(3, state)
        if reached:
            break
    assert reached and sim.state.flaps_index == 3


def test_airbus_adapter_reports_a_degraded_bridge():
    """Without calculator code the FCU knobs cannot be pulled, and it says so."""
    from aipilot.sim.base import SimCapabilities

    sim = MockSim(LatLon(0, 0))
    sim.capabilities = lambda: SimCapabilities(lvars=False, calculator_code=False)
    adapter, _ = build_adapter("a330-900", sim)
    messages = []
    adapter.log = messages.append
    adapter.set_heading_magnetic(180)
    assert adapter.capabilities().degraded
    assert any("WASM" in m for m in messages)


def test_airbus_adapter_pulls_the_knob_when_the_bridge_is_there():
    sim = MockSim(LatLon(0, 0))
    adapter, _ = build_adapter("a330-900", sim)
    adapter.set_heading_magnetic(180)
    assert sim.get_lvar("__none__") is None
    # The pull goes out as gauge calculator code.
    assert adapter.fcu["heading_pull"] == "A32NX_FCU_HDG_PULL"
    assert not adapter.capabilities().degraded


def test_inibuilds_conventions_are_empty_on_purpose():
    """Guessing unpublished event names would send commands into a void."""
    from aipilot.aircraft.airbus import convention

    assert convention("inibuilds_a350") == {}
    assert convention("inibuilds_a380") == {}
