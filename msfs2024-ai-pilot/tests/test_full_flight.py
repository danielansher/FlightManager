"""Complete flights in the mock simulator.

These are the tests that matter. Every bug found while building this project
was a whole-flight bug -- a descent that started on the runway at the departure
end, an approach phase entered three hundred miles out, a lateral channel with
nothing ahead of it at fifty feet -- and none of them is visible in a unit test
of the component that contained it. So each of these flies the aeroplane from
brakes-off to a full stop and asserts on what actually happened.
"""

import pytest

from aipilot.autopilot.controller import PilotOptions
from aipilot.autopilot.phases import Phase
from aipilot.geo import distance_nm
from aipilot.perf.profiles import get_profile

from .conftest import fly_flight


def test_a_complete_flight_lands_at_the_destination(navdata):
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10",
                        wind_from_deg=250, wind_kt=25)
    assert result.completed, f"ended in {result.phase.value}"
    assert result.stop_distance_from_threshold_nm < 2.5
    assert result.sim.state.on_ground
    assert result.sim.state.ground_speed_kt <= 30


def test_the_touchdown_is_a_landing_not_an_arrival(navdata):
    """An airline touchdown is a couple of hundred feet a minute. The 3 degree
    path arrives at seven hundred, so the flare has to do real work."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    assert result.touchdown_fpm is not None
    assert -300 < result.touchdown_fpm < 0, \
        f"touched down at {result.touchdown_fpm:.0f} fpm"


def test_the_flight_visits_every_phase_in_order(navdata):
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    seen = [event.phase for event in result.pilot.log]
    order = []
    for phase in seen:
        if not order or order[-1] is not phase:
            order.append(phase)
    for phase in (Phase.TAKEOFF, Phase.CLIMB, Phase.CRUISE,
                  Phase.DESCENT, Phase.APPROACH, Phase.LANDING, Phase.ROLLOUT):
        assert phase in order, f"never entered {phase.value}"
    # And never went backwards.
    ranks = [order.index(p) for p in order]
    assert ranks == sorted(ranks)


def test_top_of_descent_is_not_declared_on_departure(navdata):
    """Regression: the descent test used to fire seconds after takeoff."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    descent_events = [e for e in result.pilot.log
                      if e.phase is Phase.DESCENT and "top of descent" in e.message.lower()]
    assert descent_events, "never announced top of descent"
    assert all(e.time_s > 300 for e in descent_events), \
        "declared top of descent in the first five minutes"


def test_approach_is_not_entered_hundreds_of_miles_out(navdata):
    """Regression: keying the approach off the active leg rather than distance
    flew an entire sector at approach speed."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    entry = next(e for e in result.pilot.log
                 if e.phase is Phase.APPROACH and "nm to run" in e.message)
    # "APPROACH -- 25 nm to run"
    miles = float(entry.message.split("--")[-1].split("nm")[0])
    assert miles <= 30


def test_the_aeroplane_reaches_its_cruise_level(navdata):
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    assert any("level at" in e.message.lower() for e in result.pilot.log)


def test_it_stays_on_the_planned_track(navdata):
    """Established in the cruise, in a sixty knot crosswind, the aeroplane
    should hold the centreline to within a fraction of a mile."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10",
                        wind_from_deg=180, wind_kt=60)
    assert result.cruise_xtk, "never established in the cruise"
    assert result.max_xtk_nm < 0.5, \
        f"wandered {result.max_xtk_nm:.2f} nm off track"


def test_it_arrives_lined_up_with_the_runway(navdata):
    """The measure that matters about an approach: where it ends up."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10",
                        wind_from_deg=180, wind_kt=40)
    assert result.gate_xtk_nm is not None, "never reached the stabilisation gate"
    assert result.gate_xtk_nm < 0.5, \
        f"{result.gate_xtk_nm:.2f} nm off the centreline at 500 ft"


@pytest.mark.parametrize("arrival", ["05L", "23R"])
def test_it_lines_up_from_either_direction(navdata, arrival):
    """One of these runways faces the arrival and one faces away from it, so
    this exercises both the straight-in join and the full circuit."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10", arrival_runway=arrival)
    assert result.completed
    assert result.gate_xtk_nm is not None and result.gate_xtk_nm < 0.5


def test_the_configuration_is_managed_through_the_flight(navdata):
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    assert result.said("gear up")
    assert result.said("gear down")
    assert result.said("flaps up")          # clean in the climb
    assert result.said("speedbrake armed")
    landing_flaps = get_profile("b787-10").landing_flaps
    assert result.said(f"flaps {landing_flaps}")


def test_flaps_are_never_extended_above_their_placard_speed(navdata):
    """The configuration schedule is gated on speed, not only on distance."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.autopilot.controller import AIPilot
    from aipilot.route.planner import plan_route
    from aipilot.sim.mock import MockAircraftModel, MockSim

    profile = get_profile("b787-10")
    origin, destination = navdata.airport("EGLL"), navdata.airport("EGCC")
    plan = plan_route(origin, destination, profile, navdata)
    runway = plan.departure_runway
    sim = MockSim(runway.threshold, runway.heading_true_deg, origin.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan)
    pilot.engage()

    violations = []
    for _ in range(int(4 * 3600 / 2)):
        state = sim.state
        setting = profile.flap(state.flaps_index)
        if setting is not None and setting.index > 0 and \
                state.ias_kt > setting.max_speed_kt + 5 and not state.on_ground:
            violations.append((round(state.ias_kt), setting.label,
                               round(setting.max_speed_kt)))
        pilot.update(2.0)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    assert not violations, f"flap overspeed: {violations[:5]}"


def test_the_ils_is_tuned_when_the_runway_has_one(navdata):
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10", arrival_runway="05L")
    assert result.plan.arrival_runway.has_ils
    assert result.said("nav1 tuned")
    assert any("111.5" in m for m in result.messages())


def test_an_approach_without_an_ils_is_flown_and_landed_anyway(navdata):
    """This is the behaviour the MSFS 2020 AI Pilot had: it always landed."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10", arrival_runway="23R")
    assert not result.plan.arrival_runway.has_ils
    assert result.said("no ils")
    assert result.completed
    assert result.said("flare")
    assert result.stop_distance_from_threshold_nm < 3.0


def test_handover_mode_gives_the_aeroplane_back_stable(navdata):
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10",
                        options=PilotOptions(autoland="handover"))
    assert result.said("your controls")
    handover = next(e for e in result.pilot.log if "your controls" in e.message.lower())
    assert handover.level == "warning"
    # It hands over configured and on speed, not in some random state.
    assert result.sim.state.gear_down_pct > 95
    assert result.sim.state.flaps_index >= get_profile("b787-10").landing_flaps_index - 1


@pytest.mark.parametrize("aircraft", ["b787-10", "b787-9", "a350-900", "a350-1000",
                                      "a380-800", "a330-900", "a320neo"])
def test_every_aircraft_in_the_fleet_completes_a_flight(navdata, aircraft):
    result = fly_flight(navdata, "EGLL", "EGCC", aircraft,
                        wind_from_deg=250, wind_kt=20)
    assert result.completed, f"{aircraft} ended in {result.phase.value}"
    assert result.stop_distance_from_threshold_nm < 3.0
    assert result.touchdown_fpm is not None and result.touchdown_fpm > -350


def test_a_long_haul_flight_completes(bundled_navdata):
    """Twenty hours, thirty-odd waypoints, and a great circle that goes
    a long way north of the direct line."""
    result = fly_flight(bundled_navdata, "EGLL", "YSSY", "a350-1000",
                        wind_from_deg=270, wind_kt=50, dt=4.0)
    assert result.completed
    assert result.max_xtk_nm < 0.5
    assert result.gate_xtk_nm is not None and result.gate_xtk_nm < 0.5
    assert 15 * 3600 < result.elapsed_s < 26 * 3600


def test_a_flight_across_the_date_line_completes(bundled_navdata):
    result = fly_flight(bundled_navdata, "RJTT", "KSFO", "a350-900",
                        wind_from_deg=250, wind_kt=90, dt=4.0)
    assert result.completed
    assert result.max_xtk_nm < 0.5


def test_a_headwind_makes_the_flight_take_longer_than_a_tailwind(bundled_navdata):
    into_wind = fly_flight(bundled_navdata, "EGLL", "KJFK", "b787-10",
                           wind_from_deg=270, wind_kt=80, dt=4.0)
    with_wind = fly_flight(bundled_navdata, "KJFK", "EGLL", "b787-10",
                           wind_from_deg=270, wind_kt=80, dt=4.0)
    assert into_wind.completed and with_wind.completed
    assert into_wind.elapsed_s > with_wind.elapsed_s * 1.15


def test_a_very_short_sector_still_works(navdata):
    """Short enough that top of descent arrives during the climb."""
    result = fly_flight(navdata, "EGLL", "EGLL", "b787-10",
                        departure_runway="09L", arrival_runway="27R")
    assert result.completed


def test_engaging_in_flight_picks_up_the_route_ahead(bundled_navdata):
    """Engaging at cruise must fly on, not turn round for the departure fix."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.autopilot.controller import AIPilot
    from aipilot.route.planner import plan_route
    from aipilot.sim.mock import MockAircraftModel, MockSim

    profile = get_profile("b787-10")
    origin = bundled_navdata.airport("EGLL")
    destination = bundled_navdata.airport("KJFK")
    plan = plan_route(origin, destination, profile, bundled_navdata)
    # Start a third of the way along the route at cruise level.
    start_leg = plan.legs[len(plan.legs) // 3]
    sim = MockSim(start_leg.position, 280.0, 0.0,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                  start_airborne_at_ft=plan.cruise_altitude_ft)
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan,
                    PilotOptions(start_airborne=True))
    pilot.engage()
    assert pilot.phase is Phase.CLIMB
    remaining_at_start = pilot.status.distance_to_destination_nm

    for _ in range(int(8 * 3600 / 4)):
        pilot.update(4.0)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    assert pilot.phase is Phase.COMPLETE
    assert remaining_at_start < plan.total_distance_nm, "should not fly the whole route"
    assert distance_nm(sim.state.position, plan.threshold_position) < 3.0


def test_an_unstable_approach_triggers_a_go_around(navdata):
    """Forced by making the aeroplane far too fast to configure in time."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.autopilot.controller import AIPilot
    from aipilot.route.planner import plan_route
    from aipilot.sim.mock import MockAircraftModel, MockSim

    profile = get_profile("b787-10")
    origin, destination = navdata.airport("EGLL"), navdata.airport("EGCC")
    plan = plan_route(origin, destination, profile, navdata, arrival_runway="23R")
    runway = plan.departure_runway
    # An aeroplane that will not slow down: deceleration an order of magnitude
    # below normal, so it arrives at the gate far too fast.
    model = MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm,
                              decel_kt_s=0.05, flap_transit_s=90.0,
                              gear_transit_s=60.0)
    sim = MockSim(runway.threshold, runway.heading_true_deg, origin.elevation_ft,
                  model=model)
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan,
                    PilotOptions(go_around_if_unstable=True, max_go_arounds=1))
    pilot.engage()
    for _ in range(int(6 * 3600 / 2)):
        pilot.update(2.0)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    assert pilot._go_arounds >= 1, "should have rejected the unstable approach"
    assert any("going around" in e.message.lower() for e in pilot.log)


def test_warnings_are_surfaced_to_the_user(bundled_navdata):
    result = fly_flight(bundled_navdata, "EGLL", "EGCC", "b787-10")
    warnings = [e for e in result.pilot.log if e.level == "warning"]
    assert warnings, "synthetic runway data must be reported"
    assert any("no runway data" in e.message.lower() for e in warnings)


@pytest.mark.parametrize("dt", [0.25, 0.5, 2.0, 5.0])
def test_the_result_does_not_depend_on_the_control_rate(navdata, dt):
    """Guidance must be robust to how often it is called.

    The command line runs at four hertz and the tests at half a hertz, and an
    accelerated replay is tempting to implement by simply taking larger steps.
    That is what makes this worth pinning: a controller that only behaves at
    one particular step size is one that will misbehave on somebody's slower
    machine.
    """
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10",
                        wind_from_deg=250, wind_kt=25, dt=dt)
    assert result.completed, f"ended in {result.phase.value} at dt={dt}"
    assert result.stop_distance_from_threshold_nm < 3.0
    assert result.touchdown_fpm is not None and result.touchdown_fpm > -350
    # Block time should agree closely regardless of step size.
    assert 25 * 60 < result.elapsed_s < 40 * 60


def test_handover_is_announced_with_time_to_react(navdata):
    """Two hundred feet is eight seconds from the runway. Someone who has been
    watching rather than flying needs more notice than that."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10",
                        options=PilotOptions(autoland="handover"))
    warning = next((e for e in result.pilot.log
                    if "stand by to take control" in e.message.lower()), None)
    handover = next((e for e in result.pilot.log
                     if "your controls" in e.message.lower()), None)
    assert warning is not None, "handed over with no warning at all"
    assert handover is not None
    assert warning.time_s < handover.time_s
    assert warning.level == "warning"


def test_lights_are_not_re_commanded_every_cycle(navdata):
    """These run four times a second for hours; each one is a network round
    trip to the simulator."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    events = result.sim.events_sent
    light_events = [e for e, _ in events
                    if "LIGHTS" in e or e.startswith("STROBES")]
    assert light_events, "the lights were never touched at all"
    assert len(light_events) < 20, f"{len(light_events)} light commands sent"


def test_the_autopilot_is_put_back_when_the_aeroplane_drops_it(navdata):
    """The complaint that started this: it engages, then quietly stops flying.

    A jittery joystick axis reads as a control input and disconnects the
    autopilot, with nothing to say it happened. Simulated here by knocking the
    autopilot out at intervals and checking the aeroplane still arrives.
    """
    from aipilot.aircraft.registry import build_adapter
    from aipilot.autopilot.controller import AIPilot
    from aipilot.route.planner import plan_route
    from aipilot.sim.mock import MockAircraftModel, MockSim

    profile = get_profile("b787-10")
    origin, destination = navdata.airport("EGLL"), navdata.airport("EGCC")
    plan = plan_route(origin, destination, profile, navdata)
    runway = plan.departure_runway
    sim = MockSim(runway.threshold, runway.heading_true_deg, origin.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan)
    pilot.engage()

    knocked_out = 0
    for step in range(int(4 * 3600 / 2)):
        # Every two minutes, something outside the aeroplane drops the autopilot.
        if step and step % 60 == 0 and not sim.state.on_ground and sim.state.ap_master:
            sim.state.ap_master = False
            knocked_out += 1
        pilot.update(2.0)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break

    assert knocked_out > 3, "the test did not actually disconnect anything"
    assert pilot.phase is Phase.COMPLETE, "did not survive the disconnects"
    assert pilot._ap_disconnects >= knocked_out - 1
    assert distance_nm(sim.state.position, plan.threshold_position) < 3.0
    messages = [e.message.lower() for e in pilot.log]
    assert any("dropped the autopilot" in m for m in messages)
    assert any("jitter" in m for m in messages), \
        "repeated disconnects should be diagnosed, not silently papered over"


def test_a_single_disconnect_is_not_over_reported(navdata):
    """One re-engagement is worth a line. It is not worth a diagnosis."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.autopilot.controller import AIPilot
    from aipilot.route.planner import plan_route
    from aipilot.sim.mock import MockAircraftModel, MockSim

    profile = get_profile("b787-10")
    origin, destination = navdata.airport("EGLL"), navdata.airport("EGCC")
    plan = plan_route(origin, destination, profile, navdata)
    runway = plan.departure_runway
    sim = MockSim(runway.threshold, runway.heading_true_deg, origin.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan)
    pilot.engage()
    dropped = False
    for _ in range(int(4 * 3600 / 2)):
        if not dropped and pilot.phase is Phase.CRUISE and sim.state.ap_master:
            sim.state.ap_master = False
            dropped = True
        pilot.update(2.0)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    assert dropped and pilot.phase is Phase.COMPLETE
    assert pilot._ap_disconnects == 1
    assert not any("jitter" in e.message.lower() for e in pilot.log)


def test_a_boeing_recovers_from_a_go_around(navdata):
    """Regression: the Boeing adapter overrode altitude mode selection without
    delegating, so after any commanded vertical speed it could never return to
    altitude capture. A go-around then sat at five hundred feet with full
    thrust and a three thousand foot target it could not climb to."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.autopilot.controller import AIPilot
    from aipilot.route.planner import plan_route
    from aipilot.sim.mock import MockAircraftModel, MockSim

    profile = get_profile("b787-10")
    origin, destination = navdata.airport("EGLL"), navdata.airport("EGCC")
    plan = plan_route(origin, destination, profile, navdata, arrival_runway="23R")
    runway = plan.departure_runway
    model = MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm,
                              decel_kt_s=0.05, flap_transit_s=90.0,
                              gear_transit_s=60.0)
    sim = MockSim(runway.threshold, runway.heading_true_deg, origin.elevation_ft,
                  model=model)
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan,
                    PilotOptions(go_around_if_unstable=True, max_go_arounds=1))
    pilot.engage()
    for _ in range(int(6 * 3600 / 2)):
        pilot.update(2.0)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    assert pilot._go_arounds >= 1
    assert pilot.phase is Phase.COMPLETE, \
        f"stuck in {pilot.phase.value} at {sim.state.altitude_ft:.0f} ft"


def test_the_level_change_command_is_not_spammed(navdata):
    """It was sent every control cycle -- ten thousand times a flight -- which
    re-commands a mode change the aeroplane is still executing."""
    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    flch = [e for e, _ in result.sim.events_sent if e == "FLIGHT_LEVEL_CHANGE_ON"]
    assert flch, "a Boeing should be given level changes at all"
    assert len(flch) < 30, f"sent {len(flch)} level-change commands"
