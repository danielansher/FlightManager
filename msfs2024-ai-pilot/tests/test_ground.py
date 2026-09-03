"""Pushback and taxi, across a synthetic airport with real taxiway geometry."""

import pytest

from aipilot.aircraft.registry import build_adapter
from aipilot.autopilot.controller import AIPilot, PilotOptions
from aipilot.autopilot.ground import TaxiGuidance, pushback_needed
from aipilot.autopilot.phases import Phase
from aipilot.geo import (
    LatLon,
    cross_track_nm,
    destination_point,
    distance_nm,
    signed_diff_deg,
)
from aipilot.navdata.base import Airport, GroundLayout, Parking, Runway, TaxiPath
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.route.taxi import GroundNetwork, build_network, simplify
from aipilot.sim.mock import MockAircraftModel, MockSim

# A small airfield: one runway, a parallel taxiway with two links, and a stand
# on a lead-in line off the taxiway -- the layout that requires a pushback.
THRESHOLD = LatLon(51.4700, -0.4800)
RUNWAY_HEADING = 90.0
RUNWAY = Runway("09", THRESHOLD, RUNWAY_HEADING, 9000.0, 100.0, width_ft=150.0)
FAR_END = destination_point(THRESHOLD, RUNWAY_HEADING, 9000.0 / 6076.11548556)

#: The parallel taxiway, 400 ft south of the runway.
_OFFSET = 400.0 / 6076.11548556
TWY_WEST = destination_point(THRESHOLD, RUNWAY_HEADING + 90.0, _OFFSET)
TWY_EAST = destination_point(FAR_END, RUNWAY_HEADING + 90.0, _OFFSET)
STAND_ENTRY = destination_point(TWY_WEST, RUNWAY_HEADING, 0.45)
STAND = destination_point(STAND_ENTRY, RUNWAY_HEADING + 90.0, 0.055)

DEPARTURE = Airport("EGXX", "Test Field", THRESHOLD, 100.0, runways=(RUNWAY,))
ARRIVAL = Airport("EGYY", "Other Field", LatLon(52.6, -1.4), 200.0,
                  runways=(Runway("27", LatLon(52.6, -1.35), 270.0, 9000.0, 200.0),))


def _layout() -> GroundLayout:
    paths = (
        TaxiPath(TWY_WEST, STAND_ENTRY, "A"),
        TaxiPath(STAND_ENTRY, TWY_EAST, "A"),
        TaxiPath(STAND_ENTRY, STAND, "A1", kind="parking"),
        TaxiPath(TWY_WEST, THRESHOLD, "L1"),          # link onto the runway
        TaxiPath(THRESHOLD, FAR_END, "09", kind="runway"),
    )
    parking = (Parking("STAND 1", STAND, RUNWAY_HEADING + 90.0),)
    return GroundLayout("EGXX", paths, parking)


@pytest.fixture
def network() -> GroundNetwork:
    built = build_network(_layout())
    assert built is not None
    return built


def _fly(start, heading, options=None, network=None, seconds=1200, dt=1.0):
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    sim = MockSim(start, heading, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, options or PilotOptions(),
                    ground=network)
    pilot.engage()
    for _ in range(int(seconds / dt)):
        pilot.update(dt)
        if pilot.phase in (Phase.CLIMB, Phase.COMPLETE, Phase.ABORTED):
            break
    return pilot, sim


# --- The network ------------------------------------------------------------
def test_the_taxiways_form_one_connected_network(network):
    assert network.usable
    route = network.route(STAND, THRESHOLD)
    assert route, "no route from the stand to the runway"
    length = sum(distance_nm(a, b) for a, b in zip(route, route[1:]))
    assert 0.4 < length < 1.5, f"route length {length:.2f} nm looks wrong"


def test_segment_endpoints_are_welded_into_junctions(network):
    """Scenery does not guarantee bitwise-identical endpoints, and without
    welding the graph falls into disconnected pieces."""
    from aipilot.route.taxi import WELD_TOLERANCE_NM

    nudged = destination_point(STAND_ENTRY, 45.0, WELD_TOLERANCE_NM * 0.4)
    layout = GroundLayout("EGXX", _layout().taxi_paths + (
        TaxiPath(nudged, destination_point(nudged, 180.0, 0.2), "B"),))
    joined = build_network(layout)
    assert joined.route(STAND, destination_point(nudged, 180.0, 0.2))


def test_a_route_to_somewhere_unconnected_is_no_route(network):
    assert network.route(STAND, LatLon(48.0, 2.0)) == []


def test_simplify_keeps_the_turns_and_drops_the_rest(network):
    route = network.route(STAND, THRESHOLD)
    reduced = simplify(route)
    assert 2 <= len(reduced) < len(route)
    assert reduced[0] == route[0] and reduced[-1] == route[-1]


def test_a_stand_on_a_lead_in_line_needs_no_pushback(network):
    needed, _distance = pushback_needed(STAND, network, None)
    assert not needed, "the stand is on its own lead-in line, so it can taxi"


def test_a_stand_off_the_network_needs_a_pushback(network):
    remote = destination_point(STAND, RUNWAY_HEADING + 90.0, 0.20)
    needed, distance = pushback_needed(remote, network, None)
    assert needed and distance > 0


# --- Taxiing ----------------------------------------------------------------
def test_it_taxis_from_the_stand_and_lines_up(network):
    pilot, sim = _fly(STAND, RUNWAY_HEADING + 90.0, network=network)
    assert pilot.phase in (Phase.TAKEOFF, Phase.CLIMB), \
        f"ended in {pilot.phase.value}"
    assert any("taxiing to 09" in e.message.lower() for e in pilot.log)
    # And it ended up on the runway, pointing the right way.
    assert abs(cross_track_nm(sim.state.position, THRESHOLD, FAR_END)) < 0.03
    assert abs(signed_diff_deg(sim.state.heading_true_deg, RUNWAY_HEADING)) < 12


def test_it_stays_on_the_pavement(network):
    """The whole point: it follows the centrelines rather than cutting across.

    Measured as deviation from the route it is following, which is the honest
    control metric -- the route lies on the taxiways by construction, so
    staying on the route *is* staying on the pavement, and unlike distance to
    the nearest taxiway it is not confounded by the aeroplane legitimately
    being on a stand at the start.

    Measured from the second leg onward. The first is a join -- the aeroplane
    starts beside it rather than on it -- so its cross-track says how far the
    stand is from the taxiway, not how well the aeroplane tracks.
    """
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    sim = MockSim(STAND, RUNWAY_HEADING + 90.0, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()

    worst = 0.0
    measured = 0
    samples: list[float] = []
    for _ in range(1200):
        pilot.update(1.0)
        if pilot.phase is Phase.TAXI and pilot.taxi is not None \
                and pilot.taxi.index >= 2 and not pilot.taxi.finished:
            route = pilot.taxi.route
            leg_start = route[pilot.taxi.index - 1]
            leg_end = route[pilot.taxi.index]
            deviation = _distance_to_segment(sim.state.position,
                                             leg_start, leg_end)
            samples.append(deviation)
            worst = max(worst, deviation)
            measured += 1
        if pilot.phase in (Phase.TAKEOFF, Phase.CLIMB):
            break
    # This guard is only here to catch "it never really taxied", and the count
    # it can expect fell when simplify stopped keeping every scenery kink: this
    # route is four points now rather than a dozen, and one of them sequences
    # without ever being the leg under test. That it taxied is asserted
    # directly below, by where it ended up.
    assert measured > 10, "barely taxied at all"
    assert pilot.phase in (Phase.TAKEOFF, Phase.CLIMB), \
        f"never got off the ground, ended in {pilot.phase.value}"
    # Two numbers, because they say different things. Typical deviation is the
    # tracking quality and should be small. The worst case is a corner cut: on
    # a sharp turn a large aeroplane at taxi speed cannot follow the corner
    # exactly, and the sparser the taxiway data the wider it goes. This is a
    # deliberately sparse synthetic layout; a real airport's taxi network is
    # much denser and cuts less.
    typical = sorted(samples)[len(samples) // 2]
    assert typical < 0.008, f"typically {typical * 6076:.0f} ft off its route"
    assert worst < 0.07, f"cut a corner by {worst * 6076:.0f} ft"


def _distance_to_segment(point, start, end):
    from aipilot.geo import along_track_nm

    length = distance_nm(start, end)
    if length < 1e-9:
        return distance_nm(point, start)
    along = along_track_nm(point, start, end)
    if along < 0:
        return distance_nm(point, start)
    if along > length:
        return distance_nm(point, end)
    return abs(cross_track_nm(point, start, end))


def test_taxi_speed_stays_sensible(network):
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    sim = MockSim(STAND, RUNWAY_HEADING + 90.0, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()
    fastest = 0.0
    for _ in range(1200):
        pilot.update(1.0)
        if pilot.phase is Phase.TAXI:
            fastest = max(fastest, sim.state.ground_speed_kt)
        if pilot.phase in (Phase.TAKEOFF, Phase.CLIMB):
            break
    assert 5.0 < fastest < 30.0, f"taxied at {fastest:.0f} kt"


def test_without_taxiway_data_it_does_not_move(network):
    """Guessing a path across an apron is how an aeroplane ends up in a
    building, so with no data it waits and says so."""
    pilot, sim = _fly(STAND, RUNWAY_HEADING + 90.0, network=None, seconds=300)
    assert pilot.phase is Phase.PREFLIGHT
    assert sim.state.ias_kt < 3.0
    assert any("does not taxi" in e.message.lower() for e in pilot.log)


def test_taxi_can_be_turned_off(network):
    pilot, sim = _fly(STAND, RUNWAY_HEADING + 90.0,
                      options=PilotOptions(taxi=False), network=network,
                      seconds=300)
    assert pilot.phase is Phase.PREFLIGHT
    assert sim.state.ias_kt < 3.0


def test_the_takeoff_roll_keeps_straight(network):
    """Nosewheel steering now has to hold the runway; nothing else does."""
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    start = destination_point(THRESHOLD, RUNWAY_HEADING, 0.05)
    sim = MockSim(start, RUNWAY_HEADING + 4.0, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()
    worst = 0.0
    for _ in range(300):
        pilot.update(1.0)
        if sim.state.on_ground and pilot.phase is Phase.TAKEOFF:
            worst = max(worst, abs(cross_track_nm(sim.state.position,
                                                  THRESHOLD, FAR_END)))
        if not sim.state.on_ground:
            break
    assert not sim.state.on_ground, "never got airborne"
    assert worst < 0.02, f"wandered {worst * 6076:.0f} ft off the centreline"


# --- Lights and cabin signs -------------------------------------------------
def test_lights_follow_the_phase_of_flight(navdata):
    """Every one of these is a toggle event, so getting them right depends on
    reading the switch rather than assuming it."""
    from .conftest import fly_flight

    result = fly_flight(navdata, "EGLL", "EGCC", "b787-10")
    final = result.sim.state
    # Cleared the runway: strobes off, beacon and nav still on.
    assert not final.light_strobe
    assert final.light_nav
    seen = {}
    for event in result.pilot.log:
        seen.setdefault(event.phase, None)
    assert Phase.CLIMB in seen and Phase.DESCENT in seen


@pytest.mark.parametrize("switch,expect_on", [
    ("light_nav", True), ("light_beacon", True), ("light_strobe", True),
    ("light_landing", True), ("seatbelt_sign", True),
])
def test_the_right_switches_are_on_during_the_takeoff_roll(network, switch,
                                                           expect_on):
    runway = RUNWAY
    lined_up = destination_point(runway.threshold, runway.heading_true_deg, 0.05)
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    sim = MockSim(lined_up, runway.heading_true_deg, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()
    for _ in range(60):
        pilot.update(1.0)
        if pilot.phase is Phase.TAKEOFF and sim.state.ground_speed_kt > 20:
            break
    assert getattr(sim.state, switch) is expect_on, \
        f"{switch} should be {'on' if expect_on else 'off'} on the takeoff roll"


def test_the_taxi_light_is_on_while_taxiing_and_off_on_the_runway(network):
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    sim = MockSim(STAND, RUNWAY_HEADING + 90.0, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()
    on_during_taxi = False
    for _ in range(1200):
        pilot.update(1.0)
        if pilot.phase is Phase.TAXI and sim.state.light_taxi:
            on_during_taxi = True
        if pilot.phase in (Phase.TAKEOFF, Phase.CLIMB):
            break
    assert on_during_taxi, "the taxi light was never on while taxiing"
    for _ in range(30):
        pilot.update(1.0)
    assert not sim.state.light_taxi, "taxi light still on during the takeoff roll"


def test_a_switch_the_pilot_moves_by_hand_is_not_fought(network):
    """The AI Pilot only acts when a switch is in the wrong position, so a
    setting that agrees with it is left alone rather than re-commanded."""
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    lined_up = destination_point(RUNWAY.threshold, RUNWAY.heading_true_deg, 0.05)
    sim = MockSim(lined_up, RUNWAY.heading_true_deg, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()
    for _ in range(120):
        pilot.update(1.0)
    before = len([e for e, _ in sim.events_sent if "LIGHT" in e or "STROBE" in e])
    for _ in range(120):
        pilot.update(1.0)
    after = len([e for e, _ in sim.events_sent if "LIGHT" in e or "STROBE" in e])
    assert after - before <= 4, "kept re-commanding switches already correct"


# --- Letting the tug go ------------------------------------------------------
# Reported from a real flight: at a JFK gate the 787 pushed back a short way,
# then stood still with its nosewheel swinging left and right and never taxied.
#
# The cause was that KEY_TUG_HEADING does not merely steer a pushback that is
# already running -- in the simulator it is how a pushback is *started*. The
# controller kept sending the tug a heading every cycle, including after asking
# it to disconnect, so the tug was re-attached as fast as it was released. The
# aeroplane was held on the stand by a tug it could not shake off, with thrust
# doing nothing and the nosewheel following the tug commands: exactly what was
# seen.
#
# It could not be reproduced here, because the mock let a tug heading through
# without attaching a tug. It does now, which is what makes these tests mean
# something.
#: Nose-in on the stand, facing the terminal, the way an aeroplane is actually
#: parked at a gate: the taxiway is directly behind it, so it cannot drive out
#: and has to be pushed back and turned. This is the case that failed at JFK.
NOSE_IN_HEADING = 180.0


def _push_and_taxi(network, options=None):
    return _fly(STAND, NOSE_IN_HEADING, options=options, network=network)


def test_a_pushback_ends_and_the_aeroplane_taxis_away(network):
    pilot, sim = _push_and_taxi(network)
    assert any("pushing back" in e.message.lower() for e in pilot.log), \
        "this stand should have needed a pushback"
    assert pilot.phase in (Phase.TAKEOFF, Phase.CLIMB), \
        f"stuck in {pilot.phase.value} -- it never got off the stand"
    assert not sim.state.pushback_attached, "the tug was never let go"


def test_the_tug_is_not_summoned_back_after_being_released(network):
    """The specific failure: released, then immediately re-attached."""
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    sim = MockSim(STAND, NOSE_IN_HEADING, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()

    detached_at = None
    for step in range(1200):
        pilot.update(1.0)
        if detached_at is None and pilot.phase is Phase.TAXI:
            detached_at = step
        if detached_at is not None:
            assert not sim.state.pushback_attached, (
                "the tug came back after the taxi started, at step "
                f"{step} (released at {detached_at})")
        if pilot.phase in (Phase.TAKEOFF, Phase.CLIMB):
            break
    assert detached_at is not None, "never reached the taxi"


def test_the_tug_is_told_its_heading_after_it_has_attached(network):
    """The heading went out in the same cycle as the request for the tug, when
    the simulator had not attached one yet and there was nothing to hear it.
    Nothing changes the heading afterwards, so the value guard in
    set_tug_heading suppressed every later send. Pushed off a Kennedy gate the
    aeroplane recorded exactly one heading for the whole push -- its gate
    heading -- went 183 ft straight back without turning at all, and was left
    nose-on to the terminal for the taxi to drive at."""
    sim = MockSim(STAND, NOSE_IN_HEADING, DEPARTURE.elevation_ft)
    adapter, _ = build_adapter("b787-10", sim)

    def headings_sent():
        return [v for e, v in sim.events_sent if e == "KEY_TUG_HEADING"]

    adapter.set_tug_heading(67.0)
    adapter.set_tug_heading(67.0)
    assert len(headings_sent()) == 1, \
        "an unchanged heading should not be resent every cycle"

    # The one case where the unchanged heading must go again: the first send
    # was thrown at a tug that did not exist yet.
    adapter.forget_tug_heading()
    adapter.set_tug_heading(67.0)
    assert len(headings_sent()) == 2, \
        "there is no way to tell the tug a heading it missed"


def test_the_aeroplane_actually_moves_after_the_pushback(network):
    """Standing still with the nosewheel swinging is the thing to catch."""
    pilot, sim = _push_and_taxi(network)
    taxi_log = [e.message for e in pilot.log if "taxiing to" in e.message.lower()]
    assert taxi_log, "it never started a taxi"
    assert distance_nm(sim.state.position, STAND) > 0.3, \
        "it barely left the stand"


def test_a_tug_that_never_reports_leaving_does_not_strand_the_aeroplane(network):
    """Some aircraft never clear PUSHBACK ATTACHED. Say so, and carry on."""
    profile = get_profile("b787-10")
    plan = plan_route(DEPARTURE, ARRIVAL, profile, None, departure_runway="09")
    sim = MockSim(STAND, NOSE_IN_HEADING, DEPARTURE.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))

    # A simulator that acknowledges nothing: the tug stays attached for ever.
    original = sim.send_event

    def stubborn(name, value=0):
        if name == "TOGGLE_PUSHBACK" and sim.state.pushback_attached:
            return
        original(name, value)

    sim.send_event = stubborn
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()
    for _ in range(600):
        pilot.update(1.0)
        if pilot.phase not in (Phase.PREFLIGHT, Phase.PUSHBACK):
            break

    assert pilot.phase is Phase.TAXI, \
        f"gave up on the stand in {pilot.phase.value}"
    assert any("tug attached" in e.message.lower() for e in pilot.log), \
        "it moved on without saying the tug never disconnected"


def test_a_stand_the_taxiways_cannot_reach_says_so_and_stops(network):
    """A pushback is only ever a couple of hundred yards. If the stand is
    further than that from any taxiway the data knows about, there is no
    route, and the only honest answer is to say so and hold.

    Entering the taxi phase with nothing to follow was the wrong answer: it
    steers nothing and commands nothing, and the phase machine will not run
    backwards, so the aeroplane stands on the apron for the rest of the day
    with no explanation."""
    # Beyond anything a push can reach: PUSHBACK_MAX_NM of push plus
    # MAX_JOIN_DISTANCE_NM of join is 0.29 nm. This was 0.20, which stopped
    # being out of reach when the push was lengthened to fit a full turn --
    # the stand had not moved, the aeroplane simply got further.
    stranded = destination_point(STAND, RUNWAY_HEADING + 90.0, 0.35)
    pilot, sim = _fly(stranded, NOSE_IN_HEADING, network=network, seconds=400)

    assert pilot.phase is Phase.PREFLIGHT, \
        f"ended up stuck in {pilot.phase.value}"
    assert not sim.state.pushback_attached, "the tug was never let go"
    messages = " ".join(e.message.lower() for e in pilot.log)
    assert "could not find a way across the taxiways" in messages
    assert "taxi out" in messages, "it never said what the pilot should do"


def test_a_taxi_route_has_no_leg_shorter_than_the_aeroplane():
    """Around a terminal the graph has nodes closer together than an airliner
    is long, and simplify judged a turn on its angle alone. The kept points
    came out 17 to 80 ft apart, so the steering target moved on before the turn
    onto the last one had finished and the nosewheel went right, left, right,
    left about a second apart. Out of a Kennedy gate: 31 turns in 1.45 nm and
    19 legs under 150 ft."""
    from aipilot.route.taxi import MIN_LEG_NM, simplify

    # A long run made of 40 ft steps that alternate 25 degrees either side:
    # every one of them is a "turn" by angle, none is a leg by length.
    step_nm = 40.0 / 6076.12
    points = [LatLon(40.6402, -73.7807)]
    heading = 69.0
    for index in range(60):
        heading += 25.0 if index % 2 == 0 else -25.0
        points.append(destination_point(points[-1], heading, step_nm))

    reduced = simplify(points)
    legs = [distance_nm(a, b) for a, b in zip(reduced, reduced[1:])]
    too_short = [leg * 6076.12 for leg in legs if leg < MIN_LEG_NM * 0.95]
    assert not too_short, (
        f"{len(too_short)} of {len(legs)} legs are shorter than the aeroplane: "
        f"{', '.join(f'{leg:.0f} ft' for leg in too_short[:6])}")


def test_a_bigger_turn_off_the_stand_gets_a_longer_push():
    """The distance and the heading were chosen independently, and the worst
    case got the shortest push: any turn past the limit meant "nose-in, needs a
    tug" and was answered with the minimum. Asked for 182 ft and 180 degrees,
    the tug managed 56 degrees of it and left the aeroplane across the apron
    for the taxi to unwind."""
    from aipilot.autopilot.ground import pushback_distance_for

    quarter = pushback_distance_for(90.0)
    about_face = pushback_distance_for(180.0)
    assert about_face > quarter, \
        "a 180 degree turn needs more room than a 90, not the same or less"

    # 1.3 deg/s at 2.6 kt is 3.4 ft per degree, measured on the aeroplane.
    turn_room_ft = (about_face - quarter) * 6076.12
    assert 250.0 < turn_room_ft < 380.0, \
        f"the extra 90 degrees was given {turn_room_ft:.0f} ft to turn in"


def test_the_tug_pushes_straight_before_it_turns():
    """A tug pulls the aeroplane clear of the stand before it swings it. Handed
    the final heading at the outset it turns from the moment it takes the
    weight, which drags the tail across the neighbouring stand."""
    from aipilot.autopilot.ground import PushbackGuidance

    start = LatLon(40.6402, -73.7807)
    push = PushbackGuidance(start, 247.0, target_distance_nm=0.12,
                            final_heading_deg=67.0,
                            straight_distance_nm=0.025)

    assert push.tug_heading == 247.0, "it turned before clearing the stand"
    assert not push.turning

    # Ten feet at a time, straight back, until the straight leg is behind us.
    position, step_nm = start, 10.0 / 6076.12
    for _ in range(400):
        position = destination_point(position, push.push_direction_deg, step_nm)
        push.advance(position)
        if push.turning:
            break

    assert push.turning, "the turn never began"
    assert push.tug_heading == 67.0, "the tug was never given the final heading"


def test_a_turning_push_does_not_end_where_a_straight_one_would():
    """Once the tug turns, the aeroplane is travelling a curve and for a large
    turn it comes back on itself. Treating the end as a straight run put it
    nowhere near where it went: 764 ft of push through 174 degrees moved a 787
    just 418 ft from the stand, on a bearing 68 degrees off the one it set off
    on. The onward route was then read from a point it never reached, and the
    taxi opened by turning 124 degrees to undo the push."""
    from aipilot.autopilot.ground import pushback_end_point
    from aipilot.geo import initial_bearing_deg

    start, heading = LatLon(40.640164, -73.780675), 247.0   # GC 31 at Kennedy

    # Measured on the aeroplane: 418 ft from the stand, bearing 135.
    end = pushback_end_point(start, heading, 174.0)
    assert 380.0 < distance_nm(start, end) * 6076.12 < 460.0, \
        f"ended {distance_nm(start, end) * 6076.12:.0f} ft out, the flight made 418"
    assert abs(signed_diff_deg(initial_bearing_deg(start, end), 135.0)) < 12.0, \
        f"bore {initial_bearing_deg(start, end):.0f}, the flight made 135"

    # A straight push is the special case, not the general one.
    straight = pushback_end_point(start, heading, 0.0)
    assert abs(signed_diff_deg(initial_bearing_deg(start, straight), 67.0)) < 1.0
    assert distance_nm(start, end) > distance_nm(start, straight) * 1.5, \
        "a turning push and a straight one cannot end in the same place"


def test_a_push_longer_than_its_turn_carries_on_straight():
    """Whatever push is left once the turn is done is spent going straight
    backwards on the new heading. Left out of the model, a 763 ft push that
    needed only 580 ft to turn put the predicted end 63 ft and 20 degrees off
    where the aeroplane actually stopped -- and the taxi, routed from that
    imaginary point, opened by turning 155 degrees."""
    from aipilot.autopilot.ground import pushback_end_point, pushback_distance_for
    from aipilot.geo import initial_bearing_deg

    start, heading, turn = LatLon(40.640164, -73.780675), 247.0, -126.3

    # Sized to its own turn, there is no tail and the two agree.
    sized = pushback_distance_for(turn)
    assert distance_nm(pushback_end_point(start, heading, turn),
                       pushback_end_point(start, heading, turn,
                                          total_nm=sized)) * 6076.12 < 5.0

    # Pushed past that, the aeroplane ends somewhere else. Measured on the
    # aeroplane: 763 ft of push ended 501 ft out on a bearing of 2.
    overshot = pushback_end_point(start, heading, turn, total_nm=763.0 / 6076.12)
    assert 470.0 < distance_nm(start, overshot) * 6076.12 < 530.0, \
        f"{distance_nm(start, overshot) * 6076.12:.0f} ft out, the flight made 501"
    assert abs(signed_diff_deg(initial_bearing_deg(start, overshot), 2.0)) < 12.0


def test_a_waypoint_it_cannot_reach_is_eventually_given_up_on():
    """Off GC 31 at Kennedy the first waypoint sat 166 to 221 ft away and 84 to
    100 degrees off the nose: never inside the capture radius, never far enough
    round to count as behind. The aeroplane orbited it at 7 kt and the taxi
    never sequenced past its first point, for as long as it was left running.
    """
    from aipilot.autopilot.ground import TaxiGuidance
    from aipilot.sim.base import SimState

    unreachable = LatLon(40.6402, -73.7807)
    beyond = destination_point(unreachable, 0.0, 3.0)
    taxi = TaxiGuidance([unreachable, beyond])

    # Circle the waypoint itself, 200 ft out, flying the tangent. It stays 200
    # ft away -- outside the capture radius -- and 90 degrees off the nose,
    # inside the angle that counts as behind. Exactly the orbit it flew.
    radius_ft = 200.0
    travelled_ft = 0.0
    step_deg = 6.0
    for step in range(240):
        bearing = (step * step_deg) % 360.0
        position = destination_point(unreachable, bearing, radius_ft / 6076.12)
        state = SimState(lat=position.lat, lon=position.lon,
                         heading_true_deg=(bearing + 90.0) % 360.0,
                         ground_speed_kt=7.0, on_ground=True)
        taxi.update(state)
        travelled_ft += 2.0 * 3.14159 * radius_ft * (step_deg / 360.0)
        if taxi.index > 0:
            break

    assert taxi.index > 0, \
        "it circled the waypoint for a mile and never gave up on it"
    assert travelled_ft < 4000.0, \
        f"took {travelled_ft:.0f} ft of circling to notice"
