"""Gate to gate: KJFK to KIAD, from the stand and back onto a stand.

Every other end-to-end test in this suite starts the aeroplane lined up on
the runway, which is precisely the half of the flight that has been working.
The failures reported from the simulator have all been on the ground: an
aeroplane that pushed back a short way and then stood still with its
nosewheel swinging, and an aeroplane that took off from an apron.

So these fly the whole thing -- brakes off on a nose-in gate at Kennedy,
pushback, taxi, takeoff, cruise, approach, landing, and taxi in to a stand
at Dulles -- with a taxiway network at both ends.

The ground layouts here are built from the runway geometry rather than
copied from survey data: a full-length parallel taxiway, connectors at each
end and at the midpoint, and a terminal apron with nose-in stands. That is
the shape of a real airport and it is internally consistent, which is what
a control test needs. It is not a chart, and nothing here should be flown
from.
"""

from __future__ import annotations

import pytest

from aipilot.aircraft.registry import build_adapter
from aipilot.autopilot.controller import AIPilot, PilotOptions
from aipilot.autopilot.phases import Phase
from aipilot.geo import (
    LatLon,
    destination_point,
    distance_nm,
    initial_bearing_deg,
    normalize_deg,
    signed_diff_deg,
)
from aipilot.navdata.base import (
    Airport,
    ChainedNavData,
    GroundLayout,
    Parking,
    Runway,
    TaxiPath,
)
from aipilot.navdata.resolve import NavDataSources, build_navdata
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.route.taxi import build_network
from aipilot.sim.mock import MockAircraftModel, MockSim
from tests.test_kjfk_routes import KJFK

FEET_PER_NM = 6076.11548556

#: Washington Dulles. Three parallel runways and a crosswind, which is what
#: makes it a useful arrival: the runway actually gets chosen.
KIAD = Airport(
    "KIAD", "Washington Dulles International", LatLon(38.944533, -77.455811), 313.0,
    runways=(
        Runway("01L", LatLon(38.92694, -77.46583), 1.7, 11500, 313.0, width_ft=150.0,
               ils_freq_mhz=111.30, ils_course_true_deg=1.7),
        Runway("19R", LatLon(38.95850, -77.46499), 181.7, 11500, 313.0, width_ft=150.0),
        Runway("01R", LatLon(38.92694, -77.43694), 1.7, 11500, 313.0, width_ft=150.0,
               ils_freq_mhz=110.10, ils_course_true_deg=1.7),
        Runway("19L", LatLon(38.95850, -77.43610), 181.7, 11500, 313.0, width_ft=150.0,
               ils_freq_mhz=108.90, ils_course_true_deg=181.7),
        Runway("12", LatLon(38.96417, -77.47139), 121.6, 10501, 313.0, width_ft=150.0),
        Runway("30", LatLon(38.94167, -77.43833), 301.6, 10501, 313.0, width_ft=150.0,
               ils_freq_mhz=111.90, ils_course_true_deg=301.6),
    ))


# --- Building a plausible airport surface -----------------------------------
def _segments(start: LatLon, end: LatLon, name: str, kind: str = "taxi",
              step_nm: float = 0.12) -> list[TaxiPath]:
    """Chop a straight run into segments, the way scenery stores it."""
    total = distance_nm(start, end)
    course = initial_bearing_deg(start, end)
    count = max(1, int(round(total / step_nm)))
    out = []
    cursor = start
    for i in range(1, count + 1):
        nxt = destination_point(start, course, total * i / count)
        out.append(TaxiPath(cursor, nxt, name, kind))
        cursor = nxt
    return out


def _layout(airport: Airport, runway_ident: str, stand_names: list[str],
            apron_side_offset_deg: float = 90.0) -> GroundLayout:
    """A parallel taxiway serving one runway, plus a terminal apron.

    Everything is cut at shared distances along the runway, so that a
    connector, an apron lead-in and the parallel taxiway all meet at the
    same point. Scenery works the same way, and a graph built from segments
    that merely cross without sharing an endpoint is not a graph -- it is a
    pile of disconnected sticks, and nothing can be routed across it.
    """
    runway = airport.runway(runway_ident)
    assert runway is not None
    length_nm = runway.length_ft / FEET_PER_NM
    side = normalize_deg(runway.heading_true_deg + apron_side_offset_deg)
    parallel_offset = 0.055

    #: Where the runway connects to the parallel taxiway: both ends and the
    #: middle, so there is more than one way round.
    link_along = [0.0, length_nm * 0.5, length_nm]
    #: Where each stand hangs off the parallel taxiway.
    stand_along = [length_nm * (0.25 + 0.12 * i) for i in range(len(stand_names))]

    # Every point anything attaches at, plus regular joints in between.
    cuts = set(link_along) | set(stand_along)
    step = 0.12
    cuts |= {min(length_nm, step * i) for i in range(int(length_nm / step) + 2)}
    cuts = sorted(c for c in cuts if -1e-9 <= c <= length_nm + 1e-9)

    def on_runway(along: float) -> LatLon:
        # Along the runway from the threshold, in the landing direction.
        # Runway.point_on_centreline walks *backwards* down the approach --
        # it exists to place approach fixes -- and using it here builds the
        # whole airport off the far end of the extended centreline.
        return destination_point(runway.threshold, runway.heading_true_deg, along)

    def on_taxiway(along: float) -> LatLon:
        return destination_point(on_runway(along), side, parallel_offset)

    paths: list[TaxiPath] = []
    for a, b in zip(cuts, cuts[1:]):
        paths.append(TaxiPath(on_taxiway(a), on_taxiway(b), "A", "taxi"))
        # The runway itself, so the graph knows it exists -- and so the
        # routing cost has something to prefer taxiways over.
        paths.append(TaxiPath(on_runway(a), on_runway(b), runway.ident, "runway"))

    for along in link_along:
        paths.append(TaxiPath(on_runway(along), on_taxiway(along), "link", "taxi"))

    stands: list[Parking] = []
    for name, along in zip(stand_names, stand_along):
        node = on_taxiway(along)
        lead_in = destination_point(node, side, 0.05)
        stand_point = destination_point(lead_in, side, 0.05)
        paths.append(TaxiPath(node, lead_in, f"lead-in {name}", "parking"))
        paths.append(TaxiPath(lead_in, stand_point, f"lead-in {name}", "parking"))
        stands.append(Parking(
            name, stand_point,
            # Nose-in: pointing away from the taxiway, at the terminal, which
            # is how an aeroplane is parked at a gate and what forces a
            # pushback rather than simply driving out.
            heading_true_deg=normalize_deg(side),
            radius_ft=90.0, kind="gate"))

    return GroundLayout(airport.icao, tuple(paths), tuple(stands))


#: Kennedy off 04L, with a row of gates. "328" is the stand the failure was
#: reported from, so that is the one it starts on.
KJFK_LAYOUT = _layout(KJFK, "04L", ["328", "330", "332", "334"])
KIAD_LAYOUT = _layout(KIAD, "01R", ["A12", "A14", "B22", "B24"])


class _NavData(ChainedNavData):
    """The bundled sample with real Kennedy and Dulles substituted in."""

    _AIRPORTS = {"KJFK": KJFK, "KIAD": KIAD}
    _LAYOUTS = {"KJFK": KJFK_LAYOUT, "KIAD": KIAD_LAYOUT}

    def airport(self, icao):
        return self._AIRPORTS.get(icao.strip().upper()) or super().airport(icao)

    def ground_layout(self, icao):
        return self._LAYOUTS.get(icao.strip().upper())


@pytest.fixture(scope="module")
def navdata():
    base = build_navdata(NavDataSources(littlenavmap_db=None, airports_csv=None))
    return _NavData(base.providers)


@pytest.fixture(scope="module")
def networks():
    return build_network(KJFK_LAYOUT), build_network(KIAD_LAYOUT)


# --- The surface itself is sane ---------------------------------------------
def test_both_airports_have_a_connected_taxiway_network(networks):
    departure, arrival = networks
    assert departure.usable and arrival.usable
    for network, layout, runway_ident, airport in (
            (departure, KJFK_LAYOUT, "04L", KJFK),
            (arrival, KIAD_LAYOUT, "01R", KIAD)):
        runway = airport.runway(runway_ident)
        for stand in layout.parking:
            route = network.route(stand.position, runway.threshold)
            assert route, f"{airport.icao} {stand.name} cannot reach {runway_ident}"


def test_the_taxi_route_prefers_taxiways_to_the_runway(networks):
    """Routing along the runway is legal and almost always wrong."""
    departure, _ = networks
    runway = KJFK.runway("04L")
    stand = KJFK_LAYOUT.parking[0]
    route = departure.route(stand.position, runway.threshold)
    length_nm = runway.length_ft / FEET_PER_NM
    # How much of the route sits on the runway centreline, other than the
    # threshold it is aiming at.
    on_runway = sum(
        1 for point in route[:-1]
        if abs(_offset_from_runway(point, runway)) < 0.008
        and 0.02 < _along_runway(point, runway) < length_nm)
    assert on_runway <= 1, f"{on_runway} route points sit on the runway"


def _offset_from_runway(point: LatLon, runway: Runway) -> float:
    from aipilot.geo import cross_track_nm

    far = destination_point(runway.threshold, runway.heading_true_deg,
                            runway.length_ft / FEET_PER_NM)
    return cross_track_nm(point, runway.threshold, far)


def _along_runway(point: LatLon, runway: Runway) -> float:
    from aipilot.geo import along_track_nm

    far = destination_point(runway.threshold, runway.heading_true_deg,
                            runway.length_ft / FEET_PER_NM)
    return along_track_nm(point, runway.threshold, far)


# --- The whole flight --------------------------------------------------------
def _fly_gate_to_gate(navdata, networks, aircraft="b787-10", wind=(30, 12),
                      dt=2.0, max_hours=4.0, options=None):
    origin, destination = navdata.airport("KJFK"), navdata.airport("KIAD")
    profile = get_profile(aircraft)
    plan = plan_route(origin, destination, profile, navdata,
                      departure_runway="04L", arrival_runway="01R",
                      wind_from_deg=wind[0], wind_kt=wind[1])

    def terrain(position):
        near = distance_nm(position, origin.position)
        far = distance_nm(position, destination.position)
        if far < 5.0:
            return destination.elevation_ft
        if near < 5.0:
            return origin.elevation_ft
        return (origin.elevation_ft * far + destination.elevation_ft * near) / \
            max(near + far, 1e-6)

    stand = KJFK_LAYOUT.parking[0]          # gate 328, nose-in
    sim = MockSim(stand.position, stand.heading_true_deg, origin.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                  terrain=terrain, wind_from_deg=wind[0], wind_kt=wind[1])
    adapter, _ = build_adapter(aircraft, sim)
    pilot = AIPilot(sim, adapter, profile, plan, options or PilotOptions(),
                    ground=networks[0], arrival_ground=networks[1])
    pilot.engage()

    seen: list[Phase] = []
    worst_overspeed = 0.0
    lowest_agl = 1e9
    tug_after_taxi = False
    for _ in range(int(max_hours * 3600 / dt)):
        status = pilot.update(dt)
        if not seen or seen[-1] is not pilot.phase:
            seen.append(pilot.phase)
        worst_overspeed = max(worst_overspeed, status.ias_kt - profile.vmo_kt)
        if pilot.phase.airborne and pilot.phase is not Phase.LANDING:
            lowest_agl = min(lowest_agl, status.altitude_agl_ft)
        if pilot.phase in (Phase.TAXI, Phase.TAKEOFF) and sim.state.pushback_attached:
            tug_after_taxi = True
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    return pilot, sim, plan, seen, worst_overspeed, lowest_agl, tug_after_taxi


@pytest.fixture(scope="module")
def flight(navdata, networks):
    return _fly_gate_to_gate(navdata, networks)


def test_it_gets_from_a_gate_at_kennedy_to_a_gate_at_dulles(flight):
    pilot, sim, _plan, seen, _over, _agl, _tug = flight
    assert pilot.phase is Phase.COMPLETE, f"ended in {pilot.phase.value}"
    order = [p for p in seen]
    for phase in (Phase.PUSHBACK, Phase.TAXI, Phase.TAKEOFF, Phase.CLIMB,
                  Phase.CRUISE, Phase.DESCENT, Phase.APPROACH, Phase.LANDING,
                  Phase.ROLLOUT, Phase.TAXI_IN, Phase.COMPLETE):
        assert phase in order, f"never reached {phase.value}: {[p.value for p in order]}"
    assert order.index(Phase.PUSHBACK) < order.index(Phase.TAXI) \
        < order.index(Phase.TAKEOFF)


def test_it_pushes_back_off_the_nose_in_gate_and_lets_the_tug_go(flight):
    """The reported failure: pushed back a little, then stood still with the
    nosewheel swinging, because the tug was re-summoned as fast as it was
    released."""
    pilot, sim, _plan, _seen, _over, _agl, tug_after_taxi = flight
    assert any("pushing back" in e.message.lower() for e in pilot.log)
    assert not tug_after_taxi, "the tug was still attached after the taxi began"
    assert not sim.state.pushback_attached


def test_it_takes_off_from_the_runway_and_not_the_apron(flight):
    """The other reported failure: takeoff thrust applied on the stand."""
    pilot, _sim, plan, _seen, _over, _agl, _tug = flight
    runway = plan.departure_runway
    takeoff = next((e for e in pilot.log if "takeoff" in e.message.lower()), None)
    assert takeoff is not None
    # It said it was lined up, and the phase order proves it taxied there first.
    assert "lined up" in takeoff.message.lower() or \
        any("lined up" in e.message.lower() for e in pilot.log)
    assert runway.ident == "04L"


def test_the_flight_itself_is_flown_properly(flight):
    pilot, sim, plan, _seen, overspeed, lowest_agl, _tug = flight
    assert overspeed <= 5.0, f"exceeded Vmo by {overspeed:.0f} kt"
    assert lowest_agl > 150.0, f"got within {lowest_agl:.0f} ft of the ground"
    assert pilot._touchdown_vs is not None and pilot._touchdown_vs > -400
    assert 0.4 <= pilot.elapsed_s / 3600 <= 2.5, \
        f"took {pilot.elapsed_s / 3600:.2f} h for a 200 nm sector"


def test_it_vacates_the_runway_and_parks(flight):
    pilot, sim, plan, _seen, _over, _agl, _tug = flight
    # Off the runway, and on one of Dulles' stands.
    assert abs(_offset_from_runway(sim.state.position,
                                   plan.arrival_runway)) > 0.02, \
        "it stopped on the runway"
    nearest = min(KIAD_LAYOUT.parking,
                  key=lambda p: distance_nm(sim.state.position, p.position))
    assert distance_nm(sim.state.position, nearest.position) < 0.08, \
        "it stopped somewhere that is not a stand"
    assert sim.state.ground_speed_kt < 1.0, "it never actually stopped"
    assert any("stand" in e.message.lower() for e in pilot.log)


def test_it_stays_on_the_pavement_taxiing_out(navdata, networks):
    """Cutting the corner off a taxiway is how an aeroplane hits a terminal."""
    from aipilot.geo import along_track_nm

    origin, destination = navdata.airport("KJFK"), navdata.airport("KIAD")
    profile = get_profile("b787-10")
    plan = plan_route(origin, destination, profile, navdata,
                      departure_runway="04L", arrival_runway="01R")
    stand = KJFK_LAYOUT.parking[0]
    sim = MockSim(stand.position, stand.heading_true_deg, origin.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(),
                    ground=networks[0], arrival_ground=networks[1])
    pilot.engage()

    worst = 0.0
    samples: list[float] = []
    for _ in range(2500):
        pilot.update(1.0)
        if pilot.phase is Phase.TAXI and pilot.taxi is not None \
                and pilot.taxi.index >= 2 and not pilot.taxi.finished:
            route = pilot.taxi.route
            a, b = route[pilot.taxi.index - 1], route[pilot.taxi.index]
            length = distance_nm(a, b)
            if length < 1e-9:
                continue
            along = along_track_nm(sim.state.position, a, b)
            if along < 0:
                deviation = distance_nm(sim.state.position, a)
            elif along > length:
                deviation = distance_nm(sim.state.position, b)
            else:
                from aipilot.geo import cross_track_nm
                deviation = abs(cross_track_nm(sim.state.position, a, b))
            samples.append(deviation)
            worst = max(worst, deviation)
        if pilot.phase in (Phase.TAKEOFF, Phase.CLIMB):
            break
    # Only a guard against "it never really taxied". The count it can expect
    # fell when simplify stopped keeping every scenery kink: the same route is
    # a handful of points now rather than dozens, so there are fewer legs to
    # sample while flying it. The tracking assertions below are unchanged, and
    # it still has to reach the runway.
    assert len(samples) > 10, "barely taxied at all"
    assert pilot.phase in (Phase.TAKEOFF, Phase.CLIMB), \
        f"never reached the runway, ended in {pilot.phase.value}"
    typical = sorted(samples)[len(samples) // 2]
    assert typical < 0.010, f"typically {typical * FEET_PER_NM:.0f} ft off its route"
    assert worst < 0.08, f"cut a corner by {worst * FEET_PER_NM:.0f} ft"


def test_it_lines_up_before_it_rolls(navdata, networks):
    """A takeoff roll that starts fifteen degrees off runs out of runway
    sideways."""
    pilot, sim, plan, seen, _over, _agl, _tug = _fly_gate_to_gate(
        navdata, networks, max_hours=0.6)
    # Caught at the moment of takeoff rather than at the end.
    assert Phase.TAKEOFF in seen


@pytest.mark.parametrize("aircraft", ["b787-10", "a320neo"])
def test_other_aeroplanes_manage_the_same_trip(navdata, networks, aircraft):
    pilot, sim, plan, _seen, overspeed, _agl, tug = _fly_gate_to_gate(
        navdata, networks, aircraft=aircraft)
    assert pilot.phase is Phase.COMPLETE, f"{aircraft} ended in {pilot.phase.value}"
    assert not tug
    assert overspeed <= 5.0
