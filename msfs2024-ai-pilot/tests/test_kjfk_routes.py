"""Real routes out of and into New York Kennedy.

A working AI Pilot is one that flies the trips someone actually flies, so these
are real sectors from a real home base, at three lengths, with real KJFK runway
geometry and ILS frequencies. They are slow -- a whole long haul each -- but
they are the tests that would have caught every failure reported from the
simulator so far, because each of those only appeared on a complete flight.
"""

import pytest

from aipilot.aircraft.registry import build_adapter
from aipilot.autopilot.controller import AIPilot, PilotOptions
from aipilot.autopilot.phases import Phase
from aipilot.geo import LatLon, destination_point, distance_nm
from aipilot.navdata.base import Airport, ChainedNavData, Runway
from aipilot.navdata.resolve import NavDataSources, build_navdata
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.sim.mock import MockAircraftModel, MockSim

#: Kennedy's four runways, as pairs, with the ILS frequencies it really has.
KJFK = Airport(
    "KJFK", "New York John F Kennedy", LatLon(40.639751, -73.778925), 13.0,
    runways=(
        Runway("04L", LatLon(40.6222, -73.7862), 31.1, 12079, 13.0, width_ft=200.0,
               ils_freq_mhz=110.90, ils_course_true_deg=31.1),
        Runway("22R", LatLon(40.6448, -73.7648), 211.1, 12079, 13.0, width_ft=200.0),
        Runway("04R", LatLon(40.6258, -73.7700), 31.1, 8400, 13.0, width_ft=200.0,
               ils_freq_mhz=109.50, ils_course_true_deg=31.1),
        Runway("22L", LatLon(40.6420, -73.7530), 211.1, 8400, 13.0, width_ft=200.0),
        Runway("13L", LatLon(40.6558, -73.7930), 133.1, 10000, 13.0, width_ft=200.0),
        Runway("31R", LatLon(40.6367, -73.7660), 313.1, 10000, 13.0, width_ft=200.0,
               ils_freq_mhz=111.50, ils_course_true_deg=313.1),
        Runway("13R", LatLon(40.6480, -73.8180), 133.1, 14511, 13.0, width_ft=200.0),
        Runway("31L", LatLon(40.6224, -73.7660), 313.1, 14511, 13.0, width_ft=200.0,
               ils_freq_mhz=111.35, ils_course_true_deg=313.1),
    ))


class _JFKNavData(ChainedNavData):
    """The bundled sample, with the real Kennedy substituted in."""

    def airport(self, icao):
        if icao.strip().upper() == "KJFK":
            return KJFK
        return super().airport(icao)


@pytest.fixture(scope="module")
def navdata():
    base = build_navdata(NavDataSources(littlenavmap_db=None, airports_csv=None))
    return _JFKNavData(base.providers)


def _fly(navdata, origin_icao, destination_icao, aircraft, wind, dt=4.0,
         max_hours=26.0):
    origin, destination = navdata.airport(origin_icao), navdata.airport(destination_icao)
    assert origin is not None and destination is not None
    profile = get_profile(aircraft)
    plan = plan_route(origin, destination, profile, navdata,
                      wind_from_deg=wind[0], wind_kt=wind[1])
    runway = plan.departure_runway

    def terrain(position):
        near = distance_nm(position, origin.position)
        far = distance_nm(position, destination.position)
        if far < 5.0:
            return destination.elevation_ft
        if near < 5.0:
            return origin.elevation_ft
        return (origin.elevation_ft * far + destination.elevation_ft * near) / \
            max(near + far, 1e-6)

    sim = MockSim(destination_point(runway.threshold, runway.heading_true_deg, 0.05),
                  runway.heading_true_deg, origin.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                  terrain=terrain, wind_from_deg=wind[0], wind_kt=wind[1])
    adapter, _ = build_adapter(aircraft, sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions())
    pilot.engage()

    worst_overspeed = 0.0
    lowest_agl = 1e9
    for _ in range(int(max_hours * 3600 / dt)):
        status = pilot.update(dt)
        worst_overspeed = max(worst_overspeed, status.ias_kt - profile.vmo_kt)
        if pilot.phase.airborne and pilot.phase is not Phase.LANDING:
            lowest_agl = min(lowest_agl, status.altitude_agl_ft)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    return pilot, sim, plan, worst_overspeed, lowest_agl


#: (label, from, to, aircraft, wind, expected block hours low/high)
ROUTES = [
    ("short haul", "KJFK", "KBOS", "b787-10", (240, 20), 0.4, 1.3),
    ("short haul", "KJFK", "KDCA", "a320neo", (300, 25), 0.4, 1.3),
    ("medium haul", "KJFK", "KLAX", "b787-10", (270, 60), 4.0, 7.0),
    ("medium haul", "KJFK", "KDEN", "a330-900", (280, 45), 2.5, 5.0),
    ("long haul", "KJFK", "EGLL", "b787-10", (270, 90), 4.5, 8.0),
    ("long haul", "KJFK", "OMDB", "a380-800", (300, 55), 9.0, 15.0),
    ("long haul", "KJFK", "RJTT", "a350-1000", (280, 70), 10.0, 17.0),
]


@pytest.mark.parametrize("label,origin,destination,aircraft,wind,low,high", ROUTES)
def test_a_real_route_flies_and_lands(navdata, label, origin, destination,
                                      aircraft, wind, low, high):
    pilot, sim, plan, overspeed, lowest_agl = _fly(navdata, origin, destination,
                                                   aircraft, wind)
    assert pilot.phase is Phase.COMPLETE, \
        f"{origin}-{destination} ended in {pilot.phase.value}"
    assert distance_nm(sim.state.position, plan.threshold_position) < 3.0
    # Firm but well inside limits. These run at a coarse four-second control
    # rate to keep a long haul quick, and the recorded rate is the last sample
    # before the wheels, so it reads worse than the touchdown actually was.
    assert pilot._touchdown_vs is not None and pilot._touchdown_vs > -400
    assert overspeed <= 5.0, f"exceeded Vmo by {overspeed:.0f} kt"
    assert lowest_agl > 150.0, f"got within {lowest_agl:.0f} ft of the ground"
    assert low <= pilot.elapsed_s / 3600 <= high, \
        f"took {pilot.elapsed_s / 3600:.1f} h"


def test_the_route_is_not_wildly_longer_than_the_direct_track(navdata):
    for _label, origin, destination, aircraft, wind, _low, _high in ROUTES:
        plan = plan_route(navdata.airport(origin), navdata.airport(destination),
                          get_profile(aircraft), navdata,
                          wind_from_deg=wind[0], wind_kt=wind[1])
        direct = distance_nm(plan.origin.position, plan.destination.position)
        assert plan.total_distance_nm < direct * 1.6 + 60, \
            f"{origin}-{destination}: {plan.total_distance_nm:.0f} nm for {direct:.0f} nm"


def test_kennedy_arrivals_use_a_runway_with_an_ils(navdata):
    """Kennedy has four. Picking one of them is what makes an autoland possible."""
    for origin, wind in (("EGLL", (270, 85)), ("KLAX", (280, 50))):
        plan = plan_route(navdata.airport(origin), KJFK, get_profile("b787-9"),
                          navdata, wind_from_deg=wind[0], wind_kt=wind[1])
        assert plan.arrival_runway.has_ils, \
            f"chose {plan.arrival_runway.ident}, which has no ILS"


def test_the_runway_chosen_at_kennedy_follows_the_wind(navdata):
    from aipilot.navdata.base import select_runway

    assert select_runway(KJFK, 310, 25).ident.startswith("31")
    assert select_runway(KJFK, 130, 25).ident.startswith("13")
    assert select_runway(KJFK, 40, 25).ident.startswith("04")
    assert select_runway(KJFK, 210, 25).ident.startswith("22")


@pytest.mark.parametrize("origin,aircraft", [("EGLL", "b787-9"), ("KLAX", "a350-900")])
def test_an_inbound_to_kennedy_lands_on_the_ils(navdata, origin, aircraft):
    pilot, sim, plan, overspeed, _agl = _fly(navdata, origin, "KJFK", aircraft,
                                             (270, 85))
    assert pilot.phase is Phase.COMPLETE
    assert plan.arrival_runway.has_ils
    assert distance_nm(sim.state.position, plan.threshold_position) < 3.0
    assert overspeed <= 5.0
