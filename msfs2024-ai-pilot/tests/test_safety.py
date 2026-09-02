"""The protections, each written against a failure that actually happened.

Every test here corresponds to a real flight that went wrong in the simulator:
takeoff thrust applied at a gate, a descent flown at four hundred and fifty
knots because the autothrottle never took the levers, and a short sector into a
valley airport that ended on a hillside.
"""

import pytest

from aipilot.aircraft.registry import build_adapter
from aipilot.autopilot.controller import AIPilot, PilotOptions
from aipilot.autopilot.phases import Phase
from aipilot.geo import LatLon, destination_point, distance_nm
from aipilot.navdata.base import Airport, Runway
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.route.profile import build_vertical_profile
from aipilot.sim.mock import MockAircraftModel, MockSim

# Los Angeles and Burbank, which is the pair that produced the crash: sixteen
# miles apart, with high ground between the two and around Burbank.
KLAX = Airport("KLAX", "Los Angeles", LatLon(33.942536, -118.408075), 125.0,
               runways=(Runway("07L", LatLon(33.9357, -118.4189), 69.0, 12923, 125.0),
                        Runway("25R", LatLon(33.9484, -118.3789), 249.0, 12923, 125.0)))
KBUR = Airport("KBUR", "Burbank", LatLon(34.200658, -118.358585), 778.0,
               runways=(Runway("15", LatLon(34.2073, -118.3660), 151.0, 6886, 778.0),
                        Runway("08", LatLon(34.1968, -118.3711), 79.0, 5802, 778.0)))


def _sim_at(position, heading, elevation=125.0, terrain=None, profile=None):
    profile = profile or get_profile("b787-10")
    return MockSim(position, heading, elevation,
                   model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                   terrain=terrain)


def _pilot(plan, sim, options=None, aircraft="b787-10"):
    profile = get_profile(aircraft)
    adapter, _ = build_adapter(aircraft, sim)
    return AIPilot(sim, adapter, profile, plan, options or PilotOptions())


# --- Not taking off from the apron ------------------------------------------
def test_it_does_not_apply_takeoff_thrust_at_the_gate():
    """The one that drove into a terminal building."""
    plan = plan_route(KLAX, KBUR, get_profile("b787-10"), None)
    gate = destination_point(KLAX.position, 300.0, 0.7)     # somewhere on the apron
    sim = _sim_at(gate, 15.0)
    pilot = _pilot(plan, sim)
    pilot.engage()
    for _ in range(200):
        pilot.update(1.0)

    assert pilot.phase is Phase.PREFLIGHT, "must not leave preflight off a runway"
    assert sim.throttle_pct < 50.0, "opened the thrust levers on the apron"
    assert sim.state.ias_kt < 5.0, "started rolling on the apron"
    assert sim.state.parking_brake or sim.state.ias_kt < 5.0
    assert any("not lined up on a runway" in e.message.lower() for e in pilot.log)
    assert any("does not taxi" in e.message.lower() for e in pilot.log)


def test_it_takes_off_once_you_line_up():
    """Having waited, it should notice the moment the aeroplane is on a runway."""
    plan = plan_route(KLAX, KBUR, get_profile("b787-10"), None)
    runway = KLAX.runway("07L")
    gate = destination_point(KLAX.position, 300.0, 0.7)
    sim = _sim_at(gate, 15.0)
    pilot = _pilot(plan, sim)
    pilot.engage()
    for _ in range(60):
        pilot.update(1.0)
    assert pilot.phase is Phase.PREFLIGHT

    # Taxi out and line up.
    lined_up = destination_point(runway.threshold, runway.heading_true_deg, 0.1)
    sim.state.lat, sim.state.lon = lined_up.lat, lined_up.lon
    sim.state.heading_true_deg = runway.heading_true_deg
    sim.target_heading = runway.heading_true_deg
    for _ in range(30):
        pilot.update(1.0)

    assert pilot.phase in (Phase.TAKEOFF, Phase.CLIMB)
    assert sim.throttle_pct > 90.0, "should have opened the levers once lined up"


def test_it_adopts_the_runway_you_actually_line_up_on():
    """The plan picks a runway from the wind; the ground controller does not."""
    plan = plan_route(KLAX, KBUR, get_profile("b787-10"), None,
                      departure_runway="07L")
    assert plan.departure_runway.ident == "07L"
    other = KLAX.runway("25R")
    lined_up = destination_point(other.threshold, other.heading_true_deg, 0.1)
    sim = _sim_at(lined_up, other.heading_true_deg)
    pilot = _pilot(plan, sim)
    pilot.engage()
    for _ in range(20):
        pilot.update(1.0)
    assert pilot.plan.departure_runway.ident == "25R"
    assert any("25R" in e.message for e in pilot.log)


def test_the_check_can_be_turned_off():
    plan = plan_route(KLAX, KBUR, get_profile("b787-10"), None)
    gate = destination_point(KLAX.position, 300.0, 0.7)
    sim = _sim_at(gate, 15.0)
    pilot = _pilot(plan, sim, PilotOptions(require_runway=False))
    pilot.engage()
    for _ in range(20):
        pilot.update(1.0)
    assert pilot.phase is not Phase.PREFLIGHT


# --- Speed ------------------------------------------------------------------
def test_the_speed_does_not_run_away_when_the_autothrottle_does_not_hold():
    """The 450 kt descent: the levers stayed where takeoff left them."""
    profile = get_profile("b787-10")
    plan = plan_route(KLAX, KBUR, profile, None)
    sim = MockSim(LatLon(34.0, -118.4), 90.0, 0.0,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                  start_airborne_at_ft=10000.0)
    pilot = _pilot(plan, sim, PilotOptions(start_airborne=True))
    pilot.engage()

    worst = 0.0
    throttled_back = False
    for _ in range(600):
        # The aeroplane's autothrottle is armed but does nothing, which is the
        # case that produced the overspeed.
        sim.state.ap_autothrottle = False
        pilot.update(1.0)
        worst = max(worst, sim.state.ias_kt)
        if sim.throttle_pct < 60.0:
            throttled_back = True
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break

    assert worst <= profile.vmo_kt + 15, f"reached {worst:.0f} kt"
    assert throttled_back, "never took the thrust levers back"


def test_an_overspeed_closes_the_levers_and_says_so():
    profile = get_profile("b787-10")
    plan = plan_route(KLAX, KBUR, profile, None)
    sim = MockSim(LatLon(34.0, -118.4), 90.0, 0.0,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                  start_airborne_at_ft=12000.0)
    pilot = _pilot(plan, sim, PilotOptions(start_airborne=True))
    pilot.engage()
    for _ in range(20):
        pilot.update(1.0)
    sim.state.ias_kt = profile.vmo_kt + 40      # something has gone very wrong
    pilot.update(1.0)
    assert sim.throttle_pct < 5.0
    assert any("overspeed" in e.message.lower() for e in pilot.log)


def test_it_leaves_a_working_autothrottle_alone():
    """When the aeroplane is holding the speed, do not fight it."""
    profile = get_profile("b787-10")
    plan = plan_route(KLAX, KBUR, profile, None)
    sim = MockSim(LatLon(34.0, -118.4), 90.0, 0.0,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                  start_airborne_at_ft=10000.0)
    pilot = _pilot(plan, sim, PilotOptions(start_airborne=True))
    pilot.engage()
    for _ in range(120):
        sim.state.ap_autothrottle = True
        sim.state.ias_kt = pilot.status.target_speed or 250.0
        pilot.update(1.0)
    throttle_events = [v for e, v in sim.events_sent if e == "THROTTLE_SET"]
    assert len(throttle_events) <= 3, "kept moving the levers unnecessarily"


# --- Terrain ----------------------------------------------------------------
def test_it_refuses_to_descend_into_rising_ground():
    """The Burbank case: a short sector towards an airport in a valley."""
    profile = get_profile("b787-10")
    plan = plan_route(KLAX, KBUR, profile, None)

    ridge_centre = LatLon(34.13, -118.38)

    def terrain(position):
        """Flat at each airport's own elevation, with a ridge between them."""
        to_lax = distance_nm(position, KLAX.position)
        to_bur = distance_nm(position, KBUR.position)
        total = max(to_lax + to_bur, 1e-6)
        base = (KLAX.elevation_ft * to_bur + KBUR.elevation_ft * to_lax) / total
        d = distance_nm(position, ridge_centre)
        if d > 4.0:
            return base
        return base + 3000.0 * (1.0 - d / 4.0)

    sim = MockSim(LatLon(34.02, -118.40), 20.0, 200.0,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                  terrain=terrain, start_airborne_at_ft=5000.0)
    pilot = _pilot(plan, sim, PilotOptions(start_airborne=True))
    pilot.engage()

    lowest_agl = 1e9
    for _ in range(900):
        pilot.update(1.0)
        if pilot.phase in (Phase.LANDING, Phase.ROLLOUT, Phase.COMPLETE):
            break
        if pilot.phase.airborne:
            lowest_agl = min(lowest_agl, sim.state.altitude_agl_ft)

    assert lowest_agl > 200.0, f"got within {lowest_agl:.0f} ft of the ground"
    assert any("terrain" in e.message.lower() for e in pilot.log)


def test_terrain_protection_never_limits_a_climb():
    """Regression: the floor was applied to every command, so a climb through
    it became a climb *to* it and the aeroplane levelled at 1,500 ft."""
    profile = get_profile("b787-10")
    plan = plan_route(KLAX, KBUR, profile, None)
    runway = KLAX.runway("07L")
    lined_up = destination_point(runway.threshold, runway.heading_true_deg, 0.1)
    sim = _sim_at(lined_up, runway.heading_true_deg, terrain=lambda _p: 100.0)
    pilot = _pilot(plan, sim)
    pilot.engage()
    for _ in range(900):
        pilot.update(1.0)
        if pilot.phase is Phase.CRUISE:
            break
    assert pilot.phase is Phase.CRUISE, "never reached the cruise level"
    assert sim.state.altitude_ft > 2500.0


# --- Short sectors ----------------------------------------------------------
def test_a_short_sector_gets_a_sensible_cruise_level():
    """Sixteen miles is not a flight level nineteen trip."""
    plan = plan_route(KLAX, KBUR, get_profile("b787-10"), None)
    assert plan.cruise_altitude_ft < 8000, \
        f"planned {plan.cruise_altitude_ft:.0f} ft for a 16 nm sector"
    assert plan.cruise_altitude_ft >= KBUR.elevation_ft + 1500
    assert any("too short to climb high" in w for w in plan.warnings)


def test_a_short_sector_does_not_fly_a_route_six_times_its_length():
    plan = plan_route(KLAX, KBUR, get_profile("b787-10"), None)
    direct = distance_nm(KLAX.position, KBUR.position)
    assert plan.total_distance_nm < direct * 5, \
        f"{plan.total_distance_nm:.0f} nm route for a {direct:.0f} nm trip"
    furthest = max(distance_nm(KLAX.position, leg.position) for leg in plan.legs)
    assert furthest < 40, f"a fix {furthest:.0f} nm from the departure airport"


def test_top_of_descent_is_inside_the_route_on_a_short_sector():
    profile = get_profile("b787-10")
    plan = plan_route(KLAX, KBUR, profile, None)
    vertical = build_vertical_profile(plan.cruise_altitude_ft,
                                      KBUR.elevation_ft, profile)
    assert vertical.top_of_descent_nm < plan.total_distance_nm, \
        "top of descent is behind the departure airport"


def test_a_short_sector_flies_and_lands():
    profile = get_profile("b787-10")
    plan = plan_route(KLAX, KBUR, profile, None)
    runway = plan.departure_runway
    lined_up = destination_point(runway.threshold, runway.heading_true_deg, 0.1)
    def terrain(position):
        """Sloping between the two, flat within five miles of either."""
        to_lax = distance_nm(position, KLAX.position)
        to_bur = distance_nm(position, KBUR.position)
        if to_bur < 5.0:
            return KBUR.elevation_ft
        if to_lax < 5.0:
            return KLAX.elevation_ft
        total = max(to_lax + to_bur, 1e-6)
        return (KLAX.elevation_ft * to_bur + KBUR.elevation_ft * to_lax) / total

    sim = _sim_at(lined_up, runway.heading_true_deg, terrain=terrain)
    pilot = _pilot(plan, sim)
    pilot.engage()
    for _ in range(int(2 * 3600)):
        pilot.update(1.0)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    assert pilot.phase is Phase.COMPLETE
    assert distance_nm(sim.state.position, plan.threshold_position) < 3.0
