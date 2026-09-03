"""Edge cases found by auditing, each reproduced before it was fixed.

Every test here stands for a defect that was real: a crash, a silent wrong
answer, or two threads flying one aeroplane. They are grouped by the file
they belong to rather than by symptom, because that is how they will be
read when one of them fails.
"""

from __future__ import annotations

import json
import os
import sqlite3
import tempfile
import threading
import time

import pytest

from aipilot.geo import LatLon
from aipilot.navdata.base import GroundLayout, TaxiPath
from aipilot.navdata.littlenavmap import LittleNavmapProvider
from aipilot.navdata.ourairports import OurAirportsProvider
from aipilot.route.taxi import WELD_TOLERANCE_NM, GroundNetwork, build_network


# --- The taxiway graph -------------------------------------------------------
def test_a_junction_in_a_crowded_grid_cell_is_not_split():
    """The grid index kept one node per cell, so a second junction in the
    same cell evicted the first -- which could then never be found again.
    Two segments meeting at bitwise identical coordinates became
    unconnected, and the route across them came back empty for no reason
    the user could see."""
    scale = WELD_TOLERANCE_NM / 60.0
    a = LatLon((1000 - 0.45) * scale, (1000 - 0.45) * scale)
    b = LatLon((1000 + 0.45) * scale, (1000 + 0.45) * scale)
    network = GroundNetwork(GroundLayout("X", (
        TaxiPath(a, LatLon(a.lat + 0.004, a.lon)),
        TaxiPath(b, LatLon(b.lat, b.lon + 0.004)),
        TaxiPath(LatLon(a.lat, a.lon), LatLon(a.lat - 0.004, a.lon)),
    )))

    at_a = [n for n in network.nodes
            if n.position.lat == a.lat and n.position.lon == a.lon]
    assert len(at_a) == 1, "the same point became two unconnected junctions"
    assert network.route(LatLon(a.lat + 0.004, a.lon),
                         LatLon(a.lat - 0.004, a.lon)), \
        "no route across a junction that is a single point"


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_a_non_finite_coordinate_does_not_stop_the_build(bad):
    """float("nan") passes every try/except float(...) on the way in from a
    scenery database, so it has to be stopped where it would detonate."""
    network = build_network(GroundLayout("X", (
        TaxiPath(LatLon(bad, 0.0), LatLon(0.0, 0.005)),
        TaxiPath(LatLon(0.0, 0.0), LatLon(0.0, 0.005)),
        TaxiPath(LatLon(0.0, 0.005), LatLon(0.001, 0.005)),
    )))
    assert network is not None and network.usable


def test_one_bad_row_does_not_generate_a_hundred_thousand_nodes():
    """A zero coordinate -- the classic scenery-export slip -- used to be
    chopped into tens of thousands of nodes, and every route request scans
    all of them."""
    layout = GroundLayout("KJFK", (
        TaxiPath(LatLon(40.64, -73.78), LatLon(0.0, 0.0)),          # to nowhere
        TaxiPath(LatLon(40.64, -73.78), LatLon(40.641, -73.78)),
        TaxiPath(LatLon(40.641, -73.78), LatLon(40.642, -73.78)),
    ))
    network = build_network(layout)
    assert network is not None
    assert len(network.nodes) < 100, f"{len(network.nodes)} nodes from three rows"


# --- Little Navmap -----------------------------------------------------------
@pytest.fixture
def scenery_db(tmp_path):
    """A database with the kinds of row third-party scenery really contains."""
    path = str(tmp_path / "little_navmap_msfs24.sqlite")
    conn = sqlite3.connect(path)
    conn.execute("CREATE TABLE airport (airport_id INTEGER PRIMARY KEY, "
                 "ident TEXT, name TEXT, laty REAL, lonx REAL, altitude REAL, "
                 "mag_var REAL)")
    conn.executemany("INSERT INTO airport VALUES (?,?,?,?,?,?,?)", [
        (1, "KJFK", "Kennedy", None, None, 13.0, -13.0),      # no position
        (2, "EGLL", "Heathrow", "N51.47", -0.46, 83.0, 0.0),  # text position
        (3, "EGCC", "Manchester", 53.35, -2.27, 257.0, 0.0),  # fine
        (4, "EGKK", "Gatwick", float("nan"), -0.19, 202.0, 0.0),
    ])
    conn.commit()
    conn.close()
    return path


@pytest.mark.parametrize("icao", ["KJFK", "EGLL", "EGKK"])
def test_an_unusable_position_is_not_an_exception(scenery_db, icao):
    """One bad row used to raise out of the middle of an airport lookup and
    take the flight with it."""
    provider = LittleNavmapProvider(scenery_db)
    try:
        assert provider.airport(icao) is None
    finally:
        provider.close()


def test_a_good_airport_still_reads(scenery_db):
    provider = LittleNavmapProvider(scenery_db)
    try:
        airport = provider.airport("EGCC")
        assert airport is not None and airport.icao == "EGCC"
    finally:
        provider.close()


# --- OurAirports CSVs --------------------------------------------------------
def _csv(tmp_path, name: str, data: bytes) -> str:
    path = tmp_path / name
    path.write_bytes(data)
    return str(path)


def test_a_file_re_saved_out_of_a_spreadsheet_still_reads(tmp_path):
    """Latin-1 and a byte order mark: what happens when someone opens the
    download in Excel and saves it again."""
    header = "id,ident,type,name,latitude_deg,longitude_deg,elevation_ft,gps_code\n"
    latin = _csv(tmp_path, "a.csv",
                 header.encode() + "1,LFPG,large,A\xe9roport,49.0,2.5,392,LFPG\n"
                 .encode("latin-1"))
    assert OurAirportsProvider(latin, None).airport("LFPG") is not None

    bom = _csv(tmp_path, "b.csv",
               b"\xef\xbb\xbf" + header.encode()
               + b"1,EGCC,large,Manchester,53.3,-2.2,257,EGCC\n")
    assert OurAirportsProvider(bom, None).airport("EGCC") is not None


def test_a_truncated_download_degrades_instead_of_raising(tmp_path):
    path = _csv(tmp_path, "c.csv",
                b'id,ident,name,latitude_deg,longitude_deg\n'
                b'1,EGLL,"Heathrow,51.4,-0.4\n2,"' + b"x" * 200000)
    provider = OurAirportsProvider(path, None)
    assert provider.airport("EGLL") is None
    assert "field larger" in provider.describe(), \
        "it must say why it has no airports, not just have none"


def test_a_failed_read_is_not_cached_as_a_successful_empty_one(tmp_path):
    """Setting the loaded flag first turned one loud failure into a silent
    'not in the navigation data' for every airport thereafter."""
    path = _csv(tmp_path, "d.csv",
                b"id,ident,name,latitude_deg,longitude_deg\n"
                b"1,EGKK,Gatwick,51.1,-0.19\n")
    provider = OurAirportsProvider(path, None)
    assert provider.airport("EGKK") is not None
    assert provider._error is None


# --- The browser control panel ----------------------------------------------
@pytest.fixture
def session():
    from aipilot.ui.server import FlightSession

    made = FlightSession()
    yield made
    made.disengage()
    if made.navdata is not None:
        made.navdata.close()


def _plan(session, **overrides):
    request = {"origin": "EGLL", "destination": "EGCC", "aircraft": "b787-10",
               "no_metar": True}
    request.update(overrides)
    return session.build_plan(request)


def test_the_panel_can_plan_a_flight(session):
    """There was no test for this at all, which is how a refactor that broke
    it outright went through a green suite."""
    reply = _plan(session)
    assert reply["ok"] and reply["origin"]["icao"] == "EGLL"
    assert reply["legs"] and reply["runway_notes"]


def test_a_failed_plan_does_not_leave_the_database_open(session):
    for _ in range(5):
        with pytest.raises(ValueError):
            _plan(session, destination="ZZZZ")
    # And the session still works afterwards.
    assert _plan(session)["ok"]


def test_two_engage_requests_do_not_start_two_flights(session):
    """Everything between the "already running" check and the control thread
    starting is slow -- connecting to the simulator, building both taxiway
    networks -- and the check used to release its lock before any of it. Two
    clicks got two threads commanding one aeroplane at four hertz, with only
    one of them reachable to stop."""
    from aipilot.ui.server import FlightSession

    _plan(session)
    original = FlightSession._make_sim
    session._make_sim = lambda *a, **k: (time.sleep(0.3),
                                         original(session, *a, **k))[1]
    outcomes: list[str] = []

    def engage():
        try:
            session.engage({"sim": "mock", "speed": 400})
            outcomes.append("started")
        except ValueError as exc:
            outcomes.append(f"refused: {exc}")

    threads = [threading.Thread(target=engage) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert outcomes.count("started") == 1, outcomes
    live = [t for t in threading.enumerate()
            if t.name == "aipilot" and t.is_alive()]
    assert len(live) == 1, f"{len(live)} control threads are flying the aeroplane"


def test_a_failed_engage_does_not_jam_the_panel(session):
    """Left set by a simulator that would not connect, the guard refused
    every later attempt and needed a restart."""
    from aipilot.ui.server import FlightSession

    _plan(session)
    session._make_sim = lambda *a, **k: (_ for _ in ()).throw(
        RuntimeError("simulator refused"))
    with pytest.raises(RuntimeError):
        session.engage({"sim": "mock"})

    original = FlightSession._make_sim
    session._make_sim = lambda *a, **k: original(session, *a, **k)
    assert session.engage({"sim": "mock", "speed": 400})["ok"]


def test_the_static_guard_needs_a_separator():
    """A prefix test with no separator lets a sibling directory through."""
    from aipilot.ui import server

    base = server.STATIC_DIR
    escape = os.path.normpath(os.path.join(base, "../" + os.path.basename(base)
                                           + "_private/secrets.txt"))
    assert not escape.startswith(base + os.sep)
    assert escape.startswith(base), \
        "this is the case a bare startswith would have allowed"


# --- Guidance ----------------------------------------------------------------
def _airport(icao, lat, lon, elevation, heading, magvar=0.0):
    from aipilot.navdata.base import Airport, Runway

    threshold = LatLon(lat, lon)
    return Airport(icao, icao, threshold, elevation, magvar_deg=magvar, runways=(
        Runway(f"{max(1, int(round(heading / 10))):02d}", threshold, heading,
               10000, elevation, width_ft=150.0),))


@pytest.fixture
def rejoin_pilot():
    """A plan whose departure runway points away from the destination, which
    is what makes the departure leg look attractive from the arrival."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.autopilot.controller import AIPilot, PilotOptions
    from aipilot.perf.profiles import get_profile
    from aipilot.route.planner import plan_route
    from aipilot.sim.mock import MockSim

    origin = _airport("EGLL", 51.4775, -0.48286, 79.0, 89.68)
    destination = _airport("EGCC", 53.3450, -2.2990, 254.0, 52.0)
    profile = get_profile("b787-10")
    plan = plan_route(origin, destination, profile, None)
    sim = MockSim(origin.position, 0.0, origin.elevation_ft)
    adapter, _ = build_adapter("b787-10", sim)
    return AIPilot(sim, adapter, profile, plan, PilotOptions()), plan, destination


def test_engaging_near_the_destination_never_flies_back_to_the_departure(rejoin_pilot):
    """A leg not yet *reached* was scored by raw distance to its start, so an
    aeroplane in the arrival sector -- where every arrival leg is behind it --
    found the departure leg nearest and turned round. A two hundred and
    seventy mile excursion, on one position in five."""
    import random

    from aipilot.geo import destination_point, distance_nm

    pilot, plan, destination = rejoin_pilot
    random.seed(7)
    wrong = 0
    for _ in range(400):
        position = destination_point(destination.position,
                                     random.uniform(0.0, 360.0),
                                     random.uniform(1.0, 60.0))
        index = pilot._closest_useful_leg(position)
        to_fly = distance_nm(position, plan[index].position) + \
            plan.distance_from_leg_to_end_nm(index)
        if to_fly > distance_nm(position, destination.position) * 2 + 60:
            wrong += 1
    assert wrong == 0, f"{wrong} of 400 positions would fly the wrong way"


def test_engaging_past_the_destination_aims_at_the_end_of_the_route(rejoin_pilot):
    from aipilot.geo import destination_point, initial_bearing_deg

    pilot, plan, destination = rejoin_pilot
    course = initial_bearing_deg(plan.origin.position, destination.position)
    beyond = destination_point(destination.position, course, 30.0)
    assert pilot._closest_useful_leg(beyond) >= len(plan) - 2


def test_the_descent_selector_is_never_above_the_aeroplane():
    """On a short sector the cruise is capped for the distance while the
    approach fixes are still built on an unclipped three degree slope, so the
    next constraint sat thousands of feet above an aeroplane being told to
    descend. On a real MCP that is a mode conflict, and the aeroplane either
    climbs or refuses to leave its level."""
    from aipilot.autopilot.phases import Phase
    from aipilot.autopilot.vertical import VerticalGuidance
    from aipilot.geo import destination_point
    from aipilot.perf.profiles import get_profile
    from aipilot.route.planner import plan_route
    from aipilot.route.profile import build_vertical_profile

    origin = _airport("AAAA", 51.0, 0.0, 100.0, 90.0)
    far = destination_point(LatLon(51.0, 0.0), 90.0, 12.0)
    destination = _airport("BBBB", far.lat, far.lon, 100.0, 270.0)
    profile = get_profile("b787-10")
    plan = plan_route(origin, destination, profile, None)
    guidance = VerticalGuidance(
        plan, profile,
        build_vertical_profile(plan.cruise_altitude_ft, 100.0, profile))

    for altitude in (1500.0, 2000.0, 3000.0):
        for index in range(1, len(plan)):
            command = guidance.update(Phase.DESCENT, altitude_ft=altitude,
                                      distance_to_go_nm=14.9,
                                      ground_speed_kt=250.0, active_index=index)
            assert command.altitude_ft <= altitude + 1e-6, (
                f"selector {command.altitude_ft:.0f} ft with the aeroplane at "
                f"{altitude:.0f} ft, leg {index}")


def test_cruise_levels_follow_the_magnetic_course():
    """The semicircular rule is defined on magnetic track. Fed a true course,
    any route within one local variation of north or south gets the other
    hemisphere's levels -- the thousand feet opposite-direction traffic is
    using."""
    from aipilot.perf.profiles import get_profile, select_cruise_altitude
    from aipilot.route.planner import magnetic_course

    profile = get_profile("b787-10")
    denver = _airport("KDEN", 39.86, -104.67, 5431.0, 170.0, magvar=8.0)
    true_course = 185.0
    assert magnetic_course(true_course, denver) == pytest.approx(177.0)
    with_variation = select_cruise_altitude(
        2000, magnetic_course(true_course, denver), profile)
    without = select_cruise_altitude(2000, true_course, profile)
    assert with_variation != without
    assert int(with_variation / 1000) % 2 == 1, "eastbound wants an odd level"


# --- The debug report on a damaged trace -------------------------------------
@pytest.mark.parametrize("records", [
    [{"t": "header", "at": 0}, {"t": "sample", "phase": "cruise", "agl": 30000}],
    [{"t": "header", "at": 0}, {"t": "sample", "at": 1, "phase": "cruise"},
     {"t": "totals", "at": 1, "commands": {"event:X": 999},
      "spans": {"event:X": [0]}}],
    [{"t": "header", "at": 0}, {"t": "sample", "at": 1, "phase": "cruise"},
     {"t": "totals", "at": 1, "commands": {"event:X": 9}, "spans": {"event:X": "?"}}],
    [{"t": "header", "at": 0}, {"t": "sample", "at": 1, "phase": "cruise"},
     {"t": "totals", "at": 1, "commands": ["a", "b"]}],
    [{"t": "header", "at": "x"}, {"t": "sample", "at": "y", "phase": "cruise"}],
    [{"t": "header", "at": 0, "vmo_kt": "fast"},
     {"t": "sample", "at": 1, "phase": "cruise", "ias": 280}],
    [{"t": "header", "at": 0},
     {"t": "sample", "at": 1, "phase": "climb", "agl": None, "on_ground": False}],
])
def test_the_report_reads_a_damaged_trace(tmp_path, records):
    """The traces worth reading come from flights that went wrong, and those
    are the traces most likely to be damaged."""
    from aipilot.debug import analyse, format_report

    path = tmp_path / "t.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in records))
    text = format_report(analyse(str(path)))
    assert "AI Pilot flight trace" in text


def test_a_missing_runway_heading_is_measured_not_guessed(tmp_path):
    """The fallback derived the heading from the designator -- "27L" -> 270 --
    and stored it as a true heading. Runway numbers are magnetic, so at
    somewhere with real variation the entire approach was built several
    degrees off the actual pavement. Where both thresholds are known the true
    heading can simply be measured."""
    airports = tmp_path / "airports.csv"
    airports.write_text(
        "id,ident,type,name,latitude_deg,longitude_deg,elevation_ft,gps_code\n"
        "1,PANC,large,Anchorage,61.1743,-149.9962,152,PANC\n")
    # Anchorage 15/33: the designator says 150 degrees, the pavement runs
    # nearer 173 true, because the variation there is about 15 degrees east.
    runways = tmp_path / "runways.csv"
    runways.write_text(
        "id,airport_ref,airport_ident,length_ft,width_ft,surface,closed,"
        "le_ident,le_latitude_deg,le_longitude_deg,le_heading_degT,"
        "he_ident,he_latitude_deg,he_longitude_deg,he_heading_degT\n"
        "1,1,PANC,10600,150,ASP,0,"
        "15,61.1889,-149.9803,,"
        "33,61.1601,-149.9720,\n")

    airport = OurAirportsProvider(str(airports), str(runways)).airport("PANC")
    assert airport is not None
    by_ident = {r.ident: r for r in airport.runways}
    assert set(by_ident) == {"15", "33"}
    # Measured between the thresholds, not 150 from the designator.
    assert 165.0 < by_ident["15"].heading_true_deg < 185.0, \
        f"{by_ident['15'].heading_true_deg:.0f} looks like the designator, not the pavement"
    reciprocal = abs(by_ident["15"].heading_true_deg
                     - by_ident["33"].heading_true_deg)
    assert abs(reciprocal - 180.0) < 1.0, "the two ends must be reciprocal"


# --- Runway limits -----------------------------------------------------------
def _field(icao, lat, lon, runways):
    from aipilot.navdata.base import Airport, Runway

    return Airport(icao, icao, LatLon(lat, lon), 0.0, runways=tuple(
        Runway(ident, LatLon(lat, lon), heading, length, 0.0, width_ft=150.0)
        for ident, heading, length in runways))


@pytest.fixture
def two_fields():
    big = _field("KJFK", 40.64, -73.78, [("04L", 31.0, 12079), ("22R", 211.0, 12079)])
    short = _field("KTEB", 40.85, -74.06, [("06", 58.0, 6013), ("24", 238.0, 6013)])
    return big, short


def _warned(plan, phrase):
    return any(phrase in w for w in plan.warnings)


def test_a_runway_too_short_for_the_aeroplane_is_flagged(two_fields):
    """Nothing checked this at all: the minimum was one global number, so an
    A380 could be planned into a six-thousand-foot runway in silence."""
    from aipilot.perf.profiles import get_profile
    from aipilot.route.planner import plan_route

    big, short = two_fields
    plan = plan_route(big, short, get_profile("a380-800"), None)
    assert _warned(plan, "wants at least 9800 ft")

    # The same field is unremarkable for something that fits.
    fine = plan_route(big, short, get_profile("a320neo"), None)
    assert not _warned(fine, "wants at least")


def test_a_crosswind_beyond_the_demonstrated_figure_is_flagged(two_fields):
    from aipilot.perf.profiles import get_profile
    from aipilot.route.planner import AirportWind, plan_route

    big, short = two_fields
    plan = plan_route(short, big, get_profile("b787-10"), None,
                      arrival_wind=AirportWind(121.0, 40.0, "the METAR"))
    assert _warned(plan, "kt of crosswind")
    assert _warned(plan, "33 kt demonstrated")


def test_a_tailwind_beyond_the_limit_is_flagged(two_fields):
    from aipilot.perf.profiles import get_profile
    from aipilot.route.planner import AirportWind, plan_route

    big, short = two_fields
    plan = plan_route(short, big, get_profile("b787-10"), None,
                      arrival_runway="22R",
                      arrival_wind=AirportWind(31.0, 20.0, "the METAR"))
    assert _warned(plan, "kt of tailwind")


def test_a_runway_it_can_use_beats_one_it_cannot(two_fields):
    """Length and limits rank above the wind: a slight headwind is not worth
    a runway the aeroplane does not fit on."""
    from aipilot.navdata.base import select_runway

    mixed = _field("XXXX", 50.0, 0.0, [
        ("09", 90.0, 12000),      # long, slight tailwind
        ("27", 270.0, 5000),      # short, into wind
    ])
    chosen = select_runway(mixed, wind_from_deg=270.0, wind_kt=8.0,
                           min_length_ft=9000.0, max_tailwind_kt=10.0)
    assert chosen.ident == "09", "took the short runway for eight knots of wind"


def test_limits_never_leave_a_flight_with_no_runway():
    """An airport where every runway is short, or the wind is across all of
    them, still has to produce an answer -- there is nowhere else to go."""
    from aipilot.navdata.base import select_runway

    tiny = _field("YYYY", 50.0, 0.0, [("09", 90.0, 3000), ("27", 270.0, 3000)])
    chosen = select_runway(tiny, wind_from_deg=180.0, wind_kt=45.0,
                           min_length_ft=9000.0, max_crosswind_kt=33.0,
                           max_tailwind_kt=10.0)
    assert chosen is not None


# --- Reported from real flights ----------------------------------------------
def test_the_takeoff_roll_does_not_slam_the_rudder():
    """A 787 at Kennedy swung hard left the moment the thrust came up.

    The "lined up" check accepted two hundred feet off the centreline and the
    roll then tried to correct that with a gain of twenty-five per nautical
    mile -- near full rudder, on the first cycle, at takeoff power.
    """
    from aipilot.autopilot.controller import (
        TAKEOFF_STEER_LIMIT,
        TAKEOFF_STEER_RATE_PER_S,
        TAKEOFF_STEER_XTK_GAIN_PER_NM,
    )

    hundred_feet = 100.0 / 6076.11548556
    assert abs(-hundred_feet * TAKEOFF_STEER_XTK_GAIN_PER_NM) < 0.15, \
        "a hundred feet off should be a nudge, not a swerve"
    assert TAKEOFF_STEER_LIMIT <= 0.4
    assert TAKEOFF_STEER_RATE_PER_S <= 0.5, "it can still snap over in one cycle"


def test_being_well_off_the_centreline_is_not_lined_up(navdata):
    """Generosity about what counts as lined up is not a kindness when the
    next thing that happens is takeoff power."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.autopilot.controller import AIPilot, PilotOptions
    from aipilot.geo import destination_point, normalize_deg
    from aipilot.perf.profiles import get_profile
    from aipilot.route.planner import plan_route
    from aipilot.sim.mock import MockSim

    profile = get_profile("b787-10")
    origin, destination = navdata.airport("EGLL"), navdata.airport("EGCC")
    plan = plan_route(origin, destination, profile, navdata)
    runway = plan.departure_runway
    sim = MockSim(runway.threshold, runway.heading_true_deg, origin.elevation_ft)
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(taxi=False))

    beside = destination_point(runway.threshold,
                               normalize_deg(runway.heading_true_deg + 90.0),
                               200.0 / 6076.11548556)
    sim.state.lat, sim.state.lon = beside.lat, beside.lon
    assert pilot._runway_under_aircraft(sim.state) is None, \
        "two hundred feet off the centreline was accepted as lined up"

    sim.state.lat, sim.state.lon = runway.threshold.lat, runway.threshold.lon
    assert pilot._runway_under_aircraft(sim.state) is not None


def test_the_tug_heading_is_sent_once_per_heading():
    """A real pushback sent it 330 times in 84 seconds for a value that never
    changed."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.sim.mock import MockSim

    sim = MockSim(LatLon(51.0, 0.0))
    adapter, _ = build_adapter("b787-10", sim)
    for _ in range(100):
        adapter.set_tug_heading(271.0)
    adapter.set_tug_heading(95.0)
    sent = [e for e, _v in sim.events_sent if e == "KEY_TUG_HEADING"]
    assert len(sent) == 2, f"sent {len(sent)} times for two headings"


def test_thrust_going_nowhere_releases_the_brakes_anyway():
    """A 787 sat at a Kennedy gate with the thrust up for two and a half
    minutes. The parking brake was on, and the release is guarded on what the
    aeroplane reports -- which an add-on running its own hydraulics may not
    report at all, so it was never sent."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.sim.mock import MockSim

    sim = MockSim(LatLon(51.0, 0.0))
    adapter, _ = build_adapter("b787-10", sim)
    sim.state.parking_brake = False            # what the aeroplane claims
    adapter.set_parking_brake(False, sim.state)
    assert not [e for e, _v in sim.events_sent if e == "PARKING_BRAKES"], \
        "the guard should suppress this, which is the whole problem"

    adapter.release_brakes_hard()
    # What matters is the brake being off afterwards, not which event carried
    # the request. Asserting the event name let a toggle pass for a release.
    assert not sim.state.parking_brake
    assert sim.wheel_brakes == 0.0


def test_the_escape_hatch_does_not_set_the_brake_it_meant_to_release():
    """Flown out of a Kennedy gate this watchdog fired every eight seconds, and
    each firing flipped the parking brake instead of releasing it: the aeroplane
    spent half of every cycle held by the brake its own rescue had just applied.
    PARKING_BRAKES is a toggle, and the escape hatch sent it blind -- so on any
    aeroplane already released it did precisely the wrong thing."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.sim.mock import MockSim

    sim = MockSim(LatLon(51.0, 0.0))
    adapter, _ = build_adapter("b787-10", sim)
    sim.state.parking_brake = False

    for firing in range(4):
        adapter.release_brakes_hard()
        assert not sim.state.parking_brake, \
            f"the parking brake came on at firing {firing + 1}"
        assert sim.wheel_brakes == 0.0


def test_releasing_the_wheel_brakes_actually_releases_them():
    """A brake axis runs -16383 to +16383, so its centre is half braking.
    Scaling 0..1 onto 0..16383 meant "no brakes" was sent as zero -- half on.
    The 787 pushed back at Kennedy would not roll afterwards at 65% N1, and the
    trace showed the release going out exactly as intended."""
    from aipilot.aircraft.registry import build_adapter
    from aipilot.sim.mock import MockSim

    sim = MockSim(LatLon(51.0, 0.0))
    adapter, _ = build_adapter("b787-10", sim)

    adapter.set_wheel_brakes(1.0)
    assert sim.wheel_brakes == 1.0, "full braking should be full"

    adapter.set_wheel_brakes(0.0)
    assert sim.wheel_brakes == 0.0, \
        f"asked for no brakes and the aeroplane kept {sim.wheel_brakes:.0%}"


def test_the_same_database_is_not_opened_twice(tmp_path, monkeypatch):
    """On Windows %APPDATA% is ~/AppData/Roaming, so both search roots named
    the same file and every lookup ran against it twice."""
    from aipilot.navdata import littlenavmap

    roaming = tmp_path / "AppData" / "Roaming" / "ABarthel" / "little_navmap_db"
    roaming.mkdir(parents=True)
    (roaming / "little_navmap_msfs.sqlite").write_bytes(b"")

    monkeypatch.setenv("APPDATA", str(tmp_path / "AppData" / "Roaming"))
    monkeypatch.setattr(os.path, "expanduser", lambda _p: str(tmp_path))
    found = littlenavmap.default_database_paths("2020")
    assert len(found) == 1, f"the same database was found {len(found)} times"


def test_the_replay_shows_how_the_aeroplane_moved(tmp_path):
    """The trace always held heading, track and every rudder command; there
    was simply no way to look at them together."""
    from aipilot.debug import analyse, format_track, read_records

    records = [
        {"t": "header", "at": 0},
        {"t": "sample", "at": 0.0, "phase": "takeoff", "pos": [40.64, -73.78],
         "hdg": 226.0, "want_hdg": 220.0, "trk": 226.0, "gs": 12.0, "ias": 12.0,
         "alt": 13.0},
        {"t": "command", "at": 0.1, "phase": "takeoff", "kind": "event",
         "name": "RUDDER_SET", "value": -9830},
        {"t": "sample", "at": 0.5, "phase": "takeoff", "pos": [40.641, -73.781],
         "hdg": 219.0, "want_hdg": 220.0, "trk": 219.0, "gs": 30.0, "ias": 30.0,
         "alt": 13.0},
    ]
    path = tmp_path / "t.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in records))
    text = format_track(analyse(str(path)), read_records(str(path)))
    assert "-0.60" in text, "the rudder that was sent is not in the replay"
    assert "226" in text and "220" in text
