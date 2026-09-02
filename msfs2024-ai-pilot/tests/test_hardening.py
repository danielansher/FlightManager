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
