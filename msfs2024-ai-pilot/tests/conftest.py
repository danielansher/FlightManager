"""Shared fixtures: a nav-data database with real runways and ILS, and a
helper that flies a complete flight in the mock simulator."""

from __future__ import annotations

import os
import sqlite3
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from aipilot.aircraft.registry import build_adapter
from aipilot.autopilot.controller import AIPilot, PilotOptions
from aipilot.autopilot.phases import Phase
from aipilot.geo import distance_nm
from aipilot.navdata.base import ChainedNavData
from aipilot.navdata.littlenavmap import LittleNavmapProvider
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.sim.mock import MockAircraftModel, MockSim

#: Two airports with real runway geometry and working ILS, so the approach and
#: autoland paths are exercised against something other than a synthetic field.
FIXTURE_AIRPORTS = [
    # (id, icao, name, lat, lon, elevation, magvar)
    (1, "EGLL", "London Heathrow", 51.4706, -0.461941, 83.0, -0.5),
    (2, "EGCC", "Manchester", 53.353744, -2.274950, 257.0, -1.5),
]

FIXTURE_RUNWAYS = [
    # (runway_id, airport_id, primary_end, secondary_end, length, width, surface)
    (100, 1, 10, 11, 12799.0, 164.0, "ASPHALT"),
    (101, 1, 12, 13, 12008.0, 164.0, "ASPHALT"),
    (200, 2, 20, 21, 10000.0, 200.0, "ASPHALT"),
]

FIXTURE_RUNWAY_ENDS = [
    # (id, name, heading, lat, lon, altitude, offset, ils_ident)
    (10, "09L", 89.68, 51.4775, -0.48286, 79.0, 0.0, "ILL"),
    (11, "27R", 269.68, 51.4750, -0.43385, 78.0, 0.0, "IRR"),
    (12, "09R", 89.68, 51.4646, -0.48286, 77.0, 0.0, "IAA"),
    (13, "27L", 269.68, 51.4622, -0.43385, 76.0, 0.0, "IBB"),
    (20, "05L", 52.0, 53.3450, -2.2990, 254.0, 0.0, "IMC"),
    (21, "23R", 232.0, 53.3625, -2.2510, 257.0, 0.0, None),
]

FIXTURE_ILS = [
    (1, "ILL", 109500, 89.68, 3.0),
    (2, "IRR", 110300, 269.68, 3.0),
    (3, "IAA", 110700, 89.68, 3.0),
    (4, "IBB", 109500, 269.68, 3.0),
    (5, "IMC", 111550, 52.0, 3.0),
]

FIXTURE_WAYPOINTS = [
    (1, "OCK", 51.3050, -0.4472),
    (2, "MID", 51.0533, -0.6250),
    (3, "HON", 52.0217, -1.6469),
]


@pytest.fixture(scope="session")
def navdb(tmp_path_factory) -> str:
    """A Little-Navmap-shaped database, built once for the whole session."""
    path = str(tmp_path_factory.mktemp("navdata") / "little_navmap_msfs24.sqlite")
    conn = sqlite3.connect(path)
    conn.executescript(
        """
        CREATE TABLE airport(airport_id INTEGER PRIMARY KEY, ident TEXT, name TEXT,
                             laty REAL, lonx REAL, altitude REAL, mag_var REAL);
        CREATE TABLE runway(runway_id INTEGER PRIMARY KEY, airport_id INT,
                            primary_end_id INT, secondary_end_id INT,
                            length REAL, width REAL, surface TEXT);
        CREATE TABLE runway_end(runway_end_id INTEGER PRIMARY KEY, name TEXT,
                                heading REAL, laty REAL, lonx REAL, altitude REAL,
                                offset_threshold REAL, ils_ident TEXT);
        CREATE TABLE ils(ils_id INTEGER PRIMARY KEY, ident TEXT, frequency INT,
                         loc_heading REAL, gs_pitch REAL);
        CREATE TABLE waypoint(waypoint_id INTEGER PRIMARY KEY, ident TEXT,
                              laty REAL, lonx REAL);
        """
    )
    conn.executemany("INSERT INTO airport VALUES(?,?,?,?,?,?,?)", FIXTURE_AIRPORTS)
    conn.executemany("INSERT INTO runway VALUES(?,?,?,?,?,?,?)", FIXTURE_RUNWAYS)
    conn.executemany("INSERT INTO runway_end VALUES(?,?,?,?,?,?,?,?)", FIXTURE_RUNWAY_ENDS)
    conn.executemany("INSERT INTO ils VALUES(?,?,?,?,?)", FIXTURE_ILS)
    conn.executemany("INSERT INTO waypoint VALUES(?,?,?,?)", FIXTURE_WAYPOINTS)
    conn.commit()
    conn.close()
    return path


@pytest.fixture
def navdata(navdb):
    provider = LittleNavmapProvider(navdb)
    yield ChainedNavData([provider])
    provider.close()


@pytest.fixture
def bundled_navdata():
    """The bundled sample, i.e. what someone gets with no data installed."""
    from aipilot.navdata.resolve import build_navdata, NavDataSources

    return build_navdata(NavDataSources(littlenavmap_db=None, airports_csv=None))


#: How long after a waypoint sequences before cross-track means anything again.
SETTLE_AFTER_TURN_S = 120.0


class FlightResult:
    """What happened, in the terms the assertions care about.

    Cross-track error is recorded in two separate places on purpose. Enroute it
    measures how well the aeroplane holds the planned track, and should be a
    fraction of a mile. Through the arrival it measures nothing useful: with no
    published procedure to fly, the approach is joined via a base leg or a full
    circuit, and being two miles off the centreline halfway round a deliberate
    ninety degree turn is the manoeuvre working, not failing. What matters
    there is where the aeroplane ends up -- so that is recorded separately, at
    the stabilisation gate, where an approach is either lined up or is not.

    Enroute samples are also taken only while *established* on a leg. The
    instant a waypoint sequences, cross-track is suddenly measured against the
    next leg, which the aeroplane has not started turning onto yet -- so the
    first sample after a sequence reads as several miles of error that does not
    exist and has not been flown. Skipping the turn measures tracking rather
    than measuring the geometry of waypoint sequencing.
    """

    def __init__(self, pilot, sim, plan, elapsed_s, cruise_xtk,
                 gate_xtk_nm=None, aborted_reason=""):
        self.pilot = pilot
        self.sim = sim
        self.plan = plan
        self.elapsed_s = elapsed_s
        self.cruise_xtk = list(cruise_xtk)
        self.gate_xtk_nm = gate_xtk_nm
        self.aborted_reason = aborted_reason

    @property
    def max_xtk_nm(self) -> float:
        """Worst cross-track anywhere in the cruise, overshoot included."""
        return max(self.cruise_xtk, default=0.0)

    @property
    def settled_xtk_nm(self) -> float:
        """Worst cross-track over the back half of the established cruise."""
        if len(self.cruise_xtk) < 4:
            return max(self.cruise_xtk, default=0.0)
        return max(self.cruise_xtk[len(self.cruise_xtk) // 2:])

    @property
    def phase(self) -> Phase:
        return self.pilot.phase

    @property
    def completed(self) -> bool:
        return self.pilot.phase is Phase.COMPLETE

    @property
    def touchdown_fpm(self):
        return self.pilot._touchdown_vs

    @property
    def stop_distance_from_threshold_nm(self) -> float:
        return distance_nm(self.sim.state.position, self.plan.threshold_position)

    def messages(self) -> list[str]:
        return [event.message for event in self.pilot.log]

    def said(self, fragment: str) -> bool:
        return any(fragment.lower() in m.lower() for m in self.messages())


def fly_flight(navdata, origin_icao: str, destination_icao: str,
               aircraft: str = "b787-10", wind_from_deg: float = 0.0,
               wind_kt: float = 0.0, options: PilotOptions | None = None,
               max_hours: float = 26.0, dt: float = 2.0,
               departure_runway: str | None = None,
               arrival_runway: str | None = None) -> FlightResult:
    """Plan and fly a complete flight in the mock simulator."""
    origin = navdata.airport(origin_icao)
    destination = navdata.airport(destination_icao)
    assert origin is not None, f"unknown airport {origin_icao}"
    assert destination is not None, f"unknown airport {destination_icao}"
    profile = get_profile(aircraft)
    assert profile is not None

    plan = plan_route(origin, destination, profile, navdata,
                      departure_runway=departure_runway, arrival_runway=arrival_runway,
                      wind_from_deg=wind_from_deg, wind_kt=wind_kt)
    runway = plan.departure_runway
    assert runway is not None

    def terrain(position):
        near = distance_nm(position, origin.position)
        far = distance_nm(position, destination.position)
        total = near + far
        if total < 1e-6:
            return origin.elevation_ft
        return (origin.elevation_ft * far + destination.elevation_ft * near) / total

    sim = MockSim(
        runway.threshold, runway.heading_true_deg, origin.elevation_ft,
        model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
        terrain=terrain, wind_from_deg=wind_from_deg, wind_kt=wind_kt,
    )
    adapter, _ = build_adapter(aircraft, sim)
    pilot = AIPilot(sim, adapter, profile, plan, options or PilotOptions())
    pilot.engage()

    cruise_xtk: list[float] = []
    gate_xtk = None
    last_index = -1
    established_after = 0.0
    steps = int(max_hours * 3600 / dt)
    for _ in range(steps):
        status = pilot.update(dt)
        if status.active_index != last_index:
            last_index = status.active_index
            established_after = pilot.elapsed_s + SETTLE_AFTER_TURN_S
        if pilot.phase is Phase.CRUISE and pilot.elapsed_s >= established_after:
            cruise_xtk.append(abs(status.cross_track_nm))
        if gate_xtk is None and pilot.phase in (Phase.APPROACH, Phase.LANDING) \
                and 0 < status.altitude_agl_ft <= 500:
            gate_xtk = abs(status.cross_track_nm)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    return FlightResult(pilot, sim, plan, pilot.elapsed_s, cruise_xtk, gate_xtk)


@pytest.fixture
def fly_flight_fn():
    """The flight helper, as a fixture (conftest is not an importable module)."""
    return fly_flight
