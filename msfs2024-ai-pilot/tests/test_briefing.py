"""Real-world sources for the runway choice: METAR, SimBrief, and the sim.

The runway a flight uses is decided almost entirely by the wind, so these
tests are really about one thing: does the plan end up on the runway a crew
would actually be given, given what each source says?
"""

from __future__ import annotations

import json

import pytest

from aipilot.briefing import resolve_winds, wind_from_sim
from aipilot.metar import CALM, MetarError, parse_metar
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import AirportWind, plan_route
from aipilot.simbrief import SimBriefError, parse_plan


# --- Reading a METAR ---------------------------------------------------------
@pytest.mark.parametrize("raw, station, direction, speed", [
    ("METAR KJFK 021751Z 24012KT 10SM FEW045 26/18 A2996", "KJFK", 240.0, 12.0),
    ("SPECI EGLL 021750Z 09015KT 9999 FEW030 18/11 Q1015", "EGLL", 90.0, 15.0),
    ("UUEE 021800Z 09004MPS 9999 SCT030 05/01 Q1012", "UUEE", 90.0, 7.8),
    # 360 is north, which normalises to 0 -- the same direction, and the
    # form the runway headings are in.
    ("LFPG 021730Z 36020G35KT 8000 -RA BKN012 14/13 Q1009", "LFPG", 0.0, 20.0),
])
def test_metar_wind_is_read(raw, station, direction, speed):
    report = parse_metar(raw)
    assert report is not None
    assert report.station == station
    assert report.wind.from_deg == pytest.approx(direction)
    assert report.wind.speed_kt == pytest.approx(speed, abs=0.1)


@pytest.mark.parametrize("raw", [
    "KLAX 021753Z 00000KT 10SM CLR 22/14 A2990",       # explicitly calm
    "EGLL 021750Z AUTO VRB03KT 9999 NCD 21/12 Q1018",  # no usable direction
    "LFPG 021730Z /////KT 8000 -RA BKN012 14/13 Q1009",  # not reported
])
def test_a_wind_with_no_direction_is_calm(raw):
    """There is nothing to steer by, so the planner must fall back rather
    than take a placeholder direction as gospel."""
    report = parse_metar(raw)
    assert report is not None
    assert report.wind.calm
    assert report.wind.from_deg is None


@pytest.mark.parametrize("raw", ["", "   ", "No data found", "<html>error</html>"])
def test_rubbish_is_not_a_metar(raw):
    assert parse_metar(raw) is None


def test_a_gust_leans_the_choice_but_does_not_decide_it():
    """Half the gust is added, the way a crew reads a gusting wind."""
    steady = parse_metar("KJFK 021751Z 24012KT 10SM CLR 26/18 A2996").wind
    gusting = parse_metar("KJFK 021751Z 24012G30KT 10SM CLR 26/18 A2996").wind
    assert steady.planning_speed_kt == pytest.approx(12.0)
    assert gusting.planning_speed_kt == pytest.approx(21.0)
    assert gusting.speed_kt == pytest.approx(12.0)   # the steady wind is untouched


# --- Reading a SimBrief plan -------------------------------------------------
def _ofp(**overrides):
    data = {
        "fetch": {"status": "Success"},
        "general": {"icao_airline": "BAW", "flight_number": "117",
                    "route": "ROBUC3 BAF Q436 EBONY/N0489F370 NERTU DCT",
                    "initial_altitude": "37000"},
        "origin": {"icao_code": "KJFK", "plan_rwy": "04L"},
        "destination": {"icao_code": "EGLL", "plan_rwy": "27R"},
        "aircraft": {"icaocode": "B77W"},
    }
    for key, value in overrides.items():
        data[key] = value
    return json.dumps(data)


def test_simbrief_plan_is_read():
    plan = parse_plan(_ofp())
    assert (plan.origin, plan.destination) == ("KJFK", "EGLL")
    assert (plan.departure_runway, plan.arrival_runway) == ("04L", "27R")
    assert plan.cruise_altitude_ft == 37000
    assert plan.aircraft_icao == "B77W"
    assert plan.callsign == "BAW117"


def test_airways_and_level_changes_are_not_mistaken_for_fixes():
    """An airway identifier that happens to match some unrelated navaid
    would bend the route across the world, so it is dropped by shape rather
    than left to the navigation data to reject."""
    fixes = parse_plan(_ofp()).route.split()
    assert fixes == ["BAF", "EBONY", "NERTU"]
    for rejected in ("ROBUC3", "Q436", "N0489F370", "DCT"):
        assert rejected not in fixes


@pytest.mark.parametrize("written, expected", [
    ("4L", "04L"), ("RW27R", "27R"), ("RWY22", "22"), ("09", "09"), ("31C", "31C"),
])
def test_runways_are_normalised_to_the_airport_data_form(written, expected):
    plan = parse_plan(_ofp(origin={"icao_code": "KJFK", "plan_rwy": written}))
    assert plan.departure_runway == expected


def test_a_failed_fetch_says_something_useful():
    with pytest.raises(SimBriefError, match="username"):
        parse_plan("<OFPError>Unknown user</OFPError>")
    with pytest.raises(SimBriefError, match="Error: no flight plan"):
        parse_plan(json.dumps({"fetch": {"status": "Error: no flight plan found"}}))


def test_a_plan_with_no_runways_is_still_usable():
    plan = parse_plan(_ofp(origin={"icao_code": "KJFK"},
                           destination={"icao_code": "EGLL"}))
    assert plan.departure_runway is None
    assert plan.notes, "the fallback to the wind should be explained"


# --- Deciding which wind to plan with ----------------------------------------
def test_a_typed_wind_beats_everything():
    winds = resolve_winds("KJFK", "EGLL", typed=(200.0, 25.0),
                          simbrief_metars=("KJFK 021751Z 09020KT", None))
    assert winds.departure.from_deg == 200.0
    assert winds.arrival.from_deg == 200.0


def test_each_end_gets_its_own_wind():
    """A single wind applied to both ends is wrong for anything longer than
    a hop: the wind at the far end decides the landing runway."""
    winds = resolve_winds("KJFK", "EGLL", use_metar=False, simbrief_metars=(
        "KJFK 021751Z 24012KT 10SM CLR 26/18 A2996",
        "EGLL 021750Z 09015KT 9999 FEW030 18/11 Q1015",
    ))
    assert winds.departure.from_deg == 240.0
    assert winds.arrival.from_deg == 90.0


def test_no_network_means_calm_not_a_crash(monkeypatch):
    """A weather service being slow must never stop a flight."""
    def explode(*_args, **_kwargs):
        raise MetarError("no route to host")

    monkeypatch.setattr("aipilot.briefing.fetch_metar", explode)
    said: list[str] = []
    winds = resolve_winds("KJFK", "EGLL", use_metar=True, report=said.append)
    assert winds.departure.speed_kt == 0.0
    assert winds.arrival.speed_kt == 0.0
    assert any("calm" in message for message in said)


def test_the_simulators_own_wind_is_offered_when_it_is_blowing():
    class State:
        wind_from_deg, wind_kt = 310.0, 18.0

    wind = wind_from_sim(State())
    assert wind is not None and wind.from_deg == 310.0

    class Still:
        wind_from_deg, wind_kt = 310.0, 1.0

    assert wind_from_sim(Still()).speed_kt == 0.0


# --- What it all adds up to: the runway on the plan --------------------------
def test_the_wind_at_each_end_picks_that_ends_runway(navdata):
    profile = get_profile("b787-10")
    origin, destination = navdata.airport("EGLL"), navdata.airport("EGCC")

    westerly = plan_route(origin, destination, profile, navdata,
                          departure_wind=AirportWind(270, 20, "the METAR"),
                          arrival_wind=AirportWind(270, 20, "the METAR"))
    easterly = plan_route(origin, destination, profile, navdata,
                          departure_wind=AirportWind(90, 20, "the METAR"),
                          arrival_wind=AirportWind(90, 20, "the METAR"))

    assert westerly.departure_runway.ident.startswith("27")
    assert easterly.departure_runway.ident.startswith("09")
    assert westerly.arrival_runway.ident != easterly.arrival_runway.ident


def test_the_plan_says_why_it_chose_each_runway(navdata):
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata,
                      departure_wind=AirportWind(270, 20, "the METAR"))
    assert len(plan.runway_notes) == 2
    assert "the METAR" in plan.runway_notes[0]
    assert "down the runway" in plan.runway_notes[0]
    # With nothing known, it must not pretend the METAR said calm.
    assert "the METAR" not in plan.runway_notes[1]


def test_a_requested_runway_is_never_overruled_by_the_wind(navdata):
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata,
                      departure_runway="09L",
                      departure_wind=AirportWind(270, 30, "the METAR"))
    assert plan.departure_runway.ident == "09L"
    assert "as asked for" in plan.runway_notes[0]


# --- The command line ---------------------------------------------------------
def _args(**overrides):
    """The parsed arguments for `aipilot plan`, so the wiring is tested the
    way it is actually reached."""
    from aipilot.cli import build_parser

    argv = ["plan"] + list(overrides.pop("argv", []))
    args = build_parser().parse_args(argv)
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def test_simbrief_fills_in_what_you_did_not_type(monkeypatch):
    from aipilot import cli

    monkeypatch.setattr(cli, "_simbrief_plan", lambda *_: parse_plan(_ofp()))
    args = _args(argv=["--simbrief", "someone"])
    cli._apply_simbrief(args, cli._simbrief_plan(args, print))

    assert (args.origin, args.destination) == ("KJFK", "EGLL")
    assert (args.departure_runway, args.arrival_runway) == ("04L", "27R")
    assert args.cruise == 37000
    assert args.route.split() == ["BAF", "EBONY", "NERTU"]


def test_what_you_typed_beats_the_simbrief_plan():
    from aipilot import cli

    args = _args(argv=["KLAX", "KSFO", "--departure-runway", "25R",
                       "--simbrief", "someone"])
    cli._apply_simbrief(args, parse_plan(_ofp()))

    assert (args.origin, args.destination) == ("KLAX", "KSFO")
    assert args.departure_runway == "25R"
    assert args.arrival_runway == "27R"       # not typed, so SimBrief's stands


def test_a_cruise_level_from_simbrief_is_read_as_feet():
    """`--cruise 370` means flight level 370, but SimBrief gives feet. A
    plan at 380 ft instead of FL380 is not a rounding error."""
    from aipilot import cli

    args = _args(argv=["--simbrief", "someone"])
    cli._apply_simbrief(args, parse_plan(_ofp()))
    assert cli._cruise_altitude(args.cruise) == 37000


def test_no_airports_and_no_simbrief_is_explained():
    from aipilot import cli

    with pytest.raises(SystemExit, match="--simbrief"):
        cli._prepare_flight(_args(argv=[]), report=lambda _m: None)


def test_the_simulators_wind_repoints_the_departure(navdata):
    """A METAR is an observation of the real world an hour ago. If the
    simulator is running a preset, or a date in 1997, only it knows which
    way the aeroplane is about to take off."""
    from aipilot import cli

    profile = get_profile("b787-10")
    origin = navdata.airport("EGLL")
    plan = plan_route(origin, navdata.airport("EGCC"), profile, navdata,
                      departure_wind=AirportWind(90, 20, "the METAR"))
    assert plan.departure_runway.ident.startswith("09")

    class Sim:
        def poll(self, _dt):
            class State:
                position = origin.position
                wind_from_deg, wind_kt = 270.0, 25.0
            return State()

    said: list[str] = []
    args = _args(argv=["EGLL", "EGCC"], airborne=False)
    cli._match_departure_to_sim_wind(Sim(), args, plan, profile, said.append)

    assert plan.departure_runway.ident.startswith("27")
    assert plan.legs[0].ident.startswith("RW27")   # the plan really was rebuilt
    assert any("simulator's wind" in message for message in said)


def test_the_simulators_wind_is_ignored_where_it_says_nothing(navdata):
    from aipilot import cli
    from aipilot.geo import LatLon

    profile = get_profile("b787-10")
    origin = navdata.airport("EGLL")

    def fly(position, wind, **overrides):
        plan = plan_route(origin, navdata.airport("EGCC"), profile, navdata,
                          departure_wind=AirportWind(90, 20, "the METAR"))
        before = plan.departure_runway.ident

        class Sim:
            def poll(self, _dt):
                class State:
                    pass
                State.position = position
                State.wind_from_deg, State.wind_kt = wind
                return State()

        cli._match_departure_to_sim_wind(
            Sim(), _args(argv=["EGLL", "EGCC"], airborne=False, **overrides),
            plan, profile, lambda _m: None)
        return before, plan.departure_runway.ident

    # Somewhere else entirely: this wind says nothing about the wind there.
    before, after = fly(LatLon(40.0, -74.0), (270.0, 25.0))
    assert before == after
    # Barely moving: not a reason to change runways.
    before, after = fly(origin.position, (270.0, 2.0))
    assert before == after
    # A runway you asked for is never overruled.
    before, after = fly(origin.position, (270.0, 25.0), departure_runway="09L")
    assert before == after
