"""Nav data providers, runway selection and route construction."""

import pytest

from aipilot.geo import LatLon, distance_nm, initial_bearing_deg, signed_diff_deg
from aipilot.navdata.base import select_runway
from aipilot.navdata.littlenavmap import LittleNavmapProvider
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import ROLLOUT_EXTENSION_NM, plan_route, resolve_route_string


def test_littlenavmap_reads_airports_runways_and_ils(navdata):
    egll = navdata.airport("EGLL")
    assert egll is not None
    assert egll.name == "London Heathrow"
    assert egll.elevation_ft == pytest.approx(83)
    assert {r.ident for r in egll.runways} == {"09L", "27R", "09R", "27L"}
    r27r = egll.runway("27R")
    assert r27r.has_ils
    # Little Navmap stores whole kilohertz; 110300 is 110.30 MHz.
    assert r27r.ils_freq_mhz == pytest.approx(110.30)
    assert r27r.glideslope_deg == pytest.approx(3.0)


def test_lookup_is_case_insensitive_and_unknown_returns_none(navdata):
    assert navdata.airport("egll") is not None
    assert navdata.airport("ZZZZ") is None


def test_database_is_opened_read_only(navdb):
    import sqlite3

    provider = LittleNavmapProvider(navdb)
    assert provider.airport("EGLL") is not None
    conn = provider._connect()
    with pytest.raises(sqlite3.Error):
        conn.execute("INSERT INTO airport VALUES(99,'XXXX','x',0,0,0,0)")
    provider.close()


def test_runway_without_ils_is_reported_as_such(navdata):
    egcc = navdata.airport("EGCC")
    assert egcc.runway("05L").has_ils
    assert not egcc.runway("23R").has_ils


def test_runway_selection_favours_the_headwind(navdata):
    egll = navdata.airport("EGLL")
    assert select_runway(egll, wind_from_deg=270, wind_kt=25).ident.startswith("27")
    assert select_runway(egll, wind_from_deg=90, wind_kt=25).ident.startswith("09")


def test_runway_selection_avoids_a_tailwind(navdata):
    """A light wind from behind must not win on runway length alone."""
    egll = navdata.airport("EGLL")
    chosen = select_runway(egll, wind_from_deg=95, wind_kt=8)
    assert chosen.ident.startswith("09")


def test_centreline_points_lie_upwind_of_the_threshold(navdata):
    runway = navdata.airport("EGLL").runway("27R")
    for distance in (5, 10, 18):
        point = runway.point_on_centreline(distance)
        assert distance_nm(point, runway.threshold) == pytest.approx(distance, rel=1e-6)
        # The approach course from that point back to the threshold is the
        # runway course.
        course = initial_bearing_deg(point, runway.threshold)
        assert signed_diff_deg(course, runway.heading_true_deg) == pytest.approx(0, abs=0.5)


def test_plan_has_the_expected_shape(navdata):
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata, wind_from_deg=270, wind_kt=15)
    phases = [leg.phase for leg in plan.legs]
    assert phases[0] == "takeoff"
    assert phases[1] == "departure"
    assert phases[-1] == "rollout"
    assert "landing" in phases
    assert plan.legs[plan.threshold_index].phase == "landing"


def test_route_continues_past_the_threshold(navdata):
    """Lateral guidance needs a fix ahead of it during the flare."""
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata)
    threshold = plan.legs[plan.threshold_index]
    last = plan.legs[-1]
    assert last is not threshold
    assert distance_nm(threshold.position, last.position) == \
        pytest.approx(ROLLOUT_EXTENSION_NM, rel=0.01)
    runway = plan.arrival_runway
    assert signed_diff_deg(initial_bearing_deg(threshold.position, last.position),
                           runway.heading_true_deg) == pytest.approx(0, abs=1)


def test_approach_altitudes_decrease_towards_the_runway(navdata):
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata)
    approach = [leg for leg in plan.legs if leg.phase in ("approach", "final", "landing")]
    altitudes = [leg.altitude_ft for leg in approach]
    assert altitudes == sorted(altitudes, reverse=True)
    assert altitudes[-1] == pytest.approx(plan.arrival_runway.elevation_ft + 50, abs=1)


def test_long_route_is_split_into_named_segments(bundled_navdata):
    plan = plan_route(bundled_navdata.airport("EGLL"), bundled_navdata.airport("YSSY"),
                      get_profile("a350-1000"), bundled_navdata)
    enroute = [leg for leg in plan.legs if leg.phase == "enroute"]
    assert len(enroute) > 20
    assert len({leg.ident for leg in enroute}) == len(enroute), "segment names must be unique"
    # Consecutive segments are all shorter than the split threshold.
    for a, b in zip(plan.legs, plan.legs[1:]):
        assert distance_nm(a.position, b.position) < 300


def test_requested_runway_is_honoured(navdata):
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata,
                      departure_runway="09L", arrival_runway="05L")
    assert plan.departure_runway.ident == "09L"
    assert plan.arrival_runway.ident == "05L"


def test_unknown_runway_falls_back_with_a_warning(navdata):
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata, departure_runway="99X")
    assert plan.departure_runway is not None
    assert any("99X" in w for w in plan.warnings)


def test_missing_ils_produces_a_warning(navdata):
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata, arrival_runway="23R")
    assert any("no ils" in w.lower() for w in plan.warnings)


def test_synthetic_runways_are_flagged(bundled_navdata):
    plan = plan_route(bundled_navdata.airport("EGLL"), bundled_navdata.airport("EGCC"),
                      get_profile("b787-10"), bundled_navdata)
    assert any("no runway data" in w for w in plan.warnings)


def test_route_string_resolves_known_fixes_and_skips_the_rest(navdata):
    start = navdata.airport("EGLL").position
    found, skipped = resolve_route_string("DCT OCK UL9 MID/N0450F350 NOTAFIX", navdata, start)
    assert [w.ident for w in found] == ["OCK", "MID"]
    assert "UL9" in skipped and "NOTAFIX" in skipped


def test_route_fixes_pointing_backwards_are_dropped(navdata):
    """A fix identifier resolved to the wrong continent must not be followed."""
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata, route="MID HON")
    idents = [leg.ident for leg in plan.legs]
    # HON is on the way north; MID is in the opposite direction and is dropped.
    assert "HON" in idents
    assert "MID" not in idents


def test_plan_distance_accounting_is_consistent(navdata):
    plan = plan_route(navdata.airport("EGLL"), navdata.airport("EGCC"),
                      get_profile("b787-10"), navdata)
    total = plan.total_distance_nm
    assert plan.distance_to_end_nm(plan.legs[0].position, 0) == pytest.approx(total, abs=0.01)
    # Halfway along, the remaining distance is smaller but positive.
    middle = len(plan.legs) // 2
    remaining = plan.distance_to_end_nm(plan.legs[middle].position, middle)
    assert 0 < remaining < total
