"""Atmosphere, speed conversions and the descent profile."""

import pytest

from aipilot.perf.profiles import (
    AircraftProfile,
    get_profile,
    load_profile_overrides,
    profile_for_icao_type,
    select_cruise_altitude,
)
from aipilot.route.profile import (
    build_vertical_profile,
    climb_speed_target,
    descent_speed_target,
    gradient_ft_per_nm,
)
from aipilot.units import (
    cas_to_tas,
    crossover_altitude_ft,
    isa_temp_c,
    mach_to_tas,
    tas_to_cas,
    tas_to_mach,
)


def test_isa_temperature_and_tropopause():
    assert isa_temp_c(0) == pytest.approx(15.0)
    assert isa_temp_c(36089) == pytest.approx(-56.5, abs=0.2)
    assert isa_temp_c(45000) == pytest.approx(-56.5)   # held above the tropopause


def test_mach_matches_published_true_airspeeds():
    # M0.85 at FL350 is a shade under 490 kt true.
    assert mach_to_tas(0.85, 35000) == pytest.approx(490, abs=4)
    assert mach_to_tas(0.80, 0) == pytest.approx(529, abs=5)
    assert tas_to_mach(mach_to_tas(0.85, 35000), 35000) == pytest.approx(0.85)


def test_cas_and_tas_round_trip_and_diverge_with_altitude():
    for altitude in (0, 10000, 25000, 39000):
        assert tas_to_cas(cas_to_tas(280, altitude), altitude) == pytest.approx(280, rel=1e-9)
    assert cas_to_tas(250, 0) == pytest.approx(250, rel=0.01)
    assert cas_to_tas(250, 35000) > 400


def test_crossover_altitude_is_where_the_two_schedules_meet():
    # The classic 300 kt / M0.85 crossover sits around FL305.
    altitude = crossover_altitude_ft(300, 0.85)
    assert altitude == pytest.approx(30900, abs=800)
    assert tas_to_mach(cas_to_tas(300, altitude), altitude) == pytest.approx(0.85, abs=0.002)


def test_speed_schedules_step_in_the_right_places():
    profile = get_profile("b787-10")
    assert climb_speed_target(5000, profile) == (250.0, False)
    assert climb_speed_target(15000, profile) == (300.0, False)
    assert climb_speed_target(38000, profile) == (0.84, True)
    assert descent_speed_target(38000, profile) == (0.84, True)
    assert descent_speed_target(5000, profile) == (250.0, False)


def test_three_degree_gradient_is_the_rule_of_thumb():
    assert gradient_ft_per_nm(3.0) == pytest.approx(318, abs=2)


def test_top_of_descent_is_near_three_times_the_flight_level():
    profile = get_profile("b787-10")
    vertical = build_vertical_profile(36000, 13, profile)
    # The crew rule of thumb is 3 nm per thousand feet, plus room to slow down.
    assert vertical.top_of_descent_nm == pytest.approx(3 * 360 / 10 + 18, abs=15)
    assert 110 < vertical.top_of_descent_nm < 145


def test_descent_path_descends_monotonically_and_lands_at_the_threshold():
    profile = get_profile("b787-10")
    vertical = build_vertical_profile(36000, 500, profile)
    previous = vertical.target_altitude_at(400)
    for distance in range(399, -1, -1):
        altitude = vertical.target_altitude_at(distance)
        assert altitude <= previous + 1e-6, f"path climbs at {distance} nm"
        previous = altitude
    assert vertical.target_altitude_at(0) == pytest.approx(550, abs=5)
    assert vertical.target_altitude_at(vertical.faf_distance_nm) == \
        pytest.approx(vertical.faf_altitude_ft, rel=0.02)


def test_descent_starts_immediately_at_top_of_descent():
    """The deceleration allowance must shallow the path, not add a level segment."""
    profile = get_profile("b787-10")
    vertical = build_vertical_profile(36000, 13, profile)
    just_inside = vertical.top_of_descent_nm - 2.0
    assert vertical.target_altitude_at(just_inside) < 36000 - 100
    assert 2.2 < vertical.effective_angle_deg < 3.0


def test_required_vertical_speed_is_negative_when_above_the_path():
    profile = get_profile("b787-10")
    vertical = build_vertical_profile(36000, 13, profile)
    on_path = vertical.target_altitude_at(80)
    assert vertical.required_vertical_speed_fpm(80, on_path + 3000, 450) < -1000
    assert vertical.required_vertical_speed_fpm(80, on_path - 3000, 450) > 0


def test_cruise_altitude_follows_the_semicircular_rule():
    profile = get_profile("b787-10")
    for course in (0, 45, 90, 179):
        level = select_cruise_altitude(3000, course, profile)
        assert int(level / 1000) % 2 == 1, f"eastbound should be odd, got {level}"
    for course in (180, 270, 359):
        level = select_cruise_altitude(3000, course, profile)
        assert int(level / 1000) % 2 == 0, f"westbound should be even, got {level}"


def test_short_sectors_cruise_lower_than_long_ones():
    profile = get_profile("b787-10")
    assert select_cruise_altitude(120, 90, profile) < select_cruise_altitude(600, 90, profile)
    assert select_cruise_altitude(600, 90, profile) < select_cruise_altitude(4000, 90, profile)


def test_cruise_altitude_never_exceeds_the_ceiling():
    for key in ("b787-10", "a350-900", "a380-800", "a330-900", "a320neo"):
        profile = get_profile(key)
        assert select_cruise_altitude(6000, 90, profile) < profile.max_altitude_ft


def test_flap_placards_are_monotonic_and_lookup_respects_them():
    for key in ("b787-10", "a350-900", "a380-800", "a330-900", "a320neo"):
        profile = get_profile(key)
        extended = [f for f in profile.flaps if f.index > 0]
        speeds = [f.max_speed_kt for f in sorted(extended, key=lambda f: f.index)]
        assert speeds == sorted(speeds, reverse=True), f"{key} placards not monotonic"
        landing = profile.landing_flaps
        assert landing is not None
        assert profile.final_approach_speed_kt < landing.max_speed_kt
        # Too fast for any flap at all.
        assert profile.flap_for_speed(400) is None
        chosen = profile.flap_for_speed(landing.max_speed_kt)
        assert chosen is not None and chosen.index == landing.index


def test_icao_type_lookup():
    assert profile_for_icao_type("B78X").key == "b787-10"
    assert profile_for_icao_type("A388").key == "a380-800"
    assert profile_for_icao_type("XXXX") is None


def test_profile_overrides_apply(tmp_path):
    import json

    original = get_profile("a380-800").cruise_mach
    path = tmp_path / "overrides.json"
    path.write_text(json.dumps({"a380-800": {"cruise_mach": 0.83}}))
    try:
        assert load_profile_overrides(str(path)) == ["a380-800"]
        assert get_profile("a380-800").cruise_mach == pytest.approx(0.83)
    finally:
        path.write_text(json.dumps({"a380-800": {"cruise_mach": original}}))
        load_profile_overrides(str(path))
