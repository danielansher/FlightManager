"""Geodesy: the foundation everything else stands on."""

import math

import pytest

from aipilot.geo import (
    LatLon,
    along_track_nm,
    cross_track_nm,
    destination_point,
    distance_nm,
    ground_speed_kt,
    initial_bearing_deg,
    interpolate_great_circle,
    normalize_deg,
    signed_diff_deg,
    turn_anticipation_nm,
    turn_radius_nm,
    wind_correction_angle_deg,
)

EGLL = LatLon(51.4706, -0.461941)
KJFK = LatLon(40.639751, -73.778925)
YSSY = LatLon(-33.946111, 151.177222)
RJTT = LatLon(35.552258, 139.779694)
KSFO = LatLon(37.618972, -122.374889)


def test_known_great_circle_distances():
    # Published great-circle distances, to within a few miles.
    assert distance_nm(EGLL, KJFK) == pytest.approx(2990, abs=15)
    assert distance_nm(EGLL, YSSY) == pytest.approx(9190, abs=40)
    assert distance_nm(RJTT, KSFO) == pytest.approx(4460, abs=25)


def test_distance_is_symmetric_and_zero_at_a_point():
    assert distance_nm(EGLL, KJFK) == pytest.approx(distance_nm(KJFK, EGLL))
    assert distance_nm(EGLL, EGLL) == pytest.approx(0.0, abs=1e-9)


def test_north_atlantic_initial_course_is_north_of_west():
    # The great circle to New York leaves London heading north of due west.
    assert initial_bearing_deg(EGLL, KJFK) == pytest.approx(288, abs=2)


def test_destination_point_round_trips():
    for bearing in (0, 45, 90, 180, 271, 359):
        for distance in (1, 50, 500, 3000):
            end = destination_point(EGLL, bearing, distance)
            assert distance_nm(EGLL, end) == pytest.approx(distance, rel=1e-6)
            assert signed_diff_deg(initial_bearing_deg(EGLL, end), bearing) == \
                pytest.approx(0.0, abs=1e-6)


def test_cross_track_sign_and_magnitude():
    """Positive is right of track -- the convention the controller relies on."""
    mid = interpolate_great_circle(EGLL, KJFK, 0.5)
    local_course = initial_bearing_deg(mid, KJFK)
    right = destination_point(mid, local_course + 90, 10.0)
    left = destination_point(mid, local_course - 90, 10.0)
    assert cross_track_nm(right, EGLL, KJFK) == pytest.approx(10.0, abs=0.05)
    assert cross_track_nm(left, EGLL, KJFK) == pytest.approx(-10.0, abs=0.05)
    assert cross_track_nm(mid, EGLL, KJFK) == pytest.approx(0.0, abs=1e-6)


def test_along_track_progresses_and_goes_negative_before_the_start():
    total = distance_nm(EGLL, KJFK)
    for fraction in (0.1, 0.25, 0.5, 0.9):
        point = interpolate_great_circle(EGLL, KJFK, fraction)
        assert along_track_nm(point, EGLL, KJFK) == pytest.approx(total * fraction, rel=0.001)
    behind = destination_point(EGLL, initial_bearing_deg(EGLL, KJFK) + 180, 25.0)
    assert along_track_nm(behind, EGLL, KJFK) == pytest.approx(-25.0, abs=0.1)


def test_signed_diff_wraps_the_short_way():
    assert signed_diff_deg(10, 350) == pytest.approx(20)
    assert signed_diff_deg(350, 10) == pytest.approx(-20)
    # An exact reversal resolves to -180 (the range is [-180, 180)).
    assert abs(signed_diff_deg(180, 0)) == pytest.approx(180)
    assert normalize_deg(-10) == pytest.approx(350)


def test_turn_radius_matches_the_standard_formula():
    # A 25 degree bank at 480 kt TAS is a little over seven miles of radius.
    assert turn_radius_nm(480, 25) == pytest.approx(7.2, abs=0.2)
    # Turn radius grows with the square of speed.
    assert turn_radius_nm(400, 25) / turn_radius_nm(200, 25) == pytest.approx(4.0, rel=0.01)


def test_turn_anticipation_is_the_arc_tangent_distance():
    radius = turn_radius_nm(450, 25)
    assert turn_anticipation_nm(450, 90, 25) == pytest.approx(radius, rel=0.01)
    assert turn_anticipation_nm(450, 0, 25) == 0.0
    # A sharper turn needs more room.
    assert turn_anticipation_nm(450, 120, 25) > turn_anticipation_nm(450, 60, 25)


def test_wind_correction_crabs_into_the_wind():
    # Wind from the south while tracking east: crab right, into it.
    wca = wind_correction_angle_deg(90, 450, 180, 100)
    assert wca == pytest.approx(math.degrees(math.asin(100 / 450)), abs=0.01)
    assert wca > 0
    # Mirror image from the north.
    assert wind_correction_angle_deg(90, 450, 0, 100) == pytest.approx(-wca, abs=0.01)
    # A pure headwind or tailwind needs no correction.
    assert wind_correction_angle_deg(90, 450, 90, 100) == pytest.approx(0.0, abs=1e-9)


def test_wind_stronger_than_the_aeroplane_does_not_explode():
    assert wind_correction_angle_deg(90, 100, 180, 400) == 0.0


def test_ground_speed_reflects_head_and_tailwinds():
    assert ground_speed_kt(90, 450, 90, 100) == pytest.approx(350, abs=0.5)   # headwind
    assert ground_speed_kt(90, 450, 270, 100) == pytest.approx(550, abs=0.5)  # tailwind
