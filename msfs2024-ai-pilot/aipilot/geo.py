"""Spherical-earth geodesy and the turn geometry the lateral guidance needs.

A sphere is the right model here. The worst-case great-circle error against
WGS-84 is about 0.3%, which on a 3000 nm leg is ~9 nm of *distance* but only a
fraction of a degree of *bearing* -- and bearing is what actually steers the
aeroplane. Every distance we care about operationally (turn anticipation,
top-of-descent, cross-track) is short enough that the error is invisible.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .units import EARTH_RADIUS_NM


@dataclass(frozen=True)
class LatLon:
    lat: float
    lon: float

    def __str__(self) -> str:  # pragma: no cover - display only
        ns = "N" if self.lat >= 0 else "S"
        ew = "E" if self.lon >= 0 else "W"
        return f"{abs(self.lat):.4f}{ns} {abs(self.lon):.4f}{ew}"


def normalize_deg(deg: float) -> float:
    """Wrap to [0, 360)."""
    return deg % 360.0


def signed_diff_deg(a: float, b: float) -> float:
    """Smallest signed rotation from ``b`` to ``a``, in [-180, 180).

    An exact reversal comes back as -180 rather than +180. Which way an
    aeroplane turns through a half circle is arbitrary, and every caller here
    either takes the absolute value or clamps to a bank limit well short of it.
    """
    return (a - b + 180.0) % 360.0 - 180.0


def distance_nm(p1: LatLon, p2: LatLon) -> float:
    """Great-circle distance via the haversine formula."""
    phi1, phi2 = math.radians(p1.lat), math.radians(p2.lat)
    dphi = phi2 - phi1
    dlam = math.radians(p2.lon - p1.lon)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * EARTH_RADIUS_NM * math.asin(math.sqrt(min(1.0, a)))


def initial_bearing_deg(p1: LatLon, p2: LatLon) -> float:
    """True course at ``p1`` for the great circle to ``p2``."""
    phi1, phi2 = math.radians(p1.lat), math.radians(p2.lat)
    dlam = math.radians(p2.lon - p1.lon)
    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return normalize_deg(math.degrees(math.atan2(y, x)))


def destination_point(origin: LatLon, bearing_deg: float, distance_nm_: float) -> LatLon:
    """Point reached by flying ``distance_nm_`` from ``origin`` on a great circle."""
    delta = distance_nm_ / EARTH_RADIUS_NM
    theta = math.radians(bearing_deg)
    phi1, lam1 = math.radians(origin.lat), math.radians(origin.lon)
    sin_phi2 = math.sin(phi1) * math.cos(delta) + math.cos(phi1) * math.sin(delta) * math.cos(theta)
    phi2 = math.asin(max(-1.0, min(1.0, sin_phi2)))
    y = math.sin(theta) * math.sin(delta) * math.cos(phi1)
    x = math.cos(delta) - math.sin(phi1) * math.sin(phi2)
    lam2 = lam1 + math.atan2(y, x)
    return LatLon(math.degrees(phi2), (math.degrees(lam2) + 540.0) % 360.0 - 180.0)


def cross_track_nm(position: LatLon, leg_start: LatLon, leg_end: LatLon) -> float:
    """Signed distance from the leg centreline.

    Positive means the aeroplane is *right* of the desired track, which is the
    convention the lateral controller expects (a positive error commands a
    left correction).
    """
    d13 = distance_nm(leg_start, position) / EARTH_RADIUS_NM
    theta13 = math.radians(initial_bearing_deg(leg_start, position))
    theta12 = math.radians(initial_bearing_deg(leg_start, leg_end))
    return math.asin(max(-1.0, min(1.0, math.sin(d13) * math.sin(theta13 - theta12)))) * EARTH_RADIUS_NM


def along_track_nm(position: LatLon, leg_start: LatLon, leg_end: LatLon) -> float:
    """Distance from ``leg_start`` to the projection of ``position`` on the leg.

    Negative if the aeroplane has not reached ``leg_start`` yet.
    """
    d13 = distance_nm(leg_start, position) / EARTH_RADIUS_NM
    xtk = cross_track_nm(position, leg_start, leg_end) / EARTH_RADIUS_NM
    cos_ratio = math.cos(d13) / math.cos(xtk)
    return math.acos(max(-1.0, min(1.0, cos_ratio))) * EARTH_RADIUS_NM * (
        1.0 if abs(signed_diff_deg(initial_bearing_deg(leg_start, position),
                                   initial_bearing_deg(leg_start, leg_end))) <= 90.0 else -1.0
    )


def turn_radius_nm(tas_kt: float, bank_deg: float) -> float:
    """Radius of a level turn at ``tas_kt`` held at ``bank_deg`` of bank."""
    if tas_kt <= 0:
        return 0.0
    bank = math.radians(max(1.0, abs(bank_deg)))
    # r = v^2 / (g tan(bank)); done in ft/s and ft, then converted to nm.
    v_fps = tas_kt * 1.68780986
    r_ft = v_fps * v_fps / (32.174 * math.tan(bank))
    return r_ft / 6076.11548556


def turn_anticipation_nm(tas_kt: float, course_change_deg: float, bank_deg: float = 25.0) -> float:
    """How far before a waypoint to start the turn so the arc joins both legs.

    Standard fly-by geometry: the tangent distance of a circular arc of radius
    ``r`` subtending the course change, i.e. ``r * tan(delta / 2)``.
    """
    delta = abs(signed_diff_deg(course_change_deg, 0.0))
    if delta < 1.0:
        return 0.0
    delta = min(delta, 175.0)  # tan blows up as the turn approaches a reversal
    return turn_radius_nm(tas_kt, bank_deg) * math.tan(math.radians(delta / 2.0))


def wind_correction_angle_deg(course_deg: float, tas_kt: float,
                              wind_from_deg: float, wind_kt: float) -> float:
    """Angle to add to a desired *track* to get the *heading* to fly.

    Returns 0 when the wind exceeds true airspeed, since no heading holds the
    track in that case.
    """
    if tas_kt <= 1.0 or wind_kt <= 0.0:
        return 0.0
    # Angle between the desired course and the direction the wind blows *towards*.
    wind_angle = math.radians(wind_from_deg - course_deg)
    sin_wca = wind_kt * math.sin(wind_angle) / tas_kt
    if abs(sin_wca) >= 1.0:
        return 0.0
    return math.degrees(math.asin(sin_wca))


def ground_speed_kt(course_deg: float, tas_kt: float,
                    wind_from_deg: float, wind_kt: float) -> float:
    """Ground speed achieved when tracking ``course_deg`` at ``tas_kt``."""
    wca = math.radians(wind_correction_angle_deg(course_deg, tas_kt, wind_from_deg, wind_kt))
    wind_angle = math.radians(wind_from_deg - course_deg)
    return tas_kt * math.cos(wca) - wind_kt * math.cos(wind_angle)


def interpolate_great_circle(p1: LatLon, p2: LatLon, fraction: float) -> LatLon:
    """Point a given fraction of the way along the great circle from p1 to p2."""
    d = distance_nm(p1, p2)
    if d < 1e-9:
        return p1
    return destination_point(p1, initial_bearing_deg(p1, p2), d * fraction)
