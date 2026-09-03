"""Fly a route in the mock and say what was wrong with it.

The point of a matrix of flights is not that they finish -- it is that every
one of them can be held to the same set of statements, so a route that breaks
one of them stands out without anybody reading a log. Each check here is
something a previous version of this program actually got wrong.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from aipilot.aircraft.registry import build_adapter
from aipilot.autopilot.controller import AIPilot, PilotOptions
from aipilot.autopilot.phases import Phase
from aipilot.geo import (
    LatLon,
    along_track_nm,
    cross_track_nm,
    destination_point,
    distance_nm,
)
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.sim.mock import MockAircraftModel, MockSim
from aipilot.units import FEET_PER_NM

from . import hubs

#: How far down the runway a touchdown may be before it is a long landing, as
#: a fraction of the runway.
TOUCHDOWN_ZONE_FRACTION = 0.40

#: Off the centreline at touchdown. Half a runway width is about a hundred
#: feet, so this is "on the paved surface".
CENTRELINE_LIMIT_FT = 110.0

#: Below this, outside the phases that belong near the ground, is a problem.
MIN_ENROUTE_AGL_FT = 500.0


@dataclass
class Result:
    origin: str
    destination: str
    aircraft: str
    distance_nm: float = 0.0
    route_nm: float = 0.0
    hours: float = 0.0
    phase: str = ""
    touchdown_along_ft: float = 0.0
    touchdown_off_ft: float = 0.0
    stop_along_ft: float = 0.0
    runway_ft: float = 0.0
    worst_overspeed_kt: float = 0.0
    lowest_agl_ft: float = 0.0
    go_arounds: int = 0
    faults: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.faults

    def __str__(self) -> str:                      # pragma: no cover - display
        mark = "ok  " if self.ok else "FAIL"
        return (f"{mark} {self.origin}-{self.destination} {self.aircraft:<10} "
                f"{self.distance_nm:5.0f} nm {self.hours:5.2f} h"
                + ("" if self.ok else "  " + "; ".join(self.faults)))


def _terrain_for(origin, destination, ridge_ft: float = 0.0):
    """Ground under the route: the two fields, joined, with an optional ridge.

    A straight interpolation between two sea-level airports gives terrain that
    can never catch anything out, so the sweep can put a ridge across the
    middle of a sector to make the descent think.
    """
    def terrain(position: LatLon) -> float:
        near = distance_nm(position, origin.position)
        far = distance_nm(position, destination.position)
        total = near + far
        if total < 1e-6:
            return origin.elevation_ft
        if far < 6.0:
            return destination.elevation_ft
        if near < 6.0:
            return origin.elevation_ft
        blended = (origin.elevation_ft * far + destination.elevation_ft * near) / total
        if ridge_ft:
            # A hump centred half way along, a few tens of miles wide.
            middle = abs(near - far) / total          # 0 at the midpoint
            blended += ridge_ft * max(0.0, 1.0 - middle * 6.0)
        return blended

    return terrain


def fly(origin_icao: str, destination_icao: str, aircraft: str = "b787-10",
        wind: tuple[float, float] = (0.0, 0.0), dt: float = 4.0,
        ridge_ft: float = 0.0, max_hours: float = 26.0,
        options: PilotOptions | None = None) -> Result:
    """Plan and fly one route, and check it against every invariant."""
    origin = hubs.airport(origin_icao)
    destination = hubs.airport(destination_icao)
    profile = get_profile(aircraft)
    assert profile is not None

    result = Result(origin_icao, destination_icao, aircraft)
    plan = plan_route(origin, destination, profile, None,
                      wind_from_deg=wind[0], wind_kt=wind[1])
    result.warnings = list(plan.warnings)
    result.distance_nm = distance_nm(origin.position, destination.position)
    result.route_nm = plan.total_distance_nm

    departure = plan.departure_runway
    arrival = plan.arrival_runway
    if departure is None or arrival is None:
        result.faults.append("no runway")
        return result
    result.runway_ft = arrival.length_ft

    sim = MockSim(departure.threshold, departure.heading_true_deg,
                  origin.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                  terrain=_terrain_for(origin, destination, ridge_ft),
                  wind_from_deg=wind[0], wind_kt=wind[1])
    adapter, _ = build_adapter(aircraft, sim)
    pilot = AIPilot(sim, adapter, profile, plan,
                    options or PilotOptions(taxi=False))
    pilot.engage()

    lowest = 1e9
    worst_overspeed = 0.0
    for _ in range(int(max_hours * 3600 / dt)):
        status = pilot.update(dt)
        worst_overspeed = max(worst_overspeed, status.ias_kt - profile.vmo_kt)
        if pilot.phase.airborne and pilot.phase not in (
                Phase.TAKEOFF, Phase.APPROACH, Phase.LANDING):
            lowest = min(lowest, status.altitude_agl_ft)
        for value in (status.altitude_ft, status.ias_kt,
                      status.position.lat, status.position.lon):
            if value != value or abs(value) == float("inf"):   # NaN or inf
                result.faults.append("the state stopped being a number")
                return result
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break

    result.phase = pilot.phase.value
    result.hours = pilot.elapsed_s / 3600.0
    result.worst_overspeed_kt = max(0.0, worst_overspeed)
    result.lowest_agl_ft = 0.0 if lowest > 1e8 else lowest
    result.go_arounds = pilot._go_arounds

    far_end = destination_point(arrival.threshold, arrival.heading_true_deg,
                                arrival.length_ft / FEET_PER_NM)
    if pilot.touchdown_position is not None:
        result.touchdown_along_ft = along_track_nm(
            pilot.touchdown_position, arrival.threshold, far_end) * FEET_PER_NM
        result.touchdown_off_ft = abs(cross_track_nm(
            pilot.touchdown_position, arrival.threshold, far_end)) * FEET_PER_NM
    result.stop_along_ft = along_track_nm(
        sim.state.position, arrival.threshold, far_end) * FEET_PER_NM

    _judge(result, plan, profile)
    return result


def _judge(result: Result, plan, profile) -> None:
    """Every statement the flight has to satisfy, in one place."""
    if result.phase != "complete":
        result.faults.append(f"ended in {result.phase or 'nothing'}")
        return

    if not 0.0 < result.touchdown_along_ft:
        result.faults.append(
            f"touched down {abs(result.touchdown_along_ft):.0f} ft short")
    elif result.touchdown_along_ft > result.runway_ft * TOUCHDOWN_ZONE_FRACTION:
        result.faults.append(
            f"landed {result.touchdown_along_ft:.0f} ft down a "
            f"{result.runway_ft:.0f} ft runway")

    if result.touchdown_off_ft > CENTRELINE_LIMIT_FT:
        result.faults.append(
            f"touched down {result.touchdown_off_ft:.0f} ft off the centreline")

    if result.stop_along_ft > result.runway_ft:
        result.faults.append(
            f"ran {result.stop_along_ft - result.runway_ft:.0f} ft past the end")

    if result.worst_overspeed_kt > 5.0:
        result.faults.append(f"exceeded Vmo by {result.worst_overspeed_kt:.0f} kt")

    if result.lowest_agl_ft and result.lowest_agl_ft < MIN_ENROUTE_AGL_FT:
        result.faults.append(
            f"came within {result.lowest_agl_ft:.0f} ft of the ground enroute")

    if result.route_nm > result.distance_nm * 1.6 + 120:
        result.faults.append(
            f"flew {result.route_nm:.0f} nm for a {result.distance_nm:.0f} nm trip")

    # A block time that is not physically possible either way.
    cruise_kt = profile.cruise_mach * 573.0
    fastest = result.route_nm / (cruise_kt * 1.35) / 1.0
    slowest = result.route_nm / max(cruise_kt * 0.45, 1.0) + 0.9
    if not (fastest - 0.35 <= result.hours <= slowest):
        result.faults.append(
            f"took {result.hours:.2f} h for {result.route_nm:.0f} nm")
