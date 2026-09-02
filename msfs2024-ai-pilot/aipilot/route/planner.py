"""Builds a flyable flight plan from two ICAO codes.

The AI Pilot deliberately does not need SID or STAR data. Published procedures
are licensed, they change every 28 days, and half the value of the original
MSFS 2020 AI Pilot was that you typed two airports and it went. So the route
here is built from geometry:

* a departure leg straight off the runway to a clean-up altitude,
* a great circle to the destination, cut into named segments so there is
  something to display progress against,
* an approach built backwards from the landing runway threshold: a downwind-
  free straight-in with an intercept point, a final approach fix, and the
  threshold itself.

If the user does have a route -- pasted from SimBrief, say -- :func:`plan_route`
will use the fixes in it that the nav data can resolve, and fall back to the
great circle for the rest.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

from ..geo import (
    LatLon,
    destination_point,
    distance_nm,
    initial_bearing_deg,
    interpolate_great_circle,
    normalize_deg,
    signed_diff_deg,
)
from ..navdata.base import Airport, NavDataProvider, Runway, Waypoint, select_runway
from ..perf.profiles import AircraftProfile, select_cruise_altitude
from .plan import FlightPlan, RouteLeg
from .profile import FAF_DISTANCE_NM

#: Where the departure leg ends: straight ahead off the runway.
DEPARTURE_LEG_NM = 6.0
DEPARTURE_ALTITUDE_AGL_FT = 3000.0

#: The straight-in approach geometry, in track miles from the threshold.
APPROACH_INTERCEPT_NM = 18.0     # where we join the extended centreline
APPROACH_GATE_NM = 10.0          # configured and slowing by here
FINAL_FIX_NM = FAF_DISTANCE_NM   # the final approach fix

#: How far the centreline is extended past the threshold, so that lateral
#: guidance still has something ahead of it during the flare and rollout.
ROLLOUT_EXTENSION_NM = 3.0

#: The largest turn the aeroplane should have to make when joining the
#: approach. Beyond this a more elaborate join is built instead.
MAX_APPROACH_ENTRY_TURN_DEG = 100.0

#: Where the base leg sits: this far out on the centreline, offset this far to
#: the side. The two together set the intercept angle, about 40 degrees.
BASE_LEG_DISTANCE_NM = 28.0
BASE_LEG_OFFSET_NM = 9.0

#: How far past the threshold the downwind is joined, so the aeroplane turns
#: onto it near the field rather than miles short of it.
DOWNWIND_ENTRY_NM = 3.0

#: Track miles an airliner needs per thousand feet, climbing and descending.
#: Used to decide whether a cruise level is reachable at all on a short sector.
CLIMB_NM_PER_1000FT = 3.0
DESCENT_NM_PER_1000FT = 3.4

#: Enroute great-circle segments longer than this get split, so that progress,
#: ETA and the map have something to work with.
MAX_SEGMENT_NM = 250.0


def _oceanic_name(point: LatLon) -> str:
    """Name a computed point the way an oceanic waypoint is named: 52N040W."""
    ns = "N" if point.lat >= 0 else "S"
    ew = "E" if point.lon >= 0 else "W"
    return f"{abs(point.lat):02.0f}{ns}{abs(point.lon):03.0f}{ew}"


def _split_great_circle(start: LatLon, end: LatLon,
                        max_segment_nm: float = MAX_SEGMENT_NM) -> list[Waypoint]:
    """Intermediate points along the great circle, excluding both endpoints."""
    total = distance_nm(start, end)
    if total <= max_segment_nm:
        return []
    count = int(math.ceil(total / max_segment_nm)) - 1
    out = []
    seen: set[str] = set()
    for i in range(1, count + 1):
        point = interpolate_great_circle(start, end, i / (count + 1))
        name = _oceanic_name(point)
        # Two segments can round to the same name near the poles.
        suffix = 1
        base = name
        while name in seen:
            suffix += 1
            name = f"{base}{suffix}"
        seen.add(name)
        out.append(Waypoint(name, point, "computed"))
    return out


def _departure_legs(origin: Airport, runway: Optional[Runway]) -> list[RouteLeg]:
    """The runway itself, then a straight-ahead climb fix."""
    legs: list[RouteLeg] = []
    if runway is None:
        legs.append(RouteLeg(Waypoint(origin.icao, origin.position, "airport"),
                             altitude_ft=origin.elevation_ft, phase="departure"))
        return legs
    legs.append(
        RouteLeg(
            Waypoint(f"RW{runway.ident}", runway.threshold, "runway"),
            altitude_ft=runway.elevation_ft,
            phase="takeoff",
            flyover=True,
        )
    )
    climb_fix = destination_point(runway.threshold, runway.heading_true_deg, DEPARTURE_LEG_NM)
    legs.append(
        RouteLeg(
            Waypoint(f"D{runway.ident}", climb_fix, "computed"),
            altitude_ft=origin.elevation_ft + DEPARTURE_ALTITUDE_AGL_FT,
            altitude_kind="at_or_above",
            phase="departure",
            flyover=True,
        )
    )
    return legs


def approach_scale(direct_distance_nm: float) -> float:
    """How much to shrink the outer approach geometry on a short sector.

    The circuit and base leg are sized for an aeroplane arriving from a long
    way away. Applied unchanged to a fifteen mile hop they are longer than the
    flight: Los Angeles to Burbank came out as a ninety-four mile route whose
    turning point was forty-five miles from the departure runway, on the far
    side of the destination and through the mountains behind it.

    The *final* approach is never scaled. Five miles at a thousand six hundred
    feet is what makes an approach stabilised, and it does not get shorter
    because the flight was short.
    """
    if direct_distance_nm >= 120.0:
        return 1.0
    return max(0.45, direct_distance_nm / 120.0)


def _approach_legs(destination: Airport, runway: Optional[Runway],
                   profile: AircraftProfile,
                   inbound_course_deg: Optional[float] = None,
                   style: str = "straight",
                   scale: float = 1.0) -> list[RouteLeg]:
    """Built backwards from the threshold along the approach course.

    A straight-in join works only when the aeroplane is already pointing more
    or less the right way. Arrive at San Francisco from the north-west for a
    westerly runway and the straight-in geometry sends it over the field, round
    through a hundred and seventy degrees, and back -- with eighteen miles in
    which to settle on the centreline, which is not enough, and it reaches the
    stabilisation gate still half a mile off.

    So when the arrival course and the approach course disagree badly, a base
    leg is inserted: a point out to the side the aeroplane is coming from,
    positioned to give roughly a forty degree intercept onto the centreline
    with the whole of the final approach left in which to settle. That is what
    a controller would give, and it is what the aeroplane can actually fly.
    """
    if runway is None:
        return [RouteLeg(Waypoint(destination.icao, destination.position, "airport"),
                         altitude_ft=destination.elevation_ft, phase="approach")]

    field_elev = runway.elevation_ft
    gradient = 6076.11548556 * math.tan(math.radians(profile.descent_angle_deg))

    def at(distance: float, phase: str, speed: Optional[float] = None,
           agl: Optional[float] = None) -> RouteLeg:
        height = agl if agl is not None else distance * gradient
        return RouteLeg(
            Waypoint(f"{runway.ident}-{distance:g}" if distance else f"RW{runway.ident}",
                     runway.point_on_centreline(distance), "computed"),
            altitude_ft=field_elev + height,
            altitude_kind="at",
            speed_kt=speed,
            phase=phase,
            flyover=True,
        )

    legs: list[RouteLeg] = []
    approach_course = runway.approach_course_true_deg
    intercept_nm = max(APPROACH_GATE_NM + 2.0, APPROACH_INTERCEPT_NM * scale)
    base_nm = max(intercept_nm + 4.0, BASE_LEG_DISTANCE_NM * scale)
    offset_nm = max(3.0, BASE_LEG_OFFSET_NM * scale)

    def offset_fix(name: str, along_nm: float, across_nm: float, side: float,
                   height_nm: float) -> RouteLeg:
        """A fix ``along_nm`` out on the centreline, ``across_nm`` to one side."""
        centre = runway.point_on_centreline(along_nm)
        position = centre if across_nm == 0 else destination_point(
            centre, normalize_deg(approach_course + 90.0 * side), across_nm
        )
        return RouteLeg(
            Waypoint(f"{runway.ident}-{name}", position, "computed"),
            altitude_ft=field_elev + height_nm * gradient,
            speed_kt=profile.terminal_speed_kt,
            phase="approach",
        )

    if style != "straight" and inbound_course_deg is not None:
        # Offset to the side the aeroplane is arriving from, so every turn is
        # towards the centreline rather than across it.
        side = -1.0 if signed_diff_deg(approach_course, inbound_course_deg) > 0 else 1.0
        if style == "circuit":
            # Nearly the opposite direction: fly a circuit. Join downwind
            # abeam the field, run out parallel to the runway, turn base
            # across, then turn final. Three legs and two ninety degree turns
            # with room between them -- which is what a radar pattern is --
            # instead of one impossible reversal.
            legs.append(offset_fix("DW", -DOWNWIND_ENTRY_NM, offset_nm, side, base_nm))
            legs.append(offset_fix("BASE", base_nm, offset_nm, side, base_nm))
            legs.append(offset_fix("FIN", base_nm, 0.0, side, base_nm))
        else:
            # A crossing arrival: one base leg gives about a forty degree
            # intercept with the whole final approach left in which to settle.
            legs.append(offset_fix("BASE", base_nm, offset_nm, side, base_nm))

    legs += [
        at(intercept_nm, "approach", profile.terminal_speed_kt),
        at(APPROACH_GATE_NM, "approach", 180.0),
        at(FINAL_FIX_NM, "final", profile.final_approach_speed_kt + 10.0),
        RouteLeg(
            Waypoint(f"RW{runway.ident}", runway.threshold, "runway"),
            altitude_ft=field_elev + 50.0,
            speed_kt=profile.final_approach_speed_kt,
            phase="landing",
            flyover=True,
        ),
        # The centreline has to continue past the threshold. Ending the route
        # at the threshold leaves the lateral channel with nothing ahead of it
        # at the exact moment the aeroplane is about to touch down, and it
        # turns back to chase a fix that is now behind -- which is a go-around
        # at fifty feet. This fix is never reached; it exists to be aimed at.
        RouteLeg(
            Waypoint(
                f"{runway.ident}-ROLL",
                destination_point(runway.threshold, runway.heading_true_deg,
                                  ROLLOUT_EXTENSION_NM),
                "computed",
            ),
            phase="rollout",
            flyover=True,
        ),
    ]
    return legs


def resolve_route_string(route: str, navdata: NavDataProvider,
                         start: LatLon) -> tuple[list[Waypoint], list[str]]:
    """Resolve a SimBrief-style route into the fixes we can actually find.

    Airway identifiers are skipped rather than expanded -- expanding them needs
    airway data this project does not require you to have -- so a route given
    as ``MID UL9 KONAN`` flies direct MID then direct KONAN. That is a slightly
    different track from the airway, never a wrong one.
    """
    found: list[Waypoint] = []
    skipped: list[str] = []
    cursor = start
    for token in route.replace(",", " ").split():
        token = token.strip().upper()
        if not token or token in ("DCT", "SID", "STAR", "DIRECT"):
            continue
        if "/" in token:                      # KONAN/N0450F350 -- strip the change
            token = token.split("/", 1)[0]
        point = navdata.waypoint(token, near=cursor)
        if point is None:
            skipped.append(token)
            continue
        found.append(point)
        cursor = point.position
    return found, skipped


def plan_route(
    origin: Airport,
    destination: Airport,
    profile: AircraftProfile,
    navdata: Optional[NavDataProvider] = None,
    departure_runway: Optional[str] = None,
    arrival_runway: Optional[str] = None,
    cruise_altitude_ft: Optional[float] = None,
    route: Optional[str] = None,
    wind_from_deg: float = 0.0,
    wind_kt: float = 0.0,
    min_runway_ft: float = 6000.0,
) -> FlightPlan:
    """Build a complete, flyable plan between two airports."""
    warnings: list[str] = []

    dep_rwy = _choose_runway(origin, departure_runway, wind_from_deg, wind_kt,
                             min_runway_ft, warnings, "departure")
    arr_rwy = _choose_runway(destination, arrival_runway, wind_from_deg, wind_kt,
                             min_runway_ft, warnings, "arrival")

    direct_course = initial_bearing_deg(origin.position, destination.position)
    direct_distance = distance_nm(origin.position, destination.position)
    requested_cruise = cruise_altitude_ft
    if cruise_altitude_ft is None:
        cruise_altitude_ft = select_cruise_altitude(direct_distance, direct_course, profile)
    cruise_altitude_ft = _reachable_cruise(
        cruise_altitude_ft, origin, destination, direct_distance
    )
    if requested_cruise is None and cruise_altitude_ft < select_cruise_altitude(
            direct_distance, direct_course, profile):
        warnings.append(
            f"{direct_distance:.0f} nm is too short to climb high and get back "
            f"down again, so the cruise level was capped at "
            f"{cruise_altitude_ft:.0f} ft."
        )

    legs = _departure_legs(origin, dep_rwy)
    enroute_start = legs[-1].position
    # The course the great circle arrives on, which decides whether the
    # approach can be joined straight in.
    inbound_course = normalize_deg(
        initial_bearing_deg(destination.position, enroute_start) + 180.0
    )
    approach = _choose_approach(destination, arr_rwy, profile, inbound_course,
                                enroute_start)
    enroute_end = approach[0].position

    middle: list[Waypoint] = []
    if route:
        if navdata is None:
            warnings.append("A route was given but there is no nav data to resolve it against.")
        else:
            resolved, skipped = resolve_route_string(route, navdata, enroute_start)
            middle = _drop_backtracks(resolved, enroute_start, enroute_end)
            if skipped:
                warnings.append(
                    "Could not resolve these route elements, so they were skipped: "
                    + " ".join(skipped[:12]) + ("..." if len(skipped) > 12 else "")
                )
            if len(middle) < len(resolved):
                warnings.append(
                    f"Dropped {len(resolved) - len(middle)} route fix(es) that pointed backwards."
                )

    enroute_legs: list[RouteLeg] = []
    chain = [enroute_start] + [w.position for w in middle] + [enroute_end]
    for index, waypoint in enumerate(middle):
        for filler in _split_great_circle(chain[index], waypoint.position):
            enroute_legs.append(RouteLeg(filler, phase="enroute"))
        enroute_legs.append(RouteLeg(waypoint, phase="enroute"))
    for filler in _split_great_circle(chain[-2], chain[-1]):
        enroute_legs.append(RouteLeg(filler, phase="enroute"))

    plan = FlightPlan(
        origin=origin,
        destination=destination,
        departure_runway=dep_rwy,
        arrival_runway=arr_rwy,
        cruise_altitude_ft=cruise_altitude_ft,
        legs=legs + enroute_legs + approach,
        warnings=warnings,
    )

    if dep_rwy and dep_rwy.surface == "synthetic":
        warnings.append(
            f"{origin.icao} has no runway data, so a runway was assumed. Fine for a "
            "demo, not for a real departure -- install Little Navmap or download "
            "the OurAirports runways.csv."
        )
    if arr_rwy and arr_rwy.surface == "synthetic":
        warnings.append(
            f"{destination.icao} has no runway data, so the approach was built to an "
            "assumed runway and will not line up with the real one."
        )
    if plan.total_distance_nm > max(direct_distance * 2.5, direct_distance + 40.0):
        warnings.append(
            f"The route is {plan.total_distance_nm:.0f} nm for a "
            f"{direct_distance:.0f} nm trip, because joining "
            f"{destination.icao}/{arr_rwy.ident if arr_rwy else '?'} from this "
            "direction needs a circuit. Pick the other runway with "
            "--arrival-runway if you would rather go straight in."
        )
    if arr_rwy and not arr_rwy.has_ils:
        warnings.append(
            f"No ILS on {destination.icao}/{arr_rwy.ident}; the approach will be flown "
            "on the computed path rather than on the aeroplane's ILS receiver."
        )
    return plan


def _reachable_cruise(cruise_ft: float, origin: Airport, destination: Airport,
                      direct_distance_nm: float) -> float:
    """Cap a cruise level at one the sector actually has room for.

    Choosing a level from distance alone gives a fifteen mile hop a cruise of
    FL190, which needs some sixty track miles to climb to and another sixty to
    come down from. The aeroplane then reaches top of descent while still
    climbing, and spends the entire flight in a descent it cannot fly.

    So: work out what is reachable in the distance available, and take the
    lower of the two.
    """
    field = max(origin.elevation_ft, destination.elevation_ft)
    usable = max(0.0, direct_distance_nm * 0.8)
    gain = usable / (CLIMB_NM_PER_1000FT + DESCENT_NM_PER_1000FT) * 1000.0
    # Never plan lower than a sensible minimum enroute height above the fields.
    floor = field + 2000.0
    capped = min(cruise_ft, max(field + gain, floor))
    return round(capped / 1000.0) * 1000.0


def _choose_approach(destination: Airport, runway: Optional[Runway],
                     profile: AircraftProfile, inbound_course_deg: float,
                     enroute_start: LatLon) -> list[RouteLeg]:
    """Pick the simplest approach join the aeroplane can actually fly.

    The turn that matters is the one at the *first approach fix* -- from the
    course the aeroplane arrives on to the course of the first approach leg --
    and that is not the same as the angle between the arrival track and the
    runway. The approach fixes sit tens of miles out and off to one side, so
    the two can differ by fifty degrees, and choosing the join from the runway
    angle alone picks a straight-in for arrivals that need a circuit.

    So rather than predicting it, each style is built and measured, and the
    first one that turns out flyable is used.
    """
    direct = distance_nm(enroute_start, destination.position)
    scale = approach_scale(direct)
    styles = ("straight", "base", "circuit")
    built: list[RouteLeg] = []
    fallback: list[RouteLeg] = []
    for style in styles:
        built = _approach_legs(destination, runway, profile, inbound_course_deg,
                               style, scale)
        if len(built) < 2:
            break
        if not fallback:
            fallback = built
        # An approach whose first fix is further away than the destination is
        # itself sends the aeroplane past the airport and back, which on a
        # short sector is most of the flight and over whatever happens to be
        # behind the field.
        reach = distance_nm(enroute_start, built[0].position)
        if reach > max(direct * 1.6, direct + 25.0):
            continue
        entry = initial_bearing_deg(enroute_start, built[0].position)
        first_leg = initial_bearing_deg(built[0].position, built[1].position)
        if abs(signed_diff_deg(first_leg, entry)) <= MAX_APPROACH_ENTRY_TURN_DEG:
            return built
    return built if built else fallback


def _choose_runway(airport: Airport, requested: Optional[str], wind_from_deg: float,
                   wind_kt: float, min_length_ft: float, warnings: list[str],
                   role: str) -> Optional[Runway]:
    if requested:
        runway = airport.runway(requested)
        if runway is not None:
            return runway
        warnings.append(
            f"{airport.icao} has no runway {requested}; picked one from the wind instead."
        )
    chosen = select_runway(airport, wind_from_deg, wind_kt, min_length_ft)
    if chosen is None:
        warnings.append(f"No {role} runway data for {airport.icao}.")
    return chosen


def _drop_backtracks(waypoints: Sequence[Waypoint], start: LatLon,
                     end: LatLon) -> list[Waypoint]:
    """Discard fixes that would send the aeroplane the wrong way.

    A pasted route can contain fixes we resolved to the wrong continent -- fix
    identifiers are not unique worldwide -- and one of those turns a flight
    plan into a sightseeing tour. Anything that increases the distance to the
    destination, or sits absurdly far off the direct track, is dropped.
    """
    out: list[Waypoint] = []
    cursor = start
    direct = distance_nm(start, end)
    for waypoint in waypoints:
        remaining = distance_nm(waypoint.position, end)
        detour = distance_nm(cursor, waypoint.position) + remaining
        if remaining > distance_nm(cursor, end):
            continue                      # points away from the destination
        if detour > direct * 1.4 + 100.0:
            continue                      # an implausible dogleg
        out.append(waypoint)
        cursor = waypoint.position
    return out


def rebuild_departure(plan, runway: Runway, profile: AircraftProfile) -> None:
    """Point an existing plan at a different departure runway, in place.

    Used when the aeroplane is lined up on a runway other than the planned one,
    which is the normal case: the plan picks a runway from the wind, and the
    pilot taxis wherever the ground controller sent them. Following them is
    better than arguing, and only the first two legs depend on it.
    """
    keep = [leg for leg in plan.legs if leg.phase not in ("takeoff", "departure")]
    plan.legs = _departure_legs(plan.origin, runway) + keep
    plan.departure_runway = runway
