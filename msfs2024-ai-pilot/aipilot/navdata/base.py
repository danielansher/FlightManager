"""Navigation data model and the provider interface.

Nav data is the one thing this project cannot ship complete: current AIRAC
procedure data is licensed, and scenery-derived runway data belongs to whoever
installed the scenery. So the AI Pilot reads whatever the user already has, in
priority order, through :class:`NavDataProvider`:

* Little Navmap's scenery database, if installed -- the best source, because it
  was compiled from the same scenery the simulator is flying over, so runway
  positions and ILS frequencies match what the aeroplane will actually receive.
* OurAirports CSV files -- public domain, one download, covers every airport
  and runway in the world but has no ILS data.
* A small bundled sample, so a fresh clone can fly a demo immediately.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Sequence

from ..geo import LatLon, distance_nm, normalize_deg


@dataclass(frozen=True)
class Runway:
    """One landing direction. A physical strip appears twice, once per end."""

    ident: str                       # "27L"
    threshold: LatLon
    heading_true_deg: float
    length_ft: float
    elevation_ft: float
    width_ft: float = 150.0
    surface: str = "unknown"
    displaced_threshold_ft: float = 0.0
    ils_freq_mhz: Optional[float] = None
    ils_course_true_deg: Optional[float] = None
    glideslope_deg: float = 3.0

    @property
    def has_ils(self) -> bool:
        return self.ils_freq_mhz is not None

    @property
    def approach_course_true_deg(self) -> float:
        """The course to fly on final: the ILS course if there is one."""
        if self.ils_course_true_deg is not None:
            return self.ils_course_true_deg
        return self.heading_true_deg

    def point_on_centreline(self, distance_nm_: float) -> LatLon:
        """A point ``distance_nm_`` out on the approach centreline."""
        from ..geo import destination_point

        return destination_point(
            self.threshold, normalize_deg(self.approach_course_true_deg + 180.0), distance_nm_
        )


@dataclass(frozen=True)
class Airport:
    icao: str
    name: str
    position: LatLon
    elevation_ft: float
    magvar_deg: float = 0.0
    runways: tuple[Runway, ...] = ()

    def runway(self, ident: str) -> Optional[Runway]:
        wanted = ident.upper().lstrip("0").replace("RW", "")
        for rwy in self.runways:
            if rwy.ident.upper().lstrip("0") == wanted:
                return rwy
        return None

@dataclass(frozen=True)
class TaxiPath:
    """One segment of taxiway centreline, as the scenery defines it."""

    start: LatLon
    end: LatLon
    name: str = ""
    kind: str = "taxi"          # "taxi", "runway", "parking", "vehicle"
    width_ft: float = 75.0

    @property
    def length_nm(self) -> float:
        return distance_nm(self.start, self.end)


@dataclass(frozen=True)
class Parking:
    """A stand, gate or ramp position."""

    name: str
    position: LatLon
    heading_true_deg: float = 0.0
    radius_ft: float = 75.0
    kind: str = "gate"

    def __str__(self) -> str:  # pragma: no cover - display only
        return self.name


@dataclass(frozen=True)
class GroundLayout:
    """Everything known about getting around an airport on the ground."""

    icao: str
    taxi_paths: tuple[TaxiPath, ...] = ()
    parking: tuple[Parking, ...] = ()

    @property
    def usable(self) -> bool:
        return len(self.taxi_paths) >= 2

    def nearest_parking(self, position: LatLon) -> Optional[Parking]:
        return min(self.parking, key=lambda p: distance_nm(position, p.position),
                   default=None)


@dataclass(frozen=True)
class Waypoint:
    """A named point: a navaid, an intersection, or something we invented."""

    ident: str
    position: LatLon
    kind: str = "fix"

    def __str__(self) -> str:  # pragma: no cover - display only
        return self.ident


class NavDataProvider(ABC):
    """Source of airports, runways and named fixes."""

    name = "abstract"

    @abstractmethod
    def airport(self, icao: str) -> Optional[Airport]:
        """Look up an airport by ICAO identifier, or ``None`` if unknown."""

    def waypoint(self, ident: str, near: Optional[LatLon] = None) -> Optional[Waypoint]:
        """Resolve a fix name. Identifiers repeat worldwide, so when ``near``
        is given the closest match wins."""
        return None

    def ground_layout(self, icao: str) -> Optional[GroundLayout]:
        """Taxiways and stands, for providers that have them."""
        return None

    def close(self) -> None:
        """Release any handles. Safe to call more than once."""

    @property
    def available(self) -> bool:
        return True

    def describe(self) -> str:
        return self.name


class ChainedNavData(NavDataProvider):
    """Tries each provider in turn and returns the first useful answer.

    Runway data is merged rather than replaced: if the first provider knows the
    airport but has no ILS for the chosen runway and a later one does, the
    later frequency is folded in. In practice that is the OurAirports-plus-
    Little-Navmap combination, which is the common case.
    """

    name = "chained"

    def __init__(self, providers: Sequence[NavDataProvider]) -> None:
        self.providers = [p for p in providers if p.available]

    def airport(self, icao: str) -> Optional[Airport]:
        best: Optional[Airport] = None
        for provider in self.providers:
            found = provider.airport(icao)
            if found is None:
                continue
            if best is None:
                best = found
            elif not any(r.has_ils for r in best.runways) and any(r.has_ils for r in found.runways):
                best = _merge_ils(best, found)
            if best.runways and any(r.has_ils for r in best.runways):
                break
        return best

    def waypoint(self, ident: str, near: Optional[LatLon] = None) -> Optional[Waypoint]:
        for provider in self.providers:
            found = provider.waypoint(ident, near)
            if found is not None:
                return found
        return None

    def ground_layout(self, icao: str) -> Optional[GroundLayout]:
        for provider in self.providers:
            layout = provider.ground_layout(icao)
            if layout is not None and layout.usable:
                return layout
        return None

    def close(self) -> None:
        for provider in self.providers:
            provider.close()

    @property
    def available(self) -> bool:
        return bool(self.providers)

    def describe(self) -> str:
        return " + ".join(p.describe() for p in self.providers) or "none"


def _merge_ils(base: Airport, other: Airport) -> Airport:
    """Copy ILS details from ``other`` onto matching runways of ``base``."""
    from dataclasses import replace

    by_ident = {r.ident.upper().lstrip("0"): r for r in other.runways}
    merged = []
    for rwy in base.runways:
        donor = by_ident.get(rwy.ident.upper().lstrip("0"))
        if donor is not None and donor.has_ils and not rwy.has_ils:
            rwy = replace(
                rwy,
                ils_freq_mhz=donor.ils_freq_mhz,
                ils_course_true_deg=donor.ils_course_true_deg,
                glideslope_deg=donor.glideslope_deg,
            )
        merged.append(rwy)
    return replace(base, runways=tuple(merged))


def wind_components_kt(runway: Runway, wind_from_deg: float,
                       wind_kt: float) -> tuple[float, float]:
    """Head- and crosswind on this runway, in knots.

    Headwind is positive down the runway, negative for a tailwind. Crosswind
    is returned as a magnitude: which side it comes from changes the
    technique, not whether the runway is usable.
    """
    import math

    from ..geo import signed_diff_deg

    angle = math.radians(signed_diff_deg(wind_from_deg, runway.heading_true_deg))
    return wind_kt * math.cos(angle), abs(wind_kt * math.sin(angle))


def select_runway(
    airport: Airport,
    wind_from_deg: float = 0.0,
    wind_kt: float = 0.0,
    min_length_ft: float = 0.0,
    prefer_ils: bool = True,
    max_crosswind_kt: float = 0.0,
    max_tailwind_kt: float = 0.0,
) -> Optional[Runway]:
    """Pick the runway an airline crew would most likely be given.

    Ranked so that a runway the aeroplane can actually use beats one it
    cannot: long enough first, then inside the crosswind and tailwind the
    type is cleared for, then by headwind, then by ILS, then by length.

    Those first two are preferences rather than filters, deliberately. An
    airport where every runway is short, or where the wind is across all of
    them, still has to produce an answer -- there is nowhere else to go, and
    refusing to pick would leave the aeroplane with no plan at all. The
    planner says so out loud instead.
    """
    candidates = list(airport.runways)
    if not candidates:
        return None

    def score(rwy: Runway) -> tuple[float, float, float, float, float]:
        headwind, crosswind = wind_components_kt(rwy, wind_from_deg, wind_kt)
        long_enough = 1.0 if rwy.length_ft >= min_length_ft else 0.0
        within_limits = 1.0
        if max_crosswind_kt and crosswind > max_crosswind_kt:
            within_limits = 0.0
        if max_tailwind_kt and headwind < -max_tailwind_kt:
            within_limits = 0.0
        # A tailwind is a real limitation rather than a preference.
        wind_score = headwind if headwind >= 0 else headwind * 4.0
        ils_score = 1.0 if (prefer_ils and rwy.has_ils) else 0.0
        return (long_enough, within_limits, round(wind_score, 1), ils_score,
                rwy.length_ft)

    return max(candidates, key=score)


