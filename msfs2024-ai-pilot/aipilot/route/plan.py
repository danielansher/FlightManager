"""The flight plan: an ordered list of fixes with constraints attached."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator, Optional

from ..geo import LatLon, distance_nm, initial_bearing_deg
from ..navdata.base import Airport, Runway, Waypoint


@dataclass
class RouteLeg:
    """One fix, plus whatever must be true when the aeroplane reaches it."""

    waypoint: Waypoint
    altitude_ft: Optional[float] = None
    altitude_kind: str = "at"            # "at", "at_or_above", "at_or_below"
    speed_kt: Optional[float] = None
    phase: str = "enroute"
    #: Fly straight through rather than cutting the corner. Used for the final
    #: approach fixes, where cutting the corner would take us off the centreline.
    flyover: bool = False

    @property
    def ident(self) -> str:
        return self.waypoint.ident

    @property
    def position(self) -> LatLon:
        return self.waypoint.position

    def satisfies(self, altitude_ft: float, tolerance_ft: float = 300.0) -> bool:
        if self.altitude_ft is None:
            return True
        if self.altitude_kind == "at_or_above":
            return altitude_ft >= self.altitude_ft - tolerance_ft
        if self.altitude_kind == "at_or_below":
            return altitude_ft <= self.altitude_ft + tolerance_ft
        return abs(altitude_ft - self.altitude_ft) <= tolerance_ft

    def __str__(self) -> str:  # pragma: no cover - display only
        bits = [self.ident]
        if self.altitude_ft is not None:
            prefix = {"at_or_above": "+", "at_or_below": "-"}.get(self.altitude_kind, "")
            bits.append(f"{prefix}{self.altitude_ft:.0f}ft")
        if self.speed_kt is not None:
            bits.append(f"{self.speed_kt:.0f}kt")
        return "/".join(bits)


@dataclass
class FlightPlan:
    """A departure runway, a route, an arrival runway and a cruise level."""

    origin: Airport
    destination: Airport
    departure_runway: Optional[Runway]
    arrival_runway: Optional[Runway]
    cruise_altitude_ft: float
    legs: list[RouteLeg] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    # -- Geometry ------------------------------------------------------------
    def __len__(self) -> int:
        return len(self.legs)

    def __iter__(self) -> Iterator[RouteLeg]:
        return iter(self.legs)

    def __getitem__(self, index: int) -> RouteLeg:
        return self.legs[index]

    @property
    def total_distance_nm(self) -> float:
        return sum(
            distance_nm(a.position, b.position) for a, b in zip(self.legs, self.legs[1:])
        )

    def distance_from_leg_to_end_nm(self, index: int) -> float:
        """Route distance from leg ``index`` to the final fix."""
        return sum(
            distance_nm(a.position, b.position)
            for a, b in zip(self.legs[index:], self.legs[index + 1:])
        )

    def distance_to_end_nm(self, position: LatLon, active_index: int) -> float:
        """Distance still to fly: direct to the active fix, then along the route."""
        if not self.legs:
            return 0.0
        index = min(max(active_index, 0), len(self.legs) - 1)
        return distance_nm(position, self.legs[index].position) + \
            self.distance_from_leg_to_end_nm(index)

    def leg_course_deg(self, index: int) -> float:
        """Course of the leg *arriving at* ``index``."""
        if index <= 0 or index >= len(self.legs):
            return 0.0
        return initial_bearing_deg(self.legs[index - 1].position, self.legs[index].position)

    def next_course_deg(self, index: int) -> Optional[float]:
        """Course of the leg *departing* ``index``, if there is one."""
        if index < 0 or index + 1 >= len(self.legs):
            return None
        return initial_bearing_deg(self.legs[index].position, self.legs[index + 1].position)

    def course_change_at_deg(self, index: int) -> float:
        """How sharply the route turns at ``index``. Zero at either end."""
        from ..geo import signed_diff_deg

        nxt = self.next_course_deg(index)
        if nxt is None or index <= 0:
            return 0.0
        return signed_diff_deg(nxt, self.leg_course_deg(index))

    # -- Description ---------------------------------------------------------
    @property
    def threshold_index(self) -> int:
        """Index of the landing threshold, which is what distances are measured to.

        Not the last leg: the route continues a few miles past the threshold so
        that lateral guidance has something ahead of it during the flare.
        """
        for i, leg in enumerate(self.legs):
            if leg.phase == "landing":
                return i
        return max(0, len(self.legs) - 1)

    @property
    def threshold_position(self) -> LatLon:
        return self.legs[self.threshold_index].position

    def index_of_phase(self, phase: str) -> Optional[int]:
        for i, leg in enumerate(self.legs):
            if leg.phase == phase:
                return i
        return None

    def describe(self) -> str:  # pragma: no cover - display only
        dep = self.departure_runway.ident if self.departure_runway else "?"
        arr = self.arrival_runway.ident if self.arrival_runway else "?"
        return (
            f"{self.origin.icao}/{dep} -> {self.destination.icao}/{arr}  "
            f"{self.total_distance_nm:.0f} nm at FL{self.cruise_altitude_ft / 100:.0f}\n"
            + "\n".join(f"  {i:>2}. {leg}  [{leg.phase}]" for i, leg in enumerate(self.legs))
        )
