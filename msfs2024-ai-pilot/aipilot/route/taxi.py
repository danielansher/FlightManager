"""Routing around an airport on the taxiways the scenery actually has.

There is no way to ask SimConnect about scenery objects, so an aeroplane
cannot be told to avoid a terminal building by seeing it. What it can do is
stay on the taxiways -- which is what "avoiding things" means on an airfield,
because the pavement is by construction the part with nothing parked on it.

So the ground network is built from Little Navmap's taxi path table, which is
compiled from the same scenery the aeroplane is sitting on, and a route across
it is found with A*. Segment endpoints are welded together on a small grid,
because scenery authors do not guarantee that the end of one segment is bitwise
identical to the start of the next, and without welding the graph falls apart
into thousands of disconnected pieces.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

from ..geo import LatLon, destination_point, distance_nm, initial_bearing_deg
from ..navdata.base import GroundLayout, Runway, TaxiPath

#: How close two segment endpoints must be to count as the same junction.
#: About five metres: closer than any real taxiway spacing, far enough apart
#: to absorb the rounding in scenery data.
WELD_TOLERANCE_NM = 0.0027

#: How far from the network the aeroplane may be and still be joined to it.
#: Stands sit off the taxiway proper, so this has to cover a lead-in line --
#: about two hundred and fifty metres -- without stretching to "somewhere on
#: the far side of the apron", which is where a pushback is needed instead.
MAX_JOIN_DISTANCE_NM = 0.14

#: Segments longer than this are split, so a route can join partway along one
#: rather than having to reach its end.
MAX_SEGMENT_NM = 0.08


#: How much more expensive a runway segment is to route along than a taxiway.
#: Not forbidden -- an aeroplane has to get onto the runway to depart and off
#: it to arrive -- but strongly discouraged, so a route never taxis half a mile
#: down an active runway because it happened to be the shortest way.
RUNWAY_COST_FACTOR = 8.0

#: The longest a single taxi path can plausibly be. Beyond this the row is
#: not a taxiway, it is a data error.
MAX_PATH_NM = 3.0


def _finite(position: LatLon) -> bool:
    return math.isfinite(position.lat) and math.isfinite(position.lon)


@dataclass
class GroundNode:
    index: int
    position: LatLon
    #: ``(neighbour, true length, routing cost)``. Length and cost differ so
    #: that distances reported to the user stay honest while the router is
    #: still discouraged from using runways as taxiways.
    edges: list[tuple[int, float, float]] = field(default_factory=list)


class GroundNetwork:
    """A navigable graph of an airport's taxiways."""

    def __init__(self, layout: GroundLayout) -> None:
        self.layout = layout
        self.nodes: list[GroundNode] = []
        #: Grid cell -> every node in it. A list, not a single node: two
        #: junctions further apart than the weld tolerance can share a cell,
        #: and storing one per cell meant the second evicted the first, which
        #: could then never be found again. Two segments meeting at bitwise
        #: identical coordinates then became unconnected, and the route across
        #: them came back empty for no visible reason.
        self._index: dict[tuple[int, int], list[int]] = {}
        for path in layout.taxi_paths:
            self._add_path(path)

    # -- Building ------------------------------------------------------------
    def _key(self, position: LatLon) -> tuple[int, int]:
        # A grid whose cells are the weld tolerance across, so nearby endpoints
        # land in the same cell and become the same junction.
        scale = WELD_TOLERANCE_NM / 60.0
        return (int(round(position.lat / scale)), int(round(position.lon / scale)))

    def _node(self, position: LatLon) -> int:
        """The junction at this position, creating one if there is none.

        Neighbouring cells are checked as well as the exact one. A plain grid
        lookup fails whenever two endpoints that ought to be the same junction
        happen to fall either side of a cell boundary, which for scenery data
        full of near-identical coordinates happens constantly -- and every time
        it does, the network quietly gains a break in it that no route can
        cross.
        """
        lat_key, lon_key = self._key(position)
        best, best_distance = None, WELD_TOLERANCE_NM
        for dlat in (-1, 0, 1):
            for dlon in (-1, 0, 1):
                for candidate in self._index.get((lat_key + dlat, lon_key + dlon), ()):
                    d = distance_nm(position, self.nodes[candidate].position)
                    if d <= best_distance:
                        best, best_distance = candidate, d
        if best is not None:
            return best
        index = len(self.nodes)
        self.nodes.append(GroundNode(index, position))
        self._index.setdefault((lat_key, lon_key), []).append(index)
        return index

    def _connect(self, a: int, b: int, length_nm: float, cost_nm: float) -> None:
        if a == b:
            return
        self.nodes[a].edges.append((b, length_nm, cost_nm))
        self.nodes[b].edges.append((a, length_nm, cost_nm))

    def _add_path(self, path: TaxiPath) -> None:
        if not (_finite(path.start) and _finite(path.end)):
            # A NaN or infinite coordinate. float("nan") passes every
            # try/except float(...) guard on the way in from the scenery
            # database, so it has to be stopped here or it detonates in the
            # grid arithmetic.
            return
        total = path.length_nm
        if total < 1e-6:
            return
        if total > MAX_PATH_NM:
            # A taxiway is not thirty miles long. One row with a zero
            # coordinate -- the classic scenery-export slip -- would otherwise
            # be chopped into tens of thousands of nodes, and every route
            # request scans all of them.
            return
        # Split long segments so a route can join partway along one.
        pieces = max(1, int(math.ceil(total / MAX_SEGMENT_NM)))
        course = initial_bearing_deg(path.start, path.end)
        factor = RUNWAY_COST_FACTOR if "runway" in path.kind.lower() else 1.0
        previous = self._node(path.start)
        for step in range(1, pieces + 1):
            point = (path.end if step == pieces
                     else destination_point(path.start, course, total * step / pieces))
            current = self._node(point)
            self._connect(previous, current, total / pieces,
                          total / pieces * factor)
            previous = current

    @property
    def usable(self) -> bool:
        return len(self.nodes) >= 2

    # -- Queries -------------------------------------------------------------
    def nearest_node(self, position: LatLon,
                     limit_nm: float = MAX_JOIN_DISTANCE_NM) -> Optional[int]:
        best, best_distance = None, limit_nm
        for node in self.nodes:
            d = distance_nm(position, node.position)
            if d < best_distance:
                best, best_distance = node.index, d
        return best

    def route(self, start: LatLon, goal: LatLon) -> list[LatLon]:
        """Shortest path across the taxiways, as a list of points to follow."""
        start_node = self.nearest_node(start)
        goal_node = self.nearest_node(goal, limit_nm=MAX_JOIN_DISTANCE_NM * 3)
        if start_node is None or goal_node is None:
            return []
        path = self._astar(start_node, goal_node)
        return [self.nodes[i].position for i in path]

    def _astar(self, start: int, goal: int) -> list[int]:
        goal_position = self.nodes[goal].position
        best_cost = {start: 0.0}
        came_from: dict[int, int] = {}
        queue = [(0.0, start)]
        closed: set[int] = set()

        while queue:
            _priority, current = heapq.heappop(queue)
            if current == goal:
                return self._rebuild(came_from, current)
            if current in closed:
                continue
            closed.add(current)
            for neighbour, _length, weight in self.nodes[current].edges:
                cost = best_cost[current] + weight
                if cost >= best_cost.get(neighbour, float("inf")):
                    continue
                best_cost[neighbour] = cost
                came_from[neighbour] = current
                estimate = distance_nm(self.nodes[neighbour].position, goal_position)
                heapq.heappush(queue, (cost + estimate, neighbour))
        return []

    def _rebuild(self, came_from: dict[int, int], current: int) -> list[int]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


def simplify(points: Iterable[LatLon], tolerance_deg: float = 4.0) -> list[LatLon]:
    """Drop points that lie on a straight run, keeping the turns.

    A* returns a point every eighty metres, which is far more than a steering
    controller needs and makes every tiny scenery kink look like a turn.
    """
    points = list(points)
    if len(points) < 3:
        return points
    out = [points[0]]
    for previous, point, following in zip(points, points[1:], points[2:]):
        incoming = initial_bearing_deg(previous, point)
        outgoing = initial_bearing_deg(point, following)
        if abs((outgoing - incoming + 180.0) % 360.0 - 180.0) >= tolerance_deg:
            out.append(point)
    out.append(points[-1])
    return out


def runway_entry_point(runway: Runway, network: Optional[GroundNetwork],
                       hold_short_nm: float = 0.04) -> LatLon:
    """Where to taxi to for departure: the runway threshold, from the side.

    Aiming at the threshold itself is right in principle, but the threshold
    point sits on the runway centreline at the very end, and the taxi network
    usually meets the runway a little way along it. Taking the nearest network
    node to the threshold gives the actual holding point.
    """
    if network is None or not network.usable:
        return runway.threshold
    entry = network.nearest_node(runway.threshold, limit_nm=0.6)
    if entry is None:
        return runway.threshold
    return network.nodes[entry].position


def build_network(layout: Optional[GroundLayout]) -> Optional[GroundNetwork]:
    if layout is None or not layout.usable:
        return None
    network = GroundNetwork(layout)
    return network if network.usable else None
