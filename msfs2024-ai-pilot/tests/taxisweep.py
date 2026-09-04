"""Taxi and pushback, measured across many real stands without a simulator.

The ground phase cannot be judged by whether the aeroplane arrives. It either
follows the painted line off the stand or it drives across the grass, and both
of those reach the runway eventually. Flying one stand in the simulator answers
one stand, takes four minutes, and needs somebody sitting there to put the
aeroplane back afterwards.

The taxiway data is on disk, so none of that is necessary. This runs the real
ground network through the mock from as many stands as asked for and reports
the numbers that say whether the taxi was any good:

    rotation   degrees turned during the push plus the turn the taxi opens
               with. An aeroplane that pushes through 174 degrees and then
               unwinds 124 of them has done something no tug driver would.
    typical    median distance from the nearest taxiway CENTRELINE, in feet --
               not from the route the program drew for itself. Those are
               different questions, and only the first one is about whether the
               aeroplane is on the pavement. Scored against its own polyline
               this sweep reported eight stands out of eight while the taxi was
               spending 38% of its time off the taxiway, because the polyline
               was off the taxiway too.
    worst      the furthest it strayed from any centreline, in feet
    off pav    the share of the taxi spent more than half a taxiway width from
               any centreline
    rev/min    times the nosewheel changed sides per minute, which is what a
               zig-zag looks like from the cockpit

Run it directly:

    python -m tests.taxisweep KJFK --runway 22R --stands 20
    python -m tests.taxisweep KJFK KLAX EGLL --stands 12
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from aipilot.aircraft.registry import build_adapter
from aipilot.autopilot.controller import AIPilot, PilotOptions
from aipilot.autopilot.phases import Phase
from aipilot.geo import (along_track_nm, cross_track_nm, distance_nm,
                         signed_diff_deg)
from aipilot.navdata.resolve import NavDataSources, build_navdata
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.route.taxi import build_network
from aipilot.sim.mock import MockAircraftModel, MockSim
from aipilot.units import FEET_PER_NM

#: What counts as a good enough taxi. Deliberately not the loosest numbers that
#: happen to pass today -- these are what the ground phase is meant to deliver.
#: Half a taxiway. The scenery says 82 ft wide at Kennedy, so 41 ft from the
#: centreline is the edge of the pavement -- and an airliner's wingtip is a
#: hundred feet beyond that again.
HALF_TAXIWAY_FT = 41.0

MAX_TYPICAL_FT = 25.0
MAX_WORST_FT = 120.0
MAX_OFF_PAVEMENT = 0.15
MAX_ROTATION_DEG = 200.0
MAX_REVERSALS_PER_MIN = 12.0


@dataclass
class Outcome:
    airport: str
    stand: str
    runway: str
    reached: bool
    rotation_deg: float
    typical_ft: float
    worst_ft: float
    reversals_per_min: float
    seconds: float
    off_pavement: float = 0.0
    note: str = ""

    @property
    def ok(self) -> bool:
        return (self.reached
                and self.typical_ft <= MAX_TYPICAL_FT
                and self.worst_ft <= MAX_WORST_FT
                and self.off_pavement <= MAX_OFF_PAVEMENT
                and self.rotation_deg <= MAX_ROTATION_DEG
                and self.reversals_per_min <= MAX_REVERSALS_PER_MIN)

    @property
    def why(self) -> str:
        if self.note:
            return self.note
        if not self.reached:
            return "never reached the runway"
        reasons = []
        if self.typical_ft > MAX_TYPICAL_FT:
            reasons.append(f"{self.typical_ft:.0f} ft off centreline")
        if self.worst_ft > MAX_WORST_FT:
            reasons.append(f"strayed {self.worst_ft:.0f} ft")
        if self.off_pavement > MAX_OFF_PAVEMENT:
            reasons.append(f"{self.off_pavement:.0%} off pavement")
        if self.rotation_deg > MAX_ROTATION_DEG:
            reasons.append(f"turned {self.rotation_deg:.0f} deg")
        if self.reversals_per_min > MAX_REVERSALS_PER_MIN:
            reasons.append(f"{self.reversals_per_min:.0f} reversals/min")
        return ", ".join(reasons)


def _deviation(position, leg_start, leg_end) -> float:
    length = distance_nm(leg_start, leg_end)
    if length < 1e-9:
        return distance_nm(position, leg_start)
    along = along_track_nm(position, leg_start, leg_end)
    if along < 0:
        return distance_nm(position, leg_start)
    if along > length:
        return distance_nm(position, leg_end)
    return abs(cross_track_nm(position, leg_start, leg_end))


class Pavement:
    """Every taxiway centreline, on a grid, so "am I on the pavement" is cheap.

    Scanning all of them per sample is thousands of segments times thousands of
    samples per run. Bucketing by a grid cell a little larger than the longest
    segment turns it into a handful of candidates.
    """

    CELL_DEG = 0.004        # about 1450 ft of latitude

    def __init__(self, layout) -> None:
        self.cells: dict[tuple[int, int], list] = {}
        for path in layout.taxi_paths:
            if distance_nm(path.start, path.end) < 1e-9:
                continue
            for point in (path.start, path.end):
                key = (int(point.lat / self.CELL_DEG),
                       int(point.lon / self.CELL_DEG))
                self.cells.setdefault(key, []).append((path.start, path.end))

    def distance_ft(self, position) -> float:
        lat = int(position.lat / self.CELL_DEG)
        lon = int(position.lon / self.CELL_DEG)
        best = 1e9
        for dlat in (-1, 0, 1):
            for dlon in (-1, 0, 1):
                for a, b in self.cells.get((lat + dlat, lon + dlon), ()):
                    d = _deviation(position, a, b)
                    if d < best:
                        best = d
        return best * FEET_PER_NM


def run_one(navdata, network, origin, destination, stand, runway_ident,
            aircraft: str = "b787-9", limit_s: int = 1500,
            pavement: "Pavement | None" = None) -> Outcome:
    """Push back and taxi from one stand to one runway, and score it."""
    profile = get_profile(aircraft)
    try:
        plan = plan_route(origin, destination, profile, navdata,
                          departure_runway=runway_ident)
    except Exception as exc:                    # noqa: BLE001 -- reported, not raised
        return Outcome(origin.icao, stand.name, runway_ident, False,
                       0.0, 0.0, 0.0, 0.0, 0.0, f"no plan: {exc}")

    sim = MockSim(stand.position, stand.heading_true_deg, origin.elevation_ft,
                  model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    adapter, _ = build_adapter(aircraft, sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(), ground=network)
    pilot.engage()

    start_heading = stand.heading_true_deg
    push_end_heading: Optional[float] = None
    taxi_open_heading: Optional[float] = None
    samples: list[float] = []
    worst = 0.0
    reversals, previous_sign, ground_s = 0, 0, 0.0
    reached = False

    for _ in range(limit_s):
        pilot.update(1.0)
        phase = pilot.phase
        if phase in (Phase.PUSHBACK, Phase.TAXI):
            ground_s += 1.0
            sign = (sim.steering > 0.05) - (sim.steering < -0.05)
            if sign and previous_sign and sign != previous_sign:
                reversals += 1
            if sign:
                previous_sign = sign
        if phase is Phase.TAXI:
            if push_end_heading is None:
                push_end_heading = sim.state.heading_true_deg
            elif taxi_open_heading is None and sim.state.ground_speed_kt > 2.0:
                # The heading it settles on once it is actually rolling: the
                # turn the taxi opens with, which is the one that undoes a push.
                taxi_open_heading = sim.state.heading_true_deg
            if pavement is not None:
                # Distance from the pavement, not from the polyline the program
                # drew for itself. A route that is off the taxiway scores
                # perfectly against itself while the aeroplane is on the grass,
                # which is how this sweep reported eight stands out of eight
                # through a taxi spending a third of its time off the taxiway.
                deviation = pavement.distance_ft(sim.state.position)
                samples.append(deviation)
                worst = max(worst, deviation)
        if phase in (Phase.TAKEOFF, Phase.CLIMB):
            reached = True
            break
        if phase is Phase.ABORTED:
            break

    push_turn = (abs(signed_diff_deg(push_end_heading, start_heading))
                 if push_end_heading is not None else 0.0)
    open_turn = (abs(signed_diff_deg(taxi_open_heading, push_end_heading))
                 if taxi_open_heading is not None and push_end_heading is not None
                 else 0.0)

    off = (sum(1 for d in samples if d > HALF_TAXIWAY_FT) / len(samples)
           if samples else 0.0)
    return Outcome(
        airport=origin.icao, stand=stand.name, runway=runway_ident,
        reached=reached, rotation_deg=push_turn + open_turn,
        typical_ft=statistics.median(samples) if samples else 0.0,
        worst_ft=worst, off_pavement=off,
        reversals_per_min=(reversals / ground_s * 60.0) if ground_s > 0 else 0.0,
        seconds=ground_s)


def sweep(icaos: list[str], runway: Optional[str], stands: int,
          aircraft: str = "b787-9", destination_icao: str = "KIAD") -> list[Outcome]:
    navdata = build_navdata(NavDataSources())
    destination = navdata.airport(destination_icao)
    outcomes: list[Outcome] = []
    for icao in icaos:
        origin = navdata.airport(icao)
        if origin is None:
            print(f"{icao}: not in the nav data")
            continue
        layout = navdata.ground_layout(icao)
        network = build_network(layout) if layout is not None else None
        if network is None:
            print(f"{icao}: no taxiway data")
            continue
        pavement = Pavement(layout)
        idents = [runway] if runway else [r.ident for r in origin.runways[:2]]
        for stand in list(layout.parking)[:stands]:
            for ident in idents:
                outcomes.append(run_one(navdata, network, origin, destination,
                                        stand, ident, aircraft,
                                        pavement=pavement))
    return outcomes


def report(outcomes: list[Outcome]) -> int:
    if not outcomes:
        print("nothing ran")
        return 1
    print(f"{'airport':<8}{'stand':<12}{'rwy':<5}{'ok':<4}"
          f"{'rot':>6}{'typ ft':>8}{'worst':>7}{'off pav':>9}{'rev/min':>9}  why")
    for outcome in outcomes:
        print(f"{outcome.airport:<8}{outcome.stand[:11]:<12}{outcome.runway:<5}"
              f"{'yes' if outcome.ok else 'NO':<4}{outcome.rotation_deg:6.0f}"
              f"{outcome.typical_ft:8.0f}{outcome.worst_ft:7.0f}"
              f"{outcome.off_pavement:8.0%} {outcome.reversals_per_min:9.1f}  "
              f"{'' if outcome.ok else outcome.why}")
    good = [o for o in outcomes if o.ok]
    print(f"\n{len(good)}/{len(outcomes)} acceptable")
    for name, values in (
            ("rotation", [o.rotation_deg for o in outcomes]),
            ("typical ft", [o.typical_ft for o in outcomes if o.reached]),
            ("worst ft", [o.worst_ft for o in outcomes if o.reached]),
            ("off pavement %", [o.off_pavement * 100 for o in outcomes if o.reached]),
            ("rev/min", [o.reversals_per_min for o in outcomes if o.reached])):
        if values:
            print(f"  {name:<11} median {statistics.median(values):7.1f}   "
                  f"max {max(values):7.1f}")
    return 0 if len(good) == len(outcomes) else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure the ground phase across many real stands.")
    parser.add_argument("airports", nargs="+", help="ICAO codes")
    parser.add_argument("--runway", default=None,
                        help="force one departure runway, e.g. 22R")
    parser.add_argument("--stands", type=int, default=10,
                        help="how many stands per airport (default 10)")
    parser.add_argument("--aircraft", default="b787-9")
    args = parser.parse_args()
    return report(sweep(args.airports, args.runway, args.stands, args.aircraft))


if __name__ == "__main__":
    raise SystemExit(main())
