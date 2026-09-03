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
    typical    median distance from the route it is following, in feet
    worst      the widest corner cut, in feet
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
MAX_TYPICAL_FT = 60.0
MAX_WORST_FT = 500.0
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
    note: str = ""

    @property
    def ok(self) -> bool:
        return (self.reached
                and self.typical_ft <= MAX_TYPICAL_FT
                and self.worst_ft <= MAX_WORST_FT
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
            reasons.append(f"{self.typical_ft:.0f} ft off route")
        if self.worst_ft > MAX_WORST_FT:
            reasons.append(f"cut {self.worst_ft:.0f} ft")
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


def run_one(navdata, network, origin, destination, stand, runway_ident,
            aircraft: str = "b787-9", limit_s: int = 1500) -> Outcome:
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
            if pilot.taxi is not None and pilot.taxi.index >= 1 \
                    and not pilot.taxi.finished:
                route = pilot.taxi.route
                deviation = _deviation(sim.state.position,
                                       route[pilot.taxi.index - 1],
                                       route[pilot.taxi.index])
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

    return Outcome(
        airport=origin.icao, stand=stand.name, runway=runway_ident,
        reached=reached, rotation_deg=push_turn + open_turn,
        typical_ft=(statistics.median(samples) * FEET_PER_NM) if samples else 0.0,
        worst_ft=worst * FEET_PER_NM,
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
        idents = [runway] if runway else [r.ident for r in origin.runways[:2]]
        for stand in list(layout.parking)[:stands]:
            for ident in idents:
                outcomes.append(run_one(navdata, network, origin, destination,
                                        stand, ident, aircraft))
    return outcomes


def report(outcomes: list[Outcome]) -> int:
    if not outcomes:
        print("nothing ran")
        return 1
    print(f"{'airport':<8}{'stand':<12}{'rwy':<5}{'ok':<4}"
          f"{'rot':>6}{'typ ft':>8}{'worst':>7}{'rev/min':>9}  why")
    for outcome in outcomes:
        print(f"{outcome.airport:<8}{outcome.stand[:11]:<12}{outcome.runway:<5}"
              f"{'yes' if outcome.ok else 'NO':<4}{outcome.rotation_deg:6.0f}"
              f"{outcome.typical_ft:8.0f}{outcome.worst_ft:7.0f}"
              f"{outcome.reversals_per_min:9.1f}  "
              f"{'' if outcome.ok else outcome.why}")
    good = [o for o in outcomes if o.ok]
    print(f"\n{len(good)}/{len(outcomes)} acceptable")
    for name, values in (
            ("rotation", [o.rotation_deg for o in outcomes]),
            ("typical ft", [o.typical_ft for o in outcomes if o.reached]),
            ("worst ft", [o.worst_ft for o in outcomes if o.reached]),
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
