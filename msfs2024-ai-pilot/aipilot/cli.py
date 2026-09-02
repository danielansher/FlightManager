"""Command line interface.

    python -m aipilot fly EGLL KJFK --aircraft b787-10
    python -m aipilot fly EGLL EGCC --sim mock --speed 60
    python -m aipilot plan OMDB WSSS --aircraft a380-800
    python -m aipilot ui
    python -m aipilot doctor
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

from .aircraft.registry import available_aircraft, build_adapter, resolve_key
from .autopilot.controller import AIPilot, PilotOptions, PilotStatus
from .autopilot.phases import FlightEvent, Phase
from .geo import distance_nm
from .navdata.base import NavDataProvider
from .navdata.resolve import NavDataSources, build_navdata
from .perf.profiles import get_profile, load_profile_overrides
from .route.planner import plan_route
from .sim.base import SimBackend, SimBackendError

DEFAULT_RATE_HZ = 4.0


# --- Argument parsing --------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    from . import __version__

    parser = argparse.ArgumentParser(
        prog="aipilot",
        description="An AI Pilot for Microsoft Flight Simulator: type two "
                    "airports, and it flies the aeroplane there.",
    )
    parser.add_argument("--version", action="version",
                        version=f"AI Pilot {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_common(sub: argparse.ArgumentParser) -> None:
        sub.add_argument("origin", help="departure airport ICAO code, e.g. EGLL")
        sub.add_argument("destination", help="arrival airport ICAO code, e.g. KJFK")
        sub.add_argument("-a", "--aircraft", default="b787-10",
                         help="aircraft key or alias (default: b787-10)")
        sub.add_argument("--departure-runway", help="force a departure runway, e.g. 27R")
        sub.add_argument("--arrival-runway", help="force an arrival runway, e.g. 04L")
        sub.add_argument("--cruise", type=float,
                         help="cruise altitude in feet, or a flight level under 500")
        sub.add_argument("--route", help='enroute fixes, e.g. "MID DVR KONAN"')
        sub.add_argument("--wind", help="planning wind as DIR/SPEED, e.g. 250/35")
        sub.add_argument("--msfs", choices=("2020", "2024"), default=None,
                         help="which simulator you are flying, when both are "
                              "installed; decides which Little Navmap database to use")
        sub.add_argument("--navdata", help="path to a Little Navmap sqlite database")
        sub.add_argument("--airports-csv", help="path to an OurAirports airports.csv")
        sub.add_argument("--runways-csv", help="path to an OurAirports runways.csv")
        sub.add_argument("--profiles", help="JSON file of performance overrides")

    fly = subparsers.add_parser("fly", help="plan a flight and fly it")
    add_common(fly)
    fly.add_argument("--sim", choices=("msfs", "mock"), default="msfs",
                     help="msfs flies the real simulator; mock flies offline (default: msfs)")
    fly.add_argument("--speed", type=float, default=1.0,
                     help="time multiplier for the mock simulator (default: 1)")
    fly.add_argument("--autoland", choices=("auto", "ils", "handover"), default="auto",
                     help="auto lands every approach; ils only where there is an ILS; "
                          "handover always gives the landing back to you")
    fly.add_argument("--airborne", action="store_true",
                     help="the aeroplane is already flying when you engage")
    fly.add_argument("--no-lights", action="store_true",
                     help="do not touch the lights or cabin signs")
    fly.add_argument("--no-taxi", action="store_true",
                     help="do not push back or taxi; wait to be lined up")
    fly.add_argument("--no-config", action="store_true",
                     help="do not move the gear or flaps")
    fly.add_argument("--no-go-around", action="store_true",
                     help="continue an unstable approach instead of going around")
    fly.add_argument("--quiet", action="store_true", help="log only, no status line")

    plan = subparsers.add_parser("plan", help="print the flight plan and stop")
    add_common(plan)

    ui = subparsers.add_parser("ui", help="serve the web control panel")
    ui.add_argument("--port", type=int, default=8711)
    ui.add_argument("--host", default="127.0.0.1")
    ui.add_argument("--open", action="store_true", help="open a browser window")

    subparsers.add_parser("aircraft", help="list the aeroplanes it knows how to fly")

    find = subparsers.add_parser(
        "find-simconnect",
        help="search this machine for SimConnect.dll, which several tools you "
             "may already have will have installed")
    find.add_argument("--copy", action="store_true",
                      help="copy the first one found into place automatically")
    find.add_argument("--quick", action="store_true",
                      help="only check the usual locations; do not scan drives")
    find.add_argument("--seconds", type=float, default=90.0,
                      help="how long to spend scanning (default: 90)")

    doctor = subparsers.add_parser(
        "doctor", help="check the simulator connection, the WASM bridge and the nav data")
    doctor.add_argument("--msfs", choices=("2020", "2024"), default=None,
                        help="which simulator to check the navigation data for")
    doctor.add_argument("--navdata", help="path to a Little Navmap sqlite database")
    doctor.add_argument("--airports-csv", help="path to an OurAirports airports.csv")
    doctor.add_argument("--runways-csv", help="path to an OurAirports runways.csv")

    lvars = subparsers.add_parser(
        "lvars", help="discover an aircraft's local variables through the WASM bridge")
    lvars.add_argument("names", nargs="*", help="specific variables to watch")
    lvars.add_argument("--seconds", type=float, default=20.0,
                       help="how long to watch for (default: 20)")

    return parser


def _wind(text: Optional[str]) -> tuple[float, float]:
    if not text:
        return (0.0, 0.0)
    try:
        direction, speed = text.replace("@", "/").split("/", 1)
        return (float(direction) % 360.0, abs(float(speed)))
    except ValueError:
        raise SystemExit(f"Could not read the wind {text!r}. Use DIRECTION/SPEED, e.g. 250/35.")


def _cruise_altitude(value: Optional[float]) -> Optional[float]:
    """Accept either feet or a flight level."""
    if value is None:
        return None
    return value * 100.0 if value < 500 else value


def _navdata_from_args(args) -> NavDataProvider:
    return build_navdata(NavDataSources(
        littlenavmap_db=getattr(args, "navdata", None),
        airports_csv=getattr(args, "airports_csv", None),
        runways_csv=getattr(args, "runways_csv", None),
        msfs_version=getattr(args, "msfs", None),
    ))


# --- Shared setup ------------------------------------------------------------
def _prepare_flight(args):
    """Resolve the aircraft, the nav data and the route. Shared by fly and plan."""
    if getattr(args, "profiles", None):
        changed = load_profile_overrides(args.profiles)
        print(f"Applied performance overrides for: {', '.join(changed) or 'nothing'}")

    key = resolve_key(args.aircraft)
    if key is None:
        print(f"Unknown aircraft {args.aircraft!r}. Known aeroplanes:", file=sys.stderr)
        for name, label in available_aircraft():
            print(f"  {name:<12} {label}", file=sys.stderr)
        raise SystemExit(2)
    profile = get_profile(key)
    assert profile is not None

    navdata = _navdata_from_args(args)
    if not navdata.available:
        raise SystemExit("No navigation data at all. See docs/INSTALL.md.")

    origin = navdata.airport(args.origin)
    destination = navdata.airport(args.destination)
    if origin is None:
        raise SystemExit(f"{args.origin.upper()} is not in the navigation data "
                         f"({navdata.describe()}).")
    if destination is None:
        raise SystemExit(f"{args.destination.upper()} is not in the navigation data "
                         f"({navdata.describe()}).")

    wind_from, wind_kt = _wind(args.wind)
    plan = plan_route(
        origin, destination, profile, navdata,
        departure_runway=args.departure_runway,
        arrival_runway=args.arrival_runway,
        cruise_altitude_ft=_cruise_altitude(args.cruise),
        route=args.route,
        wind_from_deg=wind_from, wind_kt=wind_kt,
    )
    return key, profile, navdata, plan, (wind_from, wind_kt)


# --- Commands ----------------------------------------------------------------
def command_plan(args) -> int:
    key, profile, navdata, plan, _ = _prepare_flight(args)
    from .route.profile import build_vertical_profile

    vertical = build_vertical_profile(
        plan.cruise_altitude_ft,
        plan.arrival_runway.elevation_ft if plan.arrival_runway
        else plan.destination.elevation_ft,
        profile,
    )
    print(f"Navigation data: {navdata.describe()}")
    print(f"Aircraft:        {profile.name}")
    print()
    print(plan.describe())
    print()
    print(f"Top of descent   {vertical.top_of_descent_nm:.0f} nm to run "
          f"({vertical.effective_angle_deg:.1f} degrees to the final approach fix)")
    cruise_tas = profile.cruise_mach * 573.0
    print(f"Rough block time {plan.total_distance_nm / max(cruise_tas, 1) :.1f} h "
          "in still air at cruise Mach")
    for warning in plan.warnings:
        print(f"  ! {warning}")
    navdata.close()
    return 0


def command_aircraft(args) -> int:
    print(f"{'key':<12} {'type':<6} {'cruise':<8} {'ceiling':<9} name")
    for key, label in available_aircraft():
        profile = get_profile(key)
        if profile is None:
            continue
        print(f"{key:<12} {profile.icao_type:<6} M{profile.cruise_mach:<7.2f} "
              f"FL{profile.max_altitude_ft / 100:<7.0f} {label}")
    print()
    print("Any of these can be given to --aircraft, as can an ICAO type code "
          "(B78X, A388) or a short alias (787, a350, headwind).")
    return 0


def _build_sim(args, plan, wind) -> SimBackend:
    if args.sim == "mock":
        from .sim.mock import MockAircraftModel, MockSim

        profile = get_profile(resolve_key(args.aircraft) or "generic")
        assert profile is not None
        runway = plan.departure_runway
        origin, destination = plan.origin, plan.destination

        def terrain(position):
            near = distance_nm(position, origin.position)
            far = distance_nm(position, destination.position)
            total = near + far
            if total < 1e-6:
                return origin.elevation_ft
            return (origin.elevation_ft * far + destination.elevation_ft * near) / total

        start = runway.threshold if runway else origin.position
        heading = runway.heading_true_deg if runway else 0.0
        return MockSim(
            start, heading, origin.elevation_ft,
            model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
            terrain=terrain, wind_from_deg=wind[0], wind_kt=wind[1],
            start_airborne_at_ft=plan.cruise_altitude_ft if args.airborne else None,
        )

    from .sim.mobiflight import MobiFlightBridge
    from .sim.simconnect import SimConnectBackend

    backend = SimConnectBackend()
    backend.connect()
    backend.attach_lvar_bridge(MobiFlightBridge())
    return backend


def command_fly(args) -> int:
    key, profile, navdata, plan, wind = _prepare_flight(args)
    print(f"Navigation data: {navdata.describe()}")
    print(plan.describe())
    print()

    try:
        sim = _build_sim(args, plan, wind)
    except SimBackendError as exc:
        print(f"Could not connect to the simulator: {exc}", file=sys.stderr)
        return 1

    adapter, _ = build_adapter(key, sim)
    ground = arrival_ground = None
    if not args.no_taxi:
        ground, arrival_ground = _ground_networks(navdata, plan, print)
    options = PilotOptions(
        autoland=args.autoland,
        manage_configuration=not args.no_config,
        manage_lights=not args.no_lights,
        go_around_if_unstable=not args.no_go_around,
        start_airborne=args.airborne,
        taxi=not args.no_taxi,
    )
    printer = _Printer(quiet=args.quiet)
    pilot = AIPilot(sim, adapter, profile, plan, options,
                    listener=printer.on_event, ground=ground,
                    arrival_ground=arrival_ground)

    if args.sim == "msfs":
        if not _wait_for_data(sim):
            print("Connected to SimConnect, but no flight data is arriving. Load a "
                  "flight and try again.", file=sys.stderr)
            return 1

    pilot.engage()
    # The control step stays fixed whatever the time multiplier. Speeding up by
    # taking bigger steps would be speeding up by flying worse: at a hundred
    # seconds a step the guidance is being asked to fly an aeroplane it looks
    # at once every two minutes, and it flies it into the sea. Acceleration
    # comes from waiting less between steps instead.
    period = 1.0 / DEFAULT_RATE_HZ
    speed = max(0.1, args.speed) if args.sim == "mock" else 1.0
    try:
        while pilot.phase not in (Phase.COMPLETE, Phase.ABORTED):
            started = time.monotonic()
            status = pilot.update(period)
            printer.on_status(status)
            remaining = period / speed - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        printer.finish()
        print("\nInterrupted. The autopilot is left as it is -- take over manually.")
        return 130
    finally:
        sim.close()
        navdata.close()
    printer.finish()
    return 0


def _ground_networks(navdata, plan, report):
    """Taxiway networks for both ends, so the taxi happens without being asked.

    Both, because arriving is half the job: an AI Pilot that lands and then
    abandons the aeroplane on the runway has not finished.
    """
    from .route.taxi import build_network

    networks = []
    for airport, role in ((plan.origin, "departure"), (plan.destination, "arrival")):
        network = build_network(navdata.ground_layout(airport.icao))
        if network is None:
            report(f"No taxiway data for {airport.icao}, so the {role} taxi is "
                   "yours. Everything else still runs.")
        else:
            stands = len(getattr(network.layout, "parking", ()) or ())
            report(f"{airport.icao}: {len(network.nodes)} taxiway junctions, "
                   f"{stands} stands.")
        networks.append(network)
    return networks[0], networks[1]


class _Printer:
    """Flight log above, a single live status line below."""

    def __init__(self, quiet: bool = False) -> None:
        self.quiet = quiet
        self._line_open = False
        self._last_status = 0.0

    def on_event(self, event: FlightEvent) -> None:
        self._clear()
        marker = {"warning": "!", "error": "!!"}.get(event.level, " ")
        print(f"{marker} {event}")

    def on_status(self, status: PilotStatus) -> None:
        if self.quiet:
            return
        now = time.monotonic()
        if now - self._last_status < 0.5:
            return
        self._last_status = now
        speed = (f"M{status.target_speed:.2f}" if status.target_speed_is_mach
                 else f"{status.target_speed:.0f}kt")
        line = (
            f"  {status.phase.label:<9} "
            f"{status.altitude_ft:6.0f}ft "
            f"{status.ias_kt:3.0f}kt "
            f"{status.vertical_speed_fpm:+5.0f}fpm  "
            f"-> {status.active_waypoint:<10} "
            f"{status.distance_to_destination_nm:6.1f}nm to run  "
            f"ETA {status.eta_text}  "
            f"tgt {speed}"
        )
        sys.stdout.write("\r" + line[:150].ljust(150))
        sys.stdout.flush()
        self._line_open = True

    def _clear(self) -> None:
        if self._line_open:
            sys.stdout.write("\r" + " " * 150 + "\r")
            self._line_open = False

    def finish(self) -> None:
        self._clear()


def _wait_for_data(sim, timeout: float = 8.0) -> bool:
    """Give SimConnect a moment to start streaming before we command anything."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sim.poll(0.0)
        if getattr(sim, "receiving_data", True):
            return True
        time.sleep(0.1)
    return False


def command_ui(args) -> int:
    from .ui.server import serve

    return serve(host=args.host, port=args.port, open_browser=args.open)


def command_doctor(args) -> int:
    from .doctor import run_doctor

    return run_doctor(args)


def command_find_simconnect(args) -> int:
    from .findsim import run

    return run(args)


def command_lvars(args) -> int:
    from .doctor import run_lvars

    return run_lvars(args)


COMMANDS = {
    "fly": command_fly,
    "plan": command_plan,
    "ui": command_ui,
    "aircraft": command_aircraft,
    "doctor": command_doctor,
    "find-simconnect": command_find_simconnect,
    "lvars": command_lvars,
}


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return COMMANDS[args.command](args)
