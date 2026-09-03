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
from .briefing import resolve_winds, wind_from_sim
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
        sub.add_argument("origin", nargs="?",
                         help="departure airport ICAO code, e.g. EGLL "
                              "(optional with --simbrief)")
        sub.add_argument("destination", nargs="?",
                         help="arrival airport ICAO code, e.g. KJFK "
                              "(optional with --simbrief)")
        sub.add_argument("-a", "--aircraft", default="b787-10",
                         help="aircraft key or alias (default: b787-10)")
        sub.add_argument("--departure-runway", help="force a departure runway, e.g. 27R")
        sub.add_argument("--arrival-runway", help="force an arrival runway, e.g. 04L")
        sub.add_argument("--cruise", type=float,
                         help="cruise altitude in feet, or a flight level under 500")
        sub.add_argument("--route", help='enroute fixes, e.g. "MID DVR KONAN"')
        sub.add_argument("--wind",
                         help="planning wind as DIR/SPEED, e.g. 250/35. Without "
                              "this the current METAR at each airport is used")
        sub.add_argument("--simbrief", metavar="USER",
                         help="take the route and runways from your latest "
                              "SimBrief flight plan (username or pilot ID)")
        sub.add_argument("--no-metar", action="store_true",
                         help="do not look up the weather; plan as if calm")
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
    fly.add_argument("--debug", action="store_true",
                     help="record a flight trace to logs/, for diagnosing a "
                          "problem afterwards or sending to someone")
    fly.add_argument("--debug-file", metavar="PATH",
                     help="write the trace here instead of logs/")

    plan = subparsers.add_parser("plan", help="print the flight plan and stop")
    add_common(plan)

    ui = subparsers.add_parser("ui", help="serve the web control panel")
    ui.add_argument("--port", type=int, default=8711)
    ui.add_argument("--host", default="127.0.0.1")
    ui.add_argument("--open", action="store_true", help="open a browser window")

    subparsers.add_parser("aircraft", help="list the aeroplanes it knows how to fly")

    taxi = subparsers.add_parser(
        "taxi",
        help="print the taxi route across an airport, without the simulator "
             "running -- the taxiway data is on disk, so this needs no flight")
    taxi.add_argument("airport", help="ICAO code, e.g. KJFK")
    taxi.add_argument("--stand", help="stand to start from (default: the first)")
    taxi.add_argument("--runway", help="runway to taxi to (default: the longest)")
    taxi.add_argument("--msfs", choices=("2020", "2024"), default=None)
    taxi.add_argument("--navdata", help="path to a Little Navmap sqlite database")
    taxi.add_argument("--stands", action="store_true",
                      help="list the stands and stop")

    report = subparsers.add_parser(
        "debug-report",
        help="summarise a flight trace recorded with --debug, and say what "
             "looks wrong")
    report.add_argument("path", help="the .jsonl file written by --debug")
    report.add_argument("--events", action="store_true",
                        help="also print the whole event log")
    report.add_argument("--track", nargs="*", metavar="PHASE",
                        help="replay how the aeroplane moved, cycle by cycle: "
                             "heading, commanded heading, track, speed and the "
                             "rudder that was sent. Name phases to narrow it, "
                             'e.g. --track taxi takeoff')

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
    doctor.add_argument("--airport", metavar="ICAO",
                        help="check one airport in particular: its runways, "
                             "its ILS, and whether it has the taxiway data "
                             "needed to push back and taxi")
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
def _simbrief_plan(args, report):
    """Fetch the SimBrief plan, if one was asked for.

    A failure here stops the flight rather than being shrugged off: if you
    asked to fly your SimBrief plan, quietly flying a different route
    instead is not a helpful thing to do.
    """
    if not getattr(args, "simbrief", None):
        return None
    from .simbrief import SimBriefError, fetch_plan

    try:
        plan = fetch_plan(args.simbrief)
    except SimBriefError as exc:
        raise SystemExit(f"{exc}")
    report(plan.describe())
    for note in plan.notes:
        report(f"  ! {note}")
    return plan


def _apply_simbrief(args, brief) -> None:
    """Fill in anything you did not type from the SimBrief plan.

    Typed arguments always win. The SimBrief plan is a default, not an
    override: asking for a different runway on the command line has to
    mean something.
    """
    if brief is None:
        return
    if not args.origin:
        args.origin = brief.origin
    if not args.destination:
        args.destination = brief.destination
    if not args.departure_runway:
        args.departure_runway = brief.departure_runway
    if not args.arrival_runway:
        args.arrival_runway = brief.arrival_runway
    if not args.route and brief.route:
        args.route = brief.route
    if args.cruise is None and brief.cruise_altitude_ft:
        # _cruise_altitude reads anything under 500 as a flight level, and
        # SimBrief gives feet, so hand it feet it cannot misread.
        args.cruise = max(500.0, brief.cruise_altitude_ft)


def _prepare_flight(args, report=print):
    """Resolve the aircraft, the nav data and the route. Shared by fly and plan."""
    if getattr(args, "profiles", None):
        report(f"Applied performance overrides for: "
               f"{', '.join(load_profile_overrides(args.profiles)) or 'nothing'}")

    brief = _simbrief_plan(args, report)
    _apply_simbrief(args, brief)

    if not args.origin or not args.destination:
        raise SystemExit(
            "Give a departure and an arrival airport, e.g. `fly KJFK EGLL`, or "
            "use --simbrief to take them from your SimBrief flight plan."
        )

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

    winds = resolve_winds(
        origin.icao,
        destination.icao,
        typed=_wind(args.wind) if args.wind else None,
        use_metar=not getattr(args, "no_metar", False),
        simbrief_metars=(brief.origin_metar, brief.destination_metar) if brief else None,
        report=report,
    )

    plan = plan_route(
        origin, destination, profile, navdata,
        departure_runway=args.departure_runway,
        arrival_runway=args.arrival_runway,
        cruise_altitude_ft=_cruise_altitude(args.cruise),
        route=args.route,
        departure_wind=winds.departure,
        arrival_wind=winds.arrival,
    )
    # The mock simulator flies in the wind it is given, and the departure
    # wind is the one it starts in.
    return key, profile, navdata, plan, (winds.departure.from_deg,
                                         winds.departure.speed_kt)


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
    for note in plan.runway_notes:
        print(f"  {note}")
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
    for note in plan.runway_notes:
        print(f"  {note}")
    print()

    try:
        sim = _build_sim(args, plan, wind)
    except SimBackendError as exc:
        print(f"Could not connect to the simulator: {exc}", file=sys.stderr)
        return 1

    recorder = _start_recorder(args)
    if recorder is not None:
        from .debug import RecordingBackend

        sim = RecordingBackend(sim, recorder)

    adapter, _ = build_adapter(key, sim)

    if args.sim == "msfs":
        if not _wait_for_data(sim):
            print("Connected to SimConnect, but no flight data is arriving. Load a "
                  "flight and try again.", file=sys.stderr)
            sim.close()
            navdata.close()
            return 1
        _match_departure_to_sim_wind(sim, args, plan, profile, print)

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

    def on_event(event) -> None:
        printer.on_event(event)
        if recorder is not None:
            recorder.event(event.time_s, event.phase.value, event.message,
                           event.level)

    pilot = AIPilot(sim, adapter, profile, plan, options,
                    listener=on_event, ground=ground,
                    arrival_ground=arrival_ground)
    if recorder is not None:
        _write_header(recorder, args, key, profile, navdata, plan, options,
                      sim, ground, arrival_ground)

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
            if recorder is not None:
                recorder.sample(pilot.elapsed_s, sim.poll(0.0), status)
            remaining = period / speed - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        printer.finish()
        print("\nInterrupted. The autopilot is left as it is -- take over manually.")
        return 130
    finally:
        if recorder is not None:
            recorder.finish(phase=pilot.phase.value,
                            reason=pilot.status.message,
                            elapsed_s=round(pilot.elapsed_s, 1))
            print(f"\nFlight trace written to {recorder.path}")
            print(f"Summarise it with:  python -m aipilot debug-report "
                  f"{recorder.path}")
        elif pilot.phase is not Phase.COMPLETE:
            # It did not finish, and there is nothing to look at afterwards.
            # Say so now rather than leaving the next attempt just as blind.
            print("\nThat did not finish. Run it again with --debug and it "
                  "will record what happened:")
            print(f"  python -m aipilot fly {args.origin} {args.destination} "
                  "--debug")
        sim.close()
        navdata.close()
    printer.finish()
    return 0


def _match_departure_to_sim_wind(sim, args, plan, profile, report) -> None:
    """Depart into the wind the simulator actually has.

    A METAR is an observation of the real world an hour ago. The simulator
    might be running live weather, a preset, or a date in 1997, and only it
    knows which. Since the aeroplane is sitting at the departure airport
    with the sim already telling us the wind, use that -- it is not a
    forecast, it is the wind we are about to take off into.

    Only the departure end: the wind at the destination is hours away and
    the simulator has nothing to say about it yet.
    """
    if args.departure_runway or args.airborne or plan.departure_runway is None:
        return

    from .geo import distance_nm
    from .navdata.base import select_runway
    from .route.planner import MIN_RUNWAY_FT, rebuild_departure

    state = sim.poll(0.0)
    if state is None:
        return
    wind = wind_from_sim(state)
    if wind is None or wind.speed_kt < 3.0:
        return
    # If we are not at the departure airport, the wind here says nothing
    # about the wind there.
    if distance_nm(state.position, plan.origin.position) > 30.0:
        return

    chosen = select_runway(plan.origin, wind.from_deg, wind.speed_kt,
                           MIN_RUNWAY_FT)
    if chosen is None or chosen.ident == plan.departure_runway.ident:
        return

    was = plan.departure_runway.ident
    rebuild_departure(plan, chosen, profile)
    report(f"  The simulator's wind is {wind.from_deg:03.0f} at "
           f"{wind.speed_kt:.0f} kt, so departing runway {chosen.ident} "
           f"instead of {was}.")


def _start_recorder(args):
    """Open a flight trace, if one was asked for."""
    if not getattr(args, "debug", False) and not getattr(args, "debug_file", None):
        return None
    from .debug import FlightRecorder, default_path

    path = getattr(args, "debug_file", None) or default_path(
        args.origin or "", args.destination or "")
    try:
        return FlightRecorder(path)
    except OSError as exc:
        print(f"Could not open a flight trace at {path}: {exc}", file=sys.stderr)
        return None


def _write_header(recorder, args, key, profile, navdata, plan, options, sim,
                  ground, arrival_ground) -> None:
    from . import __version__
    from .debug import redact

    dep = plan.departure_runway.ident if plan.departure_runway else "?"
    arr = plan.arrival_runway.ident if plan.arrival_runway else "?"
    recorder.header(
        version=__version__,
        aircraft=f"{key} ({profile.name})",
        vmo_kt=profile.vmo_kt,
        sim=redact(getattr(sim, "host_description", None) or sim.name),
        navdata=redact(navdata.describe()),
        route=(f"{plan.origin.icao}/{dep} -> {plan.destination.icao}/{arr}, "
               f"{plan.total_distance_nm:.0f} nm at "
               f"FL{plan.cruise_altitude_ft / 100:.0f}, {len(plan.legs)} legs"),
        runway_notes=list(plan.runway_notes),
        warnings=list(plan.warnings),
        ground=_ground_summary(ground, arrival_ground),
        options={
            "autoland": options.autoland,
            "taxi": options.taxi,
            "lights": options.manage_lights,
            "configuration": options.manage_configuration,
            "go_around": options.go_around_if_unstable,
            "airborne": options.start_airborne,
            "sim": args.sim,
            "speed": args.speed,
        },
    )


def _ground_summary(ground, arrival_ground) -> str:
    def describe(network, role):
        if network is None:
            return f"no {role} taxiway data"
        layout = getattr(network, "layout", None)
        stands = len(getattr(layout, "parking", ()) or ())
        paths = len(getattr(layout, "taxi_paths", ()) or ())
        return f"{role}: {paths} segments, {stands} stands"

    return "; ".join((describe(ground, "departure"),
                      describe(arrival_ground, "arrival")))


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


def command_debug_report(args) -> int:
    from .debug import analyse, format_report, format_track, read_records

    try:
        report = analyse(args.path)
    except OSError as exc:
        print(f"Could not read {args.path}: {exc}", file=sys.stderr)
        return 1
    if not report.header and not report.events:
        print(f"{args.path} does not look like a flight trace.", file=sys.stderr)
        return 1

    print(format_report(report))
    if args.track is not None:
        print(format_track(report, read_records(args.path), tuple(args.track)))
    if args.events:
        print()
        print("Event log")
        print("---------")
        for event in report.events:
            minutes, seconds = divmod(int(event.get("at", 0)), 60)
            mark = {"warning": " !", "error": "!!"}.get(event.get("level"), "  ")
            print(f" {mark} [{minutes:02d}:{seconds:02d}] "
                  f"{event.get('phase', ''):<9} {event.get('message', '')}")
    # A trace with something wrong in it is worth a non-zero exit, so this can
    # be used in a script.
    return 1 if any(f.severity == "error" for f in report.findings) else 0


def command_taxi(args) -> int:
    """Show the route across an airport, and how sharp its turns are.

    The taxiway data lives in a database on disk, so none of this needs the
    simulator to be running -- which makes a ground-handling problem something
    that can be looked at between flights rather than only during one.
    """
    from .geo import distance_nm, initial_bearing_deg, signed_diff_deg
    from .route.taxi import build_network, runway_entry_point, simplify
    from .units import FEET_PER_NM

    navdata = _navdata_from_args(args)
    icao = args.airport.strip().upper()
    airport = navdata.airport(icao)
    if airport is None:
        raise SystemExit(f"{icao} is not in the navigation data "
                         f"({navdata.describe()}).")
    layout = navdata.ground_layout(icao)
    network = build_network(layout) if layout is not None else None
    if network is None or not network.usable:
        raise SystemExit(
            f"No taxiway data for {icao}. Taxiways come only from Little "
            "Navmap's scenery database; see docs/GROUND.md.")

    stands = list(layout.parking)
    if args.stands:
        print(f"{icao}: {len(stands)} stands")
        for stand in stands:
            print(f"  {stand.name:<10} {stand.kind:<10} {stand.position}")
        navdata.close()
        return 0

    stand = None
    if args.stand:
        stand = next((s for s in stands
                      if s.name.upper() == args.stand.strip().upper()), None)
        if stand is None:
            raise SystemExit(f"{icao} has no stand {args.stand!r}. "
                             "Use --stands to list them.")
    elif stands:
        stand = stands[0]
    if stand is None:
        raise SystemExit(f"No stands at {icao}.")

    runway = (airport.runway(args.runway) if args.runway
              else max(airport.runways, key=lambda r: r.length_ft, default=None))
    if runway is None:
        raise SystemExit(f"No runway data for {icao}.")

    print(f"{icao}: {len(layout.taxi_paths)} segments, "
          f"{len(network.nodes)} junctions, {len(stands)} stands")
    print(f"From stand {stand.name} to runway {runway.ident}")
    route = network.route(stand.position, runway_entry_point(runway, network))
    if not route:
        raise SystemExit("No route across the taxiways between those two.")
    reduced = simplify(route)
    raw_len = sum(distance_nm(a, b) for a, b in zip(route, route[1:]))
    print(f"  {len(route)} points, simplified to {len(reduced)}, "
          f"{raw_len:.2f} nm")
    print()
    print("   #   leg_ft   turn  heading  position")
    sharp = 0
    for index, point in enumerate(reduced):
        leg_ft = (distance_nm(reduced[index - 1], point) * FEET_PER_NM
                  if index else 0.0)
        heading = (initial_bearing_deg(reduced[index - 1], point)
                   if index else 0.0)
        turn = 0.0
        if 0 < index < len(reduced) - 1:
            turn = signed_diff_deg(
                initial_bearing_deg(point, reduced[index + 1]), heading)
        if abs(turn) > 30.0:
            sharp += 1
        print(f"  {index:2d} {leg_ft:8.0f} {turn:+6.0f} {heading:8.0f}  {point}")
    print()
    print(f"  {sharp} turns sharper than 30 degrees.")
    tight = [i for i in range(1, len(reduced))
             if distance_nm(reduced[i - 1], reduced[i]) * FEET_PER_NM < 150]
    if tight:
        print(f"  {len(tight)} legs shorter than 150 ft -- a route made of "
              "micro-segments zig-zags, because each one is shorter than the "
              "aeroplane.")
    navdata.close()
    return 0


COMMANDS = {
    "fly": command_fly,
    "taxi": command_taxi,
    "debug-report": command_debug_report,
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
