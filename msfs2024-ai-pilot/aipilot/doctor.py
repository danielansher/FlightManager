"""Diagnostics: what is connected, what works, and what does not.

``doctor`` exists because the failure modes of a program that talks to a flight
simulator are all invisible from the outside. A command that silently does
nothing looks exactly like a command that worked, and the aeroplane carries on
regardless. So rather than guessing, this checks each layer in turn and says
plainly which of them is actually working.

``lvars`` exists because of the honest gap in this project: an add-on's
autoflight panel lives in local variables whose names are not published
anywhere, and guessing them would produce commands that do nothing. So instead
of guessing, this dumps what the aeroplane really has, and lets you find the
right name by moving the knob and watching which value changes.
"""

from __future__ import annotations

import platform
import sys
import time

from .navdata.resolve import NavDataSources, build_navdata
from .sim.base import SimBackendError

TICK = "  [ ok ]"
WARN = "  [warn]"
FAIL = "  [fail]"


def _heading(text: str) -> None:
    print()
    print(text)
    print("-" * len(text))


def run_doctor(args) -> int:
    print("AI Pilot diagnostics")
    problems = 0

    _heading("Environment")
    print(f"{TICK} Python {platform.python_version()} on {platform.system()}")
    if platform.system() != "Windows":
        print(f"{WARN} Not Windows, so the simulator cannot be reached from here. "
              "Everything except the live connection can still be checked, and "
              "'--sim mock' will fly the whole flight offline.")

    problems += _check_navdata(args)
    problems += _check_simconnect()

    _heading("Summary")
    if problems == 0:
        print("Everything checks out.")
    else:
        print(f"{problems} thing(s) need attention. See docs/INSTALL.md.")
    return 0 if problems == 0 else 1


def _check_navdata(args) -> int:
    _heading("Navigation data")
    navdata = build_navdata(NavDataSources(
        littlenavmap_db=getattr(args, "navdata", None),
        airports_csv=getattr(args, "airports_csv", None),
        runways_csv=getattr(args, "runways_csv", None),
        msfs_version=getattr(args, "msfs", None),
    ))
    if not navdata.available:
        print(f"{FAIL} No navigation data found at all.")
        return 1

    print(f"{TICK} Sources, in the order they are consulted: {navdata.describe()}")
    problems = 0
    sample = navdata.airport("EGLL") or navdata.airport("KJFK")
    if sample is None:
        print(f"{FAIL} Could not look up a major airport. The data may be unreadable.")
        return 1

    print(f"{TICK} Looked up {sample.icao} ({sample.name or 'unnamed'}) at "
          f"{sample.position}, elevation {sample.elevation_ft:.0f} ft")
    real = [r for r in sample.runways if r.surface != "synthetic"]
    if not real:
        print(f"{WARN} No real runway data. Approaches will be built to assumed "
              "runways that do not line up with the scenery. Install Little "
              "Navmap, or download the OurAirports runways.csv.")
        problems += 1
    else:
        print(f"{TICK} {len(real)} runway(s) at {sample.icao}: "
              + ", ".join(r.ident for r in real[:8]))
    with_ils = [r for r in sample.runways if r.has_ils]
    if with_ils:
        example = with_ils[0]
        print(f"{TICK} ILS data present, e.g. {example.ident} on "
              f"{example.ils_freq_mhz:.2f} MHz")
    else:
        print(f"{WARN} No ILS frequencies. The AI Pilot will fly its own computed "
              "approach path and land on that, rather than on the aeroplane's ILS "
              "receiver. It still lands; it is just less precise.")
        problems += 1
    navdata.close()
    return problems


def _check_simconnect() -> int:
    _heading("Simulator connection")
    from .sim.simconnect import SimConnectBackend, find_simconnect_dll

    if platform.system() != "Windows":
        print(f"{WARN} Skipped: SimConnect is Windows only.")
        return 0

    dll = find_simconnect_dll()
    if dll is None:
        print(f"{FAIL} SimConnect.dll not found. Install the MSFS SDK, copy the DLL "
              "next to the aipilot package, or set AIPILOT_SIMCONNECT_DLL.")
        return 1
    print(f"{TICK} SimConnect.dll at {dll}")

    backend = SimConnectBackend()
    try:
        backend.connect()
    except SimBackendError as exc:
        print(f"{FAIL} {exc}")
        return 1
    print(f"{TICK} Connected to {backend.host_description}")

    problems = 0
    state = None
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        state = backend.poll(0.0)
        if backend.receiving_data:
            break
        time.sleep(0.1)

    if not backend.receiving_data:
        print(f"{FAIL} Connected, but no flight data is arriving. Load a flight and "
              "leave it running, then try again.")
        problems += 1
    else:
        assert state is not None
        print(f"{TICK} Receiving data: at {state.position}, "
              f"{state.altitude_ft:.0f} ft, {state.ias_kt:.0f} kt, "
              f"{'on the ground' if state.on_ground else 'airborne'}")
        print(f"{TICK} Autopilot reads {'engaged' if state.ap_master else 'off'}; "
              f"flaps handle {state.flaps_index}; "
              f"gear {'down' if state.gear_down_pct > 95 else 'up'}")

    problems += _check_bridge(backend)
    if backend.exception_codes:
        print(f"{WARN} SimConnect reported exception code(s) "
              f"{sorted(set(backend.exception_codes))}. Some variables may be "
              "unavailable in this aircraft.")
    backend.close()
    return problems


def _check_bridge(backend) -> int:
    _heading("Local variable bridge (MobiFlight WASM module)")
    from .sim.mobiflight import MobiFlightBridge

    bridge = MobiFlightBridge()
    backend.attach_lvar_bridge(bridge)
    if not bridge.ready:
        print(f"{WARN} Could not open the client data channels: "
              f"{bridge.last_error or 'unknown reason'}")
        print("       This only matters for aeroplanes whose autoflight panel lives "
              "in local variables. The default 787 does not need it.")
        return 0
    print(f"{TICK} Client data channels created")

    bridge.ping()
    deadline = time.monotonic() + 4.0
    responses: list[str] = []
    while time.monotonic() < deadline:
        backend.poll(0.0)
        responses += bridge.drain_responses()
        if responses:
            break
        time.sleep(0.1)

    if not responses:
        print(f"{WARN} The module did not answer. Either it is not installed, or its "
              "protocol has changed since this was written.")
        print("       Install: https://github.com/MobiFlight/MobiFlight-WASM-Module")
        print("       If it is installed and this still fails, the protocol strings "
              "can be overridden -- see aipilot/sim/mobiflight.py.")
        return 1

    print(f"{TICK} The module answered: {responses[0][:80]}")
    bridge.open_client_channels()
    print(f"{TICK} Local variables can be read and written")
    return 0


def run_lvars(args) -> int:
    """Watch an aircraft's local variables, to find the ones that matter."""
    if platform.system() != "Windows":
        print("This needs the simulator, so it only works on Windows.", file=sys.stderr)
        return 1

    from .sim.mobiflight import MobiFlightBridge
    from .sim.simconnect import SimConnectBackend

    backend = SimConnectBackend()
    try:
        backend.connect()
    except SimBackendError as exc:
        print(f"Could not connect: {exc}", file=sys.stderr)
        return 1

    bridge = MobiFlightBridge()
    backend.attach_lvar_bridge(bridge)
    backend.poll(0.0)
    time.sleep(0.5)
    bridge.open_client_channels()

    names = [n.upper().removeprefix("L:") for n in args.names]
    if not names:
        print("No variable names given.")
        print()
        print("The way to find the one you want:")
        print("  1. Load the aeroplane and let it settle.")
        print("  2. Guess a few plausible names and pass them here, for example:")
        print("       python -m aipilot lvars A32NX_FCU_HDG_PULL AIRLINER_MCP_HDG")
        print("  3. Move the knob or press the button in the cockpit while this runs.")
        print("  4. Whichever value changes is the variable you want. Put its name")
        print("     into aipilot/aircraft/profiles/fcu_conventions.json.")
        print()
        print("Names published by the add-on developer, or found in another tool's")
        print("profile for the same aeroplane, are the best starting point.")
        backend.close()
        return 0

    for name in names:
        bridge.register(name)
    print(f"Watching {len(names)} variable(s) for {args.seconds:.0f} seconds. "
          "Move the controls you are interested in.")
    print()

    seen: dict[str, float] = {}
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        backend.poll(0.0)
        for name in names:
            value = bridge.get(name)
            if value is None:
                continue
            if name not in seen:
                seen[name] = value
                print(f"  {name:<40} {value:>12.4f}   (first reading)")
            elif abs(value - seen[name]) > 1e-6:
                print(f"  {name:<40} {value:>12.4f}   (was {seen[name]:.4f})  <-- changed")
                seen[name] = value
        time.sleep(0.1)

    print()
    missing = [n for n in names if n not in seen]
    if missing:
        print("Never produced a value (the aeroplane probably does not have them): "
              + ", ".join(missing))
    if not seen:
        print("Nothing was read at all. Check 'python -m aipilot doctor' first -- "
              "the WASM module is very likely not installed.")
    backend.close()
    return 0
