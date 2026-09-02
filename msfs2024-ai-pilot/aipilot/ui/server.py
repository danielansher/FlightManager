"""A small web control panel, served from the standard library.

Deliberately no web framework. This is a tool people run on the same machine
as a flight simulator, often while the simulator is using most of the machine,
and "pip install a dependency tree" is a worse experience than a single file
that starts instantly. ``http.server`` is entirely adequate for one user on
localhost.

The pilot runs on its own thread and the HTTP handlers only read a snapshot of
its state, so a slow browser can never stall the control loop.
"""

from __future__ import annotations

import json
import os
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

from ..aircraft.registry import available_aircraft, build_adapter, resolve_key
from ..autopilot.controller import AIPilot, PilotOptions
from ..autopilot.phases import Phase
from ..geo import distance_nm
from ..navdata.resolve import NavDataSources, build_navdata
from ..perf.profiles import get_profile
from ..briefing import resolve_winds
from ..route.planner import plan_route
from ..route.profile import build_vertical_profile
from ..sim.base import SimBackendError

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
CONTROL_RATE_HZ = 4.0


class FlightSession:
    """Owns the pilot, the simulator connection and the control thread."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.pilot: Optional[AIPilot] = None
        self.sim = None
        self.navdata = None
        self.plan = None
        self.thread: Optional[threading.Thread] = None
        self.stop_flag = threading.Event()
        self.error: Optional[str] = None
        self.speed = 1.0
        self.ground_notes: list[str] = []
        self.recorder = None
        #: Set while engage() is doing its slow setup, so a second request
        #: cannot slip past the "already running" check before the control
        #: thread exists.
        self._engaging = False

    # -- Planning ------------------------------------------------------------
    def build_plan(self, request: dict) -> dict:
        key = resolve_key(request.get("aircraft", "b787-10")) or "b787-10"
        profile = get_profile(key)
        assert profile is not None

        navdata = build_navdata(NavDataSources(
            littlenavmap_db=request.get("navdata") or None,
            airports_csv=request.get("airports_csv") or None,
            runways_csv=request.get("runways_csv") or None,
        ))
        try:
            return self._plan_with(key, profile, navdata, request)
        except BaseException:
            # A plan that fails part way used to abandon the database handle
            # it had just opened. The command line closes its nav data on
            # every path; this was the one place that did not.
            navdata.close()
            raise

    def _plan_with(self, key, profile, navdata, request: dict) -> dict:
        brief = None
        if request.get("simbrief"):
            from ..simbrief import SimBriefError, fetch_plan

            try:
                brief = fetch_plan(str(request["simbrief"]))
            except SimBriefError as exc:
                raise ValueError(str(exc)) from exc
            for field in ("origin", "destination", "departure_runway",
                          "arrival_runway", "route"):
                if not request.get(field):
                    request[field] = getattr(brief, field) or ""
            if not request.get("cruise") and brief.cruise_altitude_ft:
                request["cruise"] = brief.cruise_altitude_ft

        origin = navdata.airport(request.get("origin", ""))
        destination = navdata.airport(request.get("destination", ""))
        if origin is None:
            raise ValueError(f"{request.get('origin', '').upper() or '(blank)'} "
                             "is not in the navigation data.")
        if destination is None:
            raise ValueError(f"{request.get('destination', '').upper() or '(blank)'} "
                             "is not in the navigation data.")

        typed_wind = None
        if request.get("wind_from") or request.get("wind_kt"):
            typed_wind = (float(request.get("wind_from") or 0),
                          float(request.get("wind_kt") or 0))
        wind_notes: list[str] = []
        winds = resolve_winds(
            origin.icao, destination.icao,
            typed=typed_wind,
            use_metar=not request.get("no_metar"),
            simbrief_metars=(brief.origin_metar, brief.destination_metar)
            if brief else None,
            report=wind_notes.append,
        )
        cruise = request.get("cruise")
        cruise_ft = None
        if cruise:
            cruise_ft = float(cruise)
            if cruise_ft < 500:
                cruise_ft *= 100

        plan = plan_route(
            origin, destination, profile, navdata,
            departure_runway=request.get("departure_runway") or None,
            arrival_runway=request.get("arrival_runway") or None,
            cruise_altitude_ft=cruise_ft,
            route=request.get("route") or None,
            departure_wind=winds.departure,
            arrival_wind=winds.arrival,
        )
        vertical = build_vertical_profile(
            plan.cruise_altitude_ft,
            plan.arrival_runway.elevation_ft if plan.arrival_runway
            else plan.destination.elevation_ft,
            profile,
        )
        with self.lock:
            if self.navdata is not None and self.navdata is not navdata:
                self.navdata.close()
            self.navdata = navdata
            self.plan = plan
            self._pending = (key, profile, plan,
                             (winds.departure.from_deg,
                              winds.departure.speed_kt))
        return {
            "ok": True,
            "navdata": navdata.describe(),
            "aircraft": profile.name,
            "origin": {"icao": plan.origin.icao, "name": plan.origin.name,
                       "runway": plan.departure_runway.ident if plan.departure_runway else ""},
            "destination": {"icao": plan.destination.icao, "name": plan.destination.name,
                            "runway": plan.arrival_runway.ident if plan.arrival_runway else ""},
            "cruise_ft": plan.cruise_altitude_ft,
            "distance_nm": plan.total_distance_nm,
            "top_of_descent_nm": vertical.top_of_descent_nm,
            "warnings": plan.warnings,
            "runway_notes": plan.runway_notes + wind_notes,
            "simbrief": brief.describe() if brief else "",
            "legs": [
                {"ident": leg.ident, "lat": leg.position.lat, "lon": leg.position.lon,
                 "altitude_ft": leg.altitude_ft, "speed_kt": leg.speed_kt,
                 "phase": leg.phase}
                for leg in plan.legs
            ],
            "ils": bool(plan.arrival_runway and plan.arrival_runway.has_ils),
        }

    # -- Flying --------------------------------------------------------------
    def engage(self, request: dict) -> dict:
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                raise ValueError("A flight is already running.")
            if self._engaging:
                raise ValueError("A flight is already being started.")
            pending = getattr(self, "_pending", None)
            if pending is None:
                raise ValueError("Build a flight plan first.")
            # Claimed inside the same lock as the check. Everything after this
            # is slow -- connecting to SimConnect, building the adapter and
            # both taxiway networks -- and the thread that proves a flight is
            # running does not exist until the end of it. Two clicks, or two
            # browser tabs, used to get two control threads commanding the
            # same aeroplane at four hertz, with only one of them reachable to
            # stop.
            self._engaging = True
        try:
            return self._engage(pending, request)
        finally:
            # Cleared however it ends. Left set by a failure -- a simulator
            # that will not connect, say -- the panel would refuse every later
            # attempt with "a flight is already being started" and need a
            # restart.
            with self.lock:
                self._engaging = False

    def _engage(self, pending, request: dict) -> dict:
        key, profile, plan, wind = pending

        use_mock = request.get("sim", "msfs") == "mock"
        self.speed = max(0.1, float(request.get("speed") or 1.0)) if use_mock else 1.0
        sim = self._make_sim(use_mock, plan, profile, wind,
                             bool(request.get("airborne")))

        self.recorder = None
        if request.get("debug"):
            from ..debug import FlightRecorder, RecordingBackend, default_path

            try:
                self.recorder = FlightRecorder(
                    default_path(plan.origin.icao, plan.destination.icao))
                sim = RecordingBackend(sim, self.recorder)
            except OSError as exc:
                self.recorder = None
                self.ground_notes = [f"Could not open a flight trace: {exc}"]

        adapter, _ = build_adapter(key, sim)
        options = PilotOptions(
            autoland=request.get("autoland", "auto"),
            manage_configuration=bool(request.get("manage_configuration", True)),
            manage_lights=bool(request.get("manage_lights", True)),
            go_around_if_unstable=bool(request.get("go_around", True)),
            start_airborne=bool(request.get("airborne")),
            taxi=bool(request.get("taxi", True)),
        )
        ground = arrival_ground = None
        if options.taxi and self.navdata is not None:
            from ..cli import _ground_networks

            notes: list[str] = []
            ground, arrival_ground = _ground_networks(self.navdata, plan,
                                                      notes.append)
            self.ground_notes = notes
        listener = None
        if self.recorder is not None:
            def listener(event) -> None:                # noqa: F811
                self.recorder.event(event.time_s, event.phase.value,
                                    event.message, event.level)

        pilot = AIPilot(sim, adapter, profile, plan, options, ground=ground,
                        arrival_ground=arrival_ground, listener=listener)
        if self.recorder is not None:
            from .. import __version__
            from ..debug import redact
            from ..cli import _ground_summary

            self.recorder.header(
                version=__version__,
                aircraft=f"{key} ({profile.name})",
                vmo_kt=profile.vmo_kt,
                sim=redact(getattr(sim, "host_description", None) or sim.name),
                navdata=redact(self.navdata.describe()) if self.navdata else "",
                route=(f"{plan.origin.icao} -> {plan.destination.icao}, "
                       f"{plan.total_distance_nm:.0f} nm at "
                       f"FL{plan.cruise_altitude_ft / 100:.0f}"),
                runway_notes=list(plan.runway_notes),
                warnings=list(plan.warnings),
                ground=_ground_summary(ground, arrival_ground),
                options={"autoland": options.autoland, "taxi": options.taxi,
                         "sim": "mock" if use_mock else "msfs",
                         "speed": self.speed},
            )

        with self.lock:
            self.sim = sim
            self.pilot = pilot
            self.error = None
            self.stop_flag.clear()
        pilot.engage()
        for note in getattr(self, "ground_notes", []):
            pilot.log.add(0.0, pilot.phase, note)
        self.thread = threading.Thread(target=self._run, daemon=True, name="aipilot")
        self.thread.start()
        return {"ok": True, "adapter": adapter.describe(),
                "trace": self.recorder.path if self.recorder else ""}

    def _make_sim(self, use_mock: bool, plan, profile, wind, airborne: bool):
        if use_mock:
            from ..sim.mock import MockAircraftModel, MockSim

            origin, destination = plan.origin, plan.destination

            def terrain(position):
                near = distance_nm(position, origin.position)
                far = distance_nm(position, destination.position)
                total = near + far
                if total < 1e-6:
                    return origin.elevation_ft
                return (origin.elevation_ft * far + destination.elevation_ft * near) / total

            runway = plan.departure_runway
            return MockSim(
                runway.threshold if runway else origin.position,
                runway.heading_true_deg if runway else 0.0,
                origin.elevation_ft,
                model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm),
                terrain=terrain, wind_from_deg=wind[0], wind_kt=wind[1],
                start_airborne_at_ft=plan.cruise_altitude_ft if airborne else None,
            )

        from ..sim.mobiflight import MobiFlightBridge
        from ..sim.simconnect import SimConnectBackend

        backend = SimConnectBackend()
        backend.connect()
        backend.attach_lvar_bridge(MobiFlightBridge())
        return backend

    def _run(self) -> None:
        period = 1.0 / CONTROL_RATE_HZ
        pilot = self.pilot
        assert pilot is not None
        try:
            while not self.stop_flag.is_set() and \
                    pilot.phase not in (Phase.COMPLETE, Phase.ABORTED):
                started = time.monotonic()
                status = pilot.update(period)
                if self.recorder is not None and self.sim is not None:
                    self.recorder.sample(pilot.elapsed_s, self.sim.poll(0.0),
                                         status)
                remaining = period / self.speed - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
        except Exception as exc:                     # keep the UI alive and honest
            with self.lock:
                self.error = f"{type(exc).__name__}: {exc}"
        finally:
            if self.recorder is not None:
                try:
                    self.recorder.finish(phase=pilot.phase.value,
                                         reason=pilot.status.message,
                                         elapsed_s=round(pilot.elapsed_s, 1))
                except Exception:
                    pass
            if self.sim is not None:
                try:
                    self.sim.close()
                except Exception:
                    pass

    def disengage(self) -> dict:
        self.stop_flag.set()
        with self.lock:
            if self.pilot is not None:
                self.pilot.disengage("disengaged from the control panel")
        return {"ok": True}

    # -- Reporting -----------------------------------------------------------
    def snapshot(self, since: int = 0) -> dict:
        with self.lock:
            pilot = self.pilot
            error = self.error
        if pilot is None:
            return {"running": False, "error": error, "events": [], "event_count": 0}
        status = pilot.status
        events = pilot.log.since(since)
        return {
            "running": pilot.engaged,
            "error": error,
            "phase": status.phase.value,
            "phase_label": status.phase.label,
            "message": status.message,
            "lat": status.position.lat,
            "lon": status.position.lon,
            "altitude_ft": status.altitude_ft,
            "altitude_agl_ft": status.altitude_agl_ft,
            "ias_kt": status.ias_kt,
            "mach": status.mach,
            "ground_speed_kt": status.ground_speed_kt,
            "vertical_speed_fpm": status.vertical_speed_fpm,
            "heading_true_deg": status.heading_true_deg,
            "track_true_deg": status.track_true_deg,
            "active_waypoint": status.active_waypoint,
            "active_index": status.active_index,
            "distance_to_waypoint_nm": status.distance_to_waypoint_nm,
            "distance_to_destination_nm": status.distance_to_destination_nm,
            "cross_track_nm": status.cross_track_nm,
            "eta": status.eta_text,
            "elapsed_s": status.time_enroute_s,
            "target_altitude_ft": status.target_altitude_ft,
            "target_speed": status.target_speed,
            "target_speed_is_mach": status.target_speed_is_mach,
            "commanded_vs_fpm": status.commanded_vs_fpm,
            "path_deviation_ft": status.path_deviation_ft,
            "top_of_descent_nm": status.top_of_descent_nm,
            "flaps_index": status.flaps_index,
            "gear_down": status.gear_down,
            "autoland": status.autoland,
            "go_arounds": status.go_arounds,
            "event_count": len(pilot.log),
            "events": [
                {"time_s": e.time_s, "phase": e.phase.label,
                 "message": e.message, "level": e.level}
                for e in events
            ],
        }


SESSION = FlightSession()


class Handler(BaseHTTPRequestHandler):
    server_version = "AIPilot"

    def log_message(self, *args) -> None:      # quiet; the flight log is the output
        pass

    # -- Plumbing ------------------------------------------------------------
    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, payload: Any, status: int = 200) -> None:
        self._send(status, json.dumps(payload).encode(), "application/json")

    def _static(self, name: str) -> None:
        # The separator matters: without it "/static/../static_private/x"
        # normalises to a path that still starts with the string STATIC_DIR
        # and passes the guard.
        path = os.path.normpath(os.path.join(STATIC_DIR, name))
        if not path.startswith(STATIC_DIR + os.sep) or not os.path.isfile(path):
            self._send(404, b"not found", "text/plain")
            return
        kind = {".html": "text/html", ".css": "text/css",
                ".js": "application/javascript"}.get(os.path.splitext(path)[1],
                                                     "application/octet-stream")
        with open(path, "rb") as handle:
            self._send(200, handle.read(), kind + "; charset=utf-8")

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return {}

    # -- Routes --------------------------------------------------------------
    def do_GET(self) -> None:
        try:
            self._get()
        except Exception as exc:                      # never drop the socket
            self._json({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, 500)

    def _get(self) -> None:
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._static("index.html")
        elif path.startswith("/static/"):
            self._static(path[len("/static/"):])
        elif path == "/api/aircraft":
            self._json([{"key": key, "name": name} for key, name in available_aircraft()])
        elif path == "/api/state":
            self._json(SESSION.snapshot(self._since()))
        else:
            self._send(404, b"not found", "text/plain")

    def _since(self) -> int:
        """The event cursor from the query string, if it is a number.

        Anything else means zero rather than an exception: this used to be a
        bare int(), and a stale tab or a hand-typed URL took the traceback
        through the flight log and dropped the connection.
        """
        if "?" not in self.path:
            return 0
        for part in self.path.split("?", 1)[1].split("&"):
            if part.startswith("since="):
                try:
                    return max(0, int(part[6:] or 0))
                except ValueError:
                    return 0
        return 0

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        try:
            if path == "/api/plan":
                self._json(SESSION.build_plan(self._body()))
            elif path == "/api/engage":
                self._json(SESSION.engage(self._body()))
            elif path == "/api/disengage":
                self._json(SESSION.disengage())
            else:
                self._send(404, b"not found", "text/plain")
        except (ValueError, SimBackendError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:                  # never leave the panel hanging
            self._json({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, status=500)


def serve(host: str = "127.0.0.1", port: int = 8711, open_browser: bool = False) -> int:
    server = ThreadingHTTPServer((host, port), Handler)
    url = f"http://{host}:{port}/"
    print(f"AI Pilot control panel: {url}")
    print("Press Ctrl-C to stop.")
    if open_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        SESSION.disengage()
        server.server_close()
    return 0
