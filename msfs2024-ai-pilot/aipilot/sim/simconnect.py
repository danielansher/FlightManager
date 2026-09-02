"""A self-contained SimConnect client built on ctypes.

There is no third-party dependency here on purpose. ``SimConnect.dll`` exports
plain C entry points, so ctypes can drive it directly, which keeps installation
down to "have Python, run the script" for someone who just wants to fly.

Only the calls the AI Pilot needs are bound: open/close, a data definition of
the flight state, event transmission, variable writes, and the client-data
areas used by the MobiFlight WASM bridge for local variables.

This module imports cleanly on any platform. Everything Windows-specific
happens in :meth:`SimConnectBackend.connect`, so the test suite and the mock
backend never touch it.
"""

from __future__ import annotations

import ctypes
import os
import platform
import re
import time
import struct
from ctypes import (
    POINTER,
    byref,
    c_char_p,
    c_double,
    c_float,
    c_int,
    c_long,
    c_void_p,
    create_string_buffer,
    sizeof,
)
from dataclasses import dataclass
from typing import Callable, Optional

from .base import SimBackend, SimBackendError, SimCapabilities, SimState

# --- SimConnect constants (from SimConnect.h) -------------------------------
OBJECT_ID_USER = 0
GROUP_PRIORITY_HIGHEST = 1
EVENT_FLAG_GROUPID_IS_PRIORITY = 0x00000010

DATATYPE_FLOAT64 = 4

PERIOD_NEVER = 0
PERIOD_ONCE = 1
PERIOD_VISUAL_FRAME = 2
PERIOD_SIM_FRAME = 3
PERIOD_SECOND = 4

DATA_REQUEST_FLAG_DEFAULT = 0
DATA_SET_FLAG_DEFAULT = 0

RECV_ID_NULL = 0
RECV_ID_EXCEPTION = 1
RECV_ID_OPEN = 2
RECV_ID_QUIT = 3
RECV_ID_EVENT = 4
RECV_ID_SIMOBJECT_DATA = 8
RECV_ID_SIMOBJECT_DATA_BYTYPE = 9
RECV_ID_CLIENT_DATA = 16

# SIMCONNECT_RECV_OPEN: 12 bytes of SIMCONNECT_RECV, then a 256-byte
# application name, then the version and build numbers as DWORDs.
OPEN_NAME_OFFSET = 12
OPEN_NAME_LENGTH = 256
OPEN_VERSION_OFFSET = OPEN_NAME_OFFSET + OPEN_NAME_LENGTH

# SIMCONNECT_RECV_SIMOBJECT_DATA: 12 bytes of SIMCONNECT_RECV, then seven
# DWORDs, so the payload starts here. SIMCONNECT_RECV_CLIENT_DATA derives from
# it and shares the layout.
SIMOBJECT_DATA_OFFSET = 40

E_FAIL = -2147467259

DEFINE_FLIGHT_STATE = 1
REQUEST_FLIGHT_STATE = 1
FIRST_DYNAMIC_DEFINE = 100
FIRST_EVENT_ID = 1000


@dataclass(frozen=True)
class VarSpec:
    """One row of the flight-state data definition."""

    field: str
    simvar: str
    unit: str
    kind: type = float


def _b(value: bool) -> bool:
    return bool(value)


#: The flight-state definition, in the exact order it is added to SimConnect.
#: The reply is a packed array of float64 in this order, so the order is the
#: wire format -- append to the end rather than inserting.
FLIGHT_STATE_VARS: tuple[VarSpec, ...] = (
    VarSpec("lat", "PLANE LATITUDE", "degrees"),
    VarSpec("lon", "PLANE LONGITUDE", "degrees"),
    VarSpec("altitude_ft", "INDICATED ALTITUDE", "feet"),
    VarSpec("altitude_agl_ft", "PLANE ALT ABOVE GROUND", "feet"),
    VarSpec("ground_elevation_ft", "GROUND ALTITUDE", "feet"),
    VarSpec("pitch_deg", "PLANE PITCH DEGREES", "degrees"),
    VarSpec("bank_deg", "PLANE BANK DEGREES", "degrees"),
    VarSpec("heading_true_deg", "PLANE HEADING DEGREES TRUE", "degrees"),
    VarSpec("heading_mag_deg", "PLANE HEADING DEGREES MAGNETIC", "degrees"),
    VarSpec("track_true_deg", "GPS GROUND TRUE TRACK", "degrees"),
    VarSpec("magvar_deg", "MAGVAR", "degrees"),
    VarSpec("ias_kt", "AIRSPEED INDICATED", "knots"),
    VarSpec("tas_kt", "AIRSPEED TRUE", "knots"),
    VarSpec("ground_speed_kt", "GROUND VELOCITY", "knots"),
    VarSpec("mach", "AIRSPEED MACH", "mach"),
    VarSpec("vertical_speed_fpm", "VERTICAL SPEED", "feet per minute"),
    VarSpec("wind_from_deg", "AMBIENT WIND DIRECTION", "degrees"),
    VarSpec("wind_kt", "AMBIENT WIND VELOCITY", "knots"),
    VarSpec("sea_level_pressure_inhg", "SEA LEVEL PRESSURE", "inHg"),
    VarSpec("ambient_temp_c", "AMBIENT TEMPERATURE", "celsius"),
    VarSpec("on_ground", "SIM ON GROUND", "bool", bool),
    VarSpec("gear_down_pct", "GEAR TOTAL PCT EXTENDED", "percent"),
    VarSpec("flaps_index", "FLAPS HANDLE INDEX", "number", int),
    VarSpec("flaps_pct", "TRAILING EDGE FLAPS LEFT PERCENT", "percent"),
    VarSpec("spoilers_pct", "SPOILERS HANDLE POSITION", "percent"),
    VarSpec("parking_brake", "BRAKE PARKING POSITION", "bool", bool),
    VarSpec("total_weight_lb", "TOTAL WEIGHT", "pounds"),
    VarSpec("fuel_lb", "FUEL TOTAL QUANTITY WEIGHT", "pounds"),
    VarSpec("engine_count", "NUMBER OF ENGINES", "number", int),
    VarSpec("engines_running", "GENERAL ENG COMBUSTION:1", "bool", bool),
    VarSpec("ap_master", "AUTOPILOT MASTER", "bool", bool),
    VarSpec("ap_heading_lock", "AUTOPILOT HEADING LOCK", "bool", bool),
    VarSpec("ap_altitude_lock", "AUTOPILOT ALTITUDE LOCK", "bool", bool),
    VarSpec("ap_nav_lock", "AUTOPILOT NAV1 LOCK", "bool", bool),
    VarSpec("ap_approach_hold", "AUTOPILOT APPROACH HOLD", "bool", bool),
    VarSpec("ap_glideslope_hold", "AUTOPILOT GLIDESLOPE HOLD", "bool", bool),
    VarSpec("ap_backcourse_hold", "AUTOPILOT BACKCOURSE HOLD", "bool", bool),
    VarSpec("ap_autothrottle", "AUTOPILOT THROTTLE ARM", "bool", bool),
    VarSpec("ap_heading_bug_deg", "AUTOPILOT HEADING LOCK DIR", "degrees"),
    VarSpec("ap_altitude_target_ft", "AUTOPILOT ALTITUDE LOCK VAR", "feet"),
    VarSpec("ap_vs_target_fpm", "AUTOPILOT VERTICAL HOLD VAR", "feet per minute"),
    VarSpec("ap_airspeed_target_kt", "AUTOPILOT AIRSPEED HOLD VAR", "knots"),
    VarSpec("nav1_freq_mhz", "NAV ACTIVE FREQUENCY:1", "MHz"),
    VarSpec("nav1_has_localizer", "NAV HAS LOCALIZER:1", "bool", bool),
    VarSpec("nav1_localizer_error_deg", "NAV RADIAL ERROR:1", "degrees"),
    VarSpec("nav1_has_glideslope", "NAV HAS GLIDE SLOPE:1", "bool", bool),
    VarSpec("nav1_glideslope_error_deg", "NAV GLIDE SLOPE ERROR:1", "degrees"),
    VarSpec("nav1_obs_deg", "NAV OBS:1", "degrees"),
    VarSpec("light_beacon", "LIGHT BEACON", "bool", bool),
    VarSpec("light_nav", "LIGHT NAV", "bool", bool),
    VarSpec("light_landing", "LIGHT LANDING", "bool", bool),
    VarSpec("light_taxi", "LIGHT TAXI", "bool", bool),
    VarSpec("light_strobe", "LIGHT STROBE", "bool", bool),
    VarSpec("light_wing", "LIGHT WING", "bool", bool),
    VarSpec("light_logo", "LIGHT LOGO", "bool", bool),
    VarSpec("seatbelt_sign", "CABIN SEATBELTS ALERT SWITCH", "bool", bool),
    VarSpec("no_smoking_sign", "CABIN NO SMOKING ALERT SWITCH", "bool", bool),
    VarSpec("pushback_attached", "PUSHBACK ATTACHED", "bool", bool),
    VarSpec("pushback_state", "PUSHBACK STATE", "number", int),
    VarSpec("sim_rate", "SIMULATION RATE", "number"),
    VarSpec("sim_time_s", "ABSOLUTE TIME", "seconds"),
    # Appended, which is the only safe way to change this list: the order is
    # the wire format the simulator sends the payload back in.
    VarSpec("throttle_percent", "GENERAL ENG THROTTLE LEVER POSITION:1",
            "percent"),
    VarSpec("engine_n1_pct", "TURB ENG N1:1", "percent"),
)

#: Where SimConnect.dll usually lives, for both simulators. The SDK copies are
#: listed first because they are the ones that are definitely a real, complete
#: SimConnect. ``AIPILOT_SIMCONNECT_DLL`` overrides all of this.
DEFAULT_DLL_LOCATIONS = (
    # MSFS 2024 SDK, MSFS 2020 SDK.
    r"C:\MSFS 2024 SDK\SimConnect SDK\lib\SimConnect.dll",
    r"C:\MSFS SDK\SimConnect SDK\lib\SimConnect.dll",
    # Simulator installs.
    r"C:\Program Files\Microsoft Flight Simulator 2024\SimConnect.dll",
    r"C:\Program Files\Microsoft Flight Simulator\SimConnect.dll",
    r"C:\Program Files (x86)\Microsoft Flight Simulator\SimConnect.dll",
    # Steam, in the default library.
    r"C:\Program Files (x86)\Steam\steamapps\common\MicrosoftFlightSimulator\SimConnect.dll",
    r"C:\Program Files (x86)\Steam\steamapps\common\Microsoft Flight Simulator 2024\SimConnect.dll",
)

#: Patterns searched under every Steam library and drive root, for the very
#: common cases of a Steam library on another drive or a non-default SDK path.
DLL_SEARCH_PATTERNS = (
    r"steamapps\common\MicrosoftFlightSimulator\SimConnect.dll",
    r"steamapps\common\Microsoft Flight Simulator 2024\SimConnect.dll",
    r"MSFS SDK\SimConnect SDK\lib\SimConnect.dll",
    r"MSFS 2024 SDK\SimConnect SDK\lib\SimConnect.dll",
)


def _steam_libraries() -> list[str]:
    """Steam library roots, read from Steam's own library folder manifest."""
    roots = []
    for base in (r"C:\Program Files (x86)\Steam", r"C:\Steam",
                 os.path.expandvars(r"%ProgramFiles(x86)%\Steam")):
        manifest = os.path.join(base, "steamapps", "libraryfolders.vdf")
        if not os.path.isfile(manifest):
            continue
        try:
            with open(manifest, "r", encoding="utf-8", errors="replace") as handle:
                text = handle.read()
        except OSError:
            continue
        # The manifest is Valve's own key-value format; the only thing wanted
        # from it is the quoted "path" values, which a regex gets without
        # taking on a parser for a format that is not ours.
        for match in re.finditer(r'"path"\s*"([^"]+)"', text):
            roots.append(match.group(1).replace("\\\\", "\\"))
    return roots


def find_simconnect_dll() -> Optional[str]:
    """Locate SimConnect.dll, for either simulator.

    Order: an explicit override, then a copy dropped next to this package
    (the friendliest thing to tell someone to do), then the known install
    locations, then a shallow search of Steam libraries and drive roots.
    """
    override = os.environ.get("AIPILOT_SIMCONNECT_DLL")
    if override:
        return override if os.path.isfile(override) else None

    local = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "SimConnect.dll")
    if os.path.isfile(local):
        return local

    for candidate in DEFAULT_DLL_LOCATIONS:
        if os.path.isfile(candidate):
            return candidate

    roots = _steam_libraries() + [f"{letter}:\\" for letter in "CDEFGH"]
    for root in roots:
        if not os.path.isdir(root):
            continue
        for pattern in DLL_SEARCH_PATTERNS:
            candidate = os.path.join(root, pattern)
            if os.path.isfile(candidate):
                return candidate
    return None


class SimConnectBackend(SimBackend):
    """Live connection to Microsoft Flight Simulator 2020 or 2024."""

    name = "simconnect"

    def __init__(self, app_name: str = "AIPilot", dll_path: Optional[str] = None,
                 on_exception: Optional[Callable[[int], None]] = None) -> None:
        self._dll_path = dll_path
        self._dll: Optional[ctypes.CDLL] = None
        self._handle = c_void_p()
        self._app_name = app_name
        self._state = SimState()
        self._event_ids: dict[str, int] = {}
        self._next_event_id = FIRST_EVENT_ID
        self._var_defines: dict[tuple[str, str], int] = {}
        self._next_define = FIRST_DYNAMIC_DEFINE
        self._on_exception = on_exception
        self._exceptions: list[int] = []
        self._received_any = False
        #: When a flight-state packet last arrived, for staleness.
        self._last_state_at: Optional[float] = None
        self._short_packets = 0
        self.host_name = ""
        self.host_version: Optional[tuple[int, int, int, int]] = None
        self.lvar_bridge = None  # set by attach_lvar_bridge()

    # -- Lifecycle -----------------------------------------------------------
    def connect(self) -> None:
        if platform.system() != "Windows":
            raise SimBackendError(
                "SimConnect is only available on Windows. Use --sim mock to fly the "
                "planner offline."
            )
        path = self._dll_path or find_simconnect_dll()
        if not path:
            raise SimBackendError(
                "SimConnect.dll not found. Install the MSFS SDK, or copy SimConnect.dll "
                "next to the aipilot package, or set AIPILOT_SIMCONNECT_DLL."
            )
        try:
            self._dll = ctypes.WinDLL(path)  # type: ignore[attr-defined]
        except OSError as exc:  # pragma: no cover - Windows only
            raise SimBackendError(f"Could not load {path}: {exc}") from exc
        self._bind_prototypes()

        hr = self._dll.SimConnect_Open(
            byref(self._handle), self._app_name.encode(), None, 0, None, 0
        )
        if hr != 0 or not self._handle:
            raise SimBackendError(
                "SimConnect_Open failed. Is the simulator running and past the main menu?"
            )
        self._define_flight_state()
        self._state.connected = True

    def close(self) -> None:
        if self._dll is not None and self._handle:
            try:
                self._dll.SimConnect_Close(self._handle)
            except OSError:  # pragma: no cover - teardown best effort
                pass
        self._handle = c_void_p()
        self._state.connected = False

    def _bind_prototypes(self) -> None:
        dll = self._dll
        assert dll is not None
        dll.SimConnect_Open.restype = c_long
        dll.SimConnect_Open.argtypes = [POINTER(c_void_p), c_char_p, c_void_p, ctypes.c_uint32,
                                        c_void_p, ctypes.c_uint32]
        dll.SimConnect_Close.restype = c_long
        dll.SimConnect_Close.argtypes = [c_void_p]
        dll.SimConnect_AddToDataDefinition.restype = c_long
        dll.SimConnect_AddToDataDefinition.argtypes = [
            c_void_p, ctypes.c_uint32, c_char_p, c_char_p, c_int, c_float, ctypes.c_uint32,
        ]
        dll.SimConnect_RequestDataOnSimObject.restype = c_long
        dll.SimConnect_RequestDataOnSimObject.argtypes = [
            c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, c_int,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
        ]
        dll.SimConnect_SetDataOnSimObject.restype = c_long
        dll.SimConnect_SetDataOnSimObject.argtypes = [
            c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, ctypes.c_uint32, c_void_p,
        ]
        dll.SimConnect_MapClientEventToSimEvent.restype = c_long
        dll.SimConnect_MapClientEventToSimEvent.argtypes = [c_void_p, ctypes.c_uint32, c_char_p]
        dll.SimConnect_TransmitClientEvent.restype = c_long
        dll.SimConnect_TransmitClientEvent.argtypes = [
            c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, ctypes.c_uint32,
        ]
        dll.SimConnect_GetNextDispatch.restype = c_long
        dll.SimConnect_GetNextDispatch.argtypes = [c_void_p, POINTER(c_void_p),
                                                   POINTER(ctypes.c_uint32)]
        # Client-data calls, used by the MobiFlight local-variable bridge.
        for fn, args in (
            ("SimConnect_MapClientDataNameToID", [c_void_p, c_char_p, ctypes.c_uint32]),
            ("SimConnect_CreateClientData", [c_void_p, ctypes.c_uint32, ctypes.c_uint32,
                                             ctypes.c_uint32]),
            ("SimConnect_AddToClientDataDefinition", [c_void_p, ctypes.c_uint32, ctypes.c_uint32,
                                                      ctypes.c_uint32, c_float, ctypes.c_uint32]),
            ("SimConnect_RequestClientData", [c_void_p, ctypes.c_uint32, ctypes.c_uint32,
                                              ctypes.c_uint32, c_int, ctypes.c_uint32,
                                              ctypes.c_uint32, ctypes.c_uint32,
                                              ctypes.c_uint32]),
            ("SimConnect_SetClientData", [c_void_p, ctypes.c_uint32, ctypes.c_uint32,
                                          ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                                          c_void_p]),
        ):
            if hasattr(dll, fn):
                getattr(dll, fn).restype = c_long
                getattr(dll, fn).argtypes = args

    # -- Data definition -----------------------------------------------------
    def _define_flight_state(self) -> None:
        dll = self._dll
        assert dll is not None
        for index, spec in enumerate(FLIGHT_STATE_VARS):
            hr = dll.SimConnect_AddToDataDefinition(
                self._handle, DEFINE_FLIGHT_STATE, spec.simvar.encode(), spec.unit.encode(),
                DATATYPE_FLOAT64, 0.0, index,
            )
            if hr != 0:
                raise SimBackendError(
                    f"SimConnect rejected the variable {spec.simvar!r} ({spec.unit})."
                )
        hr = dll.SimConnect_RequestDataOnSimObject(
            self._handle, REQUEST_FLIGHT_STATE, DEFINE_FLIGHT_STATE, OBJECT_ID_USER,
            PERIOD_SIM_FRAME, DATA_REQUEST_FLAG_DEFAULT, 0, 0, 0,
        )
        if hr != 0:
            raise SimBackendError("SimConnect refused the flight-state subscription.")

    # -- Dispatch ------------------------------------------------------------
    def poll(self, dt: float = 0.0) -> SimState:
        dll = self._dll
        if dll is None or not self._handle:
            return self._state
        data = c_void_p()
        size = ctypes.c_uint32()
        # Drain everything queued; the last flight-state message wins.
        while dll.SimConnect_GetNextDispatch(self._handle, byref(data), byref(size)) == 0:
            if not data:
                break
            self._dispatch(data, size.value)
        return self._state

    def _dispatch(self, data: c_void_p, size: int) -> None:
        header = ctypes.cast(data, POINTER(ctypes.c_uint32 * 3)).contents
        recv_id = header[2]
        if recv_id == RECV_ID_SIMOBJECT_DATA:
            self._read_flight_state(data, size)
        elif recv_id == RECV_ID_CLIENT_DATA and self.lvar_bridge is not None:
            self.lvar_bridge.on_client_data(data, size)
        elif recv_id == RECV_ID_EXCEPTION:
            words = ctypes.cast(data, POINTER(ctypes.c_uint32 * 5)).contents
            code = words[3]
            self._exceptions.append(code)
            if self._on_exception:
                self._on_exception(code)
        elif recv_id == RECV_ID_OPEN:
            self._read_open(data, size)
        elif recv_id == RECV_ID_QUIT:
            self._state.connected = False

    def _read_flight_state(self, data: c_void_p, size: int) -> None:
        # dwRequestID is the fourth DWORD; anything else belongs to someone else.
        words = ctypes.cast(data, POINTER(ctypes.c_uint32 * 4)).contents
        if words[3] != REQUEST_FLIGHT_STATE:
            return
        count = len(FLIGHT_STATE_VARS)
        needed = SIMOBJECT_DATA_OFFSET + 8 * count
        if size < needed:
            # A reply too short to hold the variables we asked for. Silently
            # dropping it leaves the aeroplane being flown on whatever was in
            # the state before, for ever, with nothing said -- so count it and
            # let staleness catch it.
            self._short_packets += 1
            return
        raw = ctypes.string_at(data, needed)[SIMOBJECT_DATA_OFFSET:]
        values = struct.unpack(f"<{count}d", raw)
        self._last_state_at = time.monotonic()
        state = self._state
        for spec, value in zip(FLIGHT_STATE_VARS, values):
            if spec.kind is bool:
                setattr(state, spec.field, value > 0.5)
            elif spec.kind is int:
                setattr(state, spec.field, int(round(value)))
            else:
                setattr(state, spec.field, value)
        state.connected = True
        self._received_any = True

    def _read_open(self, data: c_void_p, size: int) -> None:
        """Record who answered: the simulator's name and version."""
        needed = OPEN_VERSION_OFFSET + 16
        if size < OPEN_NAME_OFFSET + OPEN_NAME_LENGTH:
            return
        raw = ctypes.string_at(data, min(size, needed))
        name = raw[OPEN_NAME_OFFSET:OPEN_NAME_OFFSET + OPEN_NAME_LENGTH]
        self.host_name = name.split(b"\0", 1)[0].decode("ascii", "replace").strip()
        if size >= needed:
            major, minor, build_major, build_minor = struct.unpack(
                "<4I", raw[OPEN_VERSION_OFFSET:OPEN_VERSION_OFFSET + 16]
            )
            self.host_version = (major, minor, build_major, build_minor)

    @property
    def host_description(self) -> str:
        """What the simulator called itself when the connection opened.

        Useful mostly for the diagnostics: it confirms which simulator actually
        answered, which matters on a machine with both installed, and it is the
        only thing in the whole connection that says so.
        """
        if not self.host_name:
            return "unknown (the simulator did not identify itself)"
        if self.host_version is None:
            return self.host_name
        major, minor, build_major, build_minor = self.host_version
        return f"{self.host_name} {major}.{minor}.{build_major}.{build_minor}"

    @property
    def receiving_data(self) -> bool:
        """Whether at least one flight-state packet has arrived."""
        return self._received_any

    @property
    def data_age_s(self) -> float:
        """How long since the simulator last told us anything.

        Nothing in SimConnect announces a connection that has gone away
        without a QUIT -- the simulator crashing, or the link failing -- and
        an empty dispatch queue looks exactly like an idle one. Without this
        the AI Pilot keeps flying, at four hertz, on a snapshot that stopped
        changing minutes ago: the aeroplane appears frozen in the log while
        the program reports everything as normal.
        """
        if self._last_state_at is None:
            return float("inf") if self._received_any else 0.0
        return max(0.0, time.monotonic() - self._last_state_at)

    @property
    def short_packets(self) -> int:
        """Replies too short to hold the variables we subscribed to."""
        return self._short_packets

    @property
    def exception_codes(self) -> list[int]:
        return list(self._exceptions)

    # -- Commands ------------------------------------------------------------
    def _event_id(self, event: str) -> int:
        if event not in self._event_ids:
            assert self._dll is not None
            event_id = self._next_event_id
            self._next_event_id += 1
            hr = self._dll.SimConnect_MapClientEventToSimEvent(
                self._handle, event_id, event.encode()
            )
            if hr != 0:
                raise SimBackendError(f"SimConnect could not map the event {event!r}.")
            self._event_ids[event] = event_id
        return self._event_ids[event]

    def send_event(self, event: str, value: int = 0) -> None:
        if self._dll is None or not self._handle:
            raise SimBackendError("Not connected to the simulator.")
        # SimConnect takes the parameter as an unsigned DWORD; negative values
        # (a descent rate, for instance) go across as two's complement.
        raw = value & 0xFFFFFFFF
        hr = self._dll.SimConnect_TransmitClientEvent(
            self._handle, OBJECT_ID_USER, self._event_id(event), raw,
            GROUP_PRIORITY_HIGHEST, EVENT_FLAG_GROUPID_IS_PRIORITY,
        )
        if hr != 0:
            raise SimBackendError(f"Failed to transmit {event!r}.")

    def set_var(self, name: str, value: float, unit: str = "number") -> None:
        if self._dll is None or not self._handle:
            raise SimBackendError("Not connected to the simulator.")
        key = (name, unit)
        if key not in self._var_defines:
            define_id = self._next_define
            self._next_define += 1
            hr = self._dll.SimConnect_AddToDataDefinition(
                self._handle, define_id, name.encode(), unit.encode(), DATATYPE_FLOAT64, 0.0, 0
            )
            if hr != 0:
                raise SimBackendError(f"SimConnect rejected writing {name!r} ({unit}).")
            self._var_defines[key] = define_id
        payload = c_double(float(value))
        hr = self._dll.SimConnect_SetDataOnSimObject(
            self._handle, self._var_defines[key], OBJECT_ID_USER, DATA_SET_FLAG_DEFAULT,
            0, sizeof(payload), byref(payload),
        )
        if hr != 0:
            raise SimBackendError(f"Failed to set {name!r}.")

    # -- Local variables, via the optional WASM bridge -----------------------
    def attach_lvar_bridge(self, bridge) -> None:
        """Install a local-variable bridge (see :mod:`aipilot.sim.mobiflight`)."""
        self.lvar_bridge = bridge
        bridge.bind(self)

    def capabilities(self) -> SimCapabilities:
        has_lvars = self.lvar_bridge is not None and self.lvar_bridge.ready
        return SimCapabilities(
            simvars=True, events=True, lvars=has_lvars, calculator_code=has_lvars
        )

    def get_lvar(self, name: str) -> Optional[float]:
        if self.lvar_bridge is None:
            return None
        return self.lvar_bridge.get(name)

    def set_lvar(self, name: str, value: float) -> bool:
        if self.lvar_bridge is None:
            return False
        return self.lvar_bridge.set(name, value)

    def exec_calculator_code(self, code: str) -> bool:
        if self.lvar_bridge is None:
            return False
        return self.lvar_bridge.execute(code)

    def list_lvars(self) -> list[str]:
        if self.lvar_bridge is None:
            return []
        return self.lvar_bridge.known_variables()

    # -- Raw client-data helpers used by the bridge --------------------------
    def _require(self, fn_name: str):
        dll = self._dll
        if dll is None or not hasattr(dll, fn_name):
            raise SimBackendError(f"This SimConnect.dll does not export {fn_name}.")
        return getattr(dll, fn_name)

    def map_client_data_name(self, name: str, area_id: int) -> None:
        fn = self._require("SimConnect_MapClientDataNameToID")
        if fn(self._handle, name.encode(), area_id) != 0:
            raise SimBackendError(f"Could not map client data area {name!r}.")

    def add_client_data_definition(self, define_id: int, offset: int, size: int) -> None:
        fn = self._require("SimConnect_AddToClientDataDefinition")
        if fn(self._handle, define_id, offset, size, 0.0, 0) != 0:
            raise SimBackendError("Could not build the client data definition.")

    def request_client_data(self, area_id: int, request_id: int, define_id: int,
                            period: int = PERIOD_SIM_FRAME, flags: int = 0) -> None:
        fn = self._require("SimConnect_RequestClientData")
        if fn(self._handle, area_id, request_id, define_id, period, flags, 0, 0, 0) != 0:
            raise SimBackendError("Could not subscribe to the client data area.")

    def set_client_data(self, area_id: int, define_id: int, payload: bytes) -> None:
        fn = self._require("SimConnect_SetClientData")
        buffer = create_string_buffer(payload, len(payload))
        if fn(self._handle, area_id, define_id, 0, 0, len(payload), byref(buffer)) != 0:
            raise SimBackendError("Could not write to the client data area.")
