"""Local-variable access through the MobiFlight WASM module.

Why this exists
---------------
Plain SimConnect can read and write *simulation* variables (``A:`` vars) and
fire *key events*, and that is genuinely all it can do. An aircraft's own
systems -- the FCU knobs on an Airbus, an add-on's MCP, its autoflight logic --
live in *local* variables (``L:`` vars) inside the aircraft's gauge code, which
is a separate namespace SimConnect cannot see. Reaching it from an external
program requires a WASM module running inside the simulator to act as a proxy.

MobiFlight ships exactly such a module, it is free, it is very widely installed
in the community already, and it works in both MSFS 2020 and MSFS 2024. So
rather than shipping a WASM module of our own, the AI Pilot talks to
MobiFlight's if it is present.

Honesty about this protocol
---------------------------
The wire protocol below is not a published, versioned API -- it is the
convention MobiFlight's module uses, and it can change between module
releases. Every string and offset is therefore collected in
:class:`MobiFlightProtocol` and can be overridden from a JSON file without
touching code (``--mobiflight-protocol my_protocol.json``).

Run ``python -m aipilot doctor`` to check the bridge end to end against your
installed module: it reports whether the module answered, how many variables it
offered, and whether a write took effect. Aircraft that fly on standard events
alone (the default 787, for one) never need any of this, and the AI Pilot says
so rather than failing.
"""

from __future__ import annotations

import ctypes
import json
import struct
import time
from dataclasses import asdict, dataclass, field
from typing import Optional

from .base import SimBackendError

# Client-data area and definition identifiers. These are ours to choose; they
# only need to not collide with other client-data users in our own session.
AREA_COMMAND = 0x4D46_0001
AREA_RESPONSE = 0x4D46_0002
AREA_LVARS = 0x4D46_0003

DEFINE_COMMAND = 0x4D46_0011
DEFINE_RESPONSE = 0x4D46_0012
DEFINE_LVAR_BASE = 0x4D46_0100

REQUEST_RESPONSE = 0x4D46_0021
REQUEST_LVAR_BASE = 0x4D46_0200

SIMOBJECT_DATA_OFFSET = 40


@dataclass
class MobiFlightProtocol:
    """Every value that depends on the MobiFlight module's conventions."""

    command_area: str = "MobiFlight.Command"
    response_area: str = "MobiFlight.Response"
    lvar_area_suffix: str = ".LVars"
    command_area_suffix: str = ".Command"
    response_area_suffix: str = ".Response"
    message_size: int = 1024
    lvar_slot_size: int = 8          # one float64 per registered variable
    max_lvars: int = 512
    ping: str = "MF.Ping"
    add_client: str = "MF.Clients.Add.{client}"
    add_variable: str = "MF.SimVars.Add.{variable}"
    clear_variables: str = "MF.SimVars.Clear"
    set_expression: str = "MF.SimVars.Set.{code}"
    vars_per_frame: str = "MF.Config.MAX_VARS_PER_FRAME.Set.{count}"

    @classmethod
    def load(cls, path: str) -> "MobiFlightProtocol":
        with open(path, "r", encoding="utf-8") as handle:
            return cls(**json.load(handle))

    def dump(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(asdict(self), handle, indent=2)


class MobiFlightBridge:
    """Proxies L:Var reads, writes and calculator code through the WASM module.

    Reads are subscription-based: a variable must be *registered* once, after
    which the module streams its value into a client-data slot every frame. So
    :meth:`get` returns ``None`` the first time it sees a name and a real value
    from the next frame onward -- callers must tolerate that, and the aircraft
    adapters do by registering everything they need during setup.
    """

    def __init__(self, client_name: str = "AIPILOT",
                 protocol: Optional[MobiFlightProtocol] = None) -> None:
        self.client_name = client_name
        self.protocol = protocol or MobiFlightProtocol()
        self._backend = None
        self._slots: dict[str, int] = {}
        self._values: dict[str, float] = {}
        self._next_slot = 0
        self._client_channel_ready = False
        self._responses: list[str] = []
        self.ready = False
        self.last_error: Optional[str] = None

    # -- Setup ---------------------------------------------------------------
    def bind(self, backend) -> None:
        """Create the client-data channels and register with the module."""
        self._backend = backend
        proto = self.protocol
        try:
            backend.map_client_data_name(proto.command_area, AREA_COMMAND)
            backend.map_client_data_name(proto.response_area, AREA_RESPONSE)
            backend.add_client_data_definition(DEFINE_COMMAND, 0, proto.message_size)
            backend.add_client_data_definition(DEFINE_RESPONSE, 0, proto.message_size)
            backend.request_client_data(AREA_RESPONSE, REQUEST_RESPONSE, DEFINE_RESPONSE)
            self._send_command(proto.add_client.format(client=self.client_name))
        except SimBackendError as exc:
            self.last_error = str(exc)
            self.ready = False
            return
        self.ready = True

    def open_client_channels(self) -> None:
        """Map the per-client areas the module creates after ``MF.Clients.Add``.

        Called once the module has had a frame or two to answer; before that the
        areas do not exist and mapping them fails.
        """
        backend = self._backend
        if backend is None:
            return
        proto = self.protocol
        name = self.client_name
        try:
            backend.map_client_data_name(name + proto.command_area_suffix, AREA_COMMAND + 1)
            backend.map_client_data_name(name + proto.response_area_suffix, AREA_RESPONSE + 1)
            backend.map_client_data_name(name + proto.lvar_area_suffix, AREA_LVARS)
            self._client_channel_ready = True
        except SimBackendError as exc:
            self.last_error = str(exc)

    # -- Wire helpers --------------------------------------------------------
    def _send_command(self, command: str) -> bool:
        backend = self._backend
        if backend is None:
            return False
        payload = command.encode("ascii", "replace")[: self.protocol.message_size - 1]
        payload = payload.ljust(self.protocol.message_size, b"\0")
        area = AREA_COMMAND + 1 if self._client_channel_ready else AREA_COMMAND
        try:
            backend.set_client_data(area, DEFINE_COMMAND, payload)
        except SimBackendError as exc:
            self.last_error = str(exc)
            return False
        return True

    def on_client_data(self, data, size: int) -> None:
        """Handle a ``SIMCONNECT_RECV_CLIENT_DATA`` message."""
        words = ctypes.cast(data, ctypes.POINTER(ctypes.c_uint32 * 4)).contents
        request_id = words[3]
        if request_id == REQUEST_RESPONSE:
            raw = ctypes.string_at(data, min(size, SIMOBJECT_DATA_OFFSET
                                             + self.protocol.message_size))
            text = raw[SIMOBJECT_DATA_OFFSET:].split(b"\0", 1)[0].decode("ascii", "replace")
            if text:
                self._responses.append(text)
        elif REQUEST_LVAR_BASE <= request_id < REQUEST_LVAR_BASE + self.protocol.max_lvars:
            slot = request_id - REQUEST_LVAR_BASE
            needed = SIMOBJECT_DATA_OFFSET + 8
            if size < needed:
                return
            (value,) = struct.unpack("<d", ctypes.string_at(data, needed)[SIMOBJECT_DATA_OFFSET:])
            for name, assigned in self._slots.items():
                if assigned == slot:
                    self._values[name] = value
                    break

    # -- Public API ----------------------------------------------------------
    def register(self, name: str) -> bool:
        """Subscribe to a local variable so its value starts streaming in."""
        if name in self._slots:
            return True
        if self._next_slot >= self.protocol.max_lvars:
            self.last_error = "Too many registered local variables."
            return False
        backend = self._backend
        if backend is None:
            return False
        slot = self._next_slot
        self._next_slot += 1
        self._slots[name] = slot
        try:
            backend.add_client_data_definition(
                DEFINE_LVAR_BASE + slot, slot * self.protocol.lvar_slot_size,
                self.protocol.lvar_slot_size,
            )
            backend.request_client_data(AREA_LVARS, REQUEST_LVAR_BASE + slot,
                                        DEFINE_LVAR_BASE + slot)
        except SimBackendError as exc:
            self.last_error = str(exc)
            return False
        return self._send_command(self.protocol.add_variable.format(variable=name))

    def get(self, name: str) -> Optional[float]:
        if name not in self._slots:
            self.register(name)
            return None
        return self._values.get(name)

    def set(self, name: str, value: float) -> bool:
        return self.execute(f"{_format_number(value)} (>L:{name})")

    def execute(self, code: str) -> bool:
        return self._send_command(self.protocol.set_expression.format(code=code))

    def known_variables(self) -> list[str]:
        return sorted(self._slots)

    def drain_responses(self) -> list[str]:
        out, self._responses = self._responses, []
        return out

    def ping(self) -> bool:
        return self._send_command(self.protocol.ping)


def _format_number(value: float) -> str:
    """Render a number the way gauge RPN expects -- no exponent notation."""
    if value == int(value):
        return str(int(value))
    return f"{value:.6f}".rstrip("0").rstrip(".")
