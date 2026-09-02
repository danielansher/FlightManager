"""A flight recorder, so a problem can be diagnosed rather than guessed at.

Every fault found in this program so far was reported in prose -- "it hovered
at 1000 ft and went to 450 knots", "it pushed back a bit then the wheels went
left and right" -- and then reconstructed by reading code and guessing. That
works, slowly, and it has been wrong at least once.

What was actually needed each time was the same three things:

* what the aeroplane was doing, sampled over time;
* what the AI Pilot commanded, and when;
* **every event it sent to the simulator**.

The last one is the one nothing else captures, and it is the one that would
have found the two worst bugs in an instant. A tug heading being re-sent after
the tug was released, and a flight-level-change event sent ten thousand times
in one flight, are both invisible in any summary and unmissable in a command
trace.

The file is JSON lines: one self-describing record per line, so it can be read
by eye, grepped, or loaded whole. ``aipilot debug-report FILE`` turns one into
a short summary that can be pasted into a bug report.

Nothing personal is recorded. File paths are shortened to remove the home
directory, since a Windows path usually has a real name in it and these files
are meant to be sent to someone.
"""

from __future__ import annotations

import json
import math
import os
import platform
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Optional, TextIO

from .sim.base import SimBackend, SimCapabilities, SimState

#: Sampling. The interesting failures are on the ground and low down, so those
#: are recorded every cycle and the cruise is thinned out -- a twenty-hour
#: flight recorded at four hertz throughout is mostly a record of nothing
#: happening.
CRUISE_SAMPLE_S = 4.0
CLOSE_TO_THE_GROUND_FT = 1500.0

#: Beyond this many command records the trace switches to counting only. A
#: sane flight sends a few hundred; the only way to reach this is a bug that
#: repeats an event, and by then the trace has more than proved it.
MAX_COMMAND_RECORDS = 20000

#: What counts as sending an event far too often, per minute, in the report.
SPAM_PER_MINUTE = 60.0

#: Commands that genuinely have to be re-sent every cycle. The simulator's
#: BRAKES event is a momentary application, not a switch, so holding the
#: brakes on means sending it continuously; the same goes for the axis
#: setters. Everything else is a decision, and a decision repeated four times
#: a second is a bug.
HELD_DOWN = frozenset({
    "event:BRAKES", "event:SPOILERS_ON", "event:AXIS_LEFT_BRAKE_SET",
    "event:AXIS_RIGHT_BRAKE_SET", "event:STEERING_SET", "event:RUDDER_SET",
    "event:THROTTLE_SET", "event:AXIS_STEERING_SET",
})


def redact(text: Optional[str]) -> Optional[str]:
    """Replace the home directory with ``~``.

    A Windows nav-data path is usually ``C:\\Users\\<a real name>\\...`` and
    these files are meant to be handed to someone else.
    """
    if not text:
        return text
    home = os.path.expanduser("~")
    out = text
    if home and home != os.sep:
        out = out.replace(home, "~")
        out = out.replace(home.replace("\\", "/"), "~")
    return out


def _jsonable(value: Any) -> Any:
    """Flatten the few types that appear in a status snapshot."""
    from .autopilot.phases import Phase
    from .geo import LatLon

    if isinstance(value, LatLon):
        return [round(value.lat, 6), round(value.lon, 6)]
    if isinstance(value, Phase):
        return value.value
    if isinstance(value, float):
        return round(value, 3)
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return str(value)


class FlightRecorder:
    """Writes the trace. One instance per flight."""

    def __init__(self, path: str) -> None:
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        self._file: Optional[TextIO] = open(path, "w", encoding="utf-8")
        self._started = time.time()
        self._last_sample = -1e9
        self._command_records = 0
        self._commands_truncated = False
        self.command_counts: Counter[str] = Counter()
        #: When each command was first and last sent. The rate that matters is
        #: over the stretch a command was actually being sent, not over the
        #: whole flight: three hundred presses during a seventy-second takeoff
        #: roll average out to nothing across a four-hour trip, and that is
        #: exactly the bug this is meant to find.
        self.command_spans: dict[str, list[float]] = {}
        #: Set by the controller each cycle, so a command can be attributed to
        #: the phase it was sent in without threading the phase through the
        #: simulator wrapper.
        self.phase: str = "preflight"
        self.elapsed_s: float = 0.0

    # -- Writing --------------------------------------------------------------
    def _write(self, record_type: str, **fields: Any) -> None:
        if self._file is None:
            return
        record = {"t": record_type, "at": round(self.elapsed_s, 2)}
        # "t" and "at" belong to the record, not to the caller. A field that
        # collided with one of them used to overwrite it silently, which put
        # an autothrottle flag where every timestamp should have been and made
        # the whole trace look as though it happened at once.
        record.update({k: _jsonable(v) for k, v in fields.items()
                       if k not in ("t", "at")})
        try:
            self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
            # Flushed every time. The traces that matter most come from a
            # flight that hung and had to be killed, and a buffered line is
            # exactly the line that says why. At a few records a second the
            # cost is nothing.
            self._file.flush()
        except (OSError, ValueError):
            # A recorder that stops a flight is worse than no recorder.
            self.close()

    def header(self, **fields: Any) -> None:
        self._write(
            "header",
            recorded_at=time.strftime("%Y-%m-%d %H:%M:%S"),
            python=platform.python_version(),
            platform=platform.platform(),
            **fields,
        )

    def event(self, time_s: float, phase: str, message: str, level: str) -> None:
        self.elapsed_s = time_s
        self._write("event", phase=phase, level=level, message=message)

    def command(self, kind: str, name: str, value: Any = None) -> None:
        """One thing sent to the simulator."""
        key = f"{kind}:{name}"
        self.command_counts[key] += 1
        span = self.command_spans.get(key)
        if span is None:
            self.command_spans[key] = [self.elapsed_s, self.elapsed_s]
        else:
            span[1] = self.elapsed_s
        if self._command_records >= MAX_COMMAND_RECORDS:
            if not self._commands_truncated:
                self._commands_truncated = True
                self._write("note", message=(
                    f"More than {MAX_COMMAND_RECORDS} commands. Individual "
                    "records stop here; the totals at the end still count "
                    "everything."))
            return
        self._command_records += 1
        self._write("command", phase=self.phase, kind=kind, name=name, value=value)

    def sample(self, elapsed_s: float, state: SimState, status: Any) -> None:
        """A snapshot, thinned out at altitude and full rate near the ground."""
        self.elapsed_s = elapsed_s
        self.phase = getattr(getattr(status, "phase", None), "value", self.phase)
        close = (state.on_ground
                 or state.altitude_agl_ft < CLOSE_TO_THE_GROUND_FT)
        if not close and elapsed_s - self._last_sample < CRUISE_SAMPLE_S:
            return
        self._last_sample = elapsed_s
        self._write(
            "sample",
            phase=self.phase,
            msg=getattr(status, "message", ""),
            pos=state.position,
            alt=state.altitude_ft,
            agl=state.altitude_agl_ft,
            ias=state.ias_kt,
            mach=state.mach,
            gs=state.ground_speed_kt,
            vs=state.vertical_speed_fpm,
            hdg=state.heading_true_deg,
            trk=state.track_true_deg,
            on_ground=state.on_ground,
            ap=state.ap_master,
            athr=state.ap_autothrottle,
            thr=state.throttle_percent,
            n1=state.engine_n1_pct,
            flaps=state.flaps_pct,
            gear=state.gear_down_pct,
            spoilers=state.spoilers_pct,
            park_brake=state.parking_brake,
            tug=state.pushback_attached,
            wind=[state.wind_from_deg, state.wind_kt],
            # What the AI Pilot wants, next to what it is getting. Half of
            # every diagnosis is the gap between these two.
            want_alt=getattr(status, "target_altitude_ft", None),
            want_spd=getattr(status, "target_speed", None),
            want_mach=getattr(status, "target_speed_is_mach", None),
            want_hdg=getattr(status, "commanded_heading_deg", None),
            want_vs=getattr(status, "commanded_vs_fpm", None),
            xtk=getattr(status, "cross_track_nm", None),
            wpt=getattr(status, "active_waypoint", ""),
            to_go=getattr(status, "distance_to_destination_nm", None),
        )

    def finish(self, **fields: Any) -> None:
        self._write("totals",
                    commands=dict(self.command_counts.most_common()),
                    spans={k: v for k, v in self.command_spans.items()})
        self._write("end", **fields)
        self.close()

    def close(self) -> None:
        if self._file is not None:
            try:
                self._file.close()
            finally:
                self._file = None


class RecordingBackend(SimBackend):
    """Wraps a simulator connection and records everything sent to it.

    A decorator rather than hooks inside the adapters, so that there is
    exactly one place commands can escape through and nothing can be missed
    by an adapter that forgets to call something.
    """

    def __init__(self, inner: SimBackend, recorder: FlightRecorder) -> None:
        self.inner = inner
        self.recorder = recorder

    @property
    def name(self) -> str:                              # type: ignore[override]
        return f"recording({self.inner.name})"

    # -- Pass-through ---------------------------------------------------------
    def connect(self) -> None:
        self.inner.connect()

    def close(self) -> None:
        self.inner.close()

    def poll(self, dt: float) -> SimState:
        return self.inner.poll(dt)

    def capabilities(self) -> SimCapabilities:
        return self.inner.capabilities()

    def list_lvars(self) -> list[str]:
        return self.inner.list_lvars()

    def get_lvar(self, name: str) -> Optional[float]:
        return self.inner.get_lvar(name)

    def __getattr__(self, item: str) -> Any:
        # Backends carry extras the interface does not name -- receiving_data,
        # host_description, the mock's internals that the tests reach for.
        # Guarded, because __getattr__ asking for "inner" before __init__ has
        # set it would call itself for ever.
        if item in ("inner", "recorder"):
            raise AttributeError(item)
        return getattr(self.inner, item)

    # -- Recorded -------------------------------------------------------------
    def send_event(self, event: str, value: int = 0) -> None:
        self.recorder.command("event", event, value)
        self.inner.send_event(event, value)

    def set_var(self, name: str, value: float, unit: str = "number") -> None:
        self.recorder.command("var", name, round(float(value), 4))
        self.inner.set_var(name, value, unit)

    def set_lvar(self, name: str, value: float) -> bool:
        self.recorder.command("lvar", name, round(float(value), 4))
        return self.inner.set_lvar(name, value)

    def exec_calculator_code(self, code: str) -> bool:
        self.recorder.command("calc", code)
        return self.inner.exec_calculator_code(code)


# --- Reading one back --------------------------------------------------------
@dataclass
class Finding:
    """Something in the trace worth looking at."""

    severity: str            # "error", "warning", "note"
    summary: str
    detail: str = ""


@dataclass
class Report:
    path: str
    header: dict = field(default_factory=dict)
    end: dict = field(default_factory=dict)
    phases: list[tuple[str, float]] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)
    commands: dict = field(default_factory=dict)
    #: name -> [first sent at, last sent at], in seconds.
    command_spans: dict = field(default_factory=dict)
    samples: int = 0

    def rate_per_minute(self, name: str) -> float:
        """How often a command was sent while it was being sent at all."""
        count = _num(self.commands.get(name, 0))
        span = self.command_spans.get(name)
        if not isinstance(span, (list, tuple)) or len(span) < 2:
            window = self.duration_s
        else:
            window = max(_num(span[1]) - _num(span[0]), 0.0)
        # A burst inside one second is measured against one second, not zero.
        return count / max(window / 60.0, 1.0 / 60.0)
    duration_s: float = 0.0
    findings: list[Finding] = field(default_factory=list)


def read_records(path: str) -> list[dict]:
    out = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except ValueError:
                continue          # a half-written last line after a crash
    return out


def analyse(path: str) -> Report:
    """Turn a trace into the short version, and say what looks wrong."""
    records = read_records(path)
    report = Report(path=path)
    samples: list[dict] = []
    totals: dict = {}
    spans: dict = {}
    counted: Counter[str] = Counter()
    seen_spans: dict[str, list[float]] = {}
    for record in records:
        kind = record.get("t")
        if kind == "header":
            report.header = record
        elif kind == "end":
            report.end = record
        elif kind == "totals":
            totals = record.get("commands", {})
            spans = record.get("spans", {})
        elif kind == "event":
            report.events.append(record)
        elif kind == "sample":
            samples.append(record)
        elif kind == "command":
            key = f"{record.get('kind')}:{record.get('name')}"
            counted[key] += 1
            at = record.get("at", 0.0)
            if key in seen_spans:
                seen_spans[key][1] = at
            else:
                seen_spans[key] = [at, at]
    # The totals record is written when a flight ends, and the traces worth
    # reading are often from one that was killed before it did. Fall back to
    # counting the command records themselves, which are all still there.
    # Both halves matter: a totals record that is present but empty, or of
    # the wrong shape, must still fall back to counting the command records --
    # which is the case for every trace from a flight that was killed.
    report.commands = (totals if isinstance(totals, dict) and totals
                       else dict(counted.most_common()))
    report.command_spans = (spans if isinstance(spans, dict) and spans
                            else seen_spans)
    report.samples = len(samples)
    if records:
        report.duration_s = max(_num(r.get("at")) for r in records)

    last = None
    for sample in samples:
        if sample.get("phase") != last:
            last = sample.get("phase")
            report.phases.append((str(last), _num(sample.get("at"))))

    report.findings = _findings(report, samples)
    return report


def _num(value: Any, default: float = 0.0) -> float:
    """A number out of a trace, whatever the trace actually contains.

    The whole point of this file is reading a trace from a flight that went
    wrong, and a trace from a flight that went wrong is often damaged: a
    record with no timestamp, a field that is a string, a truncated span. The
    reader tolerates a half-written last line; the analysis on top of it used
    to fall over on any of the above.
    """
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _longest_run(samples: list[dict], predicate) -> float:
    """The longest unbroken stretch, in seconds, where ``predicate`` holds.

    Longest rather than total, because a condition that is true for one
    second at a time in thirty different places is noise, and the same
    condition true for four minutes without a break is the fault.
    """
    longest = 0.0
    start: Optional[float] = None
    previous: Optional[float] = None
    for sample in samples:
        at = sample.get("at", 0.0)
        if predicate(sample):
            if start is None:
                start = at
            previous = at
        else:
            if start is not None and previous is not None:
                longest = max(longest, previous - start)
            start = previous = None
    if start is not None and previous is not None:
        longest = max(longest, previous - start)
    return longest


def _findings(report: Report, samples: list[dict]) -> list[Finding]:
    found: list[Finding] = []

    # 1. An event sent far too often. Both of the worst bugs found so far look
    #    exactly like this and like nothing else.
    for name, count in report.commands.items():
        if count <= 50 or name in HELD_DOWN:
            continue
        rate = report.rate_per_minute(name)
        if rate > SPAM_PER_MINUTE:
            span = report.command_spans.get(name)
            over = (f" over {(_num(span[1]) - _num(span[0])) / 60:.1f} min"
                    if isinstance(span, (list, tuple)) and len(span) >= 2 else "")
            found.append(Finding(
                "error",
                f"{name} sent {count} times ({rate:.0f} a minute{over})",
                "An event repeated at this rate is a loop, not a decision. "
                "Something is re-sending it every cycle."))

    # 2. Held on the ground: power on, not moving.
    stuck = _longest_run(samples, lambda s: (
        s.get("on_ground") and (s.get("gs") or 0) < 1.0
        and (s.get("thr") or 0) > 5.0))
    if stuck > 30.0:
        found.append(Finding(
            "error",
            f"Throttle open but not moving for {stuck:.0f} s",
            "Something is holding the aeroplane: a tug still attached, the "
            "parking brake, or the wheel brakes."))

    # 3. A tug that is still there once the taxi has started.
    late_tug = [s for s in samples
                if s.get("tug") and s.get("phase") in ("taxi", "takeoff")]
    if late_tug:
        found.append(Finding(
            "error",
            f"A tug was still attached during {late_tug[0].get('phase')}",
            "The pushback did not release, so the aeroplane cannot move under "
            "its own power."))

    # 4. Overspeed.
    fastest = max(_num(s.get("ias")) for s in samples) if samples else 0.0
    vmo = _num(report.header.get("vmo_kt"))
    if vmo and fastest > vmo + 5:
        found.append(Finding(
            "error", f"Reached {fastest:.0f} kt against a Vmo of {vmo:.0f}",
            "Look at want_spd against ias in the samples around that point."))

    # 5. Low and not landing.
    low = [s for s in samples
           if not s.get("on_ground") and _num(s.get("agl"), 1e9) < 500
           and s.get("phase") not in ("takeoff", "approach", "landing")]
    if low:
        worst = min(low, key=lambda s: _num(s.get("agl"), 1e9))
        found.append(Finding(
            "error",
            f"{_num(worst.get('agl')):.0f} ft above the ground in "
            f"{worst.get('phase')}",
            f"At {_num(worst.get('at')):.0f} s. This program has no terrain "
            "awareness beyond a floor, so this is worth understanding."))

    # 6. The autopilot not holding.
    drops = sum(1 for e in report.events
                if "autopilot" in e.get("message", "").lower()
                and "disengag" in e.get("message", "").lower())
    if drops > 2:
        found.append(Finding(
            "warning", f"The autopilot dropped out {drops} times",
            "Usually a joystick axis with jitter on it, or a control bound "
            "twice. See docs/MSFS2020.md."))

    # 7. Nothing happening at all. Only when it never left the ground: a
    #    trace that is all cruise is someone who engaged in the air, not an
    #    aeroplane that is stuck.
    if (report.duration_s > 300 and len(report.phases) <= 1 and samples
            and all(s.get("on_ground") for s in samples)):
        found.append(Finding(
            "error", "The flight never left its first phase",
            f"Stuck on the ground in "
            f"{report.phases[0][0] if report.phases else 'preflight'} "
            f"for {report.duration_s / 60:.0f} minutes."))

    for event in report.events:
        if event.get("level") in ("warning", "error"):
            found.append(Finding(
                "warning" if event["level"] == "warning" else "error",
                event.get("message", ""),
                f"reported at {_num(event.get('at')):.0f} s in "
                f"{event.get('phase', '?')}"))
    return found


def format_report(report: Report) -> str:
    """The version to paste into a bug report."""
    head = report.header
    lines = [
        f"AI Pilot flight trace: {os.path.basename(report.path)}",
        "=" * 60,
    ]
    if head:
        lines += [
            f"Version    {head.get('version', '?')} on {head.get('platform', '?')}",
            f"Recorded   {head.get('recorded_at', '?')}",
            f"Aircraft   {head.get('aircraft', '?')}",
            f"Simulator  {head.get('sim', '?')}",
            f"Nav data   {head.get('navdata', '?')}",
            f"Route      {head.get('route', '?')}",
            f"Ground     {head.get('ground', 'not reported')}",
        ]
        if head.get("options"):
            lines.append(f"Options    {head['options']}")
    lines += [
        "",
        f"Duration   {report.duration_s / 60:.1f} min, {report.samples} samples",
        f"Ended      {report.end.get('phase', '?')}"
        + (f" -- {report.end['reason']}" if report.end.get("reason") else ""),
        "",
        "Phases",
        "------",
    ]
    for name, at in report.phases:
        lines.append(f"  {_num(at) / 60:7.1f} min  {name}")

    lines += ["", "Commands sent", "-------------"]
    for name, count in list(report.commands.items())[:20]:
        count = int(_num(count))
        rate = report.rate_per_minute(name)
        flag = ("  <-- far too often"
                if rate > SPAM_PER_MINUTE and count > 50
                and name not in HELD_DOWN else "")
        lines.append(f"  {count:7d}  {name}{flag}")
    if len(report.commands) > 20:
        lines.append(f"  ... and {len(report.commands) - 20} more")

    lines += ["", "What looks wrong", "----------------"]
    if not report.findings:
        lines.append("  Nothing stood out.")
    else:
        seen = set()
        for finding in report.findings:
            if finding.summary in seen:
                continue
            seen.add(finding.summary)
            mark = {"error": "!!", "warning": " !"}.get(finding.severity, "  ")
            lines.append(f" {mark} {finding.summary}")
            if finding.detail:
                lines.append(f"      {finding.detail}")
    return "\n".join(lines)


def default_path(origin: str = "", destination: str = "") -> str:
    """Where a trace goes when you do not say."""
    stamp = time.strftime("%Y%m%d-%H%M%S")
    leg = f"-{origin}{destination}" if origin and destination else ""
    return os.path.join("logs", f"flight-{stamp}{leg}.jsonl")
