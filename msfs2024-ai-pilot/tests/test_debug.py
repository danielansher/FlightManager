"""The flight recorder, and whether it would have caught the real failures.

Every fault in this program so far was reported in prose and then found by
reading code. The recorder exists so that stops being the method, so the
tests that matter are the ones that replay the faults that actually
happened and check the report names them.
"""

from __future__ import annotations

import json
import os

import pytest

from aipilot.aircraft.registry import build_adapter
from aipilot.autopilot.controller import AIPilot, PilotOptions
from aipilot.autopilot.phases import Phase
from aipilot.debug import (
    MAX_COMMAND_RECORDS,
    FlightRecorder,
    RecordingBackend,
    analyse,
    format_report,
    read_records,
    redact,
)
from aipilot.geo import LatLon
from aipilot.perf.profiles import get_profile
from aipilot.route.planner import plan_route
from aipilot.sim.base import SimState
from aipilot.sim.mock import MockAircraftModel, MockSim


# --- Writing -----------------------------------------------------------------
def test_every_command_is_captured(tmp_path):
    """A decorator round the backend, so nothing can send without being seen."""
    recorder = FlightRecorder(str(tmp_path / "t.jsonl"))
    sim = RecordingBackend(MockSim(LatLon(51.0, 0.0)), recorder)
    sim.send_event("AP_MASTER", 1)
    sim.set_var("NAV OBS:1", 123.0)
    sim.set_lvar("A32NX_FCU_SPD", 250.0)
    sim.exec_calculator_code("1 (>L:X)")
    recorder.finish(phase="complete")

    records = read_records(str(tmp_path / "t.jsonl"))
    commands = [r for r in records if r["t"] == "command"]
    assert [c["name"] for c in commands] == [
        "AP_MASTER", "NAV OBS:1", "A32NX_FCU_SPD", "1 (>L:X)"]
    assert [c["kind"] for c in commands] == ["event", "var", "lvar", "calc"]
    totals = next(r for r in records if r["t"] == "totals")
    assert totals["commands"]["event:AP_MASTER"] == 1


def test_the_wrapper_still_behaves_like_the_backend(tmp_path):
    """It has to be transparent, or wrapping it changes the flight."""
    recorder = FlightRecorder(str(tmp_path / "t.jsonl"))
    inner = MockSim(LatLon(51.0, 0.0))
    sim = RecordingBackend(inner, recorder)
    sim.send_event("AP_MASTER")
    assert sim.poll(0.0).ap_master is True
    assert sim.capabilities().events
    # Attributes the interface does not name still reach through.
    assert sim.events_sent[-1][0] == "AP_MASTER"
    recorder.close()


def test_a_field_cannot_overwrite_the_timestamp(tmp_path):
    """A sample field called "at" once replaced every timestamp with an
    autothrottle flag, and the whole trace looked as if it happened at once."""
    recorder = FlightRecorder(str(tmp_path / "t.jsonl"))
    recorder.elapsed_s = 42.0
    recorder._write("sample", at=True, t="nonsense", alt=1000)
    recorder.close()

    record = read_records(str(tmp_path / "t.jsonl"))[0]
    assert record["at"] == 42.0
    assert record["t"] == "sample"
    assert record["alt"] == 1000


def test_a_repeated_command_stops_filling_the_file(tmp_path):
    recorder = FlightRecorder(str(tmp_path / "t.jsonl"))
    for _ in range(MAX_COMMAND_RECORDS + 500):
        recorder.command("event", "SPAM")
    recorder.finish(phase="complete")

    records = read_records(str(tmp_path / "t.jsonl"))
    commands = [r for r in records if r["t"] == "command"]
    assert len(commands) == MAX_COMMAND_RECORDS
    assert any(r["t"] == "note" for r in records)
    # The count is still honest even though the records stopped.
    totals = next(r for r in records if r["t"] == "totals")
    assert totals["commands"]["event:SPAM"] == MAX_COMMAND_RECORDS + 500


def test_a_recorder_that_cannot_write_does_not_stop_the_flight(tmp_path):
    recorder = FlightRecorder(str(tmp_path / "t.jsonl"))
    recorder.close()
    recorder.command("event", "AP_MASTER")      # must not raise
    recorder.sample(1.0, SimState(), None)
    recorder.finish(phase="complete")


def test_the_home_directory_is_not_in_the_file():
    """These files are meant to be sent to someone, and a Windows nav-data
    path has a real name in it."""
    home = os.path.expanduser("~")
    assert redact(f"{home}/Documents/navdata.sqlite") == "~/Documents/navdata.sqlite"
    assert redact(None) is None
    assert redact("nothing personal") == "nothing personal"


# --- Reading, and the faults that actually happened --------------------------
def _trace(tmp_path, records: list[dict]) -> str:
    path = str(tmp_path / "trace.jsonl")
    with open(path, "w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")
    return path


def _sample(at, **fields):
    base = dict(t="sample", at=at, phase="cruise", on_ground=False, agl=30000,
                alt=35000, ias=280, gs=300, thr=60, tug=False)
    base.update(fields)
    return base


def test_it_names_an_event_that_is_being_repeated(tmp_path):
    """The tug bug and the ten-thousand-flight-level-changes bug both look
    exactly like this, and like nothing else."""
    path = _trace(tmp_path, [
        {"t": "header", "at": 0, "version": "1.4.0"},
        _sample(0), _sample(600),
        {"t": "totals", "at": 600,
         "commands": {"event:KEY_TUG_HEADING": 2400, "event:AP_MASTER": 2}},
    ])
    report = analyse(path)
    summaries = " ".join(f.summary for f in report.findings)
    assert "KEY_TUG_HEADING" in summaries
    assert "2400" in summaries
    assert "AP_MASTER" not in summaries
    assert any(f.severity == "error" for f in report.findings)


def test_it_names_an_aeroplane_held_on_the_ground(tmp_path):
    """Power on, not moving: the reported symptom at the Kennedy gate."""
    path = _trace(tmp_path, [
        {"t": "header", "at": 0},
        *[_sample(t, phase="taxi", on_ground=True, agl=0, gs=0.0, ias=0,
                  thr=40.0, tug=True) for t in range(0, 300, 10)],
        {"t": "totals", "at": 300, "commands": {}},
    ])
    report = analyse(path)
    summaries = " ".join(f.summary for f in report.findings)
    assert "not moving" in summaries
    assert "tug was still attached" in summaries


def test_it_names_an_overspeed_and_how_low_it_got(tmp_path):
    """KLAX to KBUR: 450 kt at low level, into the hills."""
    path = _trace(tmp_path, [
        {"t": "header", "at": 0, "vmo_kt": 350.0},
        _sample(0),
        _sample(400, phase="descent", ias=450, alt=1000, agl=300),
        {"t": "totals", "at": 400, "commands": {}},
    ])
    report = analyse(path)
    summaries = " ".join(f.summary for f in report.findings)
    assert "450" in summaries and "350" in summaries
    assert "300 ft above the ground" in summaries


def test_it_notices_a_flight_that_never_got_going(tmp_path):
    path = _trace(tmp_path, [
        {"t": "header", "at": 0},
        *[_sample(t, phase="preflight", on_ground=True, agl=0, gs=0, thr=0)
          for t in range(0, 900, 30)],
        {"t": "totals", "at": 900, "commands": {}},
    ])
    report = analyse(path)
    assert any("never left its first phase" in f.summary for f in report.findings)


def test_a_healthy_flight_reports_nothing_alarming(tmp_path):
    path = _trace(tmp_path, [
        {"t": "header", "at": 0, "vmo_kt": 350.0},
        *[_sample(t) for t in range(0, 3600, 60)],
        {"t": "totals", "at": 3600, "commands": {"event:HEADING_BUG_SET": 88}},
    ])
    report = analyse(path)
    assert [f for f in report.findings if f.severity == "error"] == []
    assert "Nothing stood out" in format_report(report)


def test_warnings_from_the_flight_reach_the_report(tmp_path):
    path = _trace(tmp_path, [
        {"t": "header", "at": 0},
        {"t": "event", "at": 12, "phase": "climb", "level": "warning",
         "message": "The autothrottle is armed but has not held the speed"},
        _sample(0), _sample(60),
        {"t": "totals", "at": 60, "commands": {}},
    ])
    report = analyse(path)
    assert any("autothrottle is armed" in f.summary for f in report.findings)


def test_a_half_written_file_still_reads(tmp_path):
    """The interesting case is a trace from a flight that crashed the program,
    so the last line is usually truncated."""
    path = str(tmp_path / "t.jsonl")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({"t": "header", "at": 0, "version": "1"}) + "\n")
        handle.write(json.dumps(_sample(10)) + "\n")
        handle.write('{"t":"sample","at":20,"pha')
    report = analyse(path)
    assert report.header["version"] == "1"
    assert report.samples == 1


# --- A real recorded flight --------------------------------------------------
@pytest.fixture(scope="module")
def flown(tmp_path_factory):
    from aipilot.navdata.resolve import NavDataSources, build_navdata

    navdata = build_navdata(NavDataSources(littlenavmap_db=None, airports_csv=None))
    origin, destination = navdata.airport("EGLL"), navdata.airport("EGCC")
    profile = get_profile("b787-10")
    plan = plan_route(origin, destination, profile, navdata)
    runway = plan.departure_runway

    path = str(tmp_path_factory.mktemp("trace") / "flight.jsonl")
    recorder = FlightRecorder(path)
    inner = MockSim(runway.threshold, runway.heading_true_deg,
                    origin.elevation_ft,
                    model=MockAircraftModel(max_climb_fpm=profile.max_climb_rate_fpm))
    sim = RecordingBackend(inner, recorder)
    adapter, _ = build_adapter("b787-10", sim)
    pilot = AIPilot(sim, adapter, profile, plan, PilotOptions(),
                    listener=lambda e: recorder.event(e.time_s, e.phase.value,
                                                      e.message, e.level))
    recorder.header(version="test", vmo_kt=profile.vmo_kt, aircraft="b787-10")
    pilot.engage()
    # A one-second step, not four: the thinning only shows against a control
    # rate faster than the cruise sampling interval, which is what the real
    # thing runs at.
    for _ in range(4000):
        status = pilot.update(1.0)
        recorder.sample(pilot.elapsed_s, sim.poll(0.0), status)
        if pilot.phase in (Phase.COMPLETE, Phase.ABORTED):
            break
    recorder.finish(phase=pilot.phase.value)
    return path, pilot


def test_a_real_flight_records_its_phases_in_order(flown):
    path, pilot = flown
    assert pilot.phase is Phase.COMPLETE
    report = analyse(path)
    names = [name for name, _at in report.phases]
    for phase in ("takeoff", "climb", "cruise", "descent", "approach",
                  "landing", "complete"):
        assert phase in names, f"{phase} missing from {names}"
    # Ordered in time, and not all crammed onto the same timestamp.
    times = [at for _name, at in report.phases]
    assert times == sorted(times)
    assert times[-1] > 60.0


def test_a_real_flight_is_sampled_hardest_near_the_ground(flown):
    """Where the failures are is where the detail should be."""
    path, _pilot = flown
    samples = [r for r in read_records(path) if r["t"] == "sample"]
    low = [s for s in samples if (s.get("agl") or 0) < 1500]
    high = [s for s in samples if (s.get("agl") or 0) > 10000]
    assert low and high

    def cadence(rows):
        gaps = [b["at"] - a["at"] for a, b in zip(rows, rows[1:])
                if 0 < b["at"] - a["at"] < 60]
        return sorted(gaps)[len(gaps) // 2]

    assert cadence(low) < cadence(high)


def test_a_real_flight_does_not_repeat_any_event(flown):
    """A guard against the next AUTO_THROTTLE_TO_GA -- which this recorder
    found on its very first flight, at 296 presses in one takeoff roll."""
    path, _pilot = flown
    report = analyse(path)
    minutes = report.duration_s / 60.0
    # BRAKES genuinely has to be re-sent every cycle to hold the brakes on,
    # and the spoiler command is the same. Everything else is a decision and
    # should be sent when the decision changes.
    held_down = {"event:BRAKES", "event:SPOILERS_ON", "event:AXIS_LEFT_BRAKE_SET",
                 "event:AXIS_RIGHT_BRAKE_SET", "event:STEERING_SET",
                 "event:RUDDER_SET", "event:THROTTLE_SET"}
    for name, count in report.commands.items():
        if name in held_down:
            continue
        assert count / minutes < 20, f"{name} sent {count} times in {minutes:.0f} min"


def test_the_report_is_short_enough_to_paste(flown):
    path, _pilot = flown
    text = format_report(analyse(path))
    assert len(text.splitlines()) < 80
    assert "AI Pilot flight trace" in text
    assert "Phases" in text and "Commands sent" in text


def test_a_killed_flight_still_shows_what_it_sent(tmp_path):
    """The traces worth reading are usually from a flight that hung and had
    to be killed, so the totals record at the end never got written."""
    path = str(tmp_path / "t.jsonl")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({"t": "header", "at": 0}) + "\n")
        # Once a cycle at four hertz, which is what the real fault did.
        for step in range(300):
            handle.write(json.dumps({
                "t": "command", "at": step * 0.25, "phase": "pushback",
                "kind": "event", "name": "KEY_TUG_HEADING"}) + "\n")
        handle.write(json.dumps(_sample(600, phase="pushback", on_ground=True,
                                        agl=0, gs=0, thr=30, tug=True)) + "\n")

    report = analyse(path)
    assert report.commands["event:KEY_TUG_HEADING"] == 300
    assert any("KEY_TUG_HEADING" in f.summary for f in report.findings)


def test_a_burst_in_one_phase_is_not_diluted_by_a_long_flight(tmp_path):
    """The TOGA fault: 296 presses in a seventy-second takeoff roll, inside a
    forty-minute flight. Averaged over the whole trip that is seven a minute
    and looks like nothing, which is why it had to be spotted by eye. Measured
    over the stretch it was actually being sent, it is obvious."""
    records = [{"t": "header", "at": 0, "vmo_kt": 350.0}]
    for step in range(296):
        records.append({"t": "command", "at": 0.25 * step, "phase": "takeoff",
                        "kind": "event", "name": "AUTO_THROTTLE_TO_GA"})
    records += [_sample(t) for t in range(0, 2400, 60)]
    path = _trace(tmp_path, records)

    report = analyse(path)
    assert report.duration_s > 2000, "this must look like a long flight"
    assert report.rate_per_minute("event:AUTO_THROTTLE_TO_GA") > 200
    assert any("AUTO_THROTTLE_TO_GA" in f.summary for f in report.findings)


def test_commands_that_must_be_held_down_are_not_flagged(tmp_path):
    """The simulator's BRAKES event is a momentary application, not a switch:
    holding the brakes on means sending it every cycle, and calling that a
    fault would make the report cry wolf on every landing."""
    records = [{"t": "header", "at": 0, "vmo_kt": 350.0}]
    for step in range(400):
        records.append({"t": "command", "at": 0.25 * step, "phase": "rollout",
                        "kind": "event", "name": "BRAKES"})
    records += [_sample(t) for t in range(0, 600, 60)]
    report = analyse(_trace(tmp_path, records))
    assert not any("BRAKES" in f.summary for f in report.findings)
    assert "far too often" not in format_report(report)
