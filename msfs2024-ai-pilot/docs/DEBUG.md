# When something goes wrong

Every fault found in this program so far was reported in prose — "it hovered at
1000 ft and went to 450 knots", "it pushed back a bit and then the wheels went
left and right" — and then tracked down by reading code and guessing. That
works, slowly, and it has been wrong at least once.

So there is a flight recorder. Fly with `--debug` and it writes down what
happened.

## Recording

```bash
python -m aipilot fly KJFK KIAD --debug
```

or on Windows, `windows\Fly-With-Debug.bat`, or tick **Record a debug trace**
in the browser panel. The file lands in `logs/`, named for the time and the
route.

It records four things:

| | |
|---|---|
| A header | version, aircraft, simulator, nav data, the plan, which runways were chosen and why, whether there was taxiway data at either end |
| Samples | position, altitude, speed, attitude, configuration, brakes, the tug — **and next to each, what the AI Pilot was asking for** |
| Events | everything that appears in the flight log, with its timestamp and phase |
| **Every command sent to the simulator** | the event name, its value, and the phase it was sent in |

The last one is the one nothing else captures and the one that earns its keep.
A tug heading being re-sent after the tug was released, or a flight-level-change
event sent ten thousand times in a flight, are invisible in any summary and
unmissable in a command trace. Both of those were real.

Samples are taken every cycle on the ground and below 1500 ft, and thinned out
above that. The failures are near the ground, so that is where the detail goes;
a twenty-hour cruise recorded at four hertz is mostly a record of nothing
happening.

## Reading it back

```bash
python -m aipilot debug-report logs/flight-20260902-183342-KJFKKIAD.jsonl
```

or `windows\Read-Debug-Trace.bat`, which finds the newest one. You get a page:
what was flown, the phase timeline, a count of every command sent, and a list of
what looks wrong.

```
Phases
------
      0.0 min  pushback
      1.8 min  taxi
      9.4 min  takeoff
     ...

Commands sent
-------------
     2400  event:KEY_TUG_HEADING  <-- far too often
      150  event:AP_VS_VAR_SET_ENGLISH

What looks wrong
----------------
 !! event:KEY_TUG_HEADING sent 2400 times (240 a minute)
      An event repeated at this rate is a loop, not a decision.
 !! A tug was still attached during taxi
      The pushback did not release, so the aeroplane cannot move
      under its own power.
```

It looks for:

- an event being sent far more often than any decision could need — measured
  over the stretch it was actually being sent, not over the whole flight, so a
  burst inside one phase is not averaged away by four hours of cruise;
- the throttle open with the aeroplane not moving — something is holding it;
- a tug still attached once the taxi has started;
- an overspeed against the aeroplane's own Vmo;
- flight below 500 ft that is not a takeoff, an approach or a landing;
- an autopilot that keeps dropping out;
- a flight that never left the ground or its first phase;
- everything the flight log itself flagged.

The summary is short on purpose: it is meant to be pasted into a message. The
file behind it is the real evidence — send that if you want it looked at
properly.

## What is not in it

Nothing about you. There are no keystrokes, no account details, no network
addresses. Folder paths have your home directory replaced with `~`, because a
Windows nav-data path usually has a real name in it and these files are meant to
be handed to someone else.

Have a look before you send one — it is plain text, one JSON object per line,
and you can read it in Notepad.

## Size

A short flight is a megabyte or so. A long haul is larger but not alarming,
because the cruise is thinned. If a bug makes the program repeat an event
thousands of times, the individual records stop after twenty thousand commands
and only the totals keep counting — the point has been made by then.

## Reading the file directly

One JSON object per line, each with `t` (the record type) and `at` (seconds
since engaging). So the ordinary tools work:

```bash
# Every command sent during the taxi
grep '"t":"command"' logs/flight-*.jsonl | grep '"phase":"taxi"'

# What it wanted against what it got, through the descent
python - <<'PY'
import json
for line in open("logs/flight-20260902-183342-KJFKKIAD.jsonl"):
    r = json.loads(line)
    if r.get("t") == "sample" and r.get("phase") == "descent":
        print(f"{r['at']:7.0f} alt {r['alt']:6.0f} want {r['want_alt']:6.0f} "
              f"ias {r['ias']:5.0f} want {r['want_spd']:5.0f}")
PY
```

`want_alt`, `want_spd`, `want_hdg` and `want_vs` are what the AI Pilot asked
for; `alt`, `ias`, `hdg` and `vs` are what it got. Half of every diagnosis is
the gap between the two.
