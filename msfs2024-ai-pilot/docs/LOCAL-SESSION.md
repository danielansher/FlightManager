# Briefing for a session running on the flying machine

This program is developed in one place and flown in another. Everything hard
left in it is in the gap: what an add-on aeroplane actually does with a
SimConnect event, and what a real airport's taxiway data actually looks like.
A session on the machine with the simulator can close that gap in minutes
instead of one flight per round trip.

This is the context that session needs.

## The setup

| | |
|---|---|
| Simulator | **Microsoft Flight Simulator 2020**, not 2024. Pass `--msfs 2020`. |
| Aircraft | Asobo 787-10 (`b787-10`) and the Horizon 787-10/-9. |
| Repository | `msfs2024-ai-pilot` is a **subfolder** of the `FlightManager` repo. `.git` is one level up — do not look for it beside the program. |
| Branch | `claude/msfs-2024-ai-pilot-cp3cd5` |
| Python | 3.9 or newer. **No third-party packages at all** — `ctypes`, `sqlite3`, `csv`, `urllib`, `http.server`. There is nothing to `pip install`. |
| SimConnect | `SimConnect.dll` must be findable. `python -m aipilot find-simconnect` locates one from any tool that already installed it. |
| Nav data | Little Navmap's scenery database. **The only source of taxiways** — there is no taxiway CSV, and downloading more files cannot produce one. |

## What to check first, before flying anything

```bash
python --version                     # 3.9+
git rev-parse --abbrev-ref HEAD      # claude/msfs-2024-ai-pilot-cp3cd5
git log --oneline -3
python -m pytest -q                  # all green, ~294 tests, about 20 seconds
python -m aipilot --version          # 1.7.0 or newer
python -m aipilot doctor --msfs 2020 --airport KJFK
```

In the `doctor` output, check three things:

* the nav data line names the Little Navmap database **once**, not twice —
  `littlenavmap(x) + littlenavmap(x)` was a bug fixed in 1.7.0 where
  `%APPDATA%` and `~/AppData/Roaming` resolved to the same folder;
* KJFK reports thousands of taxiway segments and hundreds of stands;
* the runways are real, not synthetic, and have ILS frequencies.

## Most of the ground work needs no simulator

The taxiway data is a database on disk. So the route across an airport can be
inspected between flights, or with the simulator shut:

```bash
python -m aipilot taxi KJFK --stands                    # list the stands
python -m aipilot taxi KJFK --stand A6 --runway 22R     # the route, turn by turn
```

It prints each leg's length, the turn onto it, and counts the turns sharper
than thirty degrees and the legs shorter than a hundred and fifty feet. That
is the whole of the zig-zag problem in one command, and it is reproducible
without leaving the terminal.

## Getting a trace worth reading

1. **Turn off the simulator's own assistance.** Options → Assistance →
   Piloting: AI Piloting **off**, auto-rudder **off**. Two things flying one
   aeroplane is not a bug in either of them.
2. **Deal with the controllers.** A joystick axis with a little jitter reads
   as a control input and disconnects the autopilot. Unplug it, or check
   nothing is bound twice. (Already done on this machine — worth re-checking
   after any driver update.)
3. **Load the flight and leave it alone.** Aircraft at a gate or on a runway,
   engines running, ready to move. Do not touch anything after this.
4. **Fly it with a trace.**
   ```bash
   python -m aipilot fly KJFK KIAD --aircraft b787-10 --msfs 2020 --debug
   ```
   or `windows\Fly-With-Debug.bat`.
5. **Do not intervene.** The trace is only worth what it records, and a
   manual input in the middle of it makes every later line ambiguous. If you
   must take over, say so afterwards and roughly when.
6. **When it goes wrong, wait ten seconds before stopping.** The interesting
   part is the aeroplane sitting in the failed state, not the moment it
   entered it. Records are flushed as they are written, so nothing is lost
   even if you kill it.
7. **Write down what you saw, in your own words** — "it pushed back about
   thirty feet then stopped and the nosewheel swung left and right" is worth
   more than any number, because it says which of several possible failures
   it was.

Then:

```bash
python -m aipilot debug-report logs\flight-....jsonl
python -m aipilot debug-report logs\flight-....jsonl --track taxi takeoff
```

The first says what looks wrong. The second replays the flight cycle by
cycle: position, heading, the heading it was *told* to hold, the track it
actually made good, speed, and the rudder that was sent. For anything on the
ground that second view is the one that matters.

**One fault per trace.** A trace containing three different problems is
harder to read than three traces containing one each.

## What is known to be still wrong

Do not spend time re-deriving these; they are open and understood.

1. **The taxi route zig-zags at dense terminal gates.** A route out of a
   Kennedy gate came out as twenty-two turns over 2.1 nm and clipped terminal
   structures. The taxiway graph welds endpoints within about sixteen feet and
   splits paths every 0.08 nm, which around a terminal produces nodes closer
   together than the aeroplane is long. `simplify()` runs at a four-degree
   tolerance, which is far too fine for that. **Reproduce it with
   `aipilot taxi`, offline, before touching any code.**
2. **The Horizon 787 may not report its brakes.** It sat at a gate with the
   thrust up and did not move: the parking-brake release is guarded on what
   the aeroplane reports, and a study-level add-on running its own hydraulics
   may report nothing. Version 1.7.0 added a watchdog that releases everything
   unconditionally after twelve seconds of open throttle and no movement — but
   whether the *event* reaches that aeroplane's brake system at all is
   unknown, and only testable here. `python -m aipilot lvars` will list the
   aircraft's local variables through the MobiFlight bridge if one is
   installed; that is how to find out what it really uses.
3. **The simulator does not identify itself.** Traces say "unknown (the
   simulator did not identify itself)", so the `SIMCONNECT_RECV_OPEN` parse is
   not working on this build. Harmless, but it means the trace cannot say
   which simulator produced it, which will matter later.
4. **The world-hub sweep has not been run.** `tests/hubs.py` has 55 airports
   from Amsterdam at -11 ft to Bogotá at 8361 ft, both polar circles and both
   sides of the antimeridian; `tests/worldfly.py` flies a route and checks it
   against every invariant. Nothing yet runs the matrix.

## What has already been fixed — do not go looking for these again

The tug being re-summoned as fast as it was released; the tug heading sent
four times a second; takeoff thrust applied on an apron; the approach never
converging on the centreline; the flare starting at eighty feet and floating
three thousand; the autopilot coming back after "your controls"; engaging in
the air near the destination flying back towards the departure; cruise levels
from true rather than magnetic course; and the takeoff roll slamming the
rudder over because "lined up" accepted two hundred feet off the centreline.

Each has a test named after the symptom in `tests/test_hardening.py`,
`tests/test_ground.py` or `tests/test_arrival_hardening.py`. If one of them
seems to be happening again, run the suite first — it is more likely to be a
new fault wearing an old face.

## House rules

* Every fix gets a test that fails before it and passes after.
* Do not loosen a tolerance to make a test pass. Two assertions in this
  project passed for months on behaviour that was plainly wrong, because the
  tolerances were set to whatever the code happened to do.
* The mock simulator in `aipilot/sim/mock.py` is only worth its fidelity. Four
  bugs hid behind places where it was kinder than the real thing. If the real
  simulator does something the mock does not, fix the mock first — then the
  bug becomes reproducible offline for everyone.
* Commit messages say what was wrong and why the fix is right, not what
  changed.
