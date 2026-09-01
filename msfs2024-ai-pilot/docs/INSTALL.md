# Setting it up

## 1. Python

Python 3.9 or newer, from [python.org](https://www.python.org/downloads/). Tick
"Add Python to PATH" during installation.

There is nothing else to install. No `pip install`, no virtual environment, no
dependency tree. Everything the AI Pilot needs is in the standard library:
SimConnect is driven through `ctypes`, the nav data through `sqlite3` and `csv`,
and the control panel through `http.server`.

Check it works before the simulator is anywhere near it:

```bash
python -m aipilot fly EGLL EGCC --sim mock --speed 200
```

That flies a complete flight offline in about ten seconds. If you see it take
off, climb, cruise, descend and land, the software is fine and everything after
this point is about connecting it to the simulator.

## 2. SimConnect.dll

The AI Pilot talks to the simulator through `SimConnect.dll`. It looks for it in
this order:

1. `%AIPILOT_SIMCONNECT_DLL%`, if you set it.
2. Next to the `aipilot` package — the easiest option: just drop the file there.
3. The usual SDK and simulator install locations.

If you do not have it, install the MSFS 2024 SDK from inside the simulator
(Options → General → Developers → SDK installer), or copy `SimConnect.dll` out
of any other tool that ships one.

```bash
python -m aipilot doctor
```

tells you which of these worked.

## 3. Navigation data

Out of the box there is a small bundled sample of major airports. It has **no
runway data**, so the AI Pilot invents a plausible runway and says so in a
warning every time. Fine for a demo; not fine for actually landing, since the
invented runway can be a mile from the real one.

Pick one of these — either is a large improvement, and both together is best.

### Little Navmap (recommended)

If you already have [Little Navmap](https://albar965.github.io/littlenavmap.html),
you already have the best data available: it is compiled from your *installed
scenery*, so its runways and ILS frequencies match what the aeroplane will
actually see, including third-party airports.

Nothing to configure. It is found automatically at:

```
%APPDATA%\ABarthel\little_navmap_db\little_navmap_msfs24.sqlite
```

Or point at it explicitly:

```bash
python -m aipilot fly EGLL KJFK --navdata "C:\path\to\little_navmap_msfs24.sqlite"
```

The database is opened strictly read-only, so it is safe to use while Little
Navmap itself is running.

### OurAirports (no ILS, but public domain and one download)

Download these two files and put them in the folder you run from:

- <https://davidmegginson.github.io/ourairports-data/airports.csv>
- <https://davidmegginson.github.io/ourairports-data/runways.csv>

They are found automatically. They give exact runway thresholds worldwide, but
no ILS frequencies — so approaches are flown on the AI Pilot's own computed path
rather than on the aeroplane's ILS receiver. It still lands; it is just less
precise, and there is no autoland.

Explicit paths, if you keep them elsewhere:

```bash
python -m aipilot fly EGLL KJFK --airports-csv path\to\airports.csv \
                                --runways-csv path\to\runways.csv
```

## 4. The WASM module — only for some aeroplanes

Skip this entirely if you fly the 787. It is not needed and nothing is missing
without it.

Aircraft whose autoflight panel lives in *local variables* — Airbuses, mostly —
need a WASM module inside the simulator to reach them, because SimConnect cannot.
The AI Pilot uses the free
[MobiFlight WASM module](https://github.com/MobiFlight/MobiFlight-WASM-Module),
which many people already have installed for their hardware.

Install it into your Community folder, restart the simulator, then:

```bash
python -m aipilot doctor
```

The report says whether the module answered. If it did not, the AI Pilot falls
back to standard events and tells you once, in the flight log, rather than
silently sending commands nowhere.

See [AIRCRAFT.md](AIRCRAFT.md) for what this actually changes per aeroplane.

## 5. Flying

Put the aeroplane on a runway, engines running, ready to go. Then:

```bash
python -m aipilot fly EGLL LFPG --aircraft b787-10
```

or open the control panel:

```bash
python -m aipilot ui --open
```

Press Ctrl-C at any time. The AI Pilot stops commanding and leaves the autopilot
exactly as it is, so you take over from a known state rather than from a
disconnect.

## When things do not work

**"SimConnect.dll not found"** — step 2.

**"Connected, but no flight data is arriving"** — the simulator is at the main
menu or still loading. Get into a flight and try again.

**It connects but the aeroplane ignores it** — you are almost certainly on an
aircraft whose autopilot is driven by local variables. Run `doctor`, check the
WASM bridge section, and read [AIRCRAFT.md](AIRCRAFT.md).

**The approach is not lined up with the runway** — no runway data. Look for the
"no runway data, so a runway was assumed" warning in the log, and do step 3.

**It picked a runway I did not want** — pass `--arrival-runway 27L`. Runway
choice comes from the planning wind, which defaults to calm; give it
`--wind 250/20` and it will choose the way the tower would.

**The descent starts too early or too late** — that is the type's performance
profile. Every number in it can be overridden without touching code:

```bash
python -m aipilot fly EGLL KJFK --profiles my_profiles.json
```

```json
{ "a380-800": { "descent_angle_deg": 2.7, "cruise_mach": 0.84 } }
```
