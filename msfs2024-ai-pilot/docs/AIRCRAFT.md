# Aircraft support, and one honest limitation

## The short version

| Aircraft | Works | Needs the WASM module |
|---|---|---|
| Boeing 787-10 / 787-9 (default, Horizon) | Fully | No |
| Airbus A380-800 | Expected to | No |
| iniBuilds A350-900 / -1000 | Expected to | Not currently used |
| Headwind A330-900neo | Fully, with the module | Yes, for managed/selected modes |
| Airbus A320neo | Fully, with the module | Yes |
| Anything else | On a generic profile | No |

## Why Boeings are easy and Airbuses are not

Asobo wire the *standard* SimConnect events — `AP_MASTER`, `HEADING_BUG_SET`,
`AP_ALT_VAR_SET_ENGLISH` and the rest — straight into the autoflight system of
most aircraft. Set a heading, the aeroplane turns. That is the whole story for a
Boeing MCP, which simply holds what it is given, and it is why the 787 needs
nothing beyond a Python install.

An Airbus FCU does not hold a value. Every knob has two states:

- **pulled** — the value you selected is flown (*selected* guidance),
- **pushed** — the FMGC's own profile is flown (*managed* guidance).

Setting a number without pulling the knob changes nothing about where the
aeroplane goes. The AI Pilot *is* the FMGC here — it computes the heading,
altitude and speed itself and needs the aeroplane to fly exactly those — so
every value it sets must be followed by a pull.

And pulls are not SimConnect events. They are HTML gauge events (`H:` events)
inside the aircraft's own code, in a namespace SimConnect cannot see at all.
Reaching them needs a WASM module running inside the simulator to act as a
proxy, which is what the MobiFlight module is for.

## The limitation, stated plainly

**The iniBuilds A350 and A380 entries are deliberately empty.**

Look in [`aipilot/aircraft/profiles/fcu_conventions.json`](../aipilot/aircraft/profiles/fcu_conventions.json)
and you will find the FlyByWire convention filled in — it is documented, it is
used by the A32NX and by aircraft built on that codebase such as the Headwind
A330neo, and it is implemented here. The iniBuilds entries have a description
and nothing else.

That is not an oversight. Those aircraft's internal event names are not
published anywhere I could verify, and a guessed name does not fail loudly — it
fails **silently**. The command goes out, nothing receives it, the aeroplane
carries on doing whatever it was doing, and the only evidence is that the
autopilot seems not to be listening. Shipping a plausible-looking guess would
make this program worse than shipping nothing, because it would look like it
worked.

So instead: those aircraft are driven with standard SimConnect events, which
they do respond to for heading, altitude, speed and approach mode. In practice
that is most of a flight. If the aeroplane reverts to its own managed profile at
some point, that is this limitation showing.

## Filling in the gap yourself

If you know the right names — from the developer, from a forum, or from another
tool's profile for the same aeroplane — it is a text edit and takes effect
immediately. No code changes.

To find them:

1. Install the MobiFlight WASM module (see [INSTALL.md](INSTALL.md) step 4) and
   confirm it with `python -m aipilot doctor`.
2. Load the aeroplane and let it settle at the gate.
3. Watch some candidate names while you move the knob:

   ```bash
   python -m aipilot lvars A32NX_FCU_HDG_PULL AIRLINER_MCP_HDG INI_FCU_HDG_PULL --seconds 30
   ```

   Turn and pull the heading knob in the cockpit while it runs. Whichever value
   changes is the one you want.
4. Put it in `fcu_conventions.json` under the aircraft's convention name, and
   point the aircraft at that convention in
   [`aipilot/aircraft/registry.py`](../aipilot/aircraft/registry.py).

If you get a set working, it is worth sharing — that file is designed to be
contributed to.

## How to tell what is actually being used

The first two lines of every flight log say so:

```
[00:00] PREFLIGHT AI Pilot engaged: EGLL/27R to LFPG/26L, 194 nm at FL290
[00:00] PREFLIGHT Airbus A330-900neo (Headwind) via flybywire FCU over WASM bridge
```

The possibilities are:

- `via Boeing MCP (standard SimConnect events)` — everything is available.
- `via <name> FCU over WASM bridge` — full Airbus control including knob pulls.
- `via <name> FCU over standard events -- WASM bridge unavailable` — the module
  is not answering. It will still fly, and it warns once in the log.
- `via standard SimConnect events (no FCU convention configured)` — the iniBuilds
  case described above.

## Adding an aeroplane

Two steps, both data.

**A performance profile** in [`aipilot/perf/profiles.py`](../aipilot/perf/profiles.py):
climb and descent speeds, cruise Mach, flap placard speeds, approach speed,
ceiling, typical cruise level. Copy the nearest existing entry and adjust.

**A registry entry** in [`aipilot/aircraft/registry.py`](../aipilot/aircraft/registry.py):
which adapter class flies it, which FCU convention it uses if any, and what
aliases you want to type.

Then fly it in the mock first — `--sim mock` — which will catch a profile with
its numbers the wrong way round long before the simulator does.
