# Pushback, taxi, lights and signs

## What it does

Given taxiway data, the AI Pilot will push back off the stand, taxi to the
departure runway on the actual taxiways, line up, and go — with the lights and
cabin signs set correctly at each stage.

```
[00:00] PREFLIGHT  Pushing back 182 ft, turning onto 270 degrees.
[00:37] PUSHBACK   Pushback complete
[00:39] PUSHBACK   Taxiing to 09: 0.90 nm, 6 turns.
[04:52] TAKEOFF    lined up, cleared for takeoff
```

## Your scenery, not a guess

**It reads Little Navmap's database, which Little Navmap compiles from the
scenery you actually have installed.** So a custom KJFK is *your* custom KJFK —
its taxiways, its stands, its runway positions — not a stock layout that
happens to share the ICAO code. That is the main reason this project uses Little
Navmap as its first data source rather than any static dataset.

The one thing to watch: Little Navmap's database is a snapshot. **After you
install or update scenery, re-run its scenery library load** (Scenery Library →
Load Scenery Library) or it will still be describing the old airport. Run
`python -m aipilot doctor` afterwards and it will tell you which database it
found and what it read out of it.

**Without taxiway data, nothing moves.** That is deliberate. If there is no
network to follow, the AI Pilot says so and waits for you to taxi out and line
up, and takes over from there. Guessing a path across an apron is how an
aeroplane ends up in a building — which is exactly what an earlier version of
this did, from a stand at Los Angeles.

## "Avoiding objects"

Worth being precise, because it is a real limitation.

**There is no obstacle sensing.** Nothing in SimConnect will tell an external
program what scenery is where — no buildings, no parked aircraft, no ground
vehicles, no jet bridges. It cannot see a thing, and any claim otherwise would
be false.

What it does instead is **stay on the taxiway centrelines**, which is what
avoiding things means on an airfield: the pavement is, by construction, the part
with nothing parked on it. In the test layout it holds the centreline to within
a few feet on the straights, and cuts corners on sharp turns roughly as much as
a real wide-body does.

What that does *not* protect against: another aircraft on the same taxiway,
ground vehicles, anything parked where it should not be, and a stand whose
lead-in line the scenery does not describe. Watch it. It is an autopilot, not a
driver.

## How the taxi route is worked out

1. Taxiway centreline segments are read from the scenery database.
2. They are welded into a graph — segment endpoints that are within about five
   metres become one junction. Scenery does not guarantee that the end of one
   segment is bitwise identical to the start of the next, and without welding
   the network falls apart into thousands of disconnected pieces.
3. A* finds the shortest path from the aeroplane to the runway holding point.
4. The route is simplified down to its turns, then extended onto the runway so
   the aeroplane lines up rather than stopping at the hold.

Steering is pure pursuit plus a cross-track term: it aims at a point ahead
*and* corrects towards the centreline. Pursuit alone is perfectly happy to
arrive at the next point having missed the whole leg in between, which on a
taxiway means crossing the grass. The lookahead distance is derived from the
turning circle rather than fixed — a lookahead point inside the turning circle
can never be reached, and the aeroplane orbits it instead.

## Pushback

Needed for two reasons, and it checks for both:

- The aeroplane cannot reach the taxiways from where it stands.
- It can, but is **pointing the wrong way** — parked nose-in, the route starts
  behind it, and an aeroplane asked to drive to a point behind it turns a
  hundred and eighty degrees on the spot across whatever the stand is next to.

The push is straight back, turning onto the heading of the first leg it then
has to taxi, and stops once clear.

**This part is best effort.** The simulator's tug is driven by two events whose
behaviour varies between aircraft, so it is deliberately simple and it says what
it is doing at each step, so a wrong turn is obvious rather than mysterious. If
your aeroplane does not respond, push back by hand — the AI Pilot will pick up
the taxi as soon as you stop.

## Lights and cabin signs

Every one of these is operated by a **toggle** event. A toggle sent without
knowing the current state does the right thing half the time, which for a
landing light means arriving at night with it off. So the AI Pilot reads the
switch position the simulator reports and acts only when it is actually wrong —
which also means it never fights a switch you set yourself.

| | On | Off |
|---|---|---|
| Nav | always | — |
| Beacon | always with engines running | shutdown |
| Taxi | pushback and taxi | lining up on the runway |
| Strobes | entering the runway | clear of the runway after landing |
| Landing | lining up | 10,000 ft climbing / clear of the runway |
| | 10,000 ft descending | |
| Wing, logo | on the ground and below 10,000 ft | above 10,000 ft |
| Seatbelts | pushback to 10,000 ft | cruise above 10,000 ft |
| | top of descent to the gate | |
| No smoking | always | — |

## Turning it off

```bash
python -m aipilot fly KJFK KBOS --no-taxi      # you taxi, it flies
python -m aipilot fly KJFK KBOS --no-lights    # do not touch any switches
```

## ATC

Not integrated yet, and worth saying what that means rather than implying it
half works.

The AI Pilot flies its own plan. It does not hear ATC, does not read back, and
will not follow a vector or a level it was not given. With **BeyondATC** running
you will get instructions it does not know about — so for now, fly it as you
would with any autopilot: take the instruction, and pass it on with
`--cruise` for a level, or `--arrival-runway` for a runway change, before you
start.

Making it follow ATC properly needs a way to receive the instructions. The
sensible shape is a small command channel — the control panel already has one —
so an instruction can be relayed to it mid-flight, either by you typing it or by
an add-on that can emit it. That is a real feature, not a small one, and it is
not written yet.
