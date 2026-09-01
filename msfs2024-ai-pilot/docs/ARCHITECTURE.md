# How it is put together, and why

## The shape

```
                  ┌──────────────┐   ┌──────────┐
                  │  CLI  /  UI  │   │  doctor  │
                  └──────┬───────┘   └────┬─────┘
                         │                │
                  ┌──────▼────────────────▼──────┐
                  │      AIPilot controller      │   phases, configuration,
                  │  autopilot/controller.py     │   go-arounds, the landing
                  └──┬────────┬─────────┬────────┘
                     │        │         │
        ┌────────────▼──┐  ┌──▼──────┐  │
        │ LateralGuide  │  │ Vertical│  │   pure computation, no simulator
        │  lateral.py   │  │ Guidance│  │
        └───────────────┘  └─────────┘  │
                     │        │         │
        ┌────────────▼────────▼─────┐   │
        │  FlightPlan + profiles    │   │   route/, perf/, navdata/
        └───────────────────────────┘   │
                                        │
                  ┌─────────────────────▼────────┐
                  │      AircraftAdapter         │   intent → button presses
                  │  aircraft/base.py, airbus.py │
                  └──────────────┬───────────────┘
                                 │
                  ┌──────────────▼───────────────┐
                  │         SimBackend           │   sim/base.py
                  └───┬──────────────────────┬───┘
                      │                      │
            ┌─────────▼────────┐   ┌─────────▼────────┐
            │ SimConnectBackend│   │     MockSim      │
            │   (+ MobiFlight) │   │  point-mass model│
            └──────────────────┘   └──────────────────┘
```

Two decisions shape everything else.

## The simulator is behind an interface, with a real second implementation

`SimBackend` has two implementations: SimConnect, and a point-mass model that
integrates position, speed and altitude against the same events the real
autopilot responds to.

This is not a stub. The mock has a wind triangle, a bank-limited turn rate, a
climb-rate ceiling that degrades with altitude, configuration changes that take
seconds to run, a surface wind gradient, and an autoland flare for aircraft that
have one. It is a *closed loop*: the guidance commands, the aeroplane responds
imperfectly, and the guidance sees the result.

That makes the interesting part of this project testable without Microsoft
Flight Simulator, on Linux, in CI, in seven seconds. Which matters more than it
sounds, because of the second decision.

## Guidance is pure, and the tests fly whole flights

`geo.py`, `route/`, `perf/`, `lateral.py` and `vertical.py` contain no I/O and no
simulator types. They take numbers and return numbers, so they can be tested
exactly.

But the bugs that actually happened were not in any of them. Every genuine defect
found while building this was a *whole-flight* bug, invisible from inside the
component that contained it:

- Top of descent was declared forty seconds after takeoff, because the test for
  "already below the approach altitude" is trivially true on the runway at the
  departure end.
- The approach phase began three hundred miles out, because it keyed off which
  leg was active rather than off distance — and on a short sector the approach
  fixes are the active leg almost immediately. The whole flight was then flown
  at 210 knots.
- Lateral guidance ran out of route at the threshold and turned back to chase a
  fix now behind the aeroplane — a go-around at fifty feet, every time.
- `clear_vertical_speed()` cleared a local cache but never told the aeroplane to
  leave vertical-speed mode, so after a go-around it sat at five hundred feet
  with full thrust and a three thousand foot target it could not climb to.
- "Fly direct to a fix" kept the leg's original origin, so an aeroplane already
  past that fix flew the leg's course outbound for ever — cross-track error
  obediently near zero the whole way, because it was precisely on the extended
  centreline, going the wrong way.
- Picking the nearest useful fix by "distance to it plus route remaining after
  it" always picks the *last* fix, because along a nearly straight route those
  two terms trade off exactly and the last one wins the tie.

Not one of those is visible in a unit test of the function containing it. All of
them are obvious the moment an aeroplane flies a complete flight and ends up in
the wrong place. So the test suite flies complete flights — brakes off to a full
stop, every aircraft, a twenty-hour long haul, a date-line crossing, a forced
go-around, four control rates — and asserts on where the aeroplane got to.

## Layer by layer

### `geo.py` — spherical geodesy
Haversine distance, great-circle bearing, cross-track and along-track, turn
radius and anticipation, the wind triangle. A sphere is the right model: the
worst-case error against WGS-84 is 0.3% of distance but a fraction of a degree of
*bearing*, and bearing is what steers the aeroplane.

### `sim/` — the boundary
Everything above works in knots, feet and degrees. SimConnect deals in radians,
metres and metres per second. All conversion happens here and never leaks
upward. `simconnect.py` is a ctypes binding of the dozen calls needed;
`mobiflight.py` is the local-variable bridge, with every protocol constant
isolated in one overridable dataclass because it is a convention rather than a
published API.

### `navdata/` — what the user already has
A provider chain: Little Navmap's scenery database, then OurAirports CSVs, then a
bundled sample. The Little Navmap provider inspects the tables it finds with
`PRAGMA table_info` and adapts, rather than assuming a schema that is not a
stable contract, and reports a clear reason instead of raising SQL errors from
inside a query.

### `route/` — geometry instead of licensed data
Procedures are licensed and change every 28 days, and half the point of the
original AI Pilot was that you typed two airports and it went. So: a great circle
split into named segments, and an approach built backwards from the threshold.

The approach join is chosen by *measurement*, not prediction. Each of three
styles — straight in, base leg, full circuit — is built, and the turn the
aeroplane would actually have to fly at the first approach fix is computed. The
first style that comes out flyable is used. Predicting it from the angle between
the arrival track and the runway does not work: the approach fixes sit tens of
miles out and off to one side, so the two can differ by fifty degrees.

### `perf/` — numbers a pilot would recognise
Climb and descent schedules, flap placard speeds, approach speeds, ceilings.
Operational data at the fidelity a crew works to, not certification data — and
all of it overridable from JSON, because it varies with weight and with each
add-on's flight model, and someone who flies one aeroplane a lot will know better
than this table does.

### `aircraft/` — intent to button presses
Adapters absorb the difference between a Boeing MCP that holds what it is given
and an Airbus FCU where a value means nothing until the knob is pulled. Two
details worth noting: flaps are moved one detent at a time against the *reported*
handle index rather than commanded absolutely, because `FLAPS_SET` scaling
differs between aircraft with different numbers of detents; and true-to-magnetic
conversion happens here, once, so guidance can work entirely in true degrees.

### `autopilot/` — when, rather than what
The controller decides when to rotate, when to raise the gear, when to leave the
cruise level, when to configure, when to hand back. Three rules run through it:

- **Phases only move forward.** Checked against a fixed order, so a blip in
  altitude or distance cannot send the aeroplane back to a phase it has left.
- **Configuration follows speed, not just distance.** Flaps come out when the
  aeroplane is slow enough for them. Backwards, and you get a flap overspeed
  twelve miles out.
- **It says when it cannot do something.** Without an ILS and an autoland-capable
  aeroplane there is no honest way to complete a landing precisely, so it either
  flies its own path and flares — and says that is what it is doing — or hands
  over, loudly.

## Things deliberately not done

**No PID controllers.** Every loop here is proportional with a clamp. Cross-track
correction is 3°/nm limited to 45°, which is the intercept a controller would
give and which rolls out without overshooting. Integral terms would add
wind-up on a channel that is already re-derived from geometry every cycle.

**No direct control-surface commands.** Everything goes through the aeroplane's
own autopilot. Fighting it for the elevator would work in one aircraft and be
wrong in the next; the flare is the one place a rate is commanded, and even that
goes through the autopilot's vertical-speed channel.

**No dependencies.** This is a tool people run on the same machine as a flight
simulator, often while the simulator is using most of it. `pip install` a
dependency tree is a worse first five minutes than a file that starts instantly.
`ctypes`, `sqlite3` and `http.server` are entirely adequate for one user on
localhost.
