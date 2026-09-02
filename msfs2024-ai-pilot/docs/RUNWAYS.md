# Which runway, and how it knows

A flight plan is mostly decided by two runways. Get them wrong and
everything downstream is wrong with them: the taxi route goes to the wrong
end of the field, the approach is built onto a runway nobody is using, and
you arrive with a tailwind.

Until recently this program assumed the wind was calm unless you typed it
in, which quietly picked whichever runway was longest and had an ILS.
That is right perhaps half the time. It now asks.

## Where the wind comes from

Each end of the flight is resolved on its own. A single wind applied to
both is wrong for anything longer than a hop -- the wind at the far end
decides the landing runway, and it has nothing to do with the wind you are
sitting in.

In order of preference:

| Source | Used for | Why it ranks there |
| --- | --- | --- |
| `--departure-runway` / `--arrival-runway` | Either end | You said so. |
| A SimBrief plan's `plan_rwy` | Either end | It is the paperwork for this flight. |
| `--wind 250/35` | Both ends | You said what it is doing. |
| The simulator's own wind | Departure only | Not a forecast: the wind the aeroplane is about to take off into. |
| The current METAR | Either end | The real world, an hour ago at worst. |
| Calm | Either end | Nothing is known, so length and ILS decide. |

The simulator only ranks that high for the departure, and only while the
aeroplane is within 30 nm of the departure airport. It has nothing to say
about the weather at a destination four hours away. If it names a
different runway from the one planned, the departure is rebuilt and the
change is printed:

```
  The simulator's wind is 271 at 22 kt, so departing runway 27L instead of 09R.
```

This matters more than it sounds. If you fly with a preset -- clear skies,
or a date wound back -- the real METAR knows nothing about your weather,
and only the sim does.

## The weather lookup

METARs come from `aviationweather.gov`, the US National Weather Service's
aviation weather service. It is free, needs no account and no key, and
covers the world rather than just the United States. Microsoft Flight
Simulator's own Live Weather is built from the same observations, so using
it lines the plan up with the weather in the sim.

It happens automatically. One request, a six second timeout, and if the
network is not there the flight plans as if calm and says so:

```
No live weather (Could not reach the weather service: timed out) --
runways were chosen as if calm.
```

Turn it off with `--no-metar` if you would rather it never went looking.

Every plan now says why it picked what it picked, which is the point:

```
KJFK departure runway 04L: the METAR 040 at 14 kt, +14 kt down the runway.
EGLL arrival runway 27R: the METAR 250 at 11 kt, +10 kt down the runway.
```

## SimBrief

If you already plan your flights in SimBrief, fly the plan you made:

```
python -m aipilot fly --simbrief YOUR_SIMBRIEF_USERNAME
```

The airports come from the plan, so you do not type them. It takes:

* origin and destination
* the planned runway at each end (`plan_rwy`)
* the route
* the initial cruise altitude
* the METAR embedded in the release, used if the live lookup fails

Anything you type on the command line wins. `--simbrief someone
--arrival-runway 27L` flies the SimBrief route onto the runway you asked
for.

Your numeric SimBrief pilot ID works in place of the username.

Only your own most recent plan is available, which is what SimBrief's API
offers. Nothing is sent to SimBrief except the identifier you pass.

### What gets dropped from the route

A SimBrief route mixes fixes with airway identifiers (`UL607`), procedure
names (`ROBUC3`) and speed and level changes (`N0489F370`). This program
flies fix to fix, so those are removed and the fixes either side of them
kept:

```
ROBUC3 BAF Q436 EBONY/N0489F370 NERTU DCT   ->   BAF EBONY NERTU
```

They are removed by shape rather than left to the navigation data to
reject, because an airway identifier that happened to match some unrelated
navaid on the far side of the world would bend the route to it.

This means SIDs and STARs are not flown as published. The departure is a
straight climb off the runway and the arrival is built from the runway
backwards. That is a real limitation, not an oversight.

## FlightRadar24

Not supported, and it is worth saying why rather than leaving it looking
like something nobody got round to.

* There is no free API. The commercial one is priced for airlines and
  aviation businesses, not for a program that plans one flight at a time.
* Scraping the website instead would breach its terms of use. I am not
  going to ship something that gets your address blocked.
* Most importantly, it would not answer the question. FlightRadar24 shows
  where aircraft are, not which runway is in use. Working out the runway
  from the tracks would mean inferring it from the approach paths of
  recent arrivals -- which is exactly what the wind already tells you,
  more directly and more reliably.

The METAR lookup gets you the same answer by the route real crews use:
the wind decides the runway. Where you want the actual operational choice
rather than the meteorological one -- a runway closed for work, or a noise
preference the wind does not explain -- name it with `--departure-runway`
or `--arrival-runway`, or plan it in SimBrief and let `--simbrief` bring
it across.
