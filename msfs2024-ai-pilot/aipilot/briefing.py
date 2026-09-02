"""Working out what the flight should be, before planning it.

Two questions have to be answered before a route can be built, and until
now both were answered by guessing:

* Which runways? Guessed from a wind of zero, which is to say guessed.
* What is the wind actually doing? Nobody said, so calm.

This module answers them from real sources -- a SimBrief flight plan if
you use one, and the current METAR at both airports -- and falls back
quietly and completely when there is no network, so a flight never fails
because a weather service was slow.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from .metar import MetarError, MetarWind, fetch_metar, parse_metar
from .route.planner import AirportWind

Reporter = Callable[[str], None]


def _quiet(_message: str) -> None:
    pass


@dataclass(frozen=True)
class WindBriefing:
    """The wind to plan each end with."""

    departure: AirportWind
    arrival: AirportWind


def resolve_winds(
    origin_icao: str,
    destination_icao: str,
    typed: Optional[tuple[float, float]] = None,
    use_metar: bool = True,
    simbrief_metars: Optional[tuple[Optional[str], Optional[str]]] = None,
    timeout_s: Optional[float] = None,
    report: Reporter = _quiet,
) -> WindBriefing:
    """Decide the planning wind at each end.

    In order of preference:

    1. A wind you typed. If you say what it is doing, that is the answer.
    2. The METAR embedded in a SimBrief plan, which is the weather that
       plan was built for.
    3. The current METAR, fetched from the aviation weather service.
    4. Calm.

    The two ends are resolved separately. A single wind applied to both is
    wrong for anything longer than a hop -- the wind at the far end is what
    decides the landing runway, and it has nothing to do with the wind you
    are sitting in.
    """
    if typed is not None and (typed[0] or typed[1]):
        given = AirportWind(typed[0] % 360.0, abs(typed[1]), "the wind you gave")
        return WindBriefing(given, given)

    departure = arrival = AirportWind.calm()

    if simbrief_metars:
        departure = _from_metar_text(simbrief_metars[0], "the SimBrief plan") or departure
        arrival = _from_metar_text(simbrief_metars[1], "the SimBrief plan") or arrival

    needed = [code for code, wind in ((origin_icao, departure), (destination_icao, arrival))
              if use_metar and code and wind.source == "no wind information"]
    if needed:
        try:
            kwargs = {"timeout_s": timeout_s} if timeout_s is not None else {}
            reports = fetch_metar(needed, **kwargs)
        except MetarError as exc:
            report(f"No live weather ({exc}) -- runways were chosen as if calm.")
            return WindBriefing(departure, arrival)
        for code, report_ in reports.items():
            wind = _to_airport_wind(report_.wind, "the METAR")
            if code == (origin_icao or "").upper() and departure.source == "no wind information":
                departure = wind
            if code == (destination_icao or "").upper() and arrival.source == "no wind information":
                arrival = wind
        missing = [code for code in needed if code.upper() not in reports]
        if missing:
            report(f"No observation for {', '.join(missing)} -- "
                   "its runway was chosen as if calm.")

    return WindBriefing(departure, arrival)


def wind_from_sim(state, source: str = "the simulator") -> Optional[AirportWind]:
    """The wind the simulator itself says is blowing.

    This beats every other source for the departure runway, because it is
    not a forecast or an observation of the real world -- it is the wind
    the aeroplane is about to take off into. Someone flying with preset
    weather, or with the clock wound back, has a sim wind that the real
    METAR knows nothing about.
    """
    direction = getattr(state, "wind_from_deg", None)
    speed = getattr(state, "wind_kt", None)
    if direction is None or speed is None:
        return None
    if speed < 3.0:
        return AirportWind.calm(source)
    return AirportWind(float(direction) % 360.0, float(speed), source)


def _from_metar_text(raw: Optional[str], source: str) -> Optional[AirportWind]:
    if not raw:
        return None
    report = parse_metar(raw)
    if report is None:
        return None
    return _to_airport_wind(report.wind, source)


def _to_airport_wind(wind: MetarWind, source: str) -> AirportWind:
    if wind.from_deg is None:
        # Variable or unreported: there is no direction to steer by, so let
        # the planner fall back to length and ILS rather than inventing one.
        return AirportWind.calm(f"{source} (variable)")
    return AirportWind(wind.from_deg, wind.planning_speed_kt, source)
