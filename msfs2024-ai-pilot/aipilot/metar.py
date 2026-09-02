"""Real-world wind, so the runway choice matches the one in use.

A runway is chosen almost entirely by the wind, and until now the planner
had no idea what the wind was doing unless you typed it in. It therefore
assumed calm, which quietly picks whichever runway happens to have an ILS
and is longest -- often the opposite end to the one everybody is using.

METAR is the free, official source for what the wind is actually doing:
the US National Weather Service publishes it worldwide at
aviationweather.gov with no key and no account. Microsoft Flight
Simulator's own "Live Weather" is built from the same observations, so
using METAR here lines the plan up with the weather in the sim.

Only the wind is parsed. Everything else in a METAR is weather the
aeroplane flies through rather than something the plan depends on.
"""

from __future__ import annotations

import re
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Iterable, Optional

#: The public, key-free observation service run by the US NWS Aviation
#: Weather Center. Worldwide, not just the United States.
METAR_URL = "https://aviationweather.gov/api/data/metar"

#: Long enough for a slow link, short enough that a blocked network does not
#: leave you staring at the sim wondering whether anything is happening.
DEFAULT_TIMEOUT_S = 6.0

#: A gust that changes which runway is sensible. Below this the steady wind
#: decides on its own.
GUST_WEIGHT = 0.5

#: Faster than any surface wind on this planet. Beyond it the observation is
#: corrupt, and a corrupt wind must not choose a runway.
MAX_PLAUSIBLE_WIND_KT = 250.0

#: The leading guard is a negative lookbehind, not \\b: a word boundary needs a
#: word character on one side, and "///25KT" begins with a slash, so the
#: not-reported-direction case could never match and a known twenty-five knot
#: wind was quietly read as calm.
_WIND_GROUP = re.compile(
    r"(?<![\w/])(?P<dir>\d{3}|VRB|///)(?P<speed>\d{2,3}|//)"
    r"(?:G(?P<gust>\d{2,3}))?(?P<unit>KT|MPS|KMH)\b"
)
_STATION = re.compile(r"\b([A-Z][A-Z0-9]{3})\s+\d{6}Z\b")

_UNIT_TO_KT = {"KT": 1.0, "MPS": 1.9438444924406, "KMH": 0.5399568034557}


class MetarError(RuntimeError):
    """The observations could not be fetched."""


@dataclass(frozen=True)
class MetarWind:
    """The wind from one observation.

    ``from_deg`` is the direction the wind is blowing *from*, in degrees
    true -- the same convention as a runway heading, which is what makes
    the comparison in :func:`select_runway` work. It is ``None`` when the
    wind is variable or was not reported, in which case there is no
    direction to steer by and the runway falls back to whatever else the
    planner prefers.
    """

    from_deg: Optional[float]
    speed_kt: float
    gust_kt: Optional[float] = None
    variable: bool = False

    @property
    def calm(self) -> bool:
        return self.from_deg is None or self.speed_kt < 3.0

    @property
    def planning_speed_kt(self) -> float:
        """The speed to choose a runway with.

        Half the gust is added, the way a crew reading a gusting wind will
        lean further into it than the steady figure alone suggests.
        """
        if self.gust_kt is None:
            return self.speed_kt
        return self.speed_kt + GUST_WEIGHT * max(0.0, self.gust_kt - self.speed_kt)

    def describe(self) -> str:
        if self.from_deg is None:
            return f"variable {self.speed_kt:.0f} kt" if self.speed_kt else "calm"
        text = f"{self.from_deg:03.0f} at {self.speed_kt:.0f} kt"
        if self.gust_kt:
            text += f", gusting {self.gust_kt:.0f}"
        return text


#: What the planner falls back to when nothing is known.
CALM = MetarWind(from_deg=None, speed_kt=0.0)


@dataclass(frozen=True)
class MetarReport:
    station: str
    wind: MetarWind
    raw: str


def parse_metar(raw: str) -> Optional[MetarReport]:
    """Read the station and the wind out of one raw METAR.

    Returns ``None`` for anything that is not recognisably a METAR, so a
    blank line or an error page from the service is simply ignored rather
    than becoming an exception halfway through a flight plan.
    """
    text = " ".join((raw or "").split()).upper()
    if not text:
        return None

    station_match = _STATION.search(text)
    if station_match is None:
        # Some feeds drop the observation time. Fall back to the first
        # token that looks like an ICAO code, skipping the report type.
        tokens = [t for t in text.split() if t not in ("METAR", "SPECI", "COR", "AUTO")]
        if not tokens or not re.fullmatch(r"[A-Z][A-Z0-9]{3}", tokens[0]):
            return None
        station = tokens[0]
    else:
        station = station_match.group(1)

    return MetarReport(station=station, wind=_parse_wind(text), raw=text)


def _parse_wind(text: str) -> MetarWind:
    match = _WIND_GROUP.search(text)
    if match is None:
        return CALM

    unit = _UNIT_TO_KT[match.group("unit")]
    speed_text = match.group("speed")
    if speed_text == "//":
        return CALM
    speed_kt = float(speed_text) * unit
    if speed_kt > MAX_PLAUSIBLE_WIND_KT:
        # A corrupt observation should not decide which runway to use.
        return CALM

    gust_text = match.group("gust")
    gust_kt = float(gust_text) * unit if gust_text else None

    direction_text = match.group("dir")
    if direction_text in ("VRB", "///"):
        return MetarWind(None, speed_kt, gust_kt, variable=direction_text == "VRB")

    direction = float(direction_text) % 360.0
    if speed_kt == 0.0:
        # 00000KT is calm; the direction is a placeholder, not north.
        return CALM
    return MetarWind(direction, speed_kt, gust_kt)


def fetch_metar(icaos: Iterable[str],
                timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, MetarReport]:
    """Fetch the latest observation for each airport.

    Raises :class:`MetarError` if the service cannot be reached. Airports
    with no observation are simply absent from the result: plenty of
    smaller fields never report, and that is not an error.
    """
    wanted = [code.strip().upper() for code in icaos if code and code.strip()]
    if not wanted:
        return {}

    url = f"{METAR_URL}?ids={','.join(dict.fromkeys(wanted))}&format=raw"
    request = urllib.request.Request(url, headers={"User-Agent": "aipilot"})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            body = response.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as exc:
        raise MetarError(
            f"The weather service answered {exc.code} for {', '.join(wanted)}."
        ) from exc
    except (urllib.error.URLError, OSError, TimeoutError) as exc:
        raise MetarError(f"Could not reach the weather service: {exc}") from exc

    reports: dict[str, MetarReport] = {}
    for line in body.splitlines():
        report = parse_metar(line)
        if report is not None:
            reports.setdefault(report.station, report)
    return reports


def wind_for(reports: dict[str, MetarReport], icao: str) -> Optional[MetarWind]:
    report = reports.get((icao or "").strip().upper())
    return report.wind if report is not None else None
