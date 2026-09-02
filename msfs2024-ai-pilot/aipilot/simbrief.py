"""Import a flight plan from SimBrief.

SimBrief is what most people already use to plan a flight before they fly
it, and its dispatch release names the runways the flight was planned for.
Pulling that in means the AI Pilot departs and arrives on the same runways
as the paperwork, and flies the same route, without any of it being typed
twice.

SimBrief publishes a documented, key-free endpoint that returns the
signed-in user's most recent flight plan:

    https://www.simbrief.com/api/xml.fetcher.php?username=NAME&json=1
    https://www.simbrief.com/api/xml.fetcher.php?userid=123456&json=1

Only the user's own latest plan is available, which is exactly what is
wanted here. Nothing is sent but the identifier you pass on the command
line.
"""

from __future__ import annotations

import json
import re
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Optional

SIMBRIEF_URL = "https://www.simbrief.com/api/xml.fetcher.php"

DEFAULT_TIMEOUT_S = 10.0


class SimBriefError(RuntimeError):
    """The flight plan could not be fetched or made sense of."""


@dataclass(frozen=True)
class SimBriefPlan:
    """The parts of a SimBrief release this program can use.

    Everything is optional: SimBrief plans vary by airline template, and a
    missing field should quietly fall back to the planner's own choice
    rather than stopping the flight.
    """

    origin: Optional[str] = None
    destination: Optional[str] = None
    alternate: Optional[str] = None
    departure_runway: Optional[str] = None
    arrival_runway: Optional[str] = None
    route: Optional[str] = None
    cruise_altitude_ft: Optional[float] = None
    aircraft_icao: Optional[str] = None
    callsign: Optional[str] = None
    origin_metar: Optional[str] = None
    destination_metar: Optional[str] = None
    notes: list[str] = field(default_factory=list)

    def describe(self) -> str:
        legs = f"{self.origin or '????'} to {self.destination or '????'}"
        bits = [f"SimBrief plan {legs}"]
        if self.callsign:
            bits.append(f"as {self.callsign}")
        if self.aircraft_icao:
            bits.append(f"in a {self.aircraft_icao}")
        runways = []
        if self.departure_runway:
            runways.append(f"off {self.departure_runway}")
        if self.arrival_runway:
            runways.append(f"onto {self.arrival_runway}")
        if runways:
            bits.append(", ".join(runways))
        if self.cruise_altitude_ft:
            bits.append(f"at FL{self.cruise_altitude_ft / 100:.0f}")
        return " ".join(bits)


def fetch_plan(user: str, timeout_s: float = DEFAULT_TIMEOUT_S) -> SimBriefPlan:
    """Fetch and read the latest SimBrief plan for ``user``.

    ``user`` may be either a SimBrief username or the numeric pilot ID
    shown on the account page; which one it is is worked out from the text.
    """
    identifier = (user or "").strip()
    if not identifier:
        raise SimBriefError("No SimBrief user given.")

    key = "userid" if identifier.isdigit() else "username"
    query = urllib.parse.urlencode({key: identifier, "json": "1"})
    request = urllib.request.Request(f"{SIMBRIEF_URL}?{query}",
                                     headers={"User-Agent": "aipilot"})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            body = response.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as exc:
        raise SimBriefError(f"SimBrief answered {exc.code} for {identifier!r}.") from exc
    except (urllib.error.URLError, OSError, TimeoutError) as exc:
        raise SimBriefError(f"Could not reach SimBrief: {exc}") from exc

    return parse_plan(body)


def parse_plan(body: str) -> SimBriefPlan:
    """Read a SimBrief JSON response.

    Kept separate from the fetch so it can be tested without a network,
    and so a saved copy of a release can be replayed.
    """
    try:
        data = json.loads(body)
    except ValueError as exc:
        # SimBrief answers a bad username with an XML error rather than
        # JSON, so say something useful instead of a decoding error.
        snippet = " ".join((body or "").split())[:120]
        raise SimBriefError(
            "SimBrief did not return a flight plan. It usually means the "
            "username is wrong, or that account has never generated one. "
            f"It said: {snippet or '(nothing)'}"
        ) from exc

    if not isinstance(data, dict):
        raise SimBriefError("SimBrief returned something unexpected.")

    status = str(_dig(data, "fetch", "status") or "").strip()
    if status and not status.lower().startswith("success"):
        raise SimBriefError(f"SimBrief could not give us a plan: {status}")

    notes: list[str] = []
    origin = _icao(_dig(data, "origin", "icao_code"))
    destination = _icao(_dig(data, "destination", "icao_code"))
    alternate = _icao(_dig(data, "alternate", "icao_code"))
    plan = SimBriefPlan(
        origin=origin,
        destination=destination,
        alternate=alternate,
        departure_runway=_runway(_dig(data, "origin", "plan_rwy")),
        arrival_runway=_runway(_dig(data, "destination", "plan_rwy")),
        route=_route(_dig(data, "general", "route"),
                     skip=(origin, destination, alternate)),
        cruise_altitude_ft=_altitude(_dig(data, "general", "initial_altitude")),
        aircraft_icao=_text(_dig(data, "aircraft", "icaocode")),
        callsign=_text(_dig(data, "atc", "callsign")) or _callsign(data),
        origin_metar=_text(_dig(data, "origin", "metar")),
        destination_metar=_text(_dig(data, "destination", "metar")),
        notes=notes,
    )

    if plan.origin is None or plan.destination is None:
        raise SimBriefError(
            "That SimBrief plan has no origin or destination in it."
        )
    if plan.departure_runway is None and plan.arrival_runway is None:
        notes.append(
            "The SimBrief plan does not name its runways, so they were "
            "chosen from the wind instead."
        )
    return plan


# --- Reading the fields ------------------------------------------------------
def _dig(data: Any, *keys: str) -> Any:
    """Walk nested dictionaries, tolerating anything missing.

    SimBrief sometimes returns a list where a single element is expected
    (an OFP with two alternates, say), so the first entry is taken.
    """
    cursor: Any = data
    for key in keys:
        if isinstance(cursor, list):
            cursor = cursor[0] if cursor else None
        if not isinstance(cursor, dict):
            return None
        cursor = cursor.get(key)
    if isinstance(cursor, list):
        cursor = cursor[0] if cursor else None
    return cursor


def _text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _icao(value: Any) -> Optional[str]:
    text = _text(value)
    if text is None:
        return None
    text = text.upper()
    return text if 3 <= len(text) <= 4 and text.isalnum() else None


def _runway(value: Any) -> Optional[str]:
    """Normalise a runway to the form the airport data uses.

    SimBrief writes runways as plain designators ("04L"), but templates
    have been seen to prefix them ("RW04L") or pad them ("4L").
    """
    text = _text(value)
    if text is None:
        return None
    text = text.upper().replace("RWY", "").replace("RW", "").strip()
    if not text:
        return None
    number, side = text[:-1], text[-1]
    if side in "LRC" and number.isdigit():
        return f"{int(number):02d}{side}"
    if text.isdigit():
        return f"{int(text):02d}"
    return text


def _route(value: Any, skip: tuple[Optional[str], ...] = ()) -> Optional[str]:
    """Strip the bits of a route string the planner cannot use.

    A SimBrief route mixes the fixes with airway identifiers (``UL607``),
    procedure names (``MERIT4``) and speed and level changes
    (``N0480F380``). This program flies fix to fix, so those are dropped
    and the fixes either side of them kept, which is exactly what a
    great-circle route with waypoints needs.

    They are removed by shape rather than left to the navigation data to
    reject, because an airway identifier that happens to match some
    unrelated navaid would silently bend the route across the world.
    """
    text = _text(value)
    if text is None:
        return None
    unwanted = {code for code in skip if code}
    kept = [token for token in text.replace("/", " ").upper().split()
            if _is_fix(token) and token not in unwanted]
    return " ".join(kept) or None


#: A speed and level change, e.g. N0480F380 or M083F350.
_SPEED_LEVEL = re.compile(r"^[NMK]\d{3,4}[FSAM]\d{3,4}$")
#: An airway, e.g. UL607, J80, Q436, A1.
_AIRWAY = re.compile(r"^[A-Z]{1,2}\d{1,3}[A-Z]?$")
#: A departure or arrival procedure, e.g. MERIT4, ROBUC3, LOGAN2A.
_PROCEDURE = re.compile(r"^[A-Z]{3,6}\d[A-Z]?$")


def _is_fix(token: str) -> bool:
    if token in ("DCT", "SID", "STAR", "NAT", "IFR", "VFR"):
        return False
    if not token.isalnum():
        return False
    if _SPEED_LEVEL.match(token) or _AIRWAY.match(token) or _PROCEDURE.match(token):
        return False
    # A bare runway or level left over from a token like KJFK/04L.
    if any(character.isdigit() for character in token) and len(token) <= 3:
        return False
    return 2 <= len(token) <= 5


def _altitude(value: Any) -> Optional[float]:
    text = _text(value)
    if text is None:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    if number <= 0:
        return None
    # SimBrief reports feet, but a template that writes a flight level
    # instead should not put the aeroplane at 380 ft.
    return number * 100.0 if number < 1000 else number


def _callsign(data: Any) -> Optional[str]:
    airline = _text(_dig(data, "general", "icao_airline"))
    number = _text(_dig(data, "general", "flight_number"))
    if airline and number:
        return f"{airline}{number}"
    return number
