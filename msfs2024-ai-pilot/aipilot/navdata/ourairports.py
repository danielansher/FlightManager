"""OurAirports CSV provider.

OurAirports publishes the whole world's airports and runways in the public
domain as two CSV files. It is the best no-strings-attached source there is:
one download, no licence to worry about, exact runway threshold coordinates.

What it does not have is ILS frequencies, so an approach flown on this data
alone is flown on the AI Pilot's own geometric path rather than on the
aeroplane's ILS receiver. Pair it with Little Navmap's database (see
:mod:`aipilot.navdata.littlenavmap`) to get both.

Download:
    https://davidmegginson.github.io/ourairports-data/airports.csv
    https://davidmegginson.github.io/ourairports-data/runways.csv
"""

from __future__ import annotations

import csv
import os
from typing import Optional

from ..geo import LatLon, destination_point, normalize_deg
from .base import Airport, NavDataProvider, Runway

AIRPORTS_URL = "https://davidmegginson.github.io/ourairports-data/airports.csv"
RUNWAYS_URL = "https://davidmegginson.github.io/ourairports-data/runways.csv"


def _f(row: dict, key: str, default: Optional[float] = None) -> Optional[float]:
    raw = (row.get(key) or "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


class OurAirportsProvider(NavDataProvider):
    """Reads ``airports.csv`` and (optionally) ``runways.csv``."""

    name = "ourairports"

    def __init__(self, airports_csv: str, runways_csv: Optional[str] = None,
                 synthesize_runways: bool = True) -> None:
        self.airports_csv = airports_csv
        self.runways_csv = runways_csv
        self.synthesize_runways = synthesize_runways
        self._airports: dict[str, dict] = {}
        self._runways: dict[str, list[dict]] = {}
        self._loaded = False
        self._error: Optional[str] = None

    @property
    def available(self) -> bool:
        return os.path.isfile(self.airports_csv)

    def describe(self) -> str:
        detail = "airports+runways" if self.runways_csv else "airports only"
        return f"ourairports({os.path.basename(self.airports_csv)}, {detail})"

    def _load(self) -> None:
        if self._loaded:
            return
        self._loaded = True
        with open(self.airports_csv, newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                # ``ident`` is the ICAO code for virtually every airport that
                # has one; ``gps_code`` is the fallback for the odd exception.
                for key in (row.get("ident"), row.get("gps_code")):
                    if key:
                        self._airports.setdefault(key.strip().upper(), row)
        if self.runways_csv and os.path.isfile(self.runways_csv):
            with open(self.runways_csv, newline="", encoding="utf-8") as handle:
                for row in csv.DictReader(handle):
                    if (row.get("closed") or "0").strip() in ("1", "yes"):
                        continue
                    ident = (row.get("airport_ident") or "").strip().upper()
                    if ident:
                        self._runways.setdefault(ident, []).append(row)

    def airport(self, icao: str) -> Optional[Airport]:
        self._load()
        row = self._airports.get(icao.strip().upper())
        if row is None:
            return None
        lat, lon = _f(row, "latitude_deg"), _f(row, "longitude_deg")
        if lat is None or lon is None:
            return None
        elevation = _f(row, "elevation_ft", 0.0) or 0.0
        position = LatLon(lat, lon)
        runways = self._build_runways(icao.strip().upper(), position, elevation)
        return Airport(
            icao=(row.get("ident") or icao).strip().upper(),
            name=(row.get("name") or "").strip(),
            position=position,
            elevation_ft=elevation,
            runways=runways,
        )

    def _build_runways(self, icao: str, arp: LatLon, elevation: float) -> tuple[Runway, ...]:
        out: list[Runway] = []
        for row in self._runways.get(icao, []):
            length = _f(row, "length_ft", 0.0) or 0.0
            width = _f(row, "width_ft", 150.0) or 150.0
            surface = (row.get("surface") or "").strip()
            for prefix in ("le", "he"):
                ident = (row.get(f"{prefix}_ident") or "").strip().upper()
                lat = _f(row, f"{prefix}_latitude_deg")
                lon = _f(row, f"{prefix}_longitude_deg")
                heading = _f(row, f"{prefix}_heading_degT")
                if not ident:
                    continue
                if heading is None:
                    # Fall back to the number in the designator: "27L" -> 270.
                    digits = "".join(c for c in ident if c.isdigit())
                    if not digits:
                        continue
                    heading = float(digits) * 10.0
                if lat is None or lon is None:
                    # No threshold coordinate: place it half the runway length
                    # back from the reference point along the reciprocal.
                    lat_lon = destination_point(arp, normalize_deg(heading + 180.0),
                                                length / 2.0 / 6076.11548556)
                    lat, lon = lat_lon.lat, lat_lon.lon
                out.append(
                    Runway(
                        ident=ident,
                        threshold=LatLon(lat, lon),
                        heading_true_deg=normalize_deg(heading),
                        length_ft=length,
                        elevation_ft=_f(row, f"{prefix}_elevation_ft", elevation) or elevation,
                        width_ft=width,
                        surface=surface,
                        displaced_threshold_ft=_f(row, f"{prefix}_displaced_threshold_ft", 0.0)
                        or 0.0,
                    )
                )
        if not out and self.synthesize_runways:
            out.extend(_synthesized_pair(arp, elevation))
        return tuple(out)


def _synthesized_pair(arp: LatLon, elevation: float) -> list[Runway]:
    """A plausible 10,000 ft runway pair centred on the reference point.

    Used only when there is no runway data at all, so that a demo flight can
    still be planned. It is flagged as ``surface="synthetic"`` and the planner
    warns when it uses one, because the threshold can be a mile from the real
    thing -- fine for exercising the logic, not for an autoland.
    """
    length = 10000.0
    half_nm = length / 2.0 / 6076.11548556
    out = []
    for heading in (90.0, 270.0):
        ident = f"{int(heading / 10):02d}"
        out.append(
            Runway(
                ident=ident,
                threshold=destination_point(arp, normalize_deg(heading + 180.0), half_nm),
                heading_true_deg=heading,
                length_ft=length,
                elevation_ft=elevation,
                surface="synthetic",
            )
        )
    return out
