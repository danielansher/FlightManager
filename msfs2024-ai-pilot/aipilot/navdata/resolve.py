"""Assembles the nav-data chain from whatever the user has installed.

Order matters. Little Navmap first, because it was built from the installed
scenery and therefore agrees with what the aeroplane will see; OurAirports
next, for exact thresholds at fields Little Navmap does not cover; the bundled
sample last, so a fresh clone can still fly a demo.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional, Sequence

from .base import ChainedNavData, NavDataProvider
from .littlenavmap import LittleNavmapProvider, default_database_paths
from .ourairports import OurAirportsProvider

BUNDLED_AIRPORTS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "data", "airports_sample.csv")


@dataclass
class NavDataSources:
    """Explicit overrides; anything left as ``None`` is auto-discovered."""

    littlenavmap_db: Optional[str] = None
    airports_csv: Optional[str] = None
    runways_csv: Optional[str] = None
    search_dirs: Sequence[str] = ()
    use_bundled: bool = True
    #: "2020" or "2024". Only matters when both simulators' Little Navmap
    #: databases are present, which is common enough to be worth handling.
    msfs_version: Optional[str] = None


def _find(name: str, extra_dirs: Sequence[str]) -> Optional[str]:
    candidates = list(extra_dirs) + [os.getcwd(), os.path.join(os.getcwd(), "navdata")]
    for directory in candidates:
        path = os.path.join(directory, name)
        if os.path.isfile(path):
            return path
    return None


def build_navdata(sources: Optional[NavDataSources] = None) -> ChainedNavData:
    """Build the provider chain, skipping anything that is not present."""
    sources = sources or NavDataSources()
    providers: list[NavDataProvider] = []

    lnm_paths = ([sources.littlenavmap_db] if sources.littlenavmap_db
                 else default_database_paths(sources.msfs_version))
    for path in lnm_paths:
        if path and os.path.isfile(path):
            providers.append(LittleNavmapProvider(path))

    airports = sources.airports_csv or _find("airports.csv", sources.search_dirs)
    runways = sources.runways_csv or _find("runways.csv", sources.search_dirs)
    if airports:
        providers.append(OurAirportsProvider(airports, runways, synthesize_runways=False))

    if sources.use_bundled:
        # The sample has no runway data at all, so it synthesizes one. That is
        # only ever good enough for a demo, and the planner says so out loud.
        providers.append(OurAirportsProvider(BUNDLED_AIRPORTS, None, synthesize_runways=True))

    return ChainedNavData(providers)
