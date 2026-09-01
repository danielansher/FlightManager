"""Maps an aircraft key to its performance profile and the adapter that flies it."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..perf.profiles import PROFILES, AircraftProfile, get_profile
from ..sim.base import SimBackend
from .airbus import AirbusAdapter, BoeingAdapter
from .base import AircraftAdapter, Logger


@dataclass(frozen=True)
class AircraftEntry:
    key: str
    adapter: type
    fcu_convention: str = ""
    aliases: tuple[str, ...] = ()


REGISTRY: dict[str, AircraftEntry] = {}


def _register(entry: AircraftEntry) -> None:
    REGISTRY[entry.key] = entry


_register(AircraftEntry("b787-10", BoeingAdapter,
                        aliases=("787", "78x", "b78x", "787-10", "dreamliner")))
_register(AircraftEntry("b787-9", BoeingAdapter, aliases=("789", "b789", "787-9")))
_register(AircraftEntry("a350-900", AirbusAdapter, "inibuilds_a350",
                        aliases=("a350", "359", "a359")))
_register(AircraftEntry("a350-1000", AirbusAdapter, "inibuilds_a350",
                        aliases=("a35k", "35k", "a350-1000")))
_register(AircraftEntry("a380-800", AirbusAdapter, "inibuilds_a380",
                        aliases=("a380", "388", "a388")))
_register(AircraftEntry("a330-900", AirbusAdapter, "flybywire",
                        aliases=("a330", "a339", "a330neo", "headwind")))
_register(AircraftEntry("a320neo", AirbusAdapter, "flybywire",
                        aliases=("a320", "a20n", "neo")))
_register(AircraftEntry("generic", AircraftAdapter, aliases=("default", "any")))


def resolve_key(name: str) -> Optional[str]:
    """Accept a key, an alias, or an ICAO type code."""
    wanted = name.strip().lower()
    if wanted in REGISTRY:
        return wanted
    for entry in REGISTRY.values():
        if wanted in entry.aliases:
            return entry.key
    for key, profile in PROFILES.items():
        if profile.icao_type.lower() == wanted:
            return key
    return None


def build_adapter(name: str, sim: SimBackend,
                  log: Optional[Logger] = None) -> tuple[AircraftAdapter, AircraftProfile]:
    """Construct the adapter and profile for an aircraft name.

    Falls back to the generic profile rather than failing, so an unrecognised
    aeroplane still flies -- conservatively, but it flies.
    """
    key = resolve_key(name) or "generic"
    entry = REGISTRY[key]
    profile = get_profile(key) or get_profile("generic")
    assert profile is not None
    if entry.adapter is AirbusAdapter:
        adapter = AirbusAdapter(sim, profile, log, entry.fcu_convention)
    else:
        adapter = entry.adapter(sim, profile, log)
    return adapter, profile


def available_aircraft() -> list[tuple[str, str]]:
    """``(key, display name)`` for everything the AI Pilot knows about."""
    out = []
    for key in REGISTRY:
        profile = get_profile(key)
        out.append((key, profile.name if profile else key))
    return out
