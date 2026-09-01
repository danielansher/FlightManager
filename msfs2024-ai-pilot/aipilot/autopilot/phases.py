"""Flight phases and the events the AI Pilot reports as it flies."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional


class Phase(str, Enum):
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    CLIMB = "climb"
    CRUISE = "cruise"
    DESCENT = "descent"
    APPROACH = "approach"
    LANDING = "landing"
    ROLLOUT = "rollout"
    COMPLETE = "complete"
    ABORTED = "aborted"

    @property
    def airborne(self) -> bool:
        return self in (Phase.CLIMB, Phase.CRUISE, Phase.DESCENT,
                        Phase.APPROACH, Phase.LANDING)

    @property
    def label(self) -> str:
        return self.value.upper()


#: The order phases normally run in. Used to reject a backwards transition,
#: which is almost always a latching bug rather than something real: an
#: aeroplane that has started its descent should not decide it is climbing
#: again because it levelled off for traffic.
PHASE_ORDER = [
    Phase.PREFLIGHT, Phase.TAKEOFF, Phase.CLIMB, Phase.CRUISE,
    Phase.DESCENT, Phase.APPROACH, Phase.LANDING, Phase.ROLLOUT, Phase.COMPLETE,
]


def phase_rank(phase: Phase) -> int:
    try:
        return PHASE_ORDER.index(phase)
    except ValueError:
        return -1


@dataclass
class FlightEvent:
    """Something worth telling the user about."""

    time_s: float
    phase: Phase
    message: str
    level: str = "info"          # "info", "warning", "error"

    def __str__(self) -> str:  # pragma: no cover - display only
        minutes, seconds = divmod(int(self.time_s), 60)
        return f"[{minutes:02d}:{seconds:02d}] {self.phase.label:<9} {self.message}"


class EventLog:
    """A bounded log that also fans out to a callback for the UI."""

    def __init__(self, limit: int = 500,
                 listener: Optional[Callable[[FlightEvent], None]] = None) -> None:
        self.events: list[FlightEvent] = []
        self.limit = limit
        self.listener = listener

    def add(self, time_s: float, phase: Phase, message: str, level: str = "info") -> FlightEvent:
        event = FlightEvent(time_s, phase, message, level)
        self.events.append(event)
        if len(self.events) > self.limit:
            del self.events[: len(self.events) - self.limit]
        if self.listener is not None:
            self.listener(event)
        return event

    def since(self, index: int) -> list[FlightEvent]:
        return self.events[max(0, index):]

    def __len__(self) -> int:
        return len(self.events)

    def __iter__(self):
        return iter(self.events)
