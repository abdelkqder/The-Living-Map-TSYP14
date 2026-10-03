"""
writer_robot/memory_decision.py
================================
"Should this observation become persistent memory?" — kept as an
independently testable module, deliberately NOT buried inside movement or
perception code (direct instruction from the project vision).

The Writer does not beacon every observation. It evaluates severity,
confidence, and whether a similar memory has already been deployed nearby
(avoiding redundant beacons clustered on the same event).

[IMPLEMENTED]
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from common.enums import EventType
from common.models import Observation

# [ASSUMPTION] thresholds — not tuned against real mission data.
MIN_SEVERITY_TO_PRESERVE   = 2
MIN_CONFIDENCE_TO_PRESERVE = 50
DEDUP_RADIUS_CELLS         = 2   # skip if a same-type beacon already exists this close


@dataclass
class MemoryDecision:
    preserve: bool
    reason:   str
    priority: int   # mirrors severity when preserved, 0 otherwise


DeployedBeacon = Tuple[EventType, int, int]   # (event_type, row, col) — what the Writer itself has deployed


def evaluate(observation: Observation, already_deployed: List[DeployedBeacon]) -> MemoryDecision:
    """
    Pure function: (observation, what's already been preserved) -> decision.
    [IMPLEMENTED]
    """
    if observation.severity < MIN_SEVERITY_TO_PRESERVE:
        return MemoryDecision(False, f"severity {observation.severity} below threshold", 0)

    if observation.confidence < MIN_CONFIDENCE_TO_PRESERVE:
        return MemoryDecision(False, f"confidence {observation.confidence}% below threshold", 0)

    for et, r, c in already_deployed:
        if et != observation.event_type:
            continue
        dist = abs(r - observation.row) + abs(c - observation.col)
        if dist <= DEDUP_RADIUS_CELLS:
            return MemoryDecision(False, f"redundant — same event type within {DEDUP_RADIUS_CELLS} cells", 0)

    return MemoryDecision(True, "severity/confidence threshold met, not redundant", observation.severity)
