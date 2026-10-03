"""
common/models.py
=================
Small, shared data shapes used by more than one package. Anything specific
to a single actor (e.g. a Mission belongs to command_post/executor_robot)
lives with that actor instead — this module is only for genuinely shared
primitives.

[IMPLEMENTED]
"""

from __future__ import annotations

from dataclasses import dataclass

from common.enums import EventType


@dataclass
class Pose:
    """2-D pose in a robot's local frame. [SIMULATED] position."""
    row: int          # grid row (discovered-map cell)
    col: int           # grid col (discovered-map cell)
    x_m: float = 0.0   # metres east  of zone entry
    y_m: float = 0.0   # metres north of zone entry
    heading: float = 0.0  # radians, for rendering only


@dataclass
class Observation:
    """
    One perception event: "something was sensed here."

    This is deliberately NOT a BeaconMessage. An Observation is a candidate;
    whether it becomes persistent physical memory is a separate decision
    (writer_robot/memory_decision.py). Keeping these as two steps is a
    direct requirement from the project vision ("selective memory").

    [SIMULATED] position and sensor values are synthetic in Phase 1.
    """
    event_type: EventType
    row: int
    col: int
    severity: int              # 1..4
    confidence: int            # 0..100, sensor-side certainty at detection time
    tick: int                  # simulation tick at time of detection
    source: str = "writer"     # "writer" | "executor" (who observed it)
