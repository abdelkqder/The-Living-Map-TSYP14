"""
common/mission.py
=================
The mission BRIEF — the message that travels Command Post -> ONA -> Executor.

It lives in `common/` (not `command_post/`) because the Executor must be able
to receive it without importing anything from the Command Post: the
architecture forbids a direct Executor<->Command Post dependency. The Command
Post builds it, the ONA carries it, the Executor reads it.

What a brief carries = what the Executor "inherits":
  targets    the memory records to visit (what / where)
  waypoints  beacon positions from the ONA toward the target (route skeleton)
  hazards    navigation-relevant memory (blocked passages) to avoid
  policy     what to do if the target turns out to be unreachable

[IMPLEMENTED] (data shapes) · [SIMULATED] (the carrying link)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Union


@dataclass
class MissionTarget:
    beacon_id:  int
    event_type: str
    x_local:    float
    y_local:    float
    severity:   int
    state:      str
    memory_id:  Optional[int] = None     # PHASE-1 PATCH: record id (defaults to beacon_id)
    version:    int = 1                  # version of the record the brief was built from
    host_beacon_id: Optional[int] = None   # beacon that currently hosts the record


@dataclass
class HazardHint:
    """A blocked / degraded passage the Executor inherits from the Living Map."""
    x_local:       float
    y_local:       float
    passage_state: str            # PassageState value
    memory_id:     int = 0
    confidence:    int = 50
    version:       int = 1
    host_beacon_id: Optional[int] = None
    severity:      int = 2


@dataclass
class BlockagePolicy:
    """
    What the Executor does when its target is unreachable (configurable, not
    hard-coded — see simulation/dynamic_debris.py for sweeps).
    """
    max_wait_ticks:          int  = 120    # how long it may linger near the blockage
    max_retries:             int  = 1      # re-plan + re-approach attempts after the wait
    request_debris_executor: bool = True   # report "needs DEBRIS" so the CP can dispatch one
    return_if_unreachable:   bool = True   # go home afterwards (False = stay and wait)


@dataclass
class Mission:
    mission_id: Union[int, str]
    targets:    List[MissionTarget] = field(default_factory=list)
    waypoints:  List[Tuple[float, float]] = field(default_factory=list)  # ONA -> target order
    hazards:    List[HazardHint] = field(default_factory=list)
    policy:     BlockagePolicy = field(default_factory=BlockagePolicy)
    search_types: List[str] = field(default_factory=list)   # baseline: explore for these event types
    memory_ref:   Optional[int] = None                       # baseline: record id to report on (NO location is given)
    version_ref:  int = 1
    issued_tick: int = 0
    reason:      str = ""                                    # CP's structured explanation

    @property
    def is_empty(self) -> bool:
        return not self.targets and not self.search_types

    def to_dict(self) -> dict:
        return {
            "mission_id": self.mission_id,
            "targets": [t.__dict__ for t in self.targets],
            "waypoints": [list(w) for w in self.waypoints],
            "hazards": [h.__dict__ for h in self.hazards],
            "search_types": list(self.search_types),
            "reason": self.reason,
        }
