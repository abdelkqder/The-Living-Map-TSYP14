"""
command_post/planner.py
=======================
Mission PRIORITY and EXECUTOR SELECTION — the Command Post's decision logic.
(The ONA never calls any of this.)

net priority of an open memory record
-------------------------------------
    base      dispatcher score  (severity, live confidence, freshness)
    x class   event-class weight       (victim > fire > gas > structural > blockage)
    x status  lifecycle factor         (ESCALATED > ACTIVE/VERIFIED > UNVERIFIED)
    - travel  W_TRAVEL x distance from the entry (metres, from local coordinates)
    + boost   when a blocked mission is waiting for this record to be cleared
    (CONTRADICTED / CLEARED records are never missions.)

Age enters through live confidence (old information is worth less) and through
the dispatcher's freshness term; every factor is returned in `components` so the
decision is explainable in logs / UI. The constants are [ASSUMPTION]s, exposed
as module constants so they can be tuned — they are not field-validated.

executor selection
------------------
Among AVAILABLE executors that can do the job: a specialist beats a GENERAL
executor (GENERAL only covers FIRE and GAS), then higher battery, then fewer
completed missions (load balancing), then id. The reason is a structured dict.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from common.enums import MemoryState, RobotCapability
from command_post.fleet import FleetEntry, FleetRegistry
from command_post.mission_dispatcher import MissionRecord

EVENT_CLASS_WEIGHT: Dict[str, float] = {
    "VICTIM_PRESENCE": 1.30, "FIRE": 1.20, "GAS": 1.00, "STRUCTURAL": 0.90, "BLOCKAGE": 0.80,
}
STATUS_FACTOR: Dict[MemoryState, float] = {
    MemoryState.UNVERIFIED: 0.80, MemoryState.VERIFIED: 1.00, MemoryState.ACTIVE: 1.00,
    MemoryState.ESCALATED: 1.10, MemoryState.CONTRADICTED: 0.0, MemoryState.CLEARED: 0.0,
}
W_TRAVEL = 0.35          # priority points per metre of straight-line distance from the entry
BLOCKER_BOOST = 0.9      # fraction of the blocked mission's score lent to the blockage record
MIN_BATTERY_TO_DISPATCH = 35

CAPABILITY_FOR_EVENT: Dict[str, RobotCapability] = {
    "FIRE": RobotCapability.FIRE, "GAS": RobotCapability.GAS,
    "VICTIM_PRESENCE": RobotCapability.VICTIM, "STRUCTURAL": RobotCapability.STRUCTURAL,
    "BLOCKAGE": RobotCapability.DEBRIS,
}


@dataclass
class Priority:
    score: float
    components: Dict[str, float]


def net_priority(mr: MissionRecord, record, boost: float = 0.0) -> Priority:
    base = mr.priority_score
    cls = EVENT_CLASS_WEIGHT.get(record.event_type, 1.0)
    st = STATUS_FACTOR.get(record.state, 1.0)
    dist = math.hypot(record.x_local, record.y_local)
    travel = W_TRAVEL * dist
    score = base * cls * st - travel + boost
    return Priority(round(max(0.0, score), 2), {
        "base": round(base, 2), "class_weight": cls, "status_factor": st,
        "travel_penalty": round(travel, 2), "distance_m": round(dist, 2),
        "blocker_boost": round(boost, 2), "age_s": round(record.age_seconds, 1),
        "live_confidence": round(record.live_confidence, 2),
    })


@dataclass
class Choice:
    executor: Optional[FleetEntry]
    reason: Dict[str, object]


def choose_executor(cap: RobotCapability, fleet: FleetRegistry) -> Choice:
    """Select among AVAILABLE executors; explain why."""
    avail = fleet.available()
    cands: List[Tuple[Tuple, FleetEntry, str]] = []
    rejected: List[str] = []
    for e in avail:
        if e.battery_pct < MIN_BATTERY_TO_DISPATCH:
            rejected.append(f"{e.executor_id}: battery {e.battery_pct}% < {MIN_BATTERY_TO_DISPATCH}%")
            continue
        if e.specialist_for(cap):
            match, rank = f"specialist {cap.value}", 0
        elif e.general_covers(cap):
            match, rank = "GENERAL fallback", 1
        else:
            rejected.append(f"{e.executor_id}: lacks {cap.value} capability")
            continue
        cands.append(((rank, -e.battery_pct, e.missions_done, e.executor_id), e, match))
    unavailable = [f"{e.executor_id}: {e.state.value}" for e in fleet.all() if not e.available]
    if not cands:
        return Choice(None, {"required": cap.value, "chosen": None,
                             "why": f"no AVAILABLE executor with {cap.value} capability",
                             "rejected": rejected, "not_available": unavailable})
    cands.sort(key=lambda t: t[0])
    _, best, match = cands[0]
    return Choice(best, {"required": cap.value, "chosen": best.executor_id, "match": match,
                         "battery_pct": best.battery_pct, "missions_done": best.missions_done,
                         "alternatives": [c[1].executor_id for c in cands[1:]],
                         "rejected": rejected, "not_available": unavailable,
                         "why": f"{match}; battery {best.battery_pct}%; {best.missions_done} mission(s) done"})
