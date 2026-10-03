"""
command_post/mission_planner.py
================================
Turns Living Map state into an Executor mission. This is the fix for the
Phase-0 issue where the ONA silently generated AND auto-approved missions,
bypassing Command Post entirely. The Command Post's role, per the vision:

    OBSERVE -> UNDERSTAND -> PRIORITIZE -> ASSIGN MISSION -> RECEIVE
    VERIFICATION -> UPDATE MAP

`review_and_assign()` is that "ASSIGN MISSION" step, made a real function
call rather than an implicit pass-through.

[IMPLEMENTED] mission construction and prioritisation.
[SIMULATED]   approval — see `approve_fn` below.
"""
from __future__ import annotations

from typing import Callable, List, Optional

from common.memory import MemoryRecord


# Mission / MissionTarget moved to common/mission.py (Phase-1 patch) so the
# Executor can receive a brief without importing the Command Post. Re-exported
# here so existing imports keep working.
from common.mission import Mission, MissionTarget  # noqa: E402,F401


ApproveFn = Callable[[List[MemoryRecord]], List[MemoryRecord]]


class MissionPlanner:
    """
    [SIMULATED] Operator approval is represented as a plain pass-through
    function by default (`approve_fn=None` approves every candidate). The
    hook is real — passing a function here that, say, prompts an operator,
    or filters by some policy, plugs into the exact same call shape without
    changing anything else in the system. That is deliberately how "a
    human stays at the mission-decision level" is meant to be added later.
    """

    def __init__(self, approve_fn: Optional[ApproveFn] = None):
        self._approve: ApproveFn = approve_fn or (lambda candidates: candidates)

    def review_and_assign(self, candidates: List[MemoryRecord]) -> Mission:
        approved = self._approve(candidates)
        targets = [
            MissionTarget(
                beacon_id=r.beacon_id, event_type=r.event_type,
                x_local=r.x_local, y_local=r.y_local,
                severity=r.severity, state=r.state.value,
                memory_id=r.memory_id, version=r.version, host_beacon_id=r.host_beacon_id,
            )
            for r in approved
        ]
        # legacy planner: a deterministic counter instead of wall-clock time
        self._n = getattr(self, "_n", 0) + 1
        return Mission(mission_id=self._n, targets=targets)
