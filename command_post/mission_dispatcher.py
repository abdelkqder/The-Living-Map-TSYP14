"""
command_post/mission_dispatcher.py
====================================
PATCH 3: Event-driven Mission Dispatcher.

Core principle (from spec modifier):
    "new memory → reassess the mission queue immediately"
    NOT "newest beacon = dispatch immediately"

The dispatcher evaluates every new or updated record against a priority
score and maintains a sorted queue. When auto_dispatch is enabled it
immediately assigns the best queued mission to an available executor;
otherwise the mission waits in the queue for a human to press
[DISPATCH EXECUTOR].

Priority formula
----------------
    score = W_SEV × severity           # 1–4 scale → 5–20 contribution
          + W_CONF × confidence_pct    # 0–100 → 0–5 contribution
          + W_FRESH × freshness        # 0–1 → 0–2 contribution
          − STALE_PENALTY              # if is_stale

    freshness = max(0, 1 − age_ticks / FRESH_TICKS)
    DISPATCH_THRESHOLD = 12.0  → below this: queued but not auto-dispatched

Calibrated against the default scenario:
    FIRE fresh  ≈ 21.4 → dispatched
    GAS  fresh  ≈ 15.6 → dispatched
    FIRE aging  ≈ 18.8 → dispatched
    GAS  stale  ≈  7.7 → queued only
    Low  event  ≈  9.5 → queued only

Reservation logic
-----------------
    QUEUED      → waiting for an executor
    ASSIGNED    → executor claimed it; no other executor may take it
    IN_PROGRESS → executor is moving to the target
    COMPLETED   → mission closed (verified or contradicted)
    FAILED      → executor went offline; re-queued at original priority

KPI tracking
------------
    beacon_to_dispatch_latency  = dispatched_tick − arrival_tick
    beacon_to_verify_latency    = completed_tick  − arrival_tick

[IMPLEMENTED] for Phase-1 simulation.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from common.enums import MemoryState, MissionStatus, RobotCapability
from common.memory import MemoryRecord


# ── Weights & policy constants ────────────────────────────────────────────────
W_SEV:             float = 5.0
W_CONF:            float = 0.05
W_FRESH:           float = 2.0
STALE_PENALTY:     float = 5.0
FRESH_TICKS:       int   = 400   # ticks over which freshness decays to zero
DISPATCH_THRESHOLD: float = 12.0  # minimum score for auto-dispatch eligibility

# event_type → required RobotCapability
_CAPABILITY_MAP: Dict[str, RobotCapability] = {
    "FIRE":             RobotCapability.FIRE,
    "GAS":              RobotCapability.GAS,
    "VICTIM_PRESENCE":  RobotCapability.VICTIM,
    "STRUCTURAL":       RobotCapability.STRUCTURAL,
    "BLOCKAGE":         RobotCapability.DEBRIS,     # PHASE-1 PATCH
}


# ── MissionRecord ─────────────────────────────────────────────────────────────

@dataclass
class MissionRecord:
    """
    Full lifecycle record for one mission.
    One MissionRecord per beacon, at most one at a time.
    """
    mission_id:           str
    beacon_id:            int
    event_type:           str
    priority_score:       float
    x_local:              float
    y_local:              float
    severity:             int
    required_capability:  RobotCapability
    arrival_tick:         int                    # tick when ONA first delivered this beacon
    status:               MissionStatus = field(default=MissionStatus.QUEUED)
    assigned_executor_id: Optional[str] = None
    dispatched_tick:      Optional[int] = None
    completed_tick:       Optional[int] = None

    # ── Latency helpers ───────────────────────────────────────────────────────
    @property
    def dispatch_latency(self) -> Optional[int]:
        if self.dispatched_tick is None:
            return None
        return self.dispatched_tick - self.arrival_tick

    @property
    def verify_latency(self) -> Optional[int]:
        if self.completed_tick is None:
            return None
        return self.completed_tick - self.arrival_tick


# ── MissionDispatcher ─────────────────────────────────────────────────────────

class MissionDispatcher:
    """
    Event-driven mission lifecycle manager.

    Lifecycle
    ---------
    on_new_record()         — called every time a new beacon reaches the CP
    next_queued()           — returns highest-priority QUEUED mission (or None)
    assign()                — mark QUEUED→ASSIGNED; reserves the target
    mark_in_progress()      — mark ASSIGNED→IN_PROGRESS
    on_mission_outcome()    — verify/contradict → COMPLETED; frees executor slot
    on_mission_failed()     — executor lost → FAILED → re-QUEUED

    LiveMapSystem calls these at the right moments; nothing in this file
    touches ExecutorRobot or GroundTruthWorld directly.
    """

    # expose constants so tests and renderer can read them
    DISPATCH_THRESHOLD = DISPATCH_THRESHOLD
    FRESH_TICKS        = FRESH_TICKS

    def __init__(self) -> None:
        self._queue:    List[MissionRecord]      = []   # sorted, highest priority first
        self._active:   Dict[str, MissionRecord] = {}   # executor_id → record
        self._history:  List[MissionRecord]      = []   # COMPLETED / FAILED archive
        self._beacon_to_mid: Dict[int, str]      = {}   # memory_id → mission_id (dedup)
        self._mseq = itertools.count(1)                         # per-dispatcher: no hidden global state
        self.log:       List[str]                = []

    # ── Primary ingest ────────────────────────────────────────────────────────

    def on_new_record(
        self, record: MemoryRecord, current_tick: int
    ) -> Optional[MissionRecord]:
        """
        Called when the Living Map ingests a beacon message.

        Returns the MissionRecord created or updated, or None if the record
        does not meet the minimum creation criteria (CONTRADICTED, duplicate
        already in a terminal state).
        """
        # Contradicted beacons never generate missions
        state_v = record.state.value if hasattr(record.state, "value") else str(record.state)
        if state_v == MemoryState.CONTRADICTED.value:
            return None

        # Missions are keyed by MEMORY id (== beacon id for single-record beacons).
        bid = record.memory_id if record.memory_id is not None else record.beacon_id

        # Update priority of an existing live mission
        if bid in self._beacon_to_mid:
            mr = self._find(self._beacon_to_mid[bid])
            if mr is not None and mr.status in (
                MissionStatus.QUEUED, MissionStatus.ASSIGNED, MissionStatus.IN_PROGRESS
            ):
                new_score = self._score(record, current_tick, mr.arrival_tick)
                mr.priority_score = new_score
                self._queue.sort(key=lambda m: m.priority_score, reverse=True)
                self.log.append(
                    f"[t={current_tick}] {mr.mission_id} priority updated → {new_score:.1f}"
                )
                return mr

        # Compute score for a potential new mission
        score = self._score(record, current_tick, current_tick)
        mid   = f"M{next(self._mseq):03d}"          # deterministic, readable id

        mr = MissionRecord(
            mission_id=mid,
            beacon_id=bid,
            event_type=record.event_type,
            priority_score=score,
            x_local=record.x_local,
            y_local=record.y_local,
            severity=record.severity,
            required_capability=_CAPABILITY_MAP.get(record.event_type, RobotCapability.GENERAL),
            arrival_tick=current_tick,
        )
        self._insert(mr)
        self._beacon_to_mid[bid] = mid
        self.log.append(
            f"[t={current_tick}] {mid} created — {record.event_type} "
            f"sev={record.severity} conf={record.confidence_pct}% score={score:.1f}"
        )
        return mr

    # ── Reservation lifecycle ─────────────────────────────────────────────────

    def next_queued(self) -> Optional[MissionRecord]:
        """Highest-priority QUEUED mission, or None."""
        for mr in self._queue:
            if mr.status == MissionStatus.QUEUED:
                return mr
        return None

    def assign(
        self, mr: MissionRecord, executor_id: str, current_tick: int
    ) -> None:
        """Reserve a mission for an executor. QUEUED → ASSIGNED."""
        mr.status               = MissionStatus.ASSIGNED
        mr.assigned_executor_id = executor_id
        mr.dispatched_tick      = current_tick
        self._active[executor_id] = mr
        self.log.append(
            f"[t={current_tick}] {mr.mission_id} → {executor_id} ASSIGNED "
            f"(latency={mr.dispatch_latency} ticks)"
        )

    def mark_in_progress(self, executor_id: str) -> None:
        """ASSIGNED → IN_PROGRESS (executor has started moving)."""
        if executor_id in self._active:
            self._active[executor_id].status = MissionStatus.IN_PROGRESS

    def on_mission_outcome(
        self, beacon_id: int, current_tick: int, *, success: bool
    ) -> None:
        """
        Called on executor verification or contradiction.
        Closes the active mission and records KPI latency.

        Handles two cases:
          A. Single-target dispatch (dispatch_executor): key = executor_id
          B. Multi-target legacy (start_executor):       key = "executor_id:beacon_id"
        """
        outcome = "VERIFIED" if success else "CONTRADICTED"

        # Case A: look for a simple executor_id key whose mission matches beacon_id
        for slot, mr in list(self._active.items()):
            if mr.beacon_id == beacon_id:
                mr.status         = MissionStatus.COMPLETED
                mr.completed_tick = current_tick
                self._history.append(mr)
                try:
                    self._queue.remove(mr)
                except ValueError:
                    pass
                del self._active[slot]
                self.log.append(
                    f"[t={current_tick}] {mr.mission_id} {outcome} "
                    f"(verify latency={mr.verify_latency} ticks)"
                )
                return

        # Case B: not in _active — search queue for an IN_PROGRESS match
        # (happens when start_executor used legacy planner without slot reservation)
        for mr in list(self._queue):
            if mr.beacon_id == beacon_id and mr.status == MissionStatus.IN_PROGRESS:
                mr.status         = MissionStatus.COMPLETED
                mr.completed_tick = current_tick
                self._history.append(mr)
                self._queue.remove(mr)
                self.log.append(
                    f"[t={current_tick}] {mr.mission_id} {outcome} "
                    f"(verify latency={mr.verify_latency} ticks)"
                )
                return

    def on_mission_failed(self, executor_id: str, current_tick: int) -> None:
        """Executor went offline. ASSIGNED/IN_PROGRESS → re-QUEUED."""
        if executor_id not in self._active:
            return
        mr = self._active.pop(executor_id)
        mr.status               = MissionStatus.QUEUED
        mr.assigned_executor_id = None
        mr.dispatched_tick      = None
        self._insert(mr)
        self.log.append(
            f"[t={current_tick}] {mr.mission_id} re-queued (executor {executor_id} failed)"
        )

    # ── PHASE-1: additional lifecycle transitions ────────────────────────────

    def cancel(self, mr: MissionRecord, current_tick: int, why: str) -> None:
        """Memory was CLEARED / CONTRADICTED before (or while) the mission ran."""
        if mr.status in (MissionStatus.COMPLETED, MissionStatus.CANCELLED):
            return
        mr.status = MissionStatus.CANCELLED
        mr.completed_tick = current_tick
        for slot, a in list(self._active.items()):
            if a is mr:
                del self._active[slot]
        if mr in self._queue:
            self._queue.remove(mr)
        self._history.append(mr)
        self.log.append(f"[t={current_tick}] {mr.mission_id} CANCELLED ({why})")

    def finish(self, mr: MissionRecord, current_tick: int, outcome: str) -> None:
        """Close an active mission with an explicit outcome label."""
        mr.status = MissionStatus.COMPLETED
        mr.completed_tick = current_tick
        for slot, a in list(self._active.items()):
            if a is mr:
                del self._active[slot]
        if mr in self._queue:
            self._queue.remove(mr)
        self._history.append(mr)
        self.log.append(f"[t={current_tick}] {mr.mission_id} COMPLETED — {outcome} "
                        f"(verify latency={mr.verify_latency} ticks)")

    def block(self, mr: MissionRecord, current_tick: int, why: str) -> None:
        """Executor reported the target unreachable: ASSIGNED/IN_PROGRESS -> BLOCKED (kept in queue)."""
        for slot, a in list(self._active.items()):
            if a is mr:
                del self._active[slot]
        mr.status = MissionStatus.BLOCKED
        mr.assigned_executor_id = None
        if mr not in self._queue:
            self._queue.append(mr)
        self.log.append(f"[t={current_tick}] {mr.mission_id} BLOCKED ({why})")

    def unblock(self, mr: MissionRecord, current_tick: int) -> None:
        if mr.status == MissionStatus.BLOCKED:
            mr.status = MissionStatus.QUEUED
            mr.dispatched_tick = None
            self._queue.sort(key=lambda m: m.priority_score, reverse=True)
            self.log.append(f"[t={current_tick}] {mr.mission_id} re-queued (blockage cleared)")

    def mission_for_memory(self, memory_id: int) -> Optional[MissionRecord]:
        mid = self._beacon_to_mid.get(memory_id)
        return self._find(mid) if mid else None

    def refresh_priority(self, mr: MissionRecord, record: MemoryRecord, current_tick: int) -> float:
        mr.priority_score = self._score(record, current_tick, mr.arrival_tick)
        self._queue.sort(key=lambda m: m.priority_score, reverse=True)
        return mr.priority_score

    # ── Accessors for system / renderer ───────────────────────────────────────

    def all_missions(self) -> List[MissionRecord]:
        """Live queue + history, for rendering the mission panel."""
        return list(self._queue) + list(self._history)

    @property
    def queued_count(self) -> int:
        return sum(1 for m in self._queue if m.status == MissionStatus.QUEUED)

    @property
    def active_count(self) -> int:
        return len(self._active)

    def is_reserved(self, beacon_id: int) -> bool:
        return any(
            mr.beacon_id == beacon_id
            for mr in self._active.values()
        )

    def metrics(self) -> dict:
        """
        KPI dict for benchmark mode (PATCH 8).
        All latency values are in simulation ticks.
        """
        completed = [m for m in self._history if m.status == MissionStatus.COMPLETED]
        dl = [m.dispatch_latency for m in completed if m.dispatch_latency is not None]
        vl = [m.verify_latency   for m in completed if m.verify_latency   is not None]
        return {
            "missions_created":         len(self._beacon_to_mid),
            "missions_completed":       len(completed),
            "missions_queued":          self.queued_count,
            "missions_active":          self.active_count,
            "avg_beacon_to_dispatch":   round(sum(dl) / len(dl), 1) if dl else None,
            "avg_beacon_to_verify":     round(sum(vl) / len(vl), 1) if vl else None,
            "min_beacon_to_dispatch":   min(dl)  if dl else None,
            "max_beacon_to_dispatch":   max(dl)  if dl else None,
        }

    # ── Internal ──────────────────────────────────────────────────────────────

    def _score(
        self,
        record: MemoryRecord,
        current_tick: int,
        arrival_tick: int,
    ) -> float:
        """Priority score — see module docstring for formula."""
        age_ticks = max(0, current_tick - arrival_tick)
        freshness = max(0.0, 1.0 - age_ticks / FRESH_TICKS)
        score = (
            W_SEV   * record.severity
            + W_CONF * record.confidence_pct
            + W_FRESH * freshness
        )
        if record.is_stale:
            score -= STALE_PENALTY
        return max(0.0, round(score, 2))

    def _insert(self, mr: MissionRecord) -> None:
        """Insert into queue maintaining descending priority order."""
        for i, existing in enumerate(self._queue):
            if existing.priority_score < mr.priority_score:
                self._queue.insert(i, mr)
                return
        self._queue.append(mr)

    def _find(self, mission_id: str) -> Optional[MissionRecord]:
        for mr in self._queue:
            if mr.mission_id == mission_id:
                return mr
        for mr in self._history:
            if mr.mission_id == mission_id:
                return mr
        return None
