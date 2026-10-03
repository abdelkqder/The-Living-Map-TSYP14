"""
tests/test_mission_dispatcher.py
==================================
PATCH 3 tests: MissionDispatcher — priority queue, reservation, latency KPIs,
auto-dispatch integration with LiveMapSystem.

Tests are grouped:
  A. Unit tests for MissionDispatcher in isolation (fake MemoryRecord)
  B. Integration tests through LiveMapSystem (real simulation)
"""
from __future__ import annotations

import time
import pytest

from command_post.mission_dispatcher import (
    DISPATCH_THRESHOLD,
    MissionDispatcher,
    MissionRecord,
)
from command_post.mission_planner import Mission
from common.enums import MissionStatus, MemoryState
from common.memory import MemoryRecord
from common.protocol import BeaconMessage
from simulation.scenario import default_scenario
from simulation.system import LiveMapSystem


# ── helpers ───────────────────────────────────────────────────────────────────

def _rec(
    beacon_id: int = 1,
    event_type: str = "FIRE",
    severity: int = 3,
    confidence_pct: int = 88,
    is_stale: bool = False,
    state: MemoryState = MemoryState.UNVERIFIED,
) -> MemoryRecord:
    """
    Create a MemoryRecord via BeaconMessage so it uses the real type.

    is_stale=True: sets initial_confidence=1 so live_confidence < STALE_THRESHOLD
    immediately.  The MemoryRecord.is_stale property is read-only (derived from
    live_confidence), so we can only control it through the initial value.
    """
    # Use initial_confidence=1 to make the record immediately stale
    init_conf = 1 if is_stale else confidence_pct
    msg = BeaconMessage(
        beacon_id=beacon_id,
        event_type=event_type,
        x_local=5.0,
        y_local=5.0,
        timestamp=int(time.time()),
        severity=severity,
        initial_confidence=init_conf,
        battery_pct=90,
    )
    rec = MemoryRecord.from_beacon_message(msg)
    # Override verification state (mark_verified/contradicted are the real path,
    # but for unit tests we set the field directly).
    if state != MemoryState.UNVERIFIED:
        rec.state = state
    return rec


def make_sys(seed: int = 1) -> LiveMapSystem:
    return LiveMapSystem(default_scenario(seed=seed))


def run_until_beacon(sys_: LiveMapSystem, n: int = 1, guard: int = 30_000) -> None:
    """Tick until at least n beacons are in the Living Map."""
    i = 0
    while len(sys_.living_map) < n and i < guard:
        sys_.update()
        i += 1


# ─────────────────────────────────────────────────────────────────────────────
# A. Unit tests for MissionDispatcher in isolation
# ─────────────────────────────────────────────────────────────────────────────

class TestDispatcherUnit:

    def test_empty_dispatcher_has_no_queued(self):
        d = MissionDispatcher()
        assert d.next_queued() is None
        assert d.queued_count == 0
        assert d.active_count == 0

    def test_high_priority_record_creates_mission(self):
        d   = MissionDispatcher()
        rec = _rec(beacon_id=1, event_type="FIRE", severity=3, confidence_pct=88)
        mr  = d.on_new_record(rec, current_tick=10)
        assert mr is not None
        assert mr.beacon_id == 1
        assert mr.event_type == "FIRE"
        assert mr.status == MissionStatus.QUEUED

    def test_mission_score_above_threshold(self):
        d   = MissionDispatcher()
        rec = _rec(beacon_id=1, severity=3, confidence_pct=88)
        mr  = d.on_new_record(rec, current_tick=0)
        assert mr.priority_score >= DISPATCH_THRESHOLD

    def test_low_severity_score_below_threshold(self):
        d   = MissionDispatcher()
        rec = _rec(beacon_id=1, severity=1, confidence_pct=50)
        mr  = d.on_new_record(rec, current_tick=0)
        # Score = 5×1 + 0.05×50 + 2×1 = 9.5 < 12.0 — still creates mission
        # but score is below threshold so auto-dispatch should not fire
        assert mr is not None
        assert mr.priority_score < DISPATCH_THRESHOLD

    def test_contradicted_record_gets_no_mission(self):
        d   = MissionDispatcher()
        rec = _rec(beacon_id=1, state=MemoryState.CONTRADICTED)
        mr  = d.on_new_record(rec, current_tick=0)
        assert mr is None
        assert d.queued_count == 0

    def test_no_duplicate_mission_for_same_beacon(self):
        d   = MissionDispatcher()
        rec = _rec(beacon_id=1, severity=3, confidence_pct=88)
        mr1 = d.on_new_record(rec, current_tick=0)
        mr2 = d.on_new_record(rec, current_tick=5)
        # Second call must return same mission (priority update), not a new one
        assert mr2 is not None
        assert mr1.mission_id == mr2.mission_id
        assert d.queued_count == 1

    def test_queue_sorted_by_priority_highest_first(self):
        d    = MissionDispatcher()
        fire = _rec(beacon_id=1, event_type="FIRE", severity=3, confidence_pct=88)
        gas  = _rec(beacon_id=2, event_type="GAS",  severity=2, confidence_pct=72)
        d.on_new_record(gas,  current_tick=0)   # add lower-priority first
        d.on_new_record(fire, current_tick=0)
        first = d.next_queued()
        assert first.event_type == "FIRE"       # FIRE must be first

    def test_assign_changes_status_to_assigned(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=0)
        d.assign(mr, "E1", current_tick=5)
        assert mr.status == MissionStatus.ASSIGNED
        assert mr.assigned_executor_id == "E1"
        assert mr.dispatched_tick == 5

    def test_assign_adds_to_active_dict(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=0)
        d.assign(mr, "E1", current_tick=5)
        assert d.active_count == 1
        assert d.is_reserved(1)

    def test_assigned_beacon_not_next_queued(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=0)
        d.assign(mr, "E1", current_tick=5)
        assert d.next_queued() is None   # only mission is ASSIGNED

    def test_mission_outcome_verified_closes_mission(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=0)
        d.assign(mr, "E1", current_tick=5)
        d.on_mission_outcome(1, current_tick=50, success=True)
        assert mr.status == MissionStatus.COMPLETED
        assert mr.completed_tick == 50
        assert d.active_count == 0
        assert not d.is_reserved(1)

    def test_mission_outcome_contradicted_closes_mission(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=0)
        d.assign(mr, "E1", current_tick=5)
        d.on_mission_outcome(1, current_tick=50, success=False)
        assert mr.status == MissionStatus.COMPLETED

    def test_failed_mission_returns_to_queue(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=0)
        d.assign(mr, "E1", current_tick=5)
        d.on_mission_failed("E1", current_tick=30)
        assert mr.status == MissionStatus.QUEUED
        assert mr.assigned_executor_id is None
        assert d.next_queued() is mr

    def test_failed_mission_reinserted_at_correct_priority(self):
        d    = MissionDispatcher()
        fire = _rec(beacon_id=1, event_type="FIRE", severity=3, confidence_pct=88)
        gas  = _rec(beacon_id=2, event_type="GAS",  severity=2, confidence_pct=72)
        mf   = d.on_new_record(fire, current_tick=0)
        mg   = d.on_new_record(gas,  current_tick=0)
        d.assign(mf, "E1", current_tick=5)
        d.on_mission_failed("E1", current_tick=30)
        # FIRE should still be first after requeue
        assert d.next_queued().event_type == "FIRE"

    def test_beacon_to_dispatch_latency(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=10)
        d.assign(mr, "E1", current_tick=20)
        assert mr.dispatch_latency == 10

    def test_beacon_to_verify_latency(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=10)
        d.assign(mr, "E1", current_tick=20)
        d.on_mission_outcome(1, current_tick=80, success=True)
        assert mr.verify_latency == 70

    def test_metrics_keys_present(self):
        d      = MissionDispatcher()
        d.on_new_record(_rec(beacon_id=1), current_tick=0)
        result = d.metrics()
        for key in (
            "missions_created", "missions_completed",
            "missions_queued", "missions_active",
            "avg_beacon_to_dispatch", "avg_beacon_to_verify",
        ):
            assert key in result, f"missing key: {key}"

    def test_metrics_latency_after_completion(self):
        d  = MissionDispatcher()
        mr = d.on_new_record(_rec(beacon_id=1), current_tick=0)
        d.assign(mr, "E1", current_tick=10)
        d.on_mission_outcome(1, current_tick=60, success=True)
        m  = d.metrics()
        assert m["avg_beacon_to_dispatch"] == 10
        assert m["avg_beacon_to_verify"]   == 60
        assert m["missions_completed"]     == 1

    def test_stale_record_gets_priority_penalty(self):
        """
        A stale record (initial_confidence=1 → is_stale immediately) should
        score lower than an otherwise identical fresh record.
        """
        d     = MissionDispatcher()
        fresh = _rec(beacon_id=1, severity=3, confidence_pct=88, is_stale=False)
        stale = _rec(beacon_id=2, severity=3, confidence_pct=88, is_stale=True)
        # Confirm the stale helper actually makes the record stale
        assert stale.is_stale, "test setup: stale record must have is_stale=True"
        mf = d.on_new_record(fresh, current_tick=0)
        ms = d.on_new_record(stale, current_tick=0)
        assert mf.priority_score > ms.priority_score

    def test_priority_decays_with_age(self):
        """A mission scored at tick 0 must outscore the same at tick FRESH_TICKS."""
        from command_post.mission_dispatcher import FRESH_TICKS
        d   = MissionDispatcher()
        rec = _rec(beacon_id=1, severity=3, confidence_pct=88)
        mr  = d.on_new_record(rec, current_tick=0)
        old_score = mr.priority_score
        # Simulate a refresh at a much later tick (freshness → 0)
        mr2 = d.on_new_record(rec, current_tick=FRESH_TICKS + 50)
        assert mr2.priority_score <= old_score


# ─────────────────────────────────────────────────────────────────────────────
# B. Integration tests through LiveMapSystem
# ─────────────────────────────────────────────────────────────────────────────

class TestDispatcherIntegration:

    def test_beacon_arrival_creates_dispatcher_mission(self):
        sys_ = make_sys()
        sys_.start_writer()
        run_until_beacon(sys_, n=1)
        assert sys_.mission_dispatcher.queued_count + \
               sys_.mission_dispatcher.active_count >= 1

    def test_dispatch_executor_uses_dispatcher_queue(self):
        sys_ = make_sys()
        sys_.start_writer()
        run_until_beacon(sys_, n=1)

        # Manually drain the writer so start_executor path is clean
        while not sys_.writer.dead:
            sys_.update()

        pre_q = sys_.mission_dispatcher.queued_count
        e     = sys_.dispatch_executor()
        assert e is not None
        # After dispatch, queue must have shrunk (mission moved to ASSIGNED)
        assert sys_.mission_dispatcher.active_count == 1
        assert sys_.mission_dispatcher.queued_count == pre_q - 1

    def test_auto_dispatch_fires_when_enabled(self):
        """With auto_dispatch=True a new beacon should immediately spawn an executor."""
        sys_ = make_sys()
        sys_.auto_dispatch = True
        sys_.deploy_writer()
        # Tick until at least one executor is deployed automatically
        for _ in range(30_000):
            sys_.update()
            if sys_.active_executors():
                break
        assert sys_.active_executors(), \
            "auto_dispatch should have deployed an executor by now"

    def test_auto_dispatch_off_leaves_mission_queued(self):
        """With auto_dispatch=False, beacons land in the queue but no executor fires."""
        sys_ = make_sys()
        sys_.auto_dispatch = False
        sys_.deploy_writer()
        run_until_beacon(sys_, n=1)
        # Drain the writer
        while not sys_.writer.dead:
            sys_.update()
        assert sys_.active_executors() == [], \
            "no executor should auto-deploy when auto_dispatch=False"
        assert sys_.mission_dispatcher.queued_count >= 1

    def test_reservation_prevents_double_dispatch(self):
        """Two consecutive dispatch_executor() calls must not assign the same beacon."""
        sys_ = make_sys()
        sys_.start_writer()
        run_until_beacon(sys_, n=1)
        while not sys_.writer.dead:
            sys_.update()

        e1 = sys_.dispatch_executor()
        e2 = sys_.dispatch_executor()   # second call — no more queued missions

        # Both executors must have different missions (or second returns None)
        if e1 is not None and e2 is not None:
            assert e1.mission.mission_id != e2.mission.mission_id

    def test_verified_callback_closes_dispatcher_mission(self):
        """
        After a sequential Writer→Executor run, the executor verifies or
        contradicts the beacon it was assigned.  That mission must be COMPLETED
        in the dispatcher.  Other beacons may still have QUEUED missions — the
        sequential flow only dispatches one executor.
        """
        sys_ = make_sys()
        sys_.start_writer()
        while sys_.phase == "writer":
            sys_.update()
        sys_.start_executor()
        while sys_.phase == "executor":
            sys_.update()

        completed = [
            m for m in sys_.mission_dispatcher.all_missions()
            if m.status == MissionStatus.COMPLETED
        ]
        assert completed, "at least one mission must be COMPLETED after executor run"
        # The assigned mission must be closed; only unassigned ones may remain QUEUED
        assigned_and_open = [
            m for m in sys_.mission_dispatcher.all_missions()
            if m.status in (MissionStatus.ASSIGNED, MissionStatus.IN_PROGRESS)
        ]
        assert assigned_and_open == [], \
            f"no mission should be stuck ASSIGNED/IN_PROGRESS: {assigned_and_open}"

    def test_dispatcher_metrics_populated_after_full_run(self):
        sys_ = make_sys()
        sys_.start_writer()
        while sys_.phase == "writer":
            sys_.update()
        sys_.start_executor()
        while sys_.phase == "executor":
            sys_.update()
        m = sys_.mission_dispatcher.metrics()
        assert m["missions_created"] >= 1
        assert m["missions_completed"] >= 1

    def test_mission_record_to_mission_converts_correctly(self):
        sys_ = make_sys()
        sys_.start_writer()
        run_until_beacon(sys_, n=1)
        mr = sys_.mission_dispatcher.next_queued()
        assert mr is not None
        mission = sys_._mission_record_to_mission(mr)
        assert isinstance(mission, Mission)
        assert len(mission.targets) == 1
        assert mission.targets[0].beacon_id == mr.beacon_id
        assert mission.targets[0].event_type == mr.event_type
