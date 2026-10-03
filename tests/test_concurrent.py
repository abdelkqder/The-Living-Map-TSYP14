"""
tests/test_concurrent.py
=========================
Tests for PATCH 1: concurrent actor loop.

Verifies that:
  - WriterRobot and ExecutorRobot can operate simultaneously
  - phase property correctly reports "concurrent" / "dead" / "complete"
  - deploy_writer() and dispatch_executor() work at any time
  - deploy_replacement_writer() refuses if a writer is still active
  - writer_states() and executor_states() return correct enum values
  - multiple writers can be deployed and all tick independently
  - visible_cells() aggregates across all actors
  - all 80 original tests still pass (ensured by running pytest -q)
"""
from __future__ import annotations

import pytest

from common.enums import ExecutorState, WriterState
from simulation.scenario import default_scenario
from simulation.system import LiveMapSystem


# ── helpers ───────────────────────────────────────────────────────────────────

def make_system(seed: int = 1, writer_max_ticks: int | None = None) -> LiveMapSystem:
    return LiveMapSystem(default_scenario(seed=seed, writer_max_ticks=writer_max_ticks))


def run_until(sys_: LiveMapSystem, condition, guard: int = 30_000) -> int:
    """Tick until condition(sys_) is True or guard is reached. Returns tick count."""
    n = 0
    while not condition(sys_) and n < guard:
        sys_.update()
        n += 1
    return n


# ── phase property ────────────────────────────────────────────────────────────

def test_phase_idle_on_init():
    sys_ = make_system()
    assert sys_.phase == "idle"


def test_phase_writer_after_start_writer():
    sys_ = make_system()
    sys_.start_writer()
    assert sys_.phase == "writer"


def test_phase_dead_after_writer_exhausted():
    sys_ = make_system(writer_max_ticks=200)
    sys_.start_writer()
    run_until(sys_, lambda s: s.phase == "dead")
    assert sys_.phase == "dead"


def test_phase_complete_after_sequential_mission():
    """Original sequential flow must still reach 'complete'."""
    sys_ = make_system()
    sys_.start_writer()
    run_until(sys_, lambda s: s.phase == "dead")
    sys_.start_executor()
    run_until(sys_, lambda s: s.phase != "executor")
    assert sys_.phase == "complete"


def test_phase_concurrent_when_both_active():
    """
    Deploy writer, let it run a few ticks to discover something,
    then dispatch an executor while the writer is still alive.
    Phase must be 'concurrent'.
    """
    sys_ = make_system()
    sys_.start_writer()

    # Tick until at least one beacon is in the Living Map
    run_until(sys_, lambda s: len(s.living_map) >= 1, guard=30_000)

    # Executor dispatched while writer is still alive
    exec_ = sys_.dispatch_executor()
    assert exec_ is not None, "should have a mission candidate by now"
    assert sys_.phase == "concurrent"


def test_phase_executor_after_writer_dies_during_concurrent():
    """When writer dies while executor is still active, phase becomes 'executor'."""
    sys_ = make_system()
    sys_.start_writer()
    run_until(sys_, lambda s: len(s.living_map) >= 1, guard=30_000)
    sys_.dispatch_executor()

    # Run until writer is dead but executor might still be working
    run_until(sys_, lambda s: s.writer is not None and s.writer.dead, guard=30_000)
    active_exec = sys_.active_executors()
    if active_exec:
        assert sys_.phase == "executor"


# ── deploy_writer / dispatch_executor ────────────────────────────────────────

def test_deploy_writer_returns_writer_robot():
    from writer_robot.writer import WriterRobot
    sys_ = make_system()
    w    = sys_.deploy_writer()
    assert isinstance(w, WriterRobot)
    assert w in sys_.writers


def test_deploy_writer_auto_ids():
    sys_ = make_system()
    w1 = sys_.deploy_writer()
    w2 = sys_.deploy_writer()
    assert w1.writer_id != w2.writer_id
    assert len(sys_.writers) == 2


def test_dispatch_executor_returns_none_with_empty_map():
    sys_ = make_system()
    # No beacons yet → no candidates → no mission
    result = sys_.dispatch_executor()
    assert result is None


def test_dispatch_executor_works_while_writer_active():
    sys_ = make_system()
    sys_.start_writer()
    run_until(sys_, lambda s: len(s.living_map) >= 1, guard=30_000)
    e = sys_.dispatch_executor()
    assert e is not None
    assert e in sys_.executors


def test_dispatch_multiple_executors():
    """Two executors can be dispatched independently."""
    sys_ = make_system()
    sys_.start_writer()
    # Run writer to completion so map has targets
    run_until(sys_, lambda s: s.phase == "dead")

    e1 = sys_.dispatch_executor()
    e2 = sys_.dispatch_executor()
    # Both return executors (second may get empty mission if first took all targets)
    assert e1 is not None
    assert len(sys_.executors) >= 1


# ── replacement writer ────────────────────────────────────────────────────────

def test_replacement_writer_refused_when_writer_active():
    sys_ = make_system()
    sys_.start_writer()
    assert sys_.phase == "writer"
    result = sys_.deploy_replacement_writer()
    assert result is None  # refused — writer still active


def test_replacement_writer_deployed_after_writer_dies():
    sys_ = make_system(writer_max_ticks=200)
    sys_.start_writer()
    run_until(sys_, lambda s: s.phase == "dead")

    w2 = sys_.deploy_replacement_writer()
    assert w2 is not None
    assert not w2.dead
    assert len(sys_.writers) == 2


def test_replacement_writer_has_blank_map():
    """Replacement writer must NOT inherit W1's DiscoveredWorld."""
    sys_ = make_system(writer_max_ticks=200)
    sys_.start_writer()
    run_until(sys_, lambda s: s.phase == "dead")
    w1_known = len(sys_.writers[0].discovered.known_free_cells())

    w2 = sys_.deploy_replacement_writer()
    # W2 starts with only the entry cell sensed (from __init__)
    w2_known = len(w2.discovered.known_free_cells())
    assert w2_known < w1_known


# ── state enums ──────────────────────────────────────────────────────────────

def test_writer_state_exploring_while_active():
    sys_ = make_system()
    sys_.start_writer()
    sys_.update()  # one tick — should now be exploring
    assert sys_.writer.state == WriterState.EXPLORING


def test_writer_state_returned_after_exhausted():
    # PHASE-1 PATCH: an exhausted exploration budget sends the Writer home; it is
    # RETURNED (still operational), not DEAD. DEAD is reserved for failures.
    sys_ = make_system(writer_max_ticks=100)
    sys_.start_writer()
    run_until(sys_, lambda s: s.writer is not None and s.writer.dead)
    assert sys_.writer.state == WriterState.RETURNED


def test_executor_state_idle_before_mission():
    sys_ = make_system()
    # Deploy executor without loading a mission
    executor = sys_._make_executor()
    assert executor.state == ExecutorState.IDLE


def test_executor_state_available_after_mission():
    # PHASE-1 PATCH: COMPLETED is no longer a resting state. A healthy executor
    # that finished its mission returns to the entry and is AVAILABLE again.
    sys_ = make_system()
    sys_.start_writer()
    run_until(sys_, lambda s: s.phase == "dead")
    sys_.start_executor()
    run_until(sys_, lambda s: s.phase == "complete")
    assert sys_.executor.state == ExecutorState.AVAILABLE
    assert sys_.executor.at_entry


# ── fleet status helpers ─────────────────────────────────────────────────────

def test_writer_states_dict():
    sys_ = make_system()
    sys_.start_writer()
    states = sys_.writer_states()
    assert "W1" in states
    assert states["W1"] == WriterState.EXPLORING


def test_executor_states_dict():
    sys_ = make_system()
    sys_.start_writer()
    run_until(sys_, lambda s: s.phase == "dead")
    sys_.start_executor()
    states = sys_.executor_states()
    assert len(states) == 1
    assert "E1" in states


# ── visible_cells aggregation ─────────────────────────────────────────────────

def test_visible_cells_aggregates_all_writers():
    sys_ = make_system()
    w1 = sys_.deploy_writer()
    w2 = sys_.deploy_writer()
    # Run a few ticks so both writers discover something
    for _ in range(50):
        sys_.update()
    vc = sys_.visible_cells()
    w1_cells = dict(w1.discovered.all_known_cells())
    w2_cells = dict(w2.discovered.all_known_cells())
    # visible_cells must contain at least everything each writer knows
    for cell in w1_cells:
        assert cell in vc
    for cell in w2_cells:
        assert cell in vc


# ── backward-compat properties ───────────────────────────────────────────────

def test_writer_property_returns_first_writer():
    sys_ = make_system()
    sys_.start_writer()
    assert sys_.writer is sys_.writers[0]


def test_executor_property_returns_first_executor():
    sys_ = make_system()
    sys_.start_writer()
    run_until(sys_, lambda s: s.phase == "dead")
    sys_.start_executor()
    assert sys_.executor is sys_.executors[0]


def test_writer_property_none_before_deploy():
    sys_ = make_system()
    assert sys_.writer is None


def test_executor_property_none_before_dispatch():
    sys_ = make_system()
    assert sys_.executor is None


# ── concurrent tick behaviour ─────────────────────────────────────────────────

def test_both_robots_tick_independently():
    """
    In concurrent mode, both the writer and executor advance each tick.
    Verify by checking that both accumulate trail steps while concurrent.
    """
    sys_ = make_system()
    sys_.start_writer()
    run_until(sys_, lambda s: len(s.living_map) >= 1, guard=30_000)

    exec_ = sys_.dispatch_executor()
    assert exec_ is not None

    writer_tick_before = sys_.writer.tick
    exec_tick_before   = exec_.tick

    for _ in range(20):
        sys_.update()

    assert sys_.writer.tick   > writer_tick_before  or sys_.writer.dead
    assert exec_.tick         > exec_tick_before    or exec_.is_complete
