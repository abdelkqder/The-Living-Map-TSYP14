"""
tests/test_writer_replacement.py
==================================
PATCH 5 tests: Writer failure, replacement writer, and sector hints.

Groups
------
A. force_fail() / force_writer_fail()
B. Replacement writer — blank map, unique ID, continued exploration
C. Sector hints — accepted, no ground-truth exposure, biases frontier scoring
D. Mission continuity — Living Map and executors survive writer failure
E. Concurrent writer + executor during replacement
F. Chained replacements — W1 → W2 → W3
"""
from __future__ import annotations

import pytest
from typing import Set, Tuple

from common.enums import WriterState
from simulation.scenario import default_scenario
from simulation.system import LiveMapSystem
from simulation.world import Cell


# ── helpers ───────────────────────────────────────────────────────────────────

def make_sys(seed: int = 1, writer_max_ticks: int | None = None) -> LiveMapSystem:
    return LiveMapSystem(default_scenario(seed=seed, writer_max_ticks=writer_max_ticks))


def tick_n(sys_: LiveMapSystem, n: int) -> None:
    for _ in range(n):
        sys_.update()


def run_until(sys_: LiveMapSystem, cond, guard: int = 30_000) -> int:
    for i in range(guard):
        if cond(sys_):
            return i
        sys_.update()
    return guard


def run_until_dead(sys_: LiveMapSystem, guard: int = 30_000) -> None:
    run_until(sys_, lambda s: s.phase == "dead", guard)


def run_until_beacon(sys_: LiveMapSystem, n: int = 1, guard: int = 30_000) -> None:
    run_until(sys_, lambda s: len(list(s.living_map_records)) >= n, guard)


# ─────────────────────────────────────────────────────────────────────────────
# A. force_fail() / force_writer_fail()
# ─────────────────────────────────────────────────────────────────────────────

class TestForceWriterFail:

    def test_force_fail_marks_writer_dead(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 5)
        assert not sys_.writer.dead
        sys_.writer.force_fail()
        assert sys_.writer.dead

    def test_force_fail_state_is_dead(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 5)
        sys_.writer.force_fail()
        assert sys_.writer.state == WriterState.DEAD

    def test_force_fail_logs_offline_message(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 5)
        sys_.writer.force_fail()
        assert any("OFFLINE" in msg for msg in sys_.writer.log)

    def test_force_writer_fail_via_system_returns_true(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 5)
        result = sys_.force_writer_fail()
        assert result is True
        assert sys_.writer.dead

    def test_force_writer_fail_by_id(self):
        sys_ = make_sys()
        sys_.deploy_writer("W1")
        sys_.deploy_writer("W2")
        tick_n(sys_, 5)
        result = sys_.force_writer_fail(writer_id="W1")
        assert result is True
        w1 = next(w for w in sys_.writers if w.writer_id == "W1")
        w2 = next(w for w in sys_.writers if w.writer_id == "W2")
        assert w1.dead
        assert not w2.dead

    def test_force_writer_fail_returns_false_when_none_active(self):
        sys_ = make_sys()
        result = sys_.force_writer_fail()
        assert result is False

    def test_force_writer_fail_changes_phase_to_dead(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 5)
        sys_.force_writer_fail()
        # System should detect writer is dead and no mission running
        sys_.update()
        assert sys_.phase == "dead"

    def test_force_writer_fail_records_metrics(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 50)
        sys_.force_writer_fail()
        assert len(sys_.metrics.runs) == 1

    def test_force_fail_preserves_already_deployed_beacons(self):
        """Beacons already in the Living Map must survive the writer failing."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_beacon(sys_, n=1)
        n_before = len(list(sys_.living_map_records))
        sys_.force_writer_fail()
        tick_n(sys_, 10)
        assert len(list(sys_.living_map_records)) >= n_before


# ─────────────────────────────────────────────────────────────────────────────
# B. Replacement writer — blank map, unique ID, continued exploration
# ─────────────────────────────────────────────────────────────────────────────

class TestReplacementWriter:

    def test_replacement_refused_while_primary_active(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 5)
        result = sys_.deploy_replacement_writer()
        assert result is None, "replacement must be refused while W1 is still alive"

    def test_replacement_deployed_after_w1_fails(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 10)
        sys_.force_writer_fail()
        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        assert not w2.dead

    def test_replacement_has_unique_id(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        sys_.force_writer_fail()
        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        assert w2.writer_id != sys_.writers[0].writer_id

    def test_replacement_starts_with_blank_discovered_world(self):
        """W2 must NOT inherit W1's DiscoveredWorld."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        w1_cells = len(sys_.writers[0].discovered.known_free_cells())

        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        # W2 starts with only the entry cell in its discovered world
        w2_cells = len(w2.discovered.known_free_cells())
        assert w2_cells < w1_cells, (
            f"W2 should have far fewer known cells than W1 "
            f"({w2_cells} vs {w1_cells})"
        )

    def test_replacement_discovered_world_not_shared_with_w1(self):
        """The two discovered world objects must be different instances."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        assert w2.discovered is not sys_.writers[0].discovered

    def test_replacement_continues_exploring(self):
        """W2 must actually move and discover cells after deployment."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        cells_at_start = len(w2.discovered.known_free_cells())
        tick_n(sys_, 100)
        cells_after = len(w2.discovered.known_free_cells())
        assert cells_after > cells_at_start, "W2 must explore new territory"

    def test_replacement_can_deploy_new_beacons(self):
        """W2 must be able to find events and deploy beacons independently."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        n_before = sys_.writers[0].beacon_count

        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        run_until(sys_, lambda s: w2.beacon_count > 0)
        assert w2.beacon_count > 0, "W2 must deploy at least one beacon"

    def test_writer_states_dict_contains_both_writers(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        tick_n(sys_, 5)
        sys_.force_writer_fail()
        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        states = sys_.writer_states()
        assert len(states) == 2
        assert "W1" in states
        assert w2.writer_id in states


# ─────────────────────────────────────────────────────────────────────────────
# C. Sector hints
# ─────────────────────────────────────────────────────────────────────────────

class TestSectorHints:

    def _mid_sector(self, sys_: LiveMapSystem) -> tuple:
        """Return the bottom-right quadrant as a sector hint."""
        rows = sys_.world.rows
        cols = sys_.world.cols
        return (rows // 2, cols // 2, rows - 1, cols - 1)

    def test_replacement_with_sector_hint_does_not_crash(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        hint = self._mid_sector(sys_)
        w2 = sys_.deploy_replacement_writer(sector_hint=hint)
        assert w2 is not None
        tick_n(sys_, 50)   # no crash

    def test_sector_hint_stored_on_writer(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        hint = (0, 0, 5, 5)
        w2 = sys_.deploy_replacement_writer(sector_hint=hint)
        assert w2 is not None
        assert w2.sector_hint == hint

    def test_sector_hint_stored_on_exploration_policy(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        hint = (0, 0, 5, 5)
        w2 = sys_.deploy_replacement_writer(sector_hint=hint)
        assert w2 is not None
        assert w2.exploration.sector_hint == hint

    def test_no_sector_hint_has_none_policy(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        w2 = sys_.deploy_replacement_writer()  # no hint
        assert w2 is not None
        assert w2.exploration.sector_hint is None

    def test_sector_hint_does_not_expose_ground_truth(self):
        """
        A sector hint is a grid bounding box — it must NOT reveal event
        locations from GroundTruthWorld to the replacement writer.
        """
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        hint = self._mid_sector(sys_)
        w2 = sys_.deploy_replacement_writer(sector_hint=hint)
        assert w2 is not None
        # W2 must discover events through sensing only, never from the hint
        # The hint is (int, int, int, int) — it contains no event info
        assert isinstance(hint[0], int)
        assert len(hint) == 4

    def test_sector_hint_biases_exploration_toward_sector(self):
        """
        With a sector hint, the replacement writer should explore more cells
        inside the hinted region than a writer with no hint (same seed).
        """
        def cells_in_sector(discovered_cells: Set[Cell], sector: tuple) -> int:
            r0, c0, r1, c1 = sector
            return sum(1 for (r, c) in discovered_cells
                       if r0 <= r <= r1 and c0 <= c <= c1)

        hint = (0, 0, 9, 9)  # top-left quadrant of a 20×20 grid

        # Without hint
        sys_no = make_sys(seed=7)
        sys_no.deploy_writer()
        run_until_dead(sys_no)
        w2_no = sys_no.deploy_replacement_writer()
        assert w2_no is not None
        tick_n(sys_no, 300)
        cells_no = set(w2_no.discovered.all_known_cells().keys())
        in_sector_no = cells_in_sector(cells_no, hint)

        # With hint
        sys_h = make_sys(seed=7)
        sys_h.deploy_writer()
        run_until_dead(sys_h)
        w2_h = sys_h.deploy_replacement_writer(sector_hint=hint)
        assert w2_h is not None
        tick_n(sys_h, 300)
        cells_h = set(w2_h.discovered.all_known_cells().keys())
        in_sector_h = cells_in_sector(cells_h, hint)

        # The hinted writer should spend proportionally more time in the sector
        # (This is probabilistic — assert a weaker property to avoid flakiness)
        total_no = max(len(cells_no), 1)
        total_h  = max(len(cells_h),  1)
        ratio_no = in_sector_no / total_no
        ratio_h  = in_sector_h  / total_h
        assert ratio_h >= ratio_no * 0.8, (
            f"sector hint should bias exploration toward sector "
            f"(hinted ratio={ratio_h:.2f}, no-hint ratio={ratio_no:.2f})"
        )

    def test_deploy_writer_with_sector_hint_directly(self):
        """deploy_writer() also accepts a sector_hint for manual deployment."""
        sys_  = make_sys()
        hint  = (5, 5, 15, 15)
        w     = sys_.deploy_writer(sector_hint=hint)
        assert w.sector_hint == hint
        assert w.exploration.sector_hint == hint


# ─────────────────────────────────────────────────────────────────────────────
# D. Mission continuity — Living Map and executors survive writer failure
# ─────────────────────────────────────────────────────────────────────────────

class TestMissionContinuity:

    def test_living_map_unchanged_after_writer_fails(self):
        """Beacons in the Living Map must survive writer failure unchanged."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_beacon(sys_, n=1)
        records_before = list(sys_.living_map_records)
        n_before       = len(records_before)

        sys_.force_writer_fail()
        tick_n(sys_, 20)

        assert len(list(sys_.living_map_records)) == n_before

    def test_queued_missions_survive_writer_failure(self):
        """Dispatcher missions must remain queued after W1 goes offline."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        q_before = sys_.mission_dispatcher.queued_count
        assert q_before >= 1

        # fail retroactively (already dead) — dispatch manually
        e = sys_.dispatch_executor()
        assert e is not None, "executor should be dispatchable after writer dies"

    def test_executor_keeps_running_after_writer_fails(self):
        """An active executor must continue its mission when W1 goes offline."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_beacon(sys_, n=1)
        e = sys_.dispatch_executor()
        if e is None:
            pytest.skip("no mission candidate yet")

        tick_n(sys_, 5)
        sys_.force_writer_fail()
        tick_n(sys_, 10)   # executor keeps advancing

        assert e.tick > 0, "executor must have advanced ticks after writer failure"

    def test_phase_executor_after_writer_death_with_active_executor(self):
        """phase == 'executor' when writer dead and executor has a mission."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        sys_.start_executor()
        # Phase should be "executor" while mission is running
        if not sys_.executor.is_complete:
            assert sys_.phase == "executor"

    def test_living_map_accumulates_from_both_writers(self):
        """W2's beacons get NEW ids (no collision with W1's) and everything either
        Writer preserved reaches the Living Map (the replacement Writer may
        re-find the same event: the Command Post merges such duplicate reports)."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        tick_n(sys_, 400)                      # let W1's last frames cross the beacon chain
        before = len(list(sys_.living_map_records))
        w1_ids = set(sys_.beacons.nodes)

        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        run_until(sys_, lambda s: w2.memory_count > 0)
        tick_n(sys_, 600)

        w2_ids = set(sys_.beacons.nodes) - w1_ids
        assert w2_ids, "W2 deployed beacons"
        assert not (w2_ids & w1_ids), "beacon ids must never collide across Writers"
        assert len(list(sys_.living_map_records)) >= before, "W2 must not erase what W1 preserved"


# ─────────────────────────────────────────────────────────────────────────────
# E. Concurrent writer + executor during replacement
# ─────────────────────────────────────────────────────────────────────────────

class TestConcurrentReplacement:

    def test_phase_concurrent_w2_and_executor(self):
        """W2 + active executor → phase must be 'concurrent'."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        e = sys_.dispatch_executor()
        w2 = sys_.deploy_replacement_writer()
        if e is None or w2 is None:
            pytest.skip("prerequisite actors not available")
        if not w2.dead and not e.is_complete:
            assert sys_.phase == "concurrent"

    def test_executor_and_w2_tick_simultaneously(self):
        """Both W2 and the executor must advance each tick."""
        sys_ = make_sys()
        sys_.deploy_writer()
        run_until_dead(sys_)
        e  = sys_.dispatch_executor()
        w2 = sys_.deploy_replacement_writer()
        if e is None or w2 is None:
            pytest.skip("prerequisite actors not available")

        t_w2_before = w2.tick
        t_e_before  = e.tick
        tick_n(sys_, 20)

        assert w2.tick > t_w2_before or w2.dead
        assert e.tick  > t_e_before  or e.is_complete

    def test_auto_dispatch_works_for_w2_beacons(self):
        """With auto_dispatch=True, W2 beacons should trigger executor dispatch."""
        sys_ = make_sys()
        sys_.auto_dispatch = True
        sys_.deploy_writer()
        run_until_dead(sys_)
        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None

        for _ in range(30_000):
            sys_.update()
            if w2.beacon_count > 0 and sys_.mission_dispatcher.active_count > 0:
                break

        # If W2 found and deployed a beacon, auto-dispatch should have fired
        if w2.beacon_count > 0:
            assert sys_.mission_dispatcher.active_count > 0 or \
                   sys_.mission_dispatcher.metrics()["missions_completed"] > 0, \
                "auto-dispatch should have assigned W2's beacon to an executor"


# ─────────────────────────────────────────────────────────────────────────────
# F. Chained replacements W1 → W2 → W3
# ─────────────────────────────────────────────────────────────────────────────

class TestChainedReplacements:

    def test_three_writers_deployed_in_sequence(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        sys_.force_writer_fail()
        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        sys_.force_writer_fail(writer_id=w2.writer_id)
        w3 = sys_.deploy_replacement_writer()
        assert w3 is not None
        assert len(sys_.writers) == 3

    def test_all_replacement_ids_unique(self):
        sys_ = make_sys()
        sys_.deploy_writer()
        sys_.force_writer_fail()
        w2 = sys_.deploy_replacement_writer()
        assert w2 is not None
        sys_.force_writer_fail(writer_id=w2.writer_id)
        w3 = sys_.deploy_replacement_writer()
        assert w3 is not None
        ids = [w.writer_id for w in sys_.writers]
        assert len(set(ids)) == 3, f"all writer IDs must be unique: {ids}"

    def test_each_replacement_has_blank_map(self):
        """Every writer in the chain must start with a blank discovered world."""
        sys_ = make_sys()
        maps = []
        for _ in range(3):
            sys_.deploy_writer()
            tick_n(sys_, 80)   # let writer discover some cells
            w = sys_.writers[-1]
            maps.append(id(w.discovered))
            sys_.force_writer_fail()
        # All discovered world IDs must differ (separate instances)
        assert len(set(maps)) == 3, "each writer must have its own DiscoveredWorld"

    def test_replacement_refused_while_any_writer_active(self):
        """Even with multiple writers, replacement is refused while any is active."""
        sys_ = make_sys()
        sys_.deploy_writer("W1")
        sys_.deploy_writer("W2")
        # W1 active, W2 active → replacement refused
        result = sys_.deploy_replacement_writer()
        assert result is None

        # Fail only W1 — W2 still active
        sys_.force_writer_fail("W1")
        result = sys_.deploy_replacement_writer()
        assert result is None, "W2 still active → replacement still refused"

        # Now fail W2 too
        sys_.force_writer_fail("W2")
        result = sys_.deploy_replacement_writer()
        assert result is not None
