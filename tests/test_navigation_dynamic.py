"""Frontier exploration, blocked-path memory, alternative routes, changed environments, discovered-map semantics."""
import random

import pytest

from common.enums import CellState, PassageState
from navigation.engine import choose_next_target, plan_route
from simulation.world import DiscoveredWorld, GroundTruthWorld, SensorSnapshot

# 7 x 9 arena with a loop: two routes between the left and the right side.
LOOP = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
]
ENTRY = (1, 1)


def known_world(grid=LOOP):
    d = DiscoveredWorld(len(grid), len(grid[0]))
    snap = SensorSnapshot(free={(r, c) for r, row in enumerate(grid) for c, v in enumerate(row) if v == 0},
                          wall={(r, c) for r, row in enumerate(grid) for c, v in enumerate(row) if v == 1},
                          event_here=None, at=ENTRY)
    d.integrate(snap)
    return d


# ── frontier exploration ──────────────────────────────────────────────────────
def test_exploration_starts_unknown_and_only_senses():
    gt = GroundTruthWorld(LOOP, [], ENTRY)
    d = DiscoveredWorld(gt.rows, gt.cols)
    assert not d.known_free_cells()
    d.integrate(gt.sense(ENTRY, 2))
    assert 0 < len(d.known_free_cells()) < sum(row.count(0) for row in LOOP)


def test_frontier_target_is_a_known_free_cell_bordering_unknown():
    gt = GroundTruthWorld(LOOP, [], ENTRY)
    d = DiscoveredWorld(gt.rows, gt.cols)
    d.integrate(gt.sense(ENTRY, 2))
    cand = choose_next_target(d, ENTRY, __import__("common.enums", fromlist=["NavMode"]).NavMode.EXPLORE)
    assert cand is not None and cand.cell in d.frontier_cells()


def test_seeded_jitter_changes_the_choice_but_is_reproducible():
    from common.enums import NavMode
    gt = GroundTruthWorld(LOOP, [], ENTRY)
    d = DiscoveredWorld(gt.rows, gt.cols)
    d.integrate(gt.sense((3, 4), 1))
    picks = {choose_next_target(d, (3, 4), NavMode.EXPLORE, rng=random.Random(s), jitter=5.0).cell for s in range(30)}
    assert len(picks) > 1                                                        # different seeds, different targets
    a = choose_next_target(d, (3, 4), NavMode.EXPLORE, rng=random.Random(9), jitter=5.0).cell
    b = choose_next_target(d, (3, 4), NavMode.EXPLORE, rng=random.Random(9), jitter=5.0).cell
    assert a == b


# ── blocked-path memory & alternative routes ──────────────────────────────────
def test_route_prefers_the_short_way_when_open():
    d = known_world()
    r = plan_route(d, (1, 1), (3, 1))
    assert r is not None and r.length == 2


def test_known_blockage_excludes_the_path_and_an_alternative_is_found():
    d = known_world()
    d.mark_passage((2, 1), PassageState.BLOCKED, "inherited")
    r = plan_route(d, (1, 1), (3, 1))
    assert r is not None and (2, 1) not in r.path
    assert r.length == 14                                                       # the long way round the loop


def test_impassable_is_never_entered_and_unreachable_is_reported():
    d = known_world()
    d.mark_passage((2, 1), PassageState.IMPASSABLE, "inherited")
    d.mark_passage((2, 7), PassageState.IMPASSABLE, "inherited")
    assert plan_route(d, (1, 1), (3, 1), optimistic=False) is None


RING = [          # a small ring: direct (1,1)->(3,1) costs 2, the way round costs 6
    [1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1],
    [1, 0, 1, 0, 1],
    [1, 0, 0, 0, 1],
    [1, 1, 1, 1, 1],
]


def test_degraded_passage_costs_are_ordered_and_finite():
    from simulation.world import PASSAGE_PENALTY
    d = known_world(RING)
    costs = {}
    for st in (PassageState.TEMPORARILY_BLOCKED, PassageState.DETOUR_REQUIRED, PassageState.ACCESSIBILITY_UNKNOWN):
        d.mark_passage((2, 1), st, "inherited")
        costs[st] = d.traversal_cost((2, 1))
        assert costs[st] == 1.0 + PASSAGE_PENALTY[st]                              # penalised, never forbidden (not None)
    assert costs[PassageState.TEMPORARILY_BLOCKED] > costs[PassageState.DETOUR_REQUIRED] > costs[PassageState.ACCESSIBILITY_UNKNOWN] > 1.0


@pytest.mark.parametrize("state", [PassageState.TEMPORARILY_BLOCKED, PassageState.DETOUR_REQUIRED])
def test_a_penalised_passage_is_avoided_when_a_cheap_alternative_exists(state):
    d = known_world(RING)
    d.mark_passage((2, 1), state, "inherited")
    r = plan_route(d, (1, 1), (3, 1))
    assert r is not None and (2, 1) not in r.path and r.length == 6


def test_a_penalised_passage_is_used_when_it_is_the_only_way():
    d = known_world(RING)
    d.mark_passage((2, 1), PassageState.TEMPORARILY_BLOCKED, "inherited")
    d.mark_passage((1, 3), PassageState.BLOCKED, "inherited")                         # the alternative is closed
    r = plan_route(d, (1, 1), (3, 1))
    assert r is not None and r.path == [(1, 1), (2, 1), (3, 1)]


def test_passage_info_expresses_every_required_state():
    names = {s.value for s in PassageState}
    assert {"BLOCKED", "IMPASSABLE", "TEMPORARILY_BLOCKED", "DETOUR_REQUIRED", "ACCESSIBILITY_UNKNOWN"} <= names


# ── changed environment: the discovered map is NOT append-only ─────────────────
def test_a_free_cell_can_become_blocked_and_blocked_can_become_free():
    gt = GroundTruthWorld(LOOP, [], ENTRY)
    d = DiscoveredWorld(gt.rows, gt.cols)
    d.integrate(gt.sense((1, 1), 3))
    assert d.is_free((1, 2))
    gt.add_debris((1, 2), tick=5)                                                  # debris appears AFTER it was seen free
    d.integrate(gt.sense((1, 1), 3), tick=6)
    assert d.state_of((1, 2)) == CellState.BLOCKED and d.is_impassable((1, 2))
    assert any(ch.cell == (1, 2) and ch.old == CellState.FREE and ch.new == CellState.BLOCKED for ch in d.changes)
    gt.clear_debris((1, 2), tick=9)
    d.integrate(gt.sense((1, 1), 3), tick=10)
    assert d.is_free((1, 2)) and not d.is_impassable((1, 2))


def test_remembered_blockage_that_is_gone_is_detected_as_a_change():
    gt = GroundTruthWorld(LOOP, [], ENTRY)
    d = DiscoveredWorld(gt.rows, gt.cols)
    d.mark_passage((1, 3), PassageState.BLOCKED, "inherited", memory_id=7)           # inherited memory says blocked
    d.integrate(gt.sense((1, 1), 3))                                                  # ... but it is actually free
    assert not d.is_impassable((1, 3))
    assert any("remembered blockage" in ch.reason for ch in d.changes)


def test_debris_blocks_sensing_line_of_sight_and_movement_in_ground_truth():
    gt = GroundTruthWorld(LOOP, [], ENTRY)
    assert gt.add_debris((1, 3)) and not gt.add_debris((1, 3))
    assert not gt.add_debris((0, 0)) and not gt.add_debris(ENTRY)                      # walls / entry refused
    assert gt.is_blocked(1, 3) and not gt.is_wall(1, 3)
    assert (1, 3) in gt.sense((1, 1), 3).blocked
    assert (1, 4) not in gt.sense((1, 1), 3).free                                      # cannot see through debris
