"""
navigation/engine.py
=====================
ONE shared navigation/mission engine, replacing the three separate ad hoc
implementations the Phase-0 prototype had (an astar() in writer_robot/, a
second astar() in simulation/system.py, and a third state machine in
executor_robot/mission.py that wasn't wired to either).

Modes (common.enums.NavMode):
  EXPLORE  — Writer: no goal, maximise information gain (frontier search).
  EXECUTE  — Executor: goal-directed frontier search biased toward a target
             cell it does not yet have a mapped path to (it only knows the
             target's coordinates from a beacon, not the tunnel layout
             between here and there — it has to discover its own route,
             same as the Writer would).
  VERIFY   — handled by the caller (executor_robot/executor.py): once the
             goal cell is reached, re-sensing + comparison happens there.
             Nothing route-planning-specific is needed for VERIFY itself.
  RETURN   — [PLANNED] not used in the Phase 1 demo.

PHASE-1 PATCH: `plan_route()` is the weighted planner the Executor uses. It
plans OPTIMISTICALLY through unexplored cells (slightly dearer than known-free
ones), refuses known/remembered obstructions, and penalises inherited
TEMPORARILY_BLOCKED / DETOUR_REQUIRED / ACCESSIBILITY_UNKNOWN passages. Because
it re-plans from whatever the robot currently believes, an Executor never
"replays" an old route — a new obstruction simply changes the answer.

Everything here operates ONLY on a DiscoveredWorld (simulation/world.py),
never on GroundTruthWorld — that separation is what makes "no cheating"
structurally true rather than a convention someone can forget.

[IMPLEMENTED]
"""

from __future__ import annotations

import random

import heapq
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Set, Tuple

from common.enums import NavMode
from simulation.world import Cell, DiscoveredWorld

# ── Tunable weights [ASSUMPTION: not field-validated, but exposed so they
#    can be tuned per the vision doc's "make weights configurable" request] ──
W_INFO_GAIN = 1.0
W_DISTANCE  = 0.6
W_RISK      = 0.3
W_GOAL      = 2.5     # only used in EXECUTE mode — pulls choices toward the goal
W_SECTOR    = 3.0     # PATCH 5 — sector-hint bias for replacement writer exploration

INFO_GAIN_RADIUS = 3  # cells; how far around a candidate we count as "would reveal"


# ── A* over known-free cells only ─────────────────────────────────────────────
def astar_known(world: DiscoveredWorld, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    """
    Shortest path from start to goal using ONLY cells the robot has already
    mapped as free. Returns None if no such path exists yet (goal may be
    unreached territory, or a currently-unknown gap separates them).
    [IMPLEMENTED]
    """
    if start == goal:
        return [start]

    # The goal cell itself may be an as-yet-unvisited target coordinate
    # (EXECUTE mode aiming at a beacon's cell before ever reaching it) — the
    # free-check on candidate neighbours below is relaxed for the goal cell
    # specifically, so a path can still be found up to it.
    def neighbours(cell: Cell):
        r, c = cell
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nxt = (r + dr, c + dc)
            if world.is_free(nxt) or nxt == goal:
                yield nxt

    h = lambda cell: abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])
    heap: List[Tuple[int, int, Cell]] = [(h(start), 0, start)]
    came: Dict[Cell, Cell] = {}
    g_cost: Dict[Cell, int] = {start: 0}
    visited: Set[Cell] = set()

    while heap:
        _, cost, cur = heapq.heappop(heap)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == goal:
            path = [cur]
            while path[-1] in came:
                path.append(came[path[-1]])
            return path[::-1]
        for nxt in neighbours(cur):
            ng = g_cost[cur] + 1
            if ng < g_cost.get(nxt, 10**9):
                g_cost[nxt] = ng
                came[nxt] = cur
                heapq.heappush(heap, (ng + h(nxt), ng, nxt))
    return None


# ── PHASE-1: weighted, replannable route search ──────────────────────────────
@dataclass
class Route:
    path: List[Cell]      # includes start and goal
    cost: float

    @property
    def length(self) -> int:
        return len(self.path) - 1


def plan_route(world: DiscoveredWorld, start: Cell, goal: Cell, *,
               optimistic: bool = True, relax: bool = False) -> Optional[Route]:
    """
    Weighted A* from `start` to `goal` using `world.traversal_cost`.
    optimistic=True allows unexplored cells (the usual Executor mode);
    optimistic=False restricts the route to known-free / inherited-free cells
    (used for the return trip and for the Writer's way home).
    Returns None when no route exists under the robot's current beliefs.
    The goal is enterable unless it is known/remembered to be obstructed.
    [IMPLEMENTED]
    """
    if start == goal:
        return Route([start], 0.0)
    if world.is_impassable(goal) and not relax:
        return None

    h = lambda cell: abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])   # admissible: min step cost is 1
    heap: List[Tuple[float, float, Cell]] = [(h(start), 0.0, start)]
    came: Dict[Cell, Cell] = {}
    g_cost: Dict[Cell, float] = {start: 0.0}
    done: Set[Cell] = set()
    while heap:
        _, g, cur = heapq.heappop(heap)
        if cur in done:
            continue
        done.add(cur)
        if cur == goal:
            path = [cur]
            while path[-1] in came:
                path.append(came[path[-1]])
            return Route(path[::-1], g)
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nxt = (cur[0] + dr, cur[1] + dc)
            step = world.traversal_cost(nxt, optimistic=optimistic, relax=relax)
            if step is None and nxt == goal and not world.is_impassable(goal) and world.in_bounds(goal):
                step = 1.0          # the goal cell itself may be unexplored
            if step is None:
                continue
            ng = g + step
            if ng < g_cost.get(nxt, float("inf")):
                g_cost[nxt] = ng
                came[nxt] = cur
                heapq.heappush(heap, (ng + h(nxt), ng, nxt))
    return None


# ── Frontier clustering ───────────────────────────────────────────────────────
def frontier_regions(world: DiscoveredWorld) -> List[Cell]:
    """
    Group adjacent frontier cells into regions (flood fill) and return one
    representative cell per region (the region member closest to its own
    centroid). Clustering avoids offering many near-duplicate candidates
    for the same unexplored pocket. [IMPLEMENTED]
    """
    cells = world.frontier_cells()
    remaining = set(cells)
    reps: List[Cell] = []
    while remaining:
        start = next(iter(remaining))
        region = {start}
        stack = [start]
        remaining.discard(start)
        while stack:
            r, c = stack.pop()
            for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (r + dr, c + dc)
                if nxt in remaining:
                    remaining.discard(nxt)
                    region.add(nxt)
                    stack.append(nxt)
        cr = sum(r for r, _ in region) / len(region)
        cc = sum(c for _, c in region) / len(region)
        rep = min(region, key=lambda cell: (cell[0] - cr) ** 2 + (cell[1] - cc) ** 2)
        reps.append(rep)
    return reps


def _local_unknown_count(world: DiscoveredWorld, cell: Cell, radius: int) -> int:
    r0, c0 = cell
    count = 0
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if abs(dr) + abs(dc) > radius:
                continue
            if world.state_of((r0 + dr, c0 + dc)).value == "UNKNOWN":
                count += 1
    return count


@dataclass
class Candidate:
    cell: Cell
    utility: float
    distance: int
    info_gain: int


def choose_next_target(
    world: DiscoveredWorld,
    start: Cell,
    mode: NavMode,
    goal: Optional[Cell] = None,
    sector: Optional[Tuple[int, int, int, int]] = None,
    rng: Optional["random.Random"] = None,
    jitter: float = 0.0,
) -> Optional[Candidate]:
    """
    Pick the next frontier cell to head toward.

    EXPLORE (Writer): maximise information gain, penalise distance/risk.
    EXECUTE (Executor): same scoring PLUS a strong bonus for frontiers that
      reduce remaining distance-to-goal.
    sector (PATCH 5): optional (row_min, col_min, row_max, col_max) bounding
      box that adds W_SECTOR to any frontier inside it — used to bias a
      replacement writer toward an under-explored region without exposing the
      hidden map. Has no effect in EXECUTE mode.
    rng / jitter (PHASE-1): when jitter > 0, each frontier's utility receives a
      seeded uniform perturbation in [-jitter, +jitter]. It breaks near-ties
      differently per seed, so two seeds explore the same unknown space along
      different trajectories. jitter=0 (default) is fully deterministic.

    Returns None when no reachable frontier remains. [IMPLEMENTED]
    """
    candidates = frontier_regions(world)
    best: Optional[Candidate] = None
    goal_dist_now = abs(start[0] - goal[0]) + abs(start[1] - goal[1]) if goal else 0

    for cell in candidates:
        path = astar_known(world, start, cell)
        if path is None:
            continue
        distance  = len(path) - 1
        info_gain = _local_unknown_count(world, cell, INFO_GAIN_RADIUS)
        risk      = distance / max(1, world.rows + world.cols)

        utility = W_INFO_GAIN * info_gain - W_DISTANCE * distance - W_RISK * risk

        if mode == NavMode.EXECUTE and goal is not None:
            goal_dist_after = abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])
            progress = goal_dist_now - goal_dist_after
            utility += W_GOAL * progress

        # PATCH 5: sector bias (EXPLORE mode only)
        if sector is not None and mode == NavMode.EXPLORE:
            r0, c0, r1, c1 = sector
            if r0 <= cell[0] <= r1 and c0 <= cell[1] <= c1:
                utility += W_SECTOR

        if rng is not None and jitter > 0.0:
            utility += rng.uniform(-jitter, jitter)

        cand = Candidate(cell=cell, utility=utility, distance=distance, info_gain=info_gain)
        if best is None or cand.utility > best.utility or (
            cand.utility == best.utility and cand.distance < best.distance
        ):
            best = cand

    return best


# ── Movement (pixel-smoothed stepping along a cell path) ──────────────────────
class MovementController:
    """
    Advances a robot's pixel position along a queued cell path, calling
    `on_cell_reached(cell)` each time a cell boundary is crossed. Shared by
    writer_robot/writer.py and executor_robot/executor.py so movement
    /animation logic exists exactly once. [IMPLEMENTED]
    """

    def __init__(self, cell_to_px: Callable[[Cell], Tuple[float, float]], speed_px: float):
        self._cell_to_px = cell_to_px
        self.speed_px = speed_px
        self.px: float = 0.0
        self.py: float = 0.0
        self.angle: float = 0.0
        self._path_px: List[Tuple[float, float]] = []
        self._path_cells: List[Cell] = []
        self.current_cell: Optional[Cell] = None
        # PHASE-1: physics hook. `can_advance(next_cell)` is asked before the
        # robot leaves a cell; False means it bumped into something that was not
        # there when it last looked (e.g. debris that appeared after sensing).
        self.can_advance: Optional[Callable[[Cell], bool]] = None
        self.bumped_cell: Optional[Cell] = None

    def teleport(self, cell: Cell) -> None:
        self.px, self.py = self._cell_to_px(cell)
        self._path_px = []
        self._path_cells = []
        self.current_cell = cell

    def set_path(self, cells: List[Cell]) -> None:
        """`cells` should NOT include the current cell — only remaining steps."""
        self._path_px = [self._cell_to_px(c) for c in cells]
        self._path_cells = list(cells)

    @property
    def is_moving(self) -> bool:
        return bool(self._path_px)

    def remaining_cells(self) -> List[Cell]:
        return list(self._path_cells)

    def cancel_path(self) -> None:
        self._path_px = []
        self._path_cells = []

    def step(self) -> Optional[Cell]:
        """Advance one tick. Returns the cell just reached this tick, else None."""
        import math
        if not self._path_px:
            return None
        # The next cell became obstructed (debris appeared after the last look)?
        if (self.can_advance is not None and self.current_cell is not None
                and not self.can_advance(self._path_cells[0])):
            self.bumped_cell = self._path_cells[0]
            if (self.px, self.py) == self._cell_to_px(self.current_cell):
                self.cancel_path()                       # still on the cell centre: just stop
            else:                                        # already part-way into the gap: back off to the cell we left
                self._path_px = [self._cell_to_px(self.current_cell)]
                self._path_cells = [self.current_cell]
            return None
        tx, ty = self._path_px[0]
        dx, dy = tx - self.px, ty - self.py
        dist = math.hypot(dx, dy)
        if dist < self.speed_px + 0.5:
            self.px, self.py = tx, ty
            self._path_px.pop(0)
            self.current_cell = self._path_cells.pop(0)
            return self.current_cell
        self.angle = math.atan2(dy, dx)
        self.px += dx / dist * self.speed_px
        self.py += dy / dist * self.speed_px
        return None
