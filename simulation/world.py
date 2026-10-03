"""
simulation/world.py
====================
The ground-truth / discovered-map split the project vision requires as an
architectural constraint, not just a convention.

GroundTruthWorld  — the complete map + hidden events. Only the simulator's
                    sensing call may read it. There is no public accessor
                    for "the full event list" or "the full grid" — see the
                    class docstring below.
DiscoveredWorld   — one robot's own reconstructed knowledge (per-cell
                    FREE/WALL/UNKNOWN, plus events it has personally
                    detected). This is the only thing writer_robot /
                    executor_robot code may depend on.

Grid <-> metric conversion also lives here, since it's a property of the
simulated arena, not of any one robot.

[IMPLEMENTED]
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from common.enums import CellState, EventType, PassageState
from common.models import Observation

Cell = Tuple[int, int]  # (row, col)

CELL_PX = 44           # pixels per grid cell (renderer uses this too)
METRES_PER_CELL = 0.5  # 1 cell = 0.5 m, matches the original prototype's scale


def cell_to_metres(cell: Cell) -> Tuple[float, float]:
    """WORLD metres measured from the arena's top-left corner. Used by the
    PHYSICS side only (radio range, rendering). Robots use EntryFrame instead."""
    r, c = cell
    return round(c * METRES_PER_CELL, 3), round(r * METRES_PER_CELL, 3)


def metres_to_cell(x_m: float, y_m: float) -> Cell:
    return round(y_m / METRES_PER_CELL), round(x_m / METRES_PER_CELL)


def line_cells(a: Cell, b: Cell) -> List[Cell]:
    """Grid cells on the straight line a -> b (Bresenham), both ends included."""
    (r0, c0), (r1, c1) = a, b
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
    err, r, c = dc - dr, r0, c0
    out: List[Cell] = [(r, c)]
    while (r, c) != (r1, c1):
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
        out.append((r, c))
    return out


class EntryFrame:
    """
    Robot-LOCAL frame <-> grid map frame (Phase-1 audit fix).

    The documentation always said local coordinates have their origin at the
    zone ENTRY; the old helpers measured from the arena corner. This class is
    the real thing:

        x_local = (col - entry_col) * res          (+x = screen right)
        y_local = (entry_row - row) * res          (+y = screen up)

    Right-handed, heading 0 = facing +x. Robots keep their map in cell indices
    anchored at the entry cell; this class converts a cell to the metres a
    BeaconMessage carries, and back for an Executor reading a brief.
    """

    def __init__(self, entry: Cell, res: float = METRES_PER_CELL) -> None:
        self.entry = entry
        self.res = res

    def cell_to_local(self, cell: Cell) -> Tuple[float, float]:
        return (round((cell[1] - self.entry[1]) * self.res, 3),
                round((self.entry[0] - cell[0]) * self.res, 3))

    def local_to_cell(self, x_m: float, y_m: float) -> Cell:
        return (self.entry[0] - round(y_m / self.res), self.entry[1] + round(x_m / self.res))

    def step_to_local_delta(self, from_cell: Cell, to_cell: Cell) -> Tuple[float, float]:
        """Metres (dx, dy) of a one-cell move, in the local frame."""
        return ((to_cell[1] - from_cell[1]) * self.res, (from_cell[0] - to_cell[0]) * self.res)


def cell_to_px(cell: Cell) -> Tuple[float, float]:
    r, c = cell
    return c * CELL_PX + CELL_PX / 2.0, r * CELL_PX + CELL_PX / 2.0


@dataclass(frozen=True)
class HiddenEvent:
    id: int
    event_type: EventType
    row: int
    col: int
    severity: int  # 1-4


@dataclass
class SensorSnapshot:
    """What GroundTruthWorld.sense() reveals for one sensing call."""
    free: Set[Cell]
    wall: Set[Cell]
    event_here: Optional[HiddenEvent]
    at: Cell
    # PHASE-1 PATCH
    blocked: Set[Cell] = field(default_factory=set)   # debris cells seen (passable when cleared)
    event_offset: Optional[Tuple[int, int]] = None    # (d_row, d_col) of event_here relative to `at`


class GroundTruthWorld:
    """
    Complete map + hidden events. [SIMULATED]

    Deliberately exposes almost nothing. `sense()` is the single doorway
    every perception call must go through. Do not add a convenience
    accessor here "just to make the Writer's life easier" — that shortcut
    is exactly what produced the Phase-0 bug this rewrite fixes (the old
    WriterRobot read a module-level EVENTS list directly). If a test or a
    robot module needs ground truth for anything other than scoring
    metrics, that is a design smell, not a missing accessor.
    """

    def __init__(self, grid: List[List[int]], events: List[HiddenEvent], entry: Cell):
        self._grid = grid
        self._events: Dict[int, HiddenEvent] = {e.id: e for e in events}
        self.entry = entry
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        # PHASE-1 PATCH: dynamic environment. Debris sits on an otherwise free
        # cell; unlike a wall it can APPEAR later and be CLEARED.
        self._debris: Dict[Cell, dict] = {}
        self._resolved: Set[int] = set()        # event ids that an executor has resolved
        self.change_log: List[str] = []         # simulator-side audit trail

    def is_wall(self, r: int, c: int) -> bool:
        if not self.in_bounds(r, c):
            return True
        return self._grid[r][c] == 1

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    # ── PHASE-1 PATCH: dynamic debris (simulator / actuator doorways) ─────────
    def is_debris(self, cell: Cell) -> bool:
        return cell in self._debris

    def is_blocked(self, r: int, c: int) -> bool:
        """Wall OR debris — what a moving robot physically cannot enter."""
        return self.is_wall(r, c) or (r, c) in self._debris

    def add_debris(self, cell: Cell, tick: int = 0, origin: str = "dynamic") -> bool:
        """Place debris on a free corridor cell. Refuses walls, the entry, and
        cells that already hold debris. Returns True if placed."""
        if self.is_wall(*cell) or cell == self.entry or cell in self._debris:
            return False
        self._debris[cell] = {"tick": tick, "origin": origin}
        self.change_log.append(f"t={tick} debris appeared at {cell} ({origin})")
        return True

    def clear_debris(self, cell: Cell, tick: int = 0) -> bool:
        """Actuator doorway used by a Debris Executor. True if debris was removed."""
        if cell not in self._debris:
            return False
        del self._debris[cell]
        self.change_log.append(f"t={tick} debris cleared at {cell}")
        return True

    def event_cells_view(self) -> Dict[Cell, Tuple[str, bool]]:
        """Simulator-side view for rendering / metrics ONLY: {cell: (event type, resolved)}."""
        return {(e.row, e.col): (e.event_type.value, e.id in self._resolved) for e in self._events.values()}

    def reachable_from_entry(self) -> Set[Cell]:
        """Cells an unobstructed robot could reach from the entry right now (walls AND debris block).
        Simulator-side; used for honest coverage / optimal-path metrics."""
        seen, q = {self.entry}, deque([self.entry])
        while q:
            r, c = q.popleft()
            for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nb = (r + dr, c + dc)
                if nb not in seen and self.in_bounds(*nb) and not self.is_blocked(*nb):
                    seen.add(nb)
                    q.append(nb)
        return seen

    def shortest_path_len(self, a: Cell, b: Cell) -> Optional[int]:
        """True shortest path length in cells (simulator-side), or None if unreachable."""
        if a == b:
            return 0
        seen, q = {a: 0}, deque([a])
        while q:
            cur = q.popleft()
            for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nb = (cur[0] + dr, cur[1] + dc)
                if nb in seen or not self.in_bounds(*nb) or self.is_blocked(*nb):
                    continue
                seen[nb] = seen[cur] + 1
                if nb == b:
                    return seen[nb]
                q.append(nb)
        return None

    def debris_cells(self) -> Dict[Cell, dict]:
        """Simulator-side view, for scenario scheduling and metrics only."""
        return dict(self._debris)

    def resolve_event(self, event_id: int, tick: int = 0) -> bool:
        """Actuator doorway: a Fire/Gas Executor's task removes the hazard."""
        if event_id in self._events and event_id not in self._resolved:
            self._resolved.add(event_id)
            self.change_log.append(f"t={tick} event {event_id} resolved")
            return True
        return False

    def actuate(self, cell: Cell, capability: str, tick: int = 0) -> Optional[str]:
        """
        PHYSICAL effect of an executor's task at `cell` (the actuator doorway —
        the counterpart of sense()). Returns an effect label, or None when the
        action did nothing. The return value is what the robot's own sensors would
        report back; it is not a window onto hidden state.
          FIRE    extinguishes a FIRE within one cell      -> "FIRE_EXTINGUISHED"
          GAS     ventilates; gas is reduced, NOT removed  -> "GAS_VENTED"
          VICTIM  assessment only; the victim stays        -> "VICTIM_ASSESSED"
          DEBRIS  removes debris on/adjacent to the cell   -> "DEBRIS_CLEARED"
        [SIMULATED]
        """
        if capability == "DEBRIS":
            r, c = cell
            for nb in ((r, c), (r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if nb in self._debris:
                    self.clear_debris(nb, tick)
                    return "DEBRIS_CLEARED"
            return None
        want = {"FIRE": EventType.FIRE, "GAS": EventType.GAS, "VICTIM": EventType.VICTIM_PRESENCE}.get(capability)
        if want is None:
            return None
        for ev in self._events.values():
            if ev.id in self._resolved or ev.event_type != want:
                continue
            if abs(ev.row - cell[0]) + abs(ev.col - cell[1]) <= 1:
                if capability == "FIRE":
                    self.resolve_event(ev.id, tick)
                    return "FIRE_EXTINGUISHED"
                return "GAS_VENTED" if capability == "GAS" else "VICTIM_ASSESSED"
        return None

    def walls_between(self, a: Cell, b: Cell) -> int:
        """Number of permanent wall cells strictly between a and b on the straight
        line (Bresenham). Debris is radio-transparent. [SIMULATED, ASSUMPTION]"""
        if a > b:                      # Bresenham is not symmetric: canonical order => a link is symmetric
            a, b = b, a
        cells = line_cells(a, b)
        return sum(1 for cell in cells[1:-1] if self.is_wall(*cell))

    def line_of_sight(self, a: Cell, b: Cell) -> bool:
        """Radio line-of-sight through the PERMANENT walls only (Bresenham).
        Debris is treated as radio-transparent. [SIMULATED, ASSUMPTION]"""
        if a > b:
            a, b = b, a
        (r0, c0), (r1, c1) = a, b
        dr, dc = abs(r1 - r0), abs(c1 - c0)
        sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
        err, r, c = dc - dr, r0, c0
        while True:
            if self.is_wall(r, c):
                return False
            if (r, c) == (r1, c1):
                return True
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr

    def all_event_ids(self) -> Set[int]:
        """For the simulator's own metrics/scoring only — never for a robot's decisions."""
        return set(self._events.keys())

    def event_by_id(self, event_id: int) -> Optional[HiddenEvent]:
        """For the simulator's own metrics/scoring only."""
        return self._events.get(event_id)

    def sense(self, from_cell: Cell, radius: int) -> SensorSnapshot:
        """
        BFS outward from `from_cell` through free cells up to `radius`
        steps, revealing free/wall cells as it goes. Walls block further
        propagation, so this behaves like local visibility through an open
        tunnel rather than an unobstructed circle. [SIMULATED][IMPLEMENTED]
        """
        r0, c0 = from_cell
        free: Set[Cell] = set()
        wall: Set[Cell] = set()
        blocked: Set[Cell] = set()
        visited: Set[Cell] = {from_cell}
        q = deque([(r0, c0, 0)])
        if not self.is_wall(r0, c0):
            free.add(from_cell)
        while q:
            r, c, d = q.popleft()
            if d >= radius:
                continue
            for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nr, nc = r + dr, c + dc
                if not self.in_bounds(nr, nc):
                    continue
                if self.is_wall(nr, nc):
                    wall.add((nr, nc))
                    continue
                if (nr, nc) in self._debris:      # PHASE-1: debris blocks view + movement
                    blocked.add((nr, nc))
                    visited.add((nr, nc))
                    continue
                if (nr, nc) not in visited:
                    visited.add((nr, nc))
                    free.add((nr, nc))
                    q.append((nr, nc, d + 1))
        # An event is "sensed" if its cell falls inside this sweep's revealed
        # free area (not only if the robot is standing exactly on top of it —
        # a fire/gas sensor has *some* detection range). If more than one
        # event falls in range, report the nearest. This also keeps "map
        # coverage" and "event detectability" consistent: once a cell has
        # been swept as FREE, anything in it has also been within sensor
        # range at least once. [SIMULATED][ASSUMPTION: detection range ==
        # mapping sensing range; a real system might separate these.]
        event_here = None
        best_dist = None
        for ev in self._events.values():
            if ev.id in self._resolved:           # PHASE-1: resolved hazards are gone
                continue
            cell = (ev.row, ev.col)
            if cell == from_cell or cell in free:
                d = abs(ev.row - from_cell[0]) + abs(ev.col - from_cell[1])
                if best_dist is None or d < best_dist:
                    best_dist, event_here = d, ev
        offset = None if event_here is None else (event_here.row - from_cell[0], event_here.col - from_cell[1])
        return SensorSnapshot(free=free, wall=wall, event_here=event_here, at=from_cell,
                              blocked=blocked, event_offset=offset)


@dataclass
class PassageInfo:
    """Navigation knowledge about one cell, from the robot's own sensing or inherited memory."""
    state:     PassageState
    source:    str = "own"            # "own" | "inherited"
    tick:      int = 0
    memory_id: Optional[int] = None


@dataclass
class CellChange:
    """One cell whose believed state changed — the raw material for 'discrepancy'."""
    cell:   Cell
    old:    CellState
    new:    CellState
    reason: str


# Route-cost penalties for inherited passage knowledge (see navigation/engine.plan_route).
PASSAGE_PENALTY = {
    PassageState.TEMPORARILY_BLOCKED: 12.0,   # expected to clear: last resort only
    PassageState.DETOUR_REQUIRED: 5.0,
    PassageState.ACCESSIBILITY_UNKNOWN: 4.0,
    PassageState.OPEN: 0.0,
}
UNKNOWN_CELL_COST = 1.5   # optimism: unexplored cells are slightly worse than known-free ones


class DiscoveredWorld:
    """
    One robot's own reconstructed map. [IMPLEMENTED]
    Starts fully UNKNOWN. Grows only through `integrate()`, fed exclusively
    by a sensor snapshot.

    PHASE-1 PATCH: the map is no longer append-only. The latest observation of
    a cell wins, so a cell that was FREE can become BLOCKED (debris appeared)
    and a BLOCKED cell can become FREE again (cleared). Every such change is
    reported in `changes`, which is how a robot notices that the world differs
    from what it (or the inherited memory) believed.

      passage_info  navigation knowledge per cell (own sensing or inherited)
      hint_free     cells inherited memory BELIEVES are free (route skeleton);
                    not observed — a sensing result may contradict them
    """

    def __init__(self, rows: int, cols: int):
        self.rows, self.cols = rows, cols
        self._cells: Dict[Cell, CellState] = {}
        self.observations: List[Observation] = []   # events this robot has itself detected
        self.passage_info: Dict[Cell, PassageInfo] = {}
        self.hint_free: Set[Cell] = set()
        self.changes: List[CellChange] = []          # changes caused by the latest integrate()
        self.total_changes = 0

    # ── queries ───────────────────────────────────────────────────────────────
    def state_of(self, cell: Cell) -> CellState:
        return self._cells.get(cell, CellState.UNKNOWN)

    def is_free(self, cell: Cell) -> bool:
        return self.state_of(cell) == CellState.FREE

    def is_known(self, cell: Cell) -> bool:
        return self.state_of(cell) != CellState.UNKNOWN

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.rows and 0 <= cell[1] < self.cols

    def is_impassable(self, cell: Cell) -> bool:
        """Known or remembered to be obstructed (wall, debris, inherited BLOCKED/IMPASSABLE)."""
        if self.state_of(cell) in (CellState.WALL, CellState.BLOCKED):
            return True
        info = self.passage_info.get(cell)
        return info is not None and info.state in (PassageState.BLOCKED, PassageState.IMPASSABLE)

    def traversal_cost(self, cell: Cell, optimistic: bool = False, relax: bool = False) -> Optional[float]:
        """Cost of entering `cell`, or None if it may not be entered.
        relax=True answers the hypothetical "what if the REMEMBERED/observed
        debris were gone": debris cells become enterable at a heavy cost (walls
        stay impassable). Used only to work out WHICH blockage cuts the way."""
        if not self.in_bounds(cell):
            return None
        if self.is_impassable(cell):
            if relax and self.state_of(cell) != CellState.WALL:
                return 20.0
            return None
        st = self.state_of(cell)
        if st == CellState.FREE or cell in self.hint_free:
            base = 1.0
        elif optimistic:
            base = UNKNOWN_CELL_COST
        else:
            return None
        info = self.passage_info.get(cell)
        return base + (PASSAGE_PENALTY.get(info.state, 0.0) if info else 0.0)

    # ── updates ───────────────────────────────────────────────────────────────
    def integrate(self, snapshot: SensorSnapshot, tick: int = 0) -> int:
        """Merge a sensor snapshot in (last observation wins). Returns count of
        newly-revealed cells; state changes of already-known cells go to `changes`."""
        newly = 0
        self.changes = []

        def _set(cell: Cell, new: CellState, reason_if_changed: str) -> None:
            nonlocal newly
            old = self.state_of(cell)
            if old == new:
                return
            if old == CellState.UNKNOWN:
                newly += 1
                if cell in self.hint_free and new != CellState.FREE:
                    self.changes.append(CellChange(cell, old, new, "inherited route cell is not free"))
            else:
                self.changes.append(CellChange(cell, old, new, reason_if_changed))
                if new == CellState.FREE:
                    newly += 1
            self._cells[cell] = new

        for cell in snapshot.free:
            _set(cell, CellState.FREE, "previously obstructed cell is now free")
            info = self.passage_info.get(cell)
            if info is not None and info.state in (PassageState.BLOCKED, PassageState.IMPASSABLE,
                                                    PassageState.TEMPORARILY_BLOCKED):
                self.changes.append(CellChange(cell, CellState.BLOCKED, CellState.FREE,
                                               "remembered blockage is not there"))
                del self.passage_info[cell]
        for cell in snapshot.wall:
            _set(cell, CellState.WALL, "cell turned out to be a wall")
        for cell in getattr(snapshot, "blocked", ()):
            _set(cell, CellState.BLOCKED, "path became blocked")
            self.passage_info[cell] = PassageInfo(PassageState.BLOCKED, "own", tick)
        self.total_changes += len(self.changes)
        return newly

    def mark_passage(self, cell: Cell, state: PassageState, source: str = "inherited",
                     tick: int = 0, memory_id: Optional[int] = None) -> None:
        """Record navigation knowledge (own observation or inherited). OPEN removes it."""
        if state == PassageState.OPEN:
            self.passage_info.pop(cell, None)
            return
        self.passage_info[cell] = PassageInfo(state, source, tick, memory_id)

    def add_hint_free(self, cells) -> None:
        self.hint_free.update(cells)

    def record_observation(self, obs: Observation) -> None:
        self.observations.append(obs)

    def known_free_cells(self) -> Set[Cell]:
        return {c for c, s in self._cells.items() if s == CellState.FREE}

    def blocked_cells(self) -> Set[Cell]:
        """Cells believed obstructed by debris (own sensing) or inherited memory."""
        out = {c for c, s in self._cells.items() if s == CellState.BLOCKED}
        out |= {c for c, i in self.passage_info.items()
                if i.state in (PassageState.BLOCKED, PassageState.IMPASSABLE)}
        return out

    def all_known_cells(self) -> Dict[Cell, CellState]:
        """Read-only copy, for rendering this robot's OWN discovered map (fog of war). [IMPLEMENTED]"""
        return dict(self._cells)

    def frontier_cells(self) -> Set[Cell]:
        """FREE cells that border at least one still-UNKNOWN cell."""
        out: Set[Cell] = set()
        for (r, c), s in self._cells.items():
            if s != CellState.FREE:
                continue
            for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                if self.state_of((r + dr, c + dc)) == CellState.UNKNOWN:
                    out.add((r, c))
                    break
        return out

    def coverage_ratio(self, total_free_cells: int) -> float:
        if total_free_cells <= 0:
            return 0.0
        return len(self.known_free_cells()) / total_free_cells
