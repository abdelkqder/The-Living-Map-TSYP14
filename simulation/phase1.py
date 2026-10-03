"""
simulation/phase1.py
====================
Shared helpers for the four Phase-1 simulations (global_demo, writer_demo,
fleet_demo, dynamic_debris) and the memory comparison (compare_memory).

Everything here is HEADLESS (no display needed) and seeded. Metrics printed by
the demos are read from the simulation objects at the end of the run — nothing is
hard-coded or estimated.

[IMPLEMENTED] — runners, reports, ASCII map.  [SIMULATED] — the world they drive.
"""
from __future__ import annotations

import argparse
import json
import random
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from common.enums import CellState, EventType, ExecutorState, MemoryState
from executor_robot.capabilities import DEFAULT_FLEET
from simulation.scenario import BASE_GRID, ENTRY, Scenario, _free_cells, _place_events
from simulation.system import LiveMapSystem
from simulation.world import Cell, HiddenEvent, METRES_PER_CELL, line_cells

GUARD_TICKS = 60_000
CHALLENGE_TYPES = [EventType.FIRE, EventType.GAS]            # the two event types the challenge requires
MIXED_TYPES = [EventType.FIRE, EventType.GAS, EventType.VICTIM_PRESENCE]


# ── scenario building ────────────────────────────────────────────────────────
def make_events(seed: int, types: Sequence[EventType]) -> List[HiddenEvent]:
    rng = random.Random(seed)
    evs = _place_events(rng, list(types))
    sev = {EventType.FIRE: 3, EventType.GAS: 2, EventType.VICTIM_PRESENCE: 4}
    return [HiddenEvent(e.id, e.event_type, e.row, e.col, sev.get(e.event_type, e.severity)) for e in evs]


def build_scenario(name: str, seed: int, types: Sequence[EventType] = CHALLENGE_TYPES, *,
                   fleet: Optional[Dict[str, int]] = None, writer_max_ticks: Optional[int] = None,
                   jitter: float = 0.0, writer_stock: int = 16, **extra) -> Scenario:
    return Scenario(name=name, seed=seed, grid=BASE_GRID, entry=ENTRY, events=make_events(seed, types),
                    writer_max_ticks=writer_max_ticks, beacon_packet_loss=0.0, difficulty="EASY",
                    fleet=fleet, explore_jitter=jitter, writer_beacon_stock=writer_stock, **extra)


# ── runners ──────────────────────────────────────────────────────────────────
def run_ticks(system: LiveMapSystem, n: int) -> None:
    for _ in range(n):
        system.update()


def run_until(system: LiveMapSystem, pred: Callable[[LiveMapSystem], bool], guard: int = GUARD_TICKS) -> bool:
    for _ in range(guard):
        if pred(system):
            return True
        system.update()
    return pred(system)


def explore_phase(system: LiveMapSystem, settle_ticks: int = 400, **deploy_kw):
    """Deploy a Writer, run it to the end (it returns to the entry), let the last frames
    cross the beacon chain. Returns the Writer."""
    w = system.deploy_writer(**deploy_kw)
    run_until(system, lambda s: w.dead)
    run_ticks(system, settle_ticks)
    return w


def all_idle(system: LiveMapSystem) -> bool:
    return all(e.state in (ExecutorState.AVAILABLE, ExecutorState.NEEDS_REPAIR, ExecutorState.NEEDS_CHARGING,
                           ExecutorState.OUT_OF_SERVICE) and (e.mission is None or e.is_complete)
               for e in system.executors) and not system.mission_dispatcher.active_count


# ── presentation ─────────────────────────────────────────────────────────────
def section(title: str) -> None:
    print(f"\n{'=' * 78}\n  {title}\n{'=' * 78}")


def table(headers: Sequence[str], rows: Iterable[Sequence[object]], widths: Optional[Sequence[int]] = None) -> None:
    rows = [[str(c) for c in r] for r in rows]
    widths = widths or [max(len(h), *(len(r[i]) for r in rows)) if rows else len(h) for i, h in enumerate(headers)]
    fmt = "  " + "  ".join("{:<" + str(w) + "}" for w in widths)
    print(fmt.format(*headers))
    print("  " + "  ".join("-" * w for w in widths))
    for r in rows:
        print(fmt.format(*[c[:w] for c, w in zip(r, widths)]))


def ascii_map(system: LiveMapSystem, *, route: Optional[Sequence[Cell]] = None,
              route2: Optional[Sequence[Cell]] = None, discovered=None, title: str = "") -> str:
    """
    A headless picture of the arena. Legend:
      #  wall     .  explored free cell     ,  free cell never explored (unknown)
      B  beacon   O  ONA/entry   x  debris   F/G/V  fire / gas / victim event   ~ resolved event
      *  route    +  second route   (events shown only where the SIMULATOR knows them — a viewer's aid)
    """
    w = system.world
    disc = discovered
    route_set, route2_set = set(route or ()), set(route2 or ())
    beacons = set(system.beacons.drop_cells.values())
    ev_at = {cell: ({"FIRE": "F", "GAS": "G", "VICTIM_PRESENCE": "V"}.get(t, "?") if not done else "~")
             for cell, (t, done) in w.event_cells_view().items()}      # "~" = resolved
    debris = w.debris_cells()
    lines = [title] if title else []
    for r in range(w.rows):
        row = []
        for c in range(w.cols):
            cell = (r, c)
            if w.is_wall(r, c):
                ch = "#"
            elif cell == w.entry:
                ch = "O"
            elif cell in debris:
                ch = "x"
            elif cell in beacons:
                ch = "B"
            elif cell in ev_at:
                ch = ev_at[cell]
            elif cell in route2_set:
                ch = "+"
            elif cell in route_set:
                ch = "*"
            elif disc is not None and disc.is_known(cell):
                ch = "."
            elif disc is None:
                ch = "."
            else:
                ch = ","
            row.append(ch)
        lines.append("  " + "".join(row))
    return "\n".join(lines)


# ── CLI ──────────────────────────────────────────────────────────────────────
def parse_args(description: str, extra: Optional[Callable[[argparse.ArgumentParser], None]] = None,
               argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    import sys
    for stream in (sys.stdout, sys.stderr):          # never crash on a legacy Windows console code page
        try:
            stream.reconfigure(errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("--seed", type=int, default=42, help="random seed (default 42)")
    ap.add_argument("--quiet", action="store_true", help="print only the final summary")
    ap.add_argument("--json", action="store_true", help="print the result dict as JSON instead of the report")
    ap.add_argument("--png", metavar="FILE", default=None, help="also save a PNG snapshot (needs pygame, headless OK)")
    if extra:
        extra(ap)
    return ap.parse_args(argv)


def emit_json(result: Dict[str, object]) -> None:
    print(json.dumps(result, indent=2, default=str))


def metres(cells: float) -> float:
    return round(cells * METRES_PER_CELL, 2)


# ── simulator-side geometry used to PLACE debris and to JUDGE results (never given to a robot) ──
def true_path(world, a: Cell, b: Cell) -> List[Cell]:
    """One shortest path a -> b through currently passable cells (BFS), simulator-side."""
    from collections import deque
    prev: Dict[Cell, Optional[Cell]] = {a: None}
    q = deque([a])
    while q:
        cur = q.popleft()
        if cur == b:
            break
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nb = (cur[0] + dr, cur[1] + dc)
            if nb not in prev and world.in_bounds(*nb) and not world.is_blocked(*nb):
                prev[nb] = cur
                q.append(nb)
    if b not in prev:
        return []
    out, cur = [], b
    while cur is not None:
        out.append(cur)
        cur = prev[cur]
    return out[::-1]


def chokepoints(world, a: Cell, b: Cell) -> List[Cell]:
    """Cells whose obstruction makes b unreachable from a (single-cell cut vertices)."""
    out = []
    for cell in true_path(world, a, b)[1:-1]:
        world.add_debris(cell, -1, "probe")
        cut = world.shortest_path_len(a, b) is None
        world.clear_debris(cell, -1)
        if cut:
            out.append(cell)
    return out


def pick_debris(world, seed: int, mode: str, a: Cell, b: Cell, n: int = 1) -> List[Cell]:
    """
    Choose where NEW debris appears (seeded):
      random      any free, non-entry, non-event cell
      on_route    a random cell on a shortest path entry -> target
      chokepoint  a random cell whose blockage cuts the target off (falls back to on_route)
      seal        debris on every free neighbour of the target: the target becomes unreachable
    """
    rng = random.Random(seed * 104729 + 7)
    event_cells = set(world.event_cells_view())
    free = sorted(c for c in world.reachable_from_entry()
                  if c != world.entry and c not in event_cells and abs(c[0] - a[0]) + abs(c[1] - a[1]) > 2)
    if mode == "seal":                      # a collapse closing EVERY way into the target cell
        r, c = b
        return sorted(nb for nb in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1))
                      if world.in_bounds(*nb) and not world.is_wall(*nb) and nb != world.entry)[:max(n, 4)]
    if mode == "random":
        pool = free
    elif mode == "chokepoint":
        pool = [c for c in chokepoints(world, a, b) if c in free] or [c for c in true_path(world, a, b)[2:-1] if c in free]
    else:
        pool = [c for c in true_path(world, a, b)[2:-1] if c in free]
    rng.shuffle(pool)
    return pool[:n]
