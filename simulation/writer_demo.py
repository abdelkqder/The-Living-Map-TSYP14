"""
simulation/writer_demo.py — Simulation B: WRITER EXPLORATION / COVERAGE
=======================================================================
Purpose: show that the Writer explores an UNKNOWN space autonomously (frontier
exploration, no stored route), handles obstacles and dead ends, finds events,
and gets back to the entry without shutting down.

    python -m simulation.writer_demo --seed 42
    python -m simulation.writer_demo --seed 42 --debris 2          # debris present when it enters
    python -m simulation.writer_demo --seed 42 --compare 5         # 5 seeds: different trajectories

Every number printed is read from the finished simulation. [SIMULATED]
"""
from __future__ import annotations

import hashlib
import random
from typing import Dict, List, Optional

from simulation.phase1 import (
    CHALLENGE_TYPES, ascii_map, build_scenario, emit_json, explore_phase, metres, parse_args, section, table,
)
from simulation.scenario import _free_cells, BASE_GRID, ENTRY
from simulation.system import LiveMapSystem
from simulation.world import Cell


def pick_existing_debris(seed: int, n: int) -> List[Cell]:
    """n random corridor cells that are not the entry or its neighbours (deterministic per seed)."""
    rng = random.Random(seed * 7919 + 13)
    cands = [c for c in _free_cells(BASE_GRID) if abs(c[0] - ENTRY[0]) + abs(c[1] - ENTRY[1]) > 3]
    rng.shuffle(cands)
    return cands[:n]


def route_fingerprint(route) -> str:
    return hashlib.sha1(repr(list(route)).encode()).hexdigest()[:8]


def run(seed: int = 42, *, debris: int = 0, jitter: float = 0.6, system: Optional[LiveMapSystem] = None) -> Dict[str, object]:
    sc = build_scenario(f"writer-seed{seed}", seed, CHALLENGE_TYPES, jitter=jitter,
                        debris_existing=pick_existing_debris(seed, debris))
    sys_ = system or LiveMapSystem(sc)
    w = explore_phase(sys_)
    reachable = sys_.world.reachable_from_entry()
    known = w.discovered.known_free_cells()
    cov = 100.0 * len(known & reachable) / max(1, len(reachable))
    dist_cells = len(w.route) - 1
    res = {
        "seed": seed, "jitter": jitter, "debris_present_at_entry": len(sc.debris_existing),
        "coverage_pct_of_reachable": round(cov, 1), "distance_cells": dist_cells,
        "distance_m_odometry": round(w.odometry.path_length_m, 2),
        "mission_ticks": w.tick, "mission_seconds": round(sys_.clock.ticks_to_seconds(w.tick), 1),
        "frontiers_selected": w.stats["frontiers_selected"], "dead_ends": w.stats["dead_ends"],
        "blocked_paths_found": w.stats["blockages_seen"], "blockages_preserved": w.stats["blockages_preserved"],
        "replans_due_to_blockage": w.stats["replans"], "alternative_routes_found": w.stats["alternative_routes"],
        "targets_abandoned": w.stats["abandoned_targets"],
        "events_found": len({(o.row, o.col) for o in w.discovered.observations if o.event_type.value != "BLOCKAGE"}),
        "events_total": len(sc.events), "beacons_placed": w.beacon_count, "relays_placed": w.stats["relays_dropped"],
        "returned_to_entry": w.returned, "return_tick": w.return_tick, "writer_state": w.state.value,
        "route_fingerprint": route_fingerprint(w.route),
        "records_at_command_post": len(sys_.living_map),
    }
    res["_system"], res["_writer"] = sys_, w
    return res


def report(res: Dict[str, object]) -> None:
    sys_, w = res["_system"], res["_writer"]
    section(f"SIMULATION B — WRITER EXPLORATION  (seed {res['seed']}, jitter {res['jitter']}, debris at entry {res['debris_present_at_entry']})")
    print("  The Writer starts with an EMPTY map at the entry and discovers the arena by sensing.")
    print("\n" + ascii_map(sys_, route=w.route, discovered=w.discovered,
                           title="  Map (',' = never explored, '*' = Writer's actual route):"))
    section("METRICS (all read from the run)")
    keys = [k for k in res if not k.startswith("_")]
    table(["metric", "value"], [(k, res[k]) for k in keys], [30, 24])
    section("WRITER LOG (decisions)")
    for line in w.log[:30]:
        print("  " + line)
    if len(w.log) > 30:
        print(f"  … {len(w.log) - 30} more lines")


def compare_seeds(seed: int, n: int, debris: int = 0) -> Dict[str, object]:
    """Same arena, different seeds => different trajectories (frontier tie-breaks + event/beacon placement)."""
    rows, prints = [], set()
    for k in range(n):
        r = run(seed + k, debris=debris)
        prints.add(r["route_fingerprint"])
        rows.append((seed + k, r["route_fingerprint"], r["distance_cells"], r["mission_ticks"],
                     r["coverage_pct_of_reachable"], r["returned_to_entry"]))
    section(f"TRAJECTORY COMPARISON over {n} seeds")
    table(["seed", "route hash", "cells", "ticks", "coverage %", "returned"], rows)
    print(f"\n  distinct trajectories: {len(prints)} of {n}")
    return {"distinct_trajectories": len(prints), "runs": n}


def main(argv=None) -> Dict[str, object]:
    args = parse_args("Simulation B — Writer exploration / coverage", lambda ap: (
        ap.add_argument("--debris", type=int, default=0, help="debris cells present when the Writer enters"),
        ap.add_argument("--jitter", type=float, default=0.6, help="seeded frontier tie-break noise (0 = deterministic)"),
        ap.add_argument("--compare", type=int, default=0, metavar="N", help="also run N seeds and compare trajectories")), argv)
    res = run(args.seed, debris=args.debris, jitter=args.jitter)
    if args.json:
        emit_json({k: v for k, v in res.items() if not k.startswith("_")})
    else:
        report(res)
        if args.compare:
            compare_seeds(args.seed, args.compare, args.debris)
    if args.png:
        from simulation.snapshot import save_png
        save_png(res["_system"], args.png, writer=res["_writer"])
    return res


if __name__ == "__main__":
    main()
