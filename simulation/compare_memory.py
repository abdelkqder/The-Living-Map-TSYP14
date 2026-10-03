"""
simulation/compare_memory.py — WITH vs WITHOUT inherited Living-Map memory
==========================================================================
Same arena, same seed, same Writer run, same dynamic debris (same cell, same tick).
The ONLY difference is what the Command Post puts into the Executor's brief:

  LIVING MAP  brief carries the preserved record location, the beacon chain to it, and
              remembered blockages.   The NEW debris is still unknown until sensed.
  BASELINE    brief carries only "find a FIRE" — no location, no beacons, no hazards;
              the Executor must explore on its own (same frontier engine the Writer uses).

Every value is read from the simulation after the run. Nothing is estimated, and
nothing in the Living-Map run knows the new debris in advance.

    python -m simulation.compare_memory --seeds 6
    python -m simulation.compare_memory --seeds 6 --placement random
[SIMULATED]
"""
from __future__ import annotations

from statistics import mean
from typing import Dict, List, Optional

from common.enums import EventType, MemoryState
from common.mission import BlockagePolicy
from simulation.phase1 import (
    all_idle, build_scenario, emit_json, explore_phase, parse_args, pick_debris, run_ticks, run_until, section, table,
)
from simulation.system import LiveMapSystem

FLEET = {"FIRE": 1, "DEBRIS": 1}
METRICS = [
    ("outbound_cells", "travel distance to the target (cells)"),
    ("optimal_cells", "true shortest path, simulator-side (cells)"),
    ("ticks_to_site", "mission time to reach the target (ticks)"),
    ("mission_ticks", "mission time incl. task + return (ticks)"),
    ("outbound_revisits", "wrong turns: cells re-entered on the way out"),
    ("cells_discovered", "cells the Executor had to explore itself"),
    ("replans", "route re-plans (a planned path became invalid)"),
    ("search_targets", "frontier re-selections (baseline explores; Living-Map run does not search)"),
    ("failed_approaches", "failed approaches (bumped into debris)"),
    ("repeated_approaches", "repeated approaches to the same blockage"),
    ("events_recovered", "events recovered (target resolved/verified)"),
]


def one(seed: int, inherit: bool, placement: str, n_debris: int) -> Dict[str, object]:
    sc = build_scenario(f"cmp-{'memory' if inherit else 'baseline'}-seed{seed}", seed, [EventType.FIRE], fleet=dict(FLEET),
                        dispatch_threshold=0.0, inherit_memory=inherit, brief_policy=BlockagePolicy(max_wait_ticks=60, max_retries=0))
    sys_ = LiveMapSystem(sc)
    explore_phase(sys_)
    target = next(iter(sc.events))
    cell = (target.row, target.col)
    debris = pick_debris(sys_.world, seed, placement, sys_.world.entry, cell, n_debris)   # same rule + seed => same cells in both modes
    for c in debris:
        sys_.add_debris(c, "new-after-writer")
    optimal = sys_.world.shortest_path_len(sys_.world.entry, cell)
    t0 = sys_.tick
    sys_.auto_dispatch = True
    run_until(sys_, lambda s: s.tick > t0 + 30 and all_idle(s) and not s.command_post.ranked_missions(auto=True))
    run_ticks(sys_, 40)
    ex = next((e for e in sys_.executors if e.capabilities[0].value == "FIRE" and e.stats["missions"] > 0), None)
    st = ex.stats if ex else {}
    rec = next((r for r in sys_.living_map.all() if r.event_type == "FIRE"), None)
    recovered = int(rec is not None and rec.state in (MemoryState.CLEARED, MemoryState.VERIFIED, MemoryState.ACTIVE, MemoryState.ESCALATED))
    return {"debris": debris, "optimal_cells": optimal, "events_recovered": recovered,
            "outbound_cells": st.get("outbound_cells"), "ticks_to_site": st.get("ticks_to_site"),
            "mission_ticks": st.get("mission_ticks"), "outbound_revisits": st.get("outbound_revisits"),
            "cells_discovered": st.get("cells_discovered"), "replans": st.get("replans"),
            "search_targets": st.get("search_targets"), "failed_approaches": st.get("failed_approaches"), "repeated_approaches": st.get("repeated_approaches"),
            "reached_target": "outbound_cells" in st}


def run(seeds: int = 6, start_seed: int = 42, placement: str = "on_route", n_debris: int = 1) -> Dict[str, object]:
    rows: List[Dict[str, object]] = []
    for k in range(seeds):
        sd = start_seed + k
        rows.append({"seed": sd, "memory": one(sd, True, placement, n_debris), "baseline": one(sd, False, placement, n_debris)})
    return {"placement": placement, "runs": rows}


def report(res: Dict[str, object]) -> None:
    runs = res["runs"]
    section(f"WITH vs WITHOUT LIVING-MAP MEMORY — {len(runs)} seeds, new debris '{res['placement']}' (identical per seed in both modes)")
    for key, label in METRICS:
        rows = []
        for r in runs:
            m, b = r["memory"].get(key), r["baseline"].get(key)
            rows.append((r["seed"], "-" if m is None else m, "-" if b is None else b))
        mv = [r["memory"][key] for r in runs if r["memory"].get(key) is not None]
        bv = [r["baseline"][key] for r in runs if r["baseline"].get(key) is not None]
        print(f"\n  {label}")
        table(["seed", "Living Map", "Baseline"], rows, [6, 12, 12])
        if mv and bv:
            print(f"  mean over runs that have a value:  Living Map {mean(mv):.1f}   Baseline {mean(bv):.1f}")
    ok_m = sum(1 for r in runs if r["memory"]["reached_target"])
    ok_b = sum(1 for r in runs if r["baseline"]["reached_target"])
    print(f"\n  runs where the Executor reached the target: Living Map {ok_m}/{len(runs)}   Baseline {ok_b}/{len(runs)}")
    print("  NOTE on 're-plans' / 'frontier re-selections': the two modes plan differently (the Living-Map Executor plans\n"
          "  optimistically through unexplored cells toward a known target and re-plans when a wall appears; the baseline\n"
          "  explores frontiers through known cells). Those two rows describe HOW each mode navigates, not which is better.")
    print("  (Values come straight from the simulation. Small N and one arena: this illustrates the effect of inherited memory,\n"
          "   it is not a statistical claim. If a row shows no advantage, that is the real result for that seed.)")


def main(argv=None) -> Dict[str, object]:
    args = parse_args("With vs without inherited Living-Map memory", lambda ap: (
        ap.add_argument("--seeds", type=int, default=6, help="number of consecutive seeds (starting at --seed)"),
        ap.add_argument("--placement", choices=["on_route", "chokepoint", "seal", "random"], default="on_route"),
        ap.add_argument("--n", type=int, default=1, help="number of new debris cells")), argv)
    res = run(args.seeds, args.seed, args.placement, args.n)
    if args.json:
        emit_json(res)
    else:
        report(res)
    return res


if __name__ == "__main__":
    main()
