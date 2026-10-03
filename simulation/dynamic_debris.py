"""
simulation/dynamic_debris.py — Simulation D: DYNAMIC DEBRIS
===========================================================
    Writer explores and leaves  ->  the environment CHANGES (new debris appears at a
    seeded random position the Writer never saw)  ->  an Executor arrives using the
    INHERITED map  ->  finds the discrepancy  ->  re-plans  ->  preserves the new
    information  ->  the Command Post updates the Living Map  ->  another decision.

    python -m simulation.dynamic_debris --seed 42                       # debris on the Executor's route
    python -m simulation.dynamic_debris --seed 42 --placement seal       # debris that makes the target unreachable
    python -m simulation.dynamic_debris --seed 42 --placement random     # anywhere (may not matter)
    python -m simulation.dynamic_debris --seed 42 --existing 2           # + debris present when the Writer entered
    python -m simulation.dynamic_debris --seed 42 --compare-seeds 6      # different seeds, different outcomes

Policy knobs (all configurable, none hard-coded): --max-wait --retries --no-request-debris --hold
The Executor does NOT know where the new debris is: it only knows what was preserved
before. [SIMULATED]
"""
from __future__ import annotations

from typing import Dict, List, Optional

from common.enums import EventType, ExecutorState
from common.mission import BlockagePolicy
from simulation.phase1 import (
    all_idle, build_scenario, emit_json, explore_phase, parse_args, pick_debris, run_ticks, run_until, section, table,
)
from simulation.system import LiveMapSystem
from simulation.writer_demo import pick_existing_debris

DEFAULT_DYN_FLEET = {"FIRE": 2, "DEBRIS": 1}


def classify(sys_: LiveMapSystem, target_exec_events: List[dict], debris_exec_used: bool) -> str:
    kinds = {e["kind"] for e in target_exec_events}
    missions = sys_.mission_dispatcher.all_missions()
    fire = [m for m in missions if m.event_type == "FIRE"]
    done = fire and all(m.status.value == "COMPLETED" for m in fire)
    if "target_unreachable" in kinds and debris_exec_used and done:
        return "BLOCKED -> Debris Executor cleared it -> mission completed on the second attempt"
    if "target_unreachable" in kinds and not done:
        return "BLOCKED -> mission left pending (no clearance available / requested)"
    if "discrepancy" in kinds and done:
        return "RE-PLANNED around the blockage -> mission completed"
    if done:
        return "debris never affected this mission (not on the Executor's path)"
    return "mission not completed"


def run(seed: int = 42, *, placement: str = "on_route", n_debris: int = 1, existing: int = 0,
        max_wait: int = 120, retries: int = 1, request_debris: bool = True, hold: bool = False,
        fleet: Optional[Dict[str, int]] = None, settle: int = 60) -> Dict[str, object]:
    policy = BlockagePolicy(max_wait_ticks=max_wait, max_retries=retries, request_debris_executor=request_debris,
                            return_if_unreachable=not hold)
    sc = build_scenario(f"dynamic-debris-seed{seed}", seed, [EventType.FIRE], fleet=fleet or dict(DEFAULT_DYN_FLEET),
                        dispatch_threshold=0.0, brief_policy=policy,
                        debris_existing=pick_existing_debris(seed, existing))
    sys_ = LiveMapSystem(sc)
    w = explore_phase(sys_)                                   # 1. Writer explores, preserves, leaves (returns home)
    target = next(iter(sc.events))
    target_cell = (target.row, target.col)
    before = {r.memory_id: (r.event_type, r.state.value, r.version) for r in sys_.living_map.all()}

    # 2. the environment changes AFTER the Writer left (the Executor is not told)
    cells = pick_debris(sys_.world, seed, placement, sys_.world.entry, target_cell, n_debris)
    for c in cells:
        sys_.add_debris(c, "new-after-writer")
    t_change = sys_.tick
    reachable_after = sys_.world.shortest_path_len(sys_.world.entry, target_cell) is not None

    # 3. the Command Post dispatches; everything else follows from the architecture
    sys_.auto_dispatch = True
    run_until(sys_, lambda s: s.tick > t_change + 30 and all_idle(s) and not s.command_post.ranked_missions(auto=True))
    run_ticks(sys_, settle)

    fire_execs = [e for e in sys_.executors if e.capabilities[0].value == "FIRE"]
    debris_execs = [e for e in sys_.executors if e.capabilities[0].value == "DEBRIS"]
    ev_all = [ev for e in fire_execs for ev in e.events]
    used = any(e.stats["missions"] > 0 for e in debris_execs)
    res = {
        "seed": seed, "placement": placement, "new_debris": cells, "existing_debris": list(sc.debris_existing),
        "target": target_cell, "t_change": t_change, "target_reachable_after_change": reachable_after, "system": sys_, "writer": w, "memory_before": before,
        "outcome": classify(sys_, ev_all, used), "debris_executor_used": used,
        "fire_replans": sum(e.stats["replans_blockage"] for e in fire_execs),
        "discrepancies": sum(e.stats["discrepancies"] for e in fire_execs),
        "blockages_recorded_by_executors": sum(e.stats["blockages_recorded"] for e in sys_.executors),
        "end_tick": sys_.tick,
    }
    return res


def report(res: Dict[str, object]) -> None:
    sys_: LiveMapSystem = res["system"]
    cp = sys_.command_post
    section(f"SIMULATION D — DYNAMIC DEBRIS  (seed {res['seed']}, placement '{res['placement']}')")
    print(f"  Event target (hidden from the Executor except via memory): FIRE at map cell {res['target']}")
    print(f"  Debris present when the Writer entered : {res['existing_debris'] or 'none'}")
    print(f"  NEW debris appeared after the Writer left (t={res['t_change']}): {res['new_debris']}"
          f"   -> target still reachable from the entry (simulator check): {res['target_reachable_after_change']}")
    print("\n" + __import__("simulation.phase1", fromlist=["ascii_map"]).ascii_map(
        sys_, discovered=None, title="  Final arena ('x' debris still present, 'B' beacons, '~' resolved event):"))

    section("MEMORY BEFORE THE ENVIRONMENT CHANGED (what the Executor inherits)")
    table(["memory", "type", "state", "version"], [(f"#{k:04X}", *v) for k, v in res["memory_before"].items()])

    section("EXECUTOR EVENTS (discrepancy detection, re-planning, memory preservation)")
    for e in sys_.executors:
        keys = [ev for ev in e.events if ev["kind"] in (
            "mission_loaded", "discrepancy", "replan", "new_blockage", "memory_write", "target_unreachable",
            "route_reopened", "retry_after_wait", "work_done", "verified", "returning", "home", "holding_position")]
        keys = [k for k in keys if k["kind"] != "replan" or "BLOCKAGE" in str(k.get("cause"))]
        if not any(k["kind"] != "mission_loaded" for k in keys):
            continue
        print(f"\n  {e.executor_id} [{e.capabilities[0].value}]  final state {e.state.value}")
        for ev in keys:
            detail = ", ".join(f"{k}={v}" for k, v in ev.items() if k not in ("tick", "kind"))
            print(f"    t={ev['tick']:>5}  {ev['kind']:<18} {detail}")

    section("COMMAND POST LOG (decisions after the change)")
    for line in cp.log:
        t = int(line.split("]")[0].strip("[t="))
        if t >= res["t_change"]:
            print("  " + line)

    section("LIVING MAP AFTER (memory shared with the Command Post)")
    table(["memory", "type", "state", "ver", "passage", "by", "location (local m)", "history (last)"],
          [(f"#{r.memory_id:04X}", r.event_type, r.state.value, f"v{r.version}", r.passage_state.value if r.passage_state else "-",
            r.source, f"({r.x_local:.1f},{r.y_local:.1f})", r.history[-1][:60]) for r in sys_.living_map.all()])
    section("OUTCOME")
    print(f"  {res['outcome']}")
    print(f"  discrepancies detected: {res['discrepancies']}  ·  re-plans caused by a blockage: {res['fire_replans']}  ·  "
          f"blockages recorded by executors: {res['blockages_recorded_by_executors']}  ·  Debris Executor used: {res['debris_executor_used']}")


def compare_seeds(seed: int, n: int, placement: str, **kw) -> List[dict]:
    rows, out = [], []
    for k in range(n):
        r = run(seed + k, placement=placement, **kw)
        rows.append((seed + k, r["new_debris"], r["outcome"][:78], r["fire_replans"], r["debris_executor_used"], r["end_tick"] - r["t_change"]))
        out.append({kk: vv for kk, vv in r.items() if kk not in ("system", "writer")})
    section(f"SAME SCENARIO, {n} DIFFERENT SEEDS (placement '{placement}')")
    table(["seed", "new debris cell", "response", "replans", "debris exec", "ticks after change"], rows, [5, 16, 78, 7, 11, 18])
    return out


def main(argv=None) -> Dict[str, object]:
    def extra(ap):
        ap.add_argument("--placement", choices=["on_route", "chokepoint", "seal", "random"], default="on_route")
        ap.add_argument("--n", type=int, default=1, help="how many new debris cells")
        ap.add_argument("--existing", type=int, default=0, help="debris already present when the Writer enters")
        ap.add_argument("--max-wait", type=int, default=120, help="ticks the Executor lingers near an obstruction")
        ap.add_argument("--retries", type=int, default=1, help="extra wait+retry cycles before giving up")
        ap.add_argument("--no-request-debris", action="store_true", help="do NOT ask for a Debris Executor")
        ap.add_argument("--hold", action="store_true", help="stay on site instead of returning when unreachable")
        ap.add_argument("--compare-seeds", type=int, default=0, metavar="N")
    args = parse_args("Simulation D — dynamic debris", extra, argv)
    kw = dict(placement=args.placement, n_debris=args.n, existing=args.existing, max_wait=args.max_wait,
              retries=args.retries, request_debris=not args.no_request_debris, hold=args.hold)
    res = run(args.seed, **kw)
    if args.json:
        emit_json({k: v for k, v in res.items() if k not in ("system", "writer")})
    else:
        report(res)
        if args.compare_seeds:
            compare_seeds(args.seed, args.compare_seeds, args.placement, **{k: v for k, v in kw.items() if k != "placement"})
    if args.png:
        from simulation.snapshot import save_png
        save_png(res["system"], args.png)
    return res


if __name__ == "__main__":
    main()
