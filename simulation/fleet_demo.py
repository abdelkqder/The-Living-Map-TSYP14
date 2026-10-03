"""
simulation/fleet_demo.py — Simulation C: EXECUTOR FLEET + MISSION PLANNING
==========================================================================
Starts from an already populated Living Map (a Writer run fills it through the
real beacon -> ONA -> Command Post path), then lets the Command Post run the
fleet: priority calculation, capability matching, assignment, navigation, task,
memory update, return, fleet status, next assignment.

    python -m simulation.fleet_demo --seed 42
    python -m simulation.fleet_demo --seed 42 --fault E02      # E02 breaks down mid-mission

The simulated fleet (configurable) is FIRE E01-E03 · VICTIM E04-E06 · GAS E07-E08 ·
DEBRIS E09-E10. Only the Writer + Executor + ONA are planned as physical hardware for
Phase 1 — the rest of the fleet exists to exercise fleet-level mission management.
[SIMULATED]
"""
from __future__ import annotations

from typing import Dict, List, Optional

from common.enums import EventType, ExecutorState
from executor_robot.capabilities import DEFAULT_FLEET
from simulation.phase1 import (
    all_idle, build_scenario, emit_json, explore_phase, parse_args, run_ticks, run_until, section, table,
)
from simulation.system import LiveMapSystem

# 4 fires + 1 gas + 1 victim: with only 3 FIRE executors, the 4th fire must WAIT and is
# assigned when an executor returns -> demonstrates "complete a mission, then get another".
FLEET_EVENT_TYPES = [EventType.FIRE, EventType.FIRE, EventType.FIRE, EventType.FIRE, EventType.GAS,
                     EventType.VICTIM_PRESENCE]


def run(seed: int = 42, fault: Optional[str] = None, fleet: Optional[Dict[str, int]] = None) -> Dict[str, object]:
    overrides = {}
    if fault:
        overrides[fault] = dict(fault_at_tick=60)         # breaks down ~6 s into its first mission
    sc = build_scenario(f"fleet-seed{seed}", seed, FLEET_EVENT_TYPES, fleet=fleet or dict(DEFAULT_FLEET),
                        dispatch_threshold=0.0, executor_overrides=overrides)
    sys_ = LiveMapSystem(sc)
    w = explore_phase(sys_)
    populated = len(sys_.living_map)
    sys_.auto_dispatch = True
    t0 = sys_.tick
    run_until(sys_, lambda s: s.tick > t0 + 50 and all_idle(s) and not s.command_post.ranked_missions(auto=True))
    run_ticks(sys_, 40)                       # let the final status frames reach the Command Post
    return {"seed": seed, "fault": fault, "system": sys_, "writer": w, "records_before_fleet": populated,
            "fleet_start_tick": t0}


def report(res: Dict[str, object]) -> Dict[str, object]:
    sys_: LiveMapSystem = res["system"]
    cp = sys_.command_post
    section(f"SIMULATION C — EXECUTOR FLEET + MISSION PLANNING  (seed {res['seed']}"
            + (f", fault injected into {res['fault']}" if res["fault"] else "") + ")")
    print(f"  Living Map populated by the Writer before the fleet started: {res['records_before_fleet']} record(s)")
    print(f"  Fleet: " + ", ".join(f"{e.executor_id}[{e.capabilities[0].value}]" for e in cp.fleet.all()))

    section("COMMAND POST DECISIONS — why each mission went to that executor")
    rows = []
    for d in cp.decisions:
        c, sel = d["components"], d["selection"]
        rows.append((f"t={d['tick']}", d["mission_id"], f"{d['event_type']} #{d['memory_id']:04X}", d["state"],
                     d["priority"], f"base {c['base']} x{c['class_weight']} x{c['status_factor']} -{c['travel_penalty']}",
                     d["executor"], sel["why"]))
    table(["when", "mission", "memory", "state", "prio", "priority components", "executor", "selection reason"], rows,
          [8, 7, 17, 10, 6, 36, 8, 44])

    section("MISSION HISTORY (final)")
    rows = [(m.mission_id, m.event_type, f"#{m.beacon_id:04X}", m.status.value, m.assigned_executor_id or "-",
             m.dispatched_tick if m.dispatched_tick is not None else "-", m.completed_tick if m.completed_tick is not None else "-")
            for m in sorted(sys_.mission_dispatcher.all_missions(), key=lambda m: m.mission_id)]
    table(["mission", "type", "memory", "status", "executor", "dispatched", "closed"], rows)

    section("EXECUTOR TIMELINES (state changes / key events)")
    for e in sys_.executors:
        keys = [ev for ev in e.events if ev["kind"] in ("mission_loaded", "on_site", "work_done", "not_found", "verified",
                                                         "memory_write", "returning", "home", "fault", "replan", "discrepancy")]
        if not keys:
            continue
        print(f"\n  {e.executor_id} [{e.capabilities[0].value}] — final state {e.state.value}, battery {int(e.battery_pct)}%")
        for ev in keys[:14]:
            detail = ", ".join(f"{k}={v}" for k, v in ev.items() if k not in ("tick", "kind"))
            print(f"    t={ev['tick']:>5}  {ev['kind']:<14} {detail}")

    section("LIVING MAP AFTER THE FLEET RAN (memory updated by executors)")
    rows = [(f"#{r.memory_id:04X}", r.event_type, r.state.value, f"v{r.version}", r.source, f"{r.age_seconds:.0f}s",
             f"{r.confidence_pct}%", " | ".join(r.history[-2:])) for r in sys_.living_map.all()]
    table(["memory", "type", "state", "ver", "last by", "age", "conf", "latest history"], rows, [8, 16, 11, 4, 7, 6, 5, 80])

    section("FLEET STATUS (as the Command Post knows it)")
    rows = [(e.executor_id, e.capabilities[0].value, e.state.value, e.category, f"{e.battery_pct}%", e.missions_done)
            for e in cp.fleet.all()]
    table(["executor", "capability", "reported state", "category", "battery", "missions done"], rows)

    reassigned = [e.executor_id for e in cp.fleet.all() if e.missions_done >= 1 and sum(1 for d in cp.decisions if d["executor"] == e.executor_id) >= 2]
    unavailable = [e.executor_id for e in cp.fleet.all() if e.category == "unavailable"]
    unassigned_after = {}
    for e in cp.fleet.all():
        if e.category == "unavailable":
            fail_t = next((ev["tick"] for ev in sys_.executors[[x.executor_id for x in sys_.executors].index(e.executor_id)].events
                           if ev["kind"] == "fault"), None)
            later = [d for d in cp.decisions if d["executor"] == e.executor_id and fail_t is not None and d["tick"] > fail_t]
            unassigned_after[e.executor_id] = len(later)
    out = {"decisions": len(cp.decisions), "missions_completed": sum(1 for m in sys_.mission_dispatcher.all_missions() if m.status.value == "COMPLETED"),
           "executors_with_2plus_missions": reassigned, "unavailable_executors": unavailable,
           "assignments_to_unavailable_after_failure": unassigned_after, "end_tick": sys_.tick}
    print("\n  SUMMARY:", out)
    return out


def main(argv=None) -> Dict[str, object]:
    args = parse_args("Simulation C — executor fleet + mission planning",
                      lambda ap: ap.add_argument("--fault", default=None, metavar="EID", help="inject a mid-mission breakdown into this executor (e.g. E02)"), argv)
    res = run(args.seed, args.fault)
    if args.json:
        sys_ = res["system"]
        emit_json({"decisions": sys_.command_post.decisions,
                   "fleet": [(e.executor_id, e.state.value, e.missions_done) for e in sys_.command_post.fleet.all()]})
    else:
        res["summary"] = report(res)
    if args.png:
        from simulation.snapshot import save_png
        save_png(res["system"], args.png)
    return res


if __name__ == "__main__":
    main()
