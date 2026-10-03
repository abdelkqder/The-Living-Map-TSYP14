"""
simulation/global_demo.py — Simulation A: GLOBAL LIVING MAP (the main showcase)
===============================================================================
The whole architecture, end to end, in one headless run:

  Writer exploration -> events -> memory creation -> multi-hop beacons -> ONA ->
  coordinate transformation -> Command Post -> priority analysis -> Executor ->
  mission -> verification -> memory update -> return -> next decision

    python -m simulation.global_demo --seed 42
    python -m simulation.global_demo --seed 5      # deeper maze position: a 7-hop beacon chain
    python -m simulation.global_demo --seed 42 --writer-fail 1500   # the Writer dies mid-exploration

There is NO Robot <-> Command Post link: everything the Command Post learns crossed
the beacon chain and the ONA, and every brief it sends is carried by the ONA mailbox.
Environment (challenge choice): Mines / Tunnels — a GPS-denied maze of corridors. [SIMULATED]
"""
from __future__ import annotations

from typing import Dict, Optional

from common.coordinates import robot_to_gps
from common.enums import EventType
from common.protocol import ONA_NODE_ID, node_label
from executor_robot.capabilities import DEFAULT_FLEET
from simulation.phase1 import (
    CHALLENGE_TYPES, all_idle, ascii_map, build_scenario, emit_json, parse_args, run_ticks, run_until, section, table,
)
from simulation.system import LiveMapSystem


def run(seed: int = 42, writer_fail: Optional[int] = None, jitter: float = 0.4) -> Dict[str, object]:
    sc = build_scenario(f"global-seed{seed}", seed, CHALLENGE_TYPES, fleet=dict(DEFAULT_FLEET), jitter=jitter,
                        writer_fail_tick=writer_fail, dispatch_threshold=0.0)
    sys_ = LiveMapSystem(sc)
    w = sys_.deploy_writer()
    run_until(sys_, lambda s: w.dead)
    run_ticks(sys_, 400)
    t_writer_done = sys_.tick
    snapshot_records = [(r.memory_id, r.event_type, r.state.value, r.version) for r in sys_.living_map.all()]
    net_after_writer = sys_.network_report()
    ona_rows = [(u.msg, u.ext, u.hop_count, u.origin) for u in sys_.ona.recent]

    sys_.auto_dispatch = True
    run_until(sys_, lambda s: s.tick > t_writer_done + 30 and all_idle(s) and not s.command_post.ranked_missions(auto=True))
    run_ticks(sys_, 40)
    return {"seed": seed, "system": sys_, "writer": w, "t_writer_done": t_writer_done,
            "records_after_writer": snapshot_records, "network": net_after_writer, "ona_recent": ona_rows}


def report(res: Dict[str, object]) -> Dict[str, object]:
    sys_: LiveMapSystem = res["system"]
    w = res["writer"]
    cp, ona = sys_.command_post, sys_.ona
    sc = sys_.scenario
    section(f"SIMULATION A — GLOBAL LIVING MAP  (seed {res['seed']}, Mines/Tunnels-style maze)")
    print("  Chain: Writer -> beacon memory/relay network -> ONA -> Command Post -> Executor fleet -> updated memory")

    section("1. WRITER — autonomous exploration, localisation by odometry, selective preservation")
    print(ascii_map(sys_, route=w.route, discovered=w.discovered, title="  Writer's discovered map (',' never explored, '*' its route, B beacon):"))
    print(f"\n  state {w.state.value} after {w.tick} ticks ({sys_.clock.ticks_to_seconds(w.tick):.0f} s simulated); "
          f"{w.stats['frontiers_selected']} frontier targets, {w.stats['dead_ends']} dead ends, "
          f"{len(w.discovered.known_free_cells())}/{sys_.total_free_cells} cells explored")
    print(f"  final dead-reckoned pose: x={w.pose.x:.2f} m  y={w.pose.y:.2f} m  heading={w.pose.heading:.2f} rad "
          f"(odometry noise: {sc.odometry_noise}; ideal by default)")
    for line in w.log[:18]:
        print("  " + line)

    section("2. BEACON NETWORK — multi-hop relay toward the ONA (radio range & walls limit every link)")
    rows = []
    for bid, node in sorted(sys_.beacons.nodes.items()):
        cell = sys_.beacons.drop_cells[bid]
        rows.append((f"B{bid}", cell, f"({node.x:.1f},{node.y:.1f})", node.hops if node.connected else "-",
                     node.parent if node.parent is not None else "-", len(node.records), node.comm_state(),
                     node.stats.forwarded, node.stats.originated))
    table(["beacon", "cell", "local m", "hops", "parent", "records", "comm state", "fwd", "orig"], rows)
    n = res["network"]
    print(f"\n  after the Writer: {n['beacons_connected_to_ona']}/{n['beacons']} beacons connected to the ONA, max {n['max_hops']} hops; "
          f"frames tx {n['frames_transmitted']} / delivered {n['frames_delivered']}; forwarded by beacons {n['forwarded_by_beacons']}; "
          f"retries {n['retries']}; duplicates suppressed {n['duplicates_suppressed']}; CRC rejects at ONA {n['ona_crc_rejected']}")

    section("3. ONA — validate, translate local -> GPS, carry (it never plans a mission)")
    lat0, lon0 = sc.ref_gps
    print(f"  reference point: lat {lat0}, lon {lon0}, local-frame heading {sc.ref_heading} deg  [ASSUMPTION — not field-measured]")
    rows = []
    for msg, ext, hops, origin in res["ona_recent"]:
        rows.append((f"B{msg.beacon_id}", f"#{ext.memory_id:04X}", msg.event_type, f"({msg.x_local:.2f},{msg.y_local:.2f})",
                     f"{msg.gps_lat:.6f},{msg.gps_lon:.6f}", node_label(ext.source), ext.state.value, f"v{ext.version}",
                     f"{msg.initial_confidence}%", hops))
    table(["beacon", "memory", "event", "local (m)", "GPS (lat,lon)", "source", "status", "ver", "conf", "relays"], rows)
    print(f"\n  ONA frame stats: {ona.mesh_stats}")

    section("4. COMMAND POST — Living Map, priorities, executor selection")
    rows = []
    for d in cp.decisions:
        c, sel = d["components"], d["selection"]
        rows.append((f"t={d['tick']}", d["mission_id"], f"{d['event_type']} #{d['memory_id']:04X}", d["priority"],
                     f"base {c['base']} x{c['class_weight']} x{c['status_factor']} -{c['travel_penalty']} (age {c['age_s']}s, conf {c['live_confidence']})",
                     d["executor"], sel["why"]))
    table(["when", "mission", "memory", "prio", "why this priority", "executor", "why this executor"], rows, [8, 7, 14, 6, 62, 8, 42])

    section("5. EXECUTORS — inherited memory, task, memory update, return")
    for e in sys_.executors:
        if e.stats["missions"] == 0:
            continue
        print(f"\n  {e.executor_id} [{e.capabilities[0].value}] final state {e.state.value}")
        for ev in e.events:
            if ev["kind"] in ("mission_loaded", "on_site", "work_done", "not_found", "verified", "memory_write", "returning", "home", "discrepancy"):
                detail = ", ".join(f"{k}={v}" for k, v in ev.items() if k not in ("tick", "kind"))
                print(f"    t={ev['tick']:>5}  {ev['kind']:<14} {detail}")

    section("6. LIVING MAP — before vs after the Executors")
    before = {m: (t, s, v) for m, t, s, v in res["records_after_writer"]}
    rows = []
    for r in sys_.living_map.all():
        b = before.get(r.memory_id)
        rows.append((f"#{r.memory_id:04X}", r.event_type, f"{b[1]} v{b[2]}" if b else "-", f"{r.state.value} v{r.version}",
                     r.source, f"{r.age_seconds:.0f}s", f"{r.confidence_pct}%", f"({r.gps_lat:.6f},{r.gps_lon:.6f})"))
    table(["memory", "type", "after Writer", "after Executors", "last by", "age", "conf", "GPS"], rows, [8, 8, 18, 20, 7, 6, 5, 22])

    section("7. ARCHITECTURE CHECK")
    cp_in = cp.stats
    print(f"  Command Post inputs (all via ONA): memory {cp_in['memory_in']}, heartbeats {cp_in['heartbeats_in']}, status {cp_in['status_in']}")
    print(f"  Command Post outputs: {cp_in['briefs_sent']} brief(s) -> ONA mailbox (delivered: {ona.briefs_delivered})")
    print("  Robots hold a RobotLink (beacons + ONA only); no transport to the Command Post exists (see tests/test_architecture.py).")
    out = {"records": len(sys_.living_map), "missions_completed": sum(1 for m in sys_.mission_dispatcher.all_missions() if m.status.value == "COMPLETED"),
           "writer_state": w.state.value, "end_tick": sys_.tick,
           "executors_available_at_end": sum(1 for e in cp.fleet.all() if e.category == "available")}
    print("\n  SUMMARY:", out)
    return out


def main(argv=None) -> Dict[str, object]:
    args = parse_args("Simulation A — global Living Map", lambda ap: (
        ap.add_argument("--writer-fail", type=int, default=None, metavar="TICK", help="the Writer fails at this tick (knowledge must survive it)"),
        ap.add_argument("--jitter", type=float, default=0.4)), argv)
    res = run(args.seed, args.writer_fail, args.jitter)
    if args.json:
        sys_ = res["system"]
        emit_json({"network": res["network"], "decisions": sys_.command_post.decisions,
                   "records": [r.to_dict() for r in sys_.living_map.all()]})
    else:
        res["summary"] = report(res)
    if args.png:
        from simulation.snapshot import save_png
        save_png(res["system"], args.png, writer=res["writer"])
    return res


if __name__ == "__main__":
    main()
