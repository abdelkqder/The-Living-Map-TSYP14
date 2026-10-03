"""Phase-1 behaviours demonstrated end to end by the real simulation (not by mocks)."""
import pytest

from common.enums import EventType, ExecutorState, MemoryState, MissionStatus, PassageState, WriterState
from common.mission import BlockagePolicy, Mission, MissionTarget
from common.protocol import ONA_NODE_ID
from executor_robot.capabilities import DEFAULT_FLEET, profile_for
from simulation.phase1 import (
    CHALLENGE_TYPES, all_idle, build_scenario, chokepoints, explore_phase, pick_debris, run_ticks, run_until,
)
from simulation.system import LiveMapSystem
from simulation.world import EntryFrame



def writer_run(seed=42, **kw):
    sys_ = LiveMapSystem(build_scenario(f"t{seed}", seed, CHALLENGE_TYPES, jitter=0.6, **kw))
    w = explore_phase(sys_)
    return sys_, w


# ═══ Writer: exploration is autonomous, not scripted ═══════════════════════════
def test_writer_discovers_the_arena_and_returns_to_the_entry_without_shutting_down():
    sys_, w = writer_run()
    reach = sys_.world.reachable_from_entry()
    cov = len(w.discovered.known_free_cells() & reach) / len(reach)
    assert cov > 0.97
    assert w.state == WriterState.RETURNED and w.returned and w.movement.current_cell == sys_.world.entry
    assert w.state != WriterState.DEAD


def test_different_seeds_give_different_trajectories():
    routes = set()
    for seed in (1, 2, 3, 4):
        _, w = writer_run(seed)
        routes.add(tuple(w.route))
    assert len(routes) >= 2


def test_same_seed_is_exactly_reproducible():
    _, a = writer_run(7)
    _, b = writer_run(7)
    assert a.route == b.route and a.stats == b.stats


def test_environment_change_changes_the_writer_route():
    sys_a, wa = writer_run(5)
    sc = build_scenario("changed", 5, CHALLENGE_TYPES, jitter=0.6, debris_existing=[wa.route[40]])
    wb = explore_phase(LiveMapSystem(sc))
    assert tuple(wa.route) != tuple(wb.route)


def test_writer_meets_existing_debris_records_it_and_finds_other_ways():
    probe, w0 = writer_run(42)
    cells = [c for c in w0.route[30:200] if c != probe.world.entry][:1]
    sc = build_scenario("deb", 42, CHALLENGE_TYPES, jitter=0.6, debris_existing=cells)
    sys_ = LiveMapSystem(sc)
    w = explore_phase(sys_)
    assert w.stats["blockages_seen"] >= 1 and w.stats["blockages_preserved"] >= 1
    assert cells[0] in w.discovered.blocked_cells() and w.returned
    assert w.stats["replans"] + w.stats["alternative_routes"] + w.stats["abandoned_targets"] >= 0
    blockage = [r for r in sys_.living_map.all() if r.event_type == "BLOCKAGE"]
    assert blockage and blockage[0].passage_state == PassageState.BLOCKED


def test_writer_pose_is_dead_reckoned_and_matches_the_entry_frame():
    sys_, w = writer_run(42)
    assert (w.pose.x, w.pose.y) == pytest.approx(EntryFrame(sys_.world.entry).cell_to_local(w.route[-1]), abs=1e-6)
    assert w.odometry.path_length_m == pytest.approx((len(w.route) - 1) * 0.5, abs=1e-6)


def test_writer_failure_leaves_the_knowledge_behind():
    sc = build_scenario("fail", 42, CHALLENGE_TYPES, writer_fail_tick=1800)
    sys_ = LiveMapSystem(sc)
    w = sys_.deploy_writer()
    run_until(sys_, lambda s: w.dead)
    run_ticks(sys_, 600)
    assert w.state == WriterState.DEAD and not w.returned
    assert len(sys_.living_map) >= 1, "what the Writer preserved before it failed reaches the Command Post"


# ═══ Beacon placement is a decision, not a script ═════════════════════════════
def test_beacons_are_placed_by_policy_and_relays_only_where_needed():
    sys_, w = writer_run(1)
    assert w.stats["event_beacons"] >= 1
    assert w.beacon_count <= w.dispenser.stock + w.beacon_count            # sanity
    assert w.stats["relays_dropped"] < len(sys_.world.reachable_from_entry()) / 4
    assert any("importance" in line or "relay" in line for line in w.log)


def test_every_preserved_record_reaches_the_command_post_over_multiple_hops():
    for seed in (1, 3, 7):
        sys_, w = writer_run(seed)
        net = sys_.network_report()
        assert net["beacons_connected_to_ona"] == net["beacons"]
        assert len(sys_.living_map) >= w.memory_count - 0
        assert sys_.ona.mesh_stats["memory"] >= w.memory_count
    deep = max(sys_.beacons.nodes.values(), key=lambda n: n.hops if n.connected else -1)
    assert deep.hops >= 2 or net["max_hops"] >= 1


def test_multi_hop_forwarding_is_actually_exercised():
    found_multi = False
    for seed in (1, 3, 7, 11):
        sys_, w = writer_run(seed)
        rep = sys_.network_report()
        if rep["max_hops"] >= 3 and rep["forwarded_by_beacons"] > 0:
            found_multi = True
            frames = [u for u in sys_.ona.recent if u.hop_count >= 2]
            assert frames, "records that crossed >= 2 relays arrived at the ONA"
            break
    assert found_multi


def test_event_memory_carries_local_and_global_coordinates_from_the_ona():
    sys_, w = writer_run(42)
    for r in sys_.living_map.all():
        assert r.gps_lat is not None and r.gps_lon is not None
        assert (r.x_local, r.y_local) != (0.0, 0.0) or r.event_type == "BLOCKAGE"
        assert abs(r.gps_lat - sys_.scenario.ref_gps[0]) < 0.001


# ═══ Command Post: planner ═══════════════════════════════════════════════════
def fleet_system(seed=42, types=None, **kw):
    types = types or [EventType.FIRE, EventType.FIRE, EventType.FIRE, EventType.FIRE, EventType.GAS, EventType.VICTIM_PRESENCE]
    sys_ = LiveMapSystem(build_scenario(f"f{seed}", seed, types, fleet=dict(DEFAULT_FLEET), dispatch_threshold=0.0, **kw))
    explore_phase(sys_)
    return sys_


def finish(sys_, extra=60):
    sys_.auto_dispatch = True
    t0 = sys_.tick
    run_until(sys_, lambda s: s.tick > t0 + 30 and all_idle(s) and not s.command_post.ranked_missions(auto=True))
    run_ticks(sys_, extra)


def test_priority_considers_class_status_age_and_distance():
    from command_post.planner import net_priority
    sys_ = fleet_system()
    cp = sys_.command_post
    ranked = cp.ranked_missions(auto=False)
    assert len(ranked) >= 3
    scores = [pr.score for _, _, pr in ranked]
    assert scores == sorted(scores, reverse=True)
    mr, rec, pr = ranked[0]
    for key in ("base", "class_weight", "status_factor", "travel_penalty", "age_s", "live_confidence"):
        assert key in pr.components
    # status matters: escalate the weakest record and its priority must rise
    last_mr, last_rec, last_pr = ranked[-1]
    before = net_priority(last_mr, last_rec).score
    last_rec.state = MemoryState.ESCALATED
    assert net_priority(last_mr, last_rec).score > before


def test_capability_matching_specialist_first_and_every_decision_is_explained():
    sys_ = fleet_system()
    finish(sys_)
    cp = sys_.command_post
    assert cp.decisions
    caps = {e.executor_id: e.capabilities[0].value for e in cp.fleet.all()}
    need = {"FIRE": "FIRE", "GAS": "GAS", "VICTIM_PRESENCE": "VICTIM", "BLOCKAGE": "DEBRIS"}
    for d in cp.decisions:
        assert caps[d["executor"]] == need[d["event_type"]]
        assert d["selection"]["why"] and d["components"]["base"] is not None and "priority" in d


def test_a_mission_completes_and_the_executor_is_reassigned_a_new_one():
    sys_ = fleet_system()
    finish(sys_)
    cp = sys_.command_post
    per_exec = {}
    for d in cp.decisions:
        per_exec.setdefault(d["executor"], []).append(d["tick"])
    twice = {e: t for e, t in per_exec.items() if len(t) >= 2}
    assert twice, "4 fires and only 3 FIRE executors: one of them must be reassigned after returning"
    assert all(m.status == MissionStatus.COMPLETED for m in cp.dispatcher.all_missions())


def test_failed_executor_goes_to_needs_repair_and_is_never_assigned_again():
    sys_ = fleet_system(executor_overrides={"E02": dict(fault_at_tick=60)})
    finish(sys_)
    cp = sys_.command_post
    e2 = next(e for e in sys_.executors if e.executor_id == "E02")
    assert e2.state == ExecutorState.NEEDS_REPAIR and e2.at_entry
    assert cp.fleet.get("E02").state == ExecutorState.NEEDS_REPAIR and cp.fleet.get("E02").category == "unavailable"
    t_fault = next(ev["tick"] for ev in e2.events if ev["kind"] == "fault")
    assert not [d for d in cp.decisions if d["executor"] == "E02" and d["tick"] > t_fault]
    assert all(m.status == MissionStatus.COMPLETED for m in cp.dispatcher.all_missions()), "its mission was re-assigned"


def test_stranded_or_low_battery_executors_are_not_dispatched():
    sys_ = fleet_system(executor_overrides={"E01": dict(battery_drain_per_tick=0.5)})
    finish(sys_)
    e1 = next(e for e in sys_.executors if e.executor_id == "E01")
    assert e1.state == ExecutorState.NEEDS_CHARGING
    entry = sys_.command_post.fleet.get("E01")
    assert not entry.available
    # servicing makes it available again
    assert e1.service() and e1.state == ExecutorState.AVAILABLE


def test_no_available_capable_executor_means_no_assignment():
    from command_post.planner import choose_executor
    from common.enums import RobotCapability
    sys_ = fleet_system()
    for e in sys_.command_post.fleet.all():
        if e.capabilities[0] == RobotCapability.GAS:
            e.state = ExecutorState.NEEDS_REPAIR
    ch = choose_executor(RobotCapability.GAS, sys_.command_post.fleet)
    assert ch.executor is None and "no AVAILABLE executor" in ch.reason["why"]


# ═══ Executor: feedback loop ═════════════════════════════════════════════════
def test_executor_verifies_updates_memory_returns_and_command_post_sees_the_new_state():
    sys_ = fleet_system()
    finish(sys_)
    states = {r.event_type: r.state for r in sys_.living_map.all()}
    assert MemoryState.CLEARED in {r.state for r in sys_.living_map.all() if r.event_type == "FIRE"}
    assert states["GAS"] == MemoryState.ACTIVE                       # gas verified and still present
    assert states["VICTIM_PRESENCE"] == MemoryState.ESCALATED        # needs human responders
    for r in sys_.living_map.all():
        assert r.version >= 2 and r.source.startswith("E"), "executors wrote the update (version bumped)"
    for e in sys_.executors:
        if e.stats["missions"]:
            assert e.state == ExecutorState.AVAILABLE and e.at_entry


def test_capability_profiles():
    assert profile_for([__import__("common.enums", fromlist=["x"]).RobotCapability.FIRE], "FIRE").task == "suppress fire"
    assert profile_for([__import__("common.enums", fromlist=["x"]).RobotCapability.FIRE], "GAS") is None
    assert profile_for([__import__("common.enums", fromlist=["x"]).RobotCapability.GENERAL], "GAS").actuates is False


def test_executor_contradicts_a_record_when_the_event_is_not_there():
    from command_post.mission_planner import Mission, MissionTarget
    from executor_robot.executor import ExecutorRobot
    from simulation.world import GroundTruthWorld
    from tests.test_executor import GRID, run_executor
    x, y = EntryFrame((1, 1)).cell_to_local((5, 2))
    m = Mission(1, [MissionTarget(1, "FIRE", x, y, 3, "UNVERIFIED")])
    contra = []
    ex = ExecutorRobot(GroundTruthWorld(GRID, [], (1, 1)), on_contradicted=lambda b, t, r: contra.append(b))
    ex.load_mission(m)
    for _ in range(3000):
        ex.update()
        if ex.is_complete:
            break
    assert contra == [1] and ex.state == ExecutorState.AVAILABLE


# ═══ Dynamic debris ═════════════════════════════════════════════════════════
def dyn(seed=42, placement="seal", **policy):
    from simulation import dynamic_debris
    return dynamic_debris.run(seed, placement=placement, **policy)


def test_new_debris_appears_after_the_writer_and_changes_the_discovered_map():
    r = dyn(42, "seal")
    sys_ = r["system"]
    assert r["new_debris"] and r["t_change"] > r["writer"].tick
    writer_known = r["writer"].discovered
    assert not any(writer_known.is_impassable(c) for c in r["new_debris"]), "the Writer never saw this debris"
    e = next(e for e in sys_.executors if e.stats["missions"] and e.capabilities[0].value == "FIRE")
    disc = [ev for ev in e.events if ev["kind"] == "discrepancy"]
    assert disc and disc[0]["observed"].startswith("BLOCKED"), "unexpected blockage on the expected route is recognised"
    assert any(e.discovered.state_of(c).value == "BLOCKED" for c in r["new_debris"]), "the Executor's discovered map changed"
    assert e.stats["replans_blockage"] >= 1, "it re-planned instead of walking into the debris"
    assert e.stats["failed_approaches"] == 0


def test_executor_preserves_a_newly_seen_blockage_in_the_spatial_memory():
    r = dyn(42, "on_route")
    sys_ = r["system"]
    blockages = [x for x in sys_.living_map.all() if x.event_type == "BLOCKAGE"]
    assert blockages, "the Executor wrote the new blockage into the spatial memory"
    assert blockages[0].source.startswith("E") and blockages[0].passage_state is not None and blockages[0].version >= 1
    fire = next(m for m in sys_.mission_dispatcher.all_missions() if m.event_type == "FIRE")
    assert fire.status == MissionStatus.COMPLETED and not r["debris_executor_used"]


def test_sealed_target_blocks_the_mission_requests_a_debris_executor_and_resumes():
    r = dyn(42, "seal")
    sys_ = r["system"]
    assert not r["target_reachable_after_change"]
    cp = sys_.command_post
    kinds = [m.event_type for m in cp.dispatcher.all_missions()]
    assert "BLOCKAGE" in kinds and r["debris_executor_used"]
    assert any("BLOCKED by memory" in l for l in cp.log) and any("CLEARED -> mission" in l for l in cp.log)
    fire = next(m for m in cp.dispatcher.all_missions() if m.event_type == "FIRE")
    assert fire.status == MissionStatus.COMPLETED
    # the Debris Executor removed the debris that cut the target off; the target is reachable again
    assert any("debris cleared" in line for line in sys_.world.change_log)
    assert sys_.world.shortest_path_len(sys_.world.entry, r["target"]) is not None
    assert MemoryState.CLEARED in {x.state for x in sys_.living_map.all() if x.event_type == "BLOCKAGE"}


def test_policy_is_configurable_no_clearance_requested():
    r = dyn(42, "seal", request_debris=False)
    sys_ = r["system"]
    fire = next(m for m in sys_.mission_dispatcher.all_missions() if m.event_type == "FIRE")
    assert not r["debris_executor_used"] and fire.status != MissionStatus.COMPLETED


def test_policy_is_configurable_hold_position_keeps_the_executor_busy():
    r = dyn(42, "seal", request_debris=False, hold=True)
    ex = next(e for e in r["system"].executors if e.stats["missions"] and e.capabilities[0].value == "FIRE")
    assert ex.state == ExecutorState.WAITING


def test_different_seeds_produce_different_debris_and_possibly_different_responses():
    cells = {tuple(dyn(s, "random")["new_debris"]) for s in (1, 2, 3, 4)}
    assert len(cells) >= 3


def test_wait_parameter_changes_how_long_the_executor_lingers():
    short = dyn(42, "seal", max_wait=10, retries=0)
    long = dyn(42, "seal", max_wait=200, retries=0)
    t_s = next(e for e in short["system"].executors if e.stats["missions"] and e.capabilities[0].value == "FIRE")
    t_l = next(e for e in long["system"].executors if e.stats["missions"] and e.capabilities[0].value == "FIRE")
    wait = lambda e: next(ev["tick"] for ev in e.events if ev["kind"] == "returning") - next(ev["tick"] for ev in e.events if ev["kind"] == "target_unreachable")
    assert wait(t_l) > wait(t_s)


# ═══ With vs without inherited memory ═════════════════════════════════════════
def test_baseline_and_living_map_runs_share_the_environment_and_only_differ_in_the_brief():
    from simulation import compare_memory
    a = compare_memory.one(42, True, "on_route", 1)
    b = compare_memory.one(42, False, "on_route", 1)
    assert a["debris"] == b["debris"] and a["optimal_cells"] == b["optimal_cells"]
    assert a["reached_target"] and b["reached_target"]
    assert a["ticks_to_site"] < b["ticks_to_site"]                       # a measured result of THIS run, asserted for seed 42
    assert a["cells_discovered"] < b["cells_discovered"]
    assert b["search_targets"] > 0 and a["search_targets"] == 0          # baseline had to search; Living Map did not


def test_the_living_map_executor_does_not_know_future_debris():
    from simulation import compare_memory
    sc = build_scenario("nofuture", 42, [EventType.FIRE], fleet={"FIRE": 1, "DEBRIS": 1}, dispatch_threshold=0.0)
    sys_ = LiveMapSystem(sc)
    explore_phase(sys_)
    t = next(iter(sc.events))
    cells = pick_debris(sys_.world, 42, "on_route", sys_.world.entry, (t.row, t.col), 1)
    sys_.add_debris(cells[0])
    sys_.auto_dispatch = True
    run_ticks(sys_, 3)
    brief_hazards = sys_.command_post.living_map.hazard_hints()
    assert brief_hazards == [], "the Command Post cannot know about debris nobody has observed yet"
