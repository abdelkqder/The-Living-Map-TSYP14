"""Executor: inherited-memory use, obstacle re-planning, bump handling, lifecycle, failure / return branches."""
import pytest

from common.enums import EventType, ExecutorState, MemoryState, PassageState, RobotCapability
from common.mission import BlockagePolicy, HazardHint, Mission, MissionTarget
from executor_robot.executor import ExecutorRobot
from simulation.world import EntryFrame, GroundTruthWorld, HiddenEvent

# two routes (top / bottom) from the entry (1,1) to the FIRE at (3,7)
GRID = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
]
ENTRY = (1, 1)
F = EntryFrame(ENTRY)


def world(debris=(), events=None):
    gt = GroundTruthWorld(GRID, events if events is not None else [HiddenEvent(1, EventType.FIRE, 3, 7, 3)], ENTRY)
    for d in debris:
        gt.add_debris(d)
    return gt


def brief(cell=(3, 7), etype="FIRE", hazards=(), policy=None, waypoints=()):
    x, y = F.cell_to_local(cell)
    return Mission("M1", [MissionTarget(1, etype, x, y, 3, "UNVERIFIED", memory_id=1)],
                   [F.cell_to_local(w) for w in waypoints],
                   [HazardHint(*F.cell_to_local(c), PassageState.BLOCKED.value, 5) for c in hazards],
                   policy or BlockagePolicy(max_wait_ticks=30, max_retries=0))


def run(ex, n=6000):
    seen = [ex.state]
    for _ in range(n):
        ex.update()
        if ex.state != seen[-1]:
            seen.append(ex.state)
        if ex.is_complete:
            break
    return seen


def executor(gt, caps=(RobotCapability.FIRE,), **kw):
    return ExecutorRobot(gt, capabilities=list(caps), **kw)


def test_full_lifecycle_visits_every_state_in_order_and_ends_available_at_the_entry():
    ex = executor(world())
    ex.load_mission(brief())
    seen = run(ex)
    S = ExecutorState
    order = [S.ASSIGNED, S.DEPLOYING, S.ON_SITE, S.WORKING, S.VERIFYING, S.UPDATE_MEMORY, S.RETURNING, S.AVAILABLE]
    idx = [seen.index(s) for s in order]
    assert idx == sorted(idx)
    assert ex.at_entry and ex.is_complete and ex.world_gt.event_cells_view()[(3, 7)] == ("FIRE", True)   # fire extinguished


def test_inherited_blocked_passage_is_never_tried_and_an_alternative_route_is_taken():
    ex = executor(world(debris=[(1, 4)]))              # debris really there AND remembered in the brief
    ex.load_mission(brief(hazards=[(1, 4)]))
    run(ex)
    assert (1, 4) not in ex.actual_route
    assert ex.stats["failed_approaches"] == 0 and ex.stats["repeated_approaches"] == 0
    assert any(c in ex.actual_route for c in [(3, 3), (3, 4), (3, 5)]), "it went round via the other corridor"
    assert ex.state == ExecutorState.AVAILABLE


def test_without_the_inherited_hazard_the_same_executor_finds_the_debris_by_sensing_and_replans():
    ex = executor(world(debris=[(1, 4)]))
    ex.load_mission(brief())                           # no hazard in the brief
    run(ex)
    assert (1, 4) not in ex.actual_route
    assert ex.stats["discrepancies"] >= 1 and ex.stats["blockages_recorded"] >= 1 or ex.stats["replans_blockage"] >= 1
    assert ex.state == ExecutorState.AVAILABLE


def test_a_remembered_blockage_that_is_gone_is_detected_and_reported_cleared():
    ex = executor(world())                             # nothing there ...
    ex.load_mission(brief(hazards=[(1, 4)]))           # ... but memory says blocked
    run(ex)
    assert any(ev["kind"] == "discrepancy" and ev["observed"] == "FREE" for ev in ex.events)
    assert ex.stats["discrepancies"] >= 1


def test_debris_appearing_between_sensing_and_moving_is_a_bump_not_a_walk_through():
    gt = world()
    ex = executor(gt)
    ex.load_mission(brief())
    for _ in range(45):                                # still on the outbound leg
        ex.update()
    nxt = ex.movement.remaining_cells()[:1]
    assert nxt and nxt[0] not in ex.actual_route, "robot is mid-route, next cell not yet entered"
    gt.add_debris(nxt[0], tick=ex.tick)                # appears right in front of it
    for _ in range(600):
        ex.update()
        if ex.stats["failed_approaches"] or ex.is_complete:
            break
    assert nxt[0] not in ex.actual_route
    assert ex.stats["failed_approaches"] >= 1 or ex.stats["replans"] >= 1


def test_unreachable_target_follows_the_policy_wait_then_report_then_return():
    gt = world(debris=[(3, 6), (2, 7)])                # both ways into (3,7) are sealed
    ex = executor(gt, link=None)
    ex.load_mission(brief(policy=BlockagePolicy(max_wait_ticks=20, max_retries=1, request_debris_executor=True, return_if_unreachable=True)))
    seen = run(ex)
    assert ExecutorState.WAITING in seen and ExecutorState.UPDATE_MEMORY in seen and ExecutorState.RETURNING in seen
    assert ex.state == ExecutorState.AVAILABLE and ex.at_entry
    assert any(ev["kind"] == "target_unreachable" for ev in ex.events) and any(ev["kind"] == "retry_after_wait" for ev in ex.events)


def test_policy_hold_position_does_not_return():
    ex = executor(world(debris=[(3, 6), (2, 7)]))
    ex.load_mission(brief(policy=BlockagePolicy(max_wait_ticks=10, max_retries=0, return_if_unreachable=False)))
    run(ex, 1500)
    assert ex.state == ExecutorState.WAITING and not ex.at_entry


def test_fault_leads_to_needs_repair_and_the_robot_still_gets_home():
    ex = executor(world(), fault_at_tick=40)
    ex.load_mission(brief())
    seen = run(ex)
    assert ExecutorState.FAILED in seen and ex.state == ExecutorState.NEEDS_REPAIR and ex.at_entry
    assert ex.service() and ex.state == ExecutorState.AVAILABLE


def test_stranding_fault_is_out_of_service():
    ex = executor(world(), fault_at_tick=40, fault_strands=True)
    ex.load_mission(brief())
    run(ex, 400)
    assert ex.state == ExecutorState.OUT_OF_SERVICE and not ex.at_entry


def test_low_battery_aborts_the_mission_returns_and_needs_charging():
    ex = executor(world(), battery_drain_per_tick=1.0)
    ex.load_mission(brief())
    seen = run(ex)
    assert ExecutorState.LOW_BATTERY in seen and ex.state == ExecutorState.NEEDS_CHARGING and ex.at_entry


def test_debris_executor_clears_a_remembered_blockage_and_reports_cleared():
    gt = world(debris=[(1, 4)], events=[])
    ex = executor(gt, caps=(RobotCapability.DEBRIS,))
    ex.load_mission(brief(cell=(1, 4), etype="BLOCKAGE", hazards=[(1, 4)]))
    run(ex)
    assert not gt.is_debris((1, 4)) and ex.state == ExecutorState.AVAILABLE
    assert not gt.debris_cells()


def test_victim_executor_verifies_but_cannot_remove_the_victim():
    gt = world(events=[HiddenEvent(1, EventType.VICTIM_PRESENCE, 3, 7, 4)])
    ex = executor(gt, caps=(RobotCapability.VICTIM,))
    ex.load_mission(brief(etype="VICTIM_PRESENCE"))
    run(ex)
    assert gt.event_cells_view()[(3, 7)] == ("VICTIM_PRESENCE", False) and ex.state == ExecutorState.AVAILABLE
    assert any(ev["kind"] == "verified" and ev["result"] == "ESCALATED" for ev in ex.events)


def test_gas_executor_leaves_the_gas_active():
    gt = world(events=[HiddenEvent(1, EventType.GAS, 3, 7, 2)])
    ex = executor(gt, caps=(RobotCapability.GAS,))
    ex.load_mission(brief(etype="GAS"))
    run(ex)
    assert any(ev["kind"] == "verified" and ev["result"] == "ACTIVE" for ev in ex.events)


def test_baseline_executor_without_a_location_still_finds_the_event_by_searching():
    ex = executor(world())
    ex.load_mission(Mission("B", [], search_types=["FIRE"], memory_ref=1))
    run(ex)
    assert ex.stats["search_targets"] > 0 and ex.state == ExecutorState.AVAILABLE
    assert ex.world_gt.event_cells_view()[(3, 7)] == ("FIRE", True)
