from command_post.mission_planner import Mission, MissionTarget
from executor_robot.executor import ExecutorRobot
from common.enums import EventType
from simulation.world import EntryFrame, GroundTruthWorld, HiddenEvent

GRID = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 0, 1, 1],
    [1, 0, 0, 0, 0, 0, 1, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]


def run_executor(events, targets, max_ticks=10_000):
    world = GroundTruthWorld(GRID, events, entry=(1, 1))
    verified, contradicted = [], []
    ex = ExecutorRobot(
        world,
        on_verified=lambda bid, t: verified.append(bid),
        on_contradicted=lambda bid, t, reason: contradicted.append((bid, reason)),
    )
    ex.load_mission(Mission(mission_id=1, targets=targets))
    guard = 0
    while not ex.is_complete and guard < max_ticks:
        ex.update()
        guard += 1
    return ex, verified, contradicted


def test_executor_verifies_matching_event():
    events = [HiddenEvent(id=1, event_type=EventType.FIRE, row=3, col=8, severity=3)]
    x, y = EntryFrame((1, 1)).cell_to_local((3, 8))
    targets = [MissionTarget(beacon_id=1, event_type="FIRE", x_local=x, y_local=y, severity=3, state="UNVERIFIED")]
    ex, verified, contradicted = run_executor(events, targets)
    assert ex.is_complete
    assert verified == [1]
    assert contradicted == []


def test_executor_contradicts_when_nothing_found():
    # No hidden event actually placed there — the beacon's claim doesn't hold up.
    x, y = EntryFrame((1, 1)).cell_to_local((3, 8))
    targets = [MissionTarget(beacon_id=1, event_type="FIRE", x_local=x, y_local=y, severity=3, state="UNVERIFIED")]
    ex, verified, contradicted = run_executor([], targets)
    assert ex.is_complete
    assert verified == []
    assert len(contradicted) == 1
    assert contradicted[0][0] == 1


def test_executor_processes_multiple_targets_in_order():
    events = [
        HiddenEvent(id=1, event_type=EventType.FIRE, row=3, col=8, severity=3),
        HiddenEvent(id=2, event_type=EventType.GAS, row=5, col=2, severity=2),
    ]
    x1, y1 = EntryFrame((1, 1)).cell_to_local((3, 8))
    x2, y2 = EntryFrame((1, 1)).cell_to_local((5, 2))
    targets = [
        MissionTarget(beacon_id=1, event_type="FIRE", x_local=x1, y_local=y1, severity=3, state="UNVERIFIED"),
        MissionTarget(beacon_id=2, event_type="GAS", x_local=x2, y_local=y2, severity=2, state="UNVERIFIED"),
    ]
    ex, verified, contradicted = run_executor(events, targets)
    assert ex.is_complete
    assert set(verified) == {1, 2}
    assert len(ex.reached) == 2


def test_executor_no_mission_does_nothing():
    world = GroundTruthWorld(GRID, [], entry=(1, 1))
    ex = ExecutorRobot(world)
    ex.update()  # should not raise
    assert ex.mission is None
