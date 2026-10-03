from common.enums import EventType
from communication.mock_transport import MockTransport
from simulation.world import GroundTruthWorld, HiddenEvent
from writer_robot.writer import WriterRobot

GRID = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 0, 1, 1],
    [1, 0, 0, 0, 0, 0, 1, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]


def run_writer(events, max_ticks=2000):
    world = GroundTruthWorld(GRID, events, entry=(1, 1))
    transport = MockTransport("beacon")
    writer = WriterRobot(world, transport, max_ticks=max_ticks)
    writer.start()
    guard = 0
    while not writer.dead and guard < 10_000:
        writer.update()
        guard += 1
    return writer, guard


def test_writer_explores_and_preserves_events():
    events = [
        HiddenEvent(id=1, event_type=EventType.FIRE, row=3, col=8, severity=3),
        HiddenEvent(id=2, event_type=EventType.GAS, row=5, col=2, severity=2),
    ]
    writer, ticks = run_writer(events)
    assert writer.beacon_count == 2
    assert writer.discovered.coverage_ratio(1) > 0  # sanity: it moved and mapped something


def test_writer_stops_within_tick_budget():
    # PHASE-1 PATCH: the budget bounds EXPLORATION. When it is spent the Writer
    # heads back to the entry (it does not "die"); `dead` is the legacy alias for
    # "no longer exploring", `returned` says it got home.
    writer, ticks = run_writer([], max_ticks=50)
    assert writer.dead is True
    assert writer.returned is True
    assert writer.tick >= 50


def test_writer_deploys_beacons_via_transport():
    events = [HiddenEvent(id=1, event_type=EventType.FIRE, row=3, col=8, severity=3)]
    world = GroundTruthWorld(GRID, events, entry=(1, 1))
    transport = MockTransport("beacon")
    received = []
    transport.subscribe(lambda raw: received.append(raw))
    writer = WriterRobot(world, transport, max_ticks=2000)
    writer.start()
    guard = 0
    while not writer.dead and guard < 10_000:
        writer.update()
        guard += 1
    assert len(received) == writer.beacon_count == 1


def test_writer_does_not_redeploy_for_same_event_twice():
    # A single event; even if re-sensed multiple times while nearby, only
    # one beacon should ever be deployed for it (selective memory dedup).
    events = [HiddenEvent(id=1, event_type=EventType.FIRE, row=3, col=8, severity=3)]
    writer, ticks = run_writer(events)
    assert writer.beacon_count == 1
