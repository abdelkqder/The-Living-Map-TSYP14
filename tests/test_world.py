from common.enums import CellState, EventType
from simulation.world import DiscoveredWorld, GroundTruthWorld, HiddenEvent

SIMPLE_GRID = [
    [1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1],
    [1, 0, 1, 0, 1],
    [1, 0, 0, 0, 1],
    [1, 1, 1, 1, 1],
]


def make_world(events=None):
    return GroundTruthWorld(SIMPLE_GRID, events or [], entry=(1, 1))


def test_sense_reveals_only_within_radius():
    world = make_world()
    snap = world.sense((1, 1), radius=1)
    assert (1, 1) in snap.free
    assert (3, 3) not in snap.free  # far corner, out of radius


def test_sense_blocked_by_walls():
    world = make_world()
    # (2,2) is a wall in the middle; sensing from (1,1) should reveal it as a wall
    snap = world.sense((1, 1), radius=3)
    assert (2, 2) in snap.wall


def test_sense_detects_event_at_exact_cell():
    ev = HiddenEvent(id=1, event_type=EventType.FIRE, row=1, col=2, severity=3)
    world = make_world([ev])
    snap = world.sense((1, 2), radius=0)
    assert snap.event_here is not None
    assert snap.event_here.id == 1


def test_sense_does_not_detect_event_outside_swept_area():
    ev = HiddenEvent(id=1, event_type=EventType.FIRE, row=3, col=3, severity=3)
    world = make_world([ev])
    snap = world.sense((1, 1), radius=1)
    assert snap.event_here is None


def test_discovered_world_starts_unknown():
    disc = DiscoveredWorld(5, 5)
    assert disc.state_of((1, 1)) == CellState.UNKNOWN
    assert disc.is_free((1, 1)) is False


def test_discovered_world_integrates_snapshot():
    world = make_world()
    disc = DiscoveredWorld(world.rows, world.cols)
    disc.integrate(world.sense((1, 1), radius=2))
    assert disc.is_free((1, 1)) is True
    assert disc.state_of((2, 2)) == CellState.WALL


def test_frontier_cells_border_unknown():
    world = make_world()
    disc = DiscoveredWorld(world.rows, world.cols)
    disc.integrate(world.sense((1, 1), radius=1))
    frontier = disc.frontier_cells()
    assert len(frontier) > 0
    for cell in frontier:
        assert disc.is_free(cell)


def test_coverage_ratio():
    world = make_world()
    disc = DiscoveredWorld(world.rows, world.cols)
    total_free = sum(1 for r in range(world.rows) for c in range(world.cols) if not world.is_wall(r, c))
    assert disc.coverage_ratio(total_free) == 0.0
    disc.integrate(world.sense((1, 1), radius=10))
    assert disc.coverage_ratio(total_free) == 1.0


# ── "No cheating" structural guard — doc4 §8 explicitly requires this ────────
# GroundTruthWorld deliberately exposes only sense()/is_wall()/in_bounds()/
# entry/rows/cols/all_event_ids()/event_by_id() as public surface. The last
# two exist ONLY for the simulator's own metrics — robot code must never
# call them. This test greps the actual robot source files rather than
# testing behaviour, because the point is to catch the shortcut being taken
# at all, not just its effects.
import pathlib

FORBIDDEN_SUBSTRINGS = ["_events", "_grid", ".all_event_ids(", ".event_by_id("]
ROBOT_SOURCE_DIRS = ["writer_robot", "executor_robot"]


def test_robot_code_never_touches_ground_truth_internals():
    root = pathlib.Path(__file__).resolve().parent.parent
    offenders = []
    for pkg in ROBOT_SOURCE_DIRS:
        for path in (root / pkg).glob("*.py"):
            text = path.read_text()
            for forbidden in FORBIDDEN_SUBSTRINGS:
                if forbidden in text:
                    offenders.append(f"{path.relative_to(root)}: found {forbidden!r}")
    assert not offenders, "Ground-truth cheating detected:\n" + "\n".join(offenders)
