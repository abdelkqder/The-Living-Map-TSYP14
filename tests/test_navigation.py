from common.enums import NavMode
from navigation.engine import astar_known, choose_next_target, frontier_regions
from simulation.world import DiscoveredWorld, GroundTruthWorld

GRID = [
    [1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1],
]


def fully_discovered():
    world = GroundTruthWorld(GRID, [], entry=(1, 1))
    disc = DiscoveredWorld(world.rows, world.cols)
    disc.integrate(world.sense((1, 1), radius=10))
    return world, disc


def test_astar_finds_shortest_path_known_cells():
    world, disc = fully_discovered()
    path = astar_known(disc, (1, 1), (3, 5))
    assert path is not None
    assert path[0] == (1, 1)
    assert path[-1] == (3, 5)
    # Manhattan distance from (1,1) to (3,5) is 6, and this grid has a clear
    # route achieving exactly that (row 1 across, then down column 5).
    assert len(path) - 1 == 6


def test_astar_returns_none_when_unreachable_through_known_cells():
    world = GroundTruthWorld(GRID, [], entry=(1, 1))
    disc = DiscoveredWorld(world.rows, world.cols)
    disc.integrate(world.sense((1, 1), radius=1))  # small sweep, most of the map still UNKNOWN
    path = astar_known(disc, (1, 1), (3, 5))
    assert path is None


def test_frontier_regions_empty_when_fully_known():
    _, disc = fully_discovered()
    assert frontier_regions(disc) == []


def test_choose_next_target_explore_returns_none_when_fully_explored():
    _, disc = fully_discovered()
    result = choose_next_target(disc, (1, 1), NavMode.EXPLORE)
    assert result is None


def test_choose_next_target_explore_picks_a_frontier():
    world = GroundTruthWorld(GRID, [], entry=(1, 1))
    disc = DiscoveredWorld(world.rows, world.cols)
    disc.integrate(world.sense((1, 1), radius=1))
    result = choose_next_target(disc, (1, 1), NavMode.EXPLORE)
    assert result is not None
    assert result.cell in disc.frontier_cells()


def test_execute_mode_prefers_progress_toward_goal():
    """
    With a goal set, EXECUTE mode should score a frontier that reduces
    distance-to-goal higher than pure EXPLORE mode would (all else equal),
    since W_GOAL adds a bonus for getting closer to the target.
    """
    world = GroundTruthWorld(GRID, [], entry=(1, 1))
    disc = DiscoveredWorld(world.rows, world.cols)
    disc.integrate(world.sense((1, 1), radius=1))

    explore_choice = choose_next_target(disc, (1, 1), NavMode.EXPLORE)
    execute_choice = choose_next_target(disc, (1, 1), NavMode.EXECUTE, goal=(3, 5))

    assert explore_choice is not None and execute_choice is not None
    # EXECUTE's chosen cell must not be farther from the goal than EXPLORE's pick
    dist_explore = abs(explore_choice.cell[0] - 3) + abs(explore_choice.cell[1] - 5)
    dist_execute = abs(execute_choice.cell[0] - 3) + abs(execute_choice.cell[1] - 5)
    assert dist_execute <= dist_explore
