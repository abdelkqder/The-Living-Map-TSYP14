from common.enums import MemoryState
from simulation.scenario import default_scenario
from simulation.system import LiveMapSystem


def run_full_mission(seed, writer_max_ticks=None, guard=30_000):
    sys_ = LiveMapSystem(default_scenario(seed=seed, writer_max_ticks=writer_max_ticks))
    sys_.start_writer()
    n = 0
    while sys_.phase == "writer" and n < guard:
        sys_.update()
        n += 1
    sys_.start_executor()
    n = 0
    while sys_.phase == "executor" and n < guard:
        sys_.update()
        n += 1
    return sys_


def test_full_pipeline_reaches_complete_with_generous_budget():
    sys_ = run_full_mission(seed=1, writer_max_ticks=None)
    assert sys_.phase == "complete"
    assert len(sys_.living_map) == 2  # both FIRE and GAS found with unlimited budget
    for rec in sys_.living_map.all():
        assert rec.state == MemoryState.VERIFIED


def test_writer_knowledge_survives_writer_death():
    """The literal point of the project: knowledge preserved before failure
    is still usable by the Executor afterwards."""
    sys_ = LiveMapSystem(default_scenario(seed=1, writer_max_ticks=None))
    sys_.start_writer()
    n = 0
    while sys_.phase == "writer" and n < 30_000:
        sys_.update()
        n += 1
    assert sys_.phase == "dead"
    beacons_before_executor = len(sys_.living_map)
    assert beacons_before_executor > 0

    sys_.start_executor()
    n = 0
    while sys_.phase == "executor" and n < 30_000:
        sys_.update()
        n += 1
    # nothing was lost or added between Writer death and mission assignment
    assert len(sys_.living_map) == beacons_before_executor


def test_no_direct_writer_to_command_post_link():
    """
    Structural check: neither robot module may couple directly to the
    Living Map / Command Post. WriterRobot must have zero command_post
    references at all. ExecutorRobot legitimately imports Mission /
    MissionTarget (plain data shapes received "via the ONA relay") from
    command_post.mission_planner, but must never import command_post's
    LivingMap/map_state or call its mutating methods directly — those only
    happen inside simulation/system.py's _on_verified/_on_contradicted
    callbacks.
    """
    import inspect
    import writer_robot.writer as writer_mod
    import executor_robot.executor as executor_mod

    writer_src = inspect.getsource(writer_mod)
    executor_src = inspect.getsource(executor_mod)

    assert "command_post" not in writer_src.lower()
    assert "LivingMap" not in writer_src

    for forbidden in ("command_post.map_state", "LivingMap", ".mark_verified(", ".mark_contradicted("):
        assert forbidden not in executor_src, f"unexpected coupling: {forbidden!r} found in executor.py"


def test_partial_budget_still_completes_a_smaller_mission():
    """Writer with a tight tick budget still produces a usable (if partial) mission."""
    sys_ = run_full_mission(seed=1, writer_max_ticks=300)
    assert sys_.phase in ("complete", "dead")  # 'dead' only if literally zero beacons were found
    if len(sys_.living_map) > 0:
        assert sys_.phase == "complete"


def test_metrics_recorded_after_writer_phase():
    sys_ = LiveMapSystem(default_scenario(seed=1, writer_max_ticks=300))
    sys_.start_writer()
    n = 0
    while sys_.phase == "writer" and n < 5000:
        sys_.update()
        n += 1
    assert len(sys_.metrics.runs) == 1
    m = sys_.metrics.runs[0]
    assert m.role == "writer"
    assert 0 <= m.coverage_pct <= 100
