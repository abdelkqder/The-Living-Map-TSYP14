"""The no-direct-communication constraint, enforced in CODE (not documentation), plus the ONA boundary."""
import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
ROBOT_PKGS = ["writer_robot", "executor_robot"]
FIELD_PKGS = ["writer_robot", "executor_robot", "beacon", "communication"]      # everything that lives inside the zone / on its radio


def imports_of(path: pathlib.Path):
    tree = ast.parse(path.read_text())
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            out.add(node.module)
            out.update(f"{node.module}.{a.name}" for a in node.names)
    return out


def test_robot_and_field_code_never_import_the_command_post():
    bad = []
    for pkg in FIELD_PKGS:
        for f in (ROOT / pkg).glob("*.py"):
            for imp in imports_of(f):
                if imp.split(".")[0] == "command_post":
                    bad.append(f"{f.relative_to(ROOT)} imports {imp}")
    assert not bad, "illegal Robot/Beacon -> Command Post dependency:\n" + "\n".join(bad)


def test_robot_code_does_not_import_the_ona_or_the_simulation_orchestrator():
    bad = []
    for pkg in ROBOT_PKGS:
        for f in (ROOT / pkg).glob("*.py"):
            for imp in imports_of(f):
                if imp.split(".")[0] == "gateway" or imp in ("simulation.system", "simulation.scenario"):
                    bad.append(f"{f.relative_to(ROOT)} imports {imp}")
    assert not bad, "robots reach the ONA only through the radio (RobotLink):\n" + "\n".join(bad)


def test_the_command_post_is_never_imported_by_the_ona_or_the_radio_layer():
    for pkg in ("gateway", "communication", "beacon"):
        for f in (ROOT / pkg).glob("*.py"):
            assert not any(i.split(".")[0] == "command_post" for i in imports_of(f)), f


def test_robotlink_exposes_no_way_to_reach_the_command_post():
    from communication.mesh import RobotLink
    public = {n for n in dir(RobotLink) if not n.startswith("_")}
    assert public == {"hello", "node_id", "outbox_size", "port", "program_beacon", "send_upstream", "tick", "uplink"} or \
        not any("command" in n.lower() or "post" in n.lower() for n in public)


def test_no_transport_in_the_system_is_shared_between_a_robot_and_the_command_post():
    from simulation.phase1 import build_scenario
    from simulation.system import LiveMapSystem
    sys_ = LiveMapSystem(build_scenario("arch", 42, fleet={"FIRE": 1}))
    w = sys_.deploy_writer()
    e = sys_.executors[0]
    cp = sys_.command_post
    reachable_from_robot = {id(w.link), id(w.link.port), id(w.transport), id(e.link), id(e.link.port)}
    for name, obj in vars(cp).items():
        assert id(obj) not in reachable_from_robot, f"Command Post attribute {name!r} is a robot-side handle"
    # the ONLY object holding callbacks into the Command Post is the ONA
    assert sys_.ona._memory_fn.__self__ is cp and sys_.ona._status_fn.__self__ is cp
    assert not hasattr(w, "command_post") and not hasattr(e, "command_post")


def test_the_radio_medium_has_no_command_post_node():
    from simulation.phase1 import build_scenario
    from simulation.system import LiveMapSystem
    sys_ = LiveMapSystem(build_scenario("arch2", 42, fleet={"FIRE": 1}))
    sys_.deploy_writer()
    kinds = {n.kind for n in sys_.medium._nodes.values()}
    assert kinds <= {"ona", "beacon", "robot"}


def test_a_robot_alone_in_the_zone_cannot_get_a_message_to_the_command_post():
    """No beacon chain => nothing reaches the Command Post, even though the robot transmits."""
    from common.protocol import FrameType
    from simulation.phase1 import build_scenario
    from simulation.system import LiveMapSystem
    sys_ = LiveMapSystem(build_scenario("arch3", 42, fleet={"FIRE": 1}))
    deep = (9, 16)                                                       # far from the ONA, no beacons deployed
    from communication.mesh import RadioPort, RobotLink
    link = RobotLink(RadioPort(sys_.medium, 130, lambda: deep, kind="robot"))
    assert link.uplink() is None
    assert link.send_upstream(FrameType.STATUS, b"x" * 8) is False       # queued, not sent
    for _ in range(200):
        sys_.update(); link.tick()
    assert sys_.command_post.stats["status_in"] == 0 and len(sys_.living_map) == 0


def test_executor_gets_its_brief_only_through_the_ona_mailbox():
    from simulation.phase1 import build_scenario, explore_phase
    from simulation.system import LiveMapSystem
    sys_ = LiveMapSystem(build_scenario("arch4", 42, fleet={"FIRE": 1, "GAS": 1}, dispatch_threshold=0.0))
    explore_phase(sys_)
    e = sys_.executors[0]
    assert e.mission is None
    sys_.auto_dispatch = True
    sys_.update(); sys_.update()
    assert sys_.ona.briefs_queued >= 1
    for _ in range(40):
        sys_.update()
    assert sys_.ona.briefs_delivered >= 1


# ── ONA boundary: it carries, it never plans ──────────────────────────────────
def test_ona_has_no_mission_planning_responsibility():
    src = (ROOT / "gateway" / "ona_gateway.py").read_text()
    tree = ast.parse(src)
    names = {n.name for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
    for forbidden in ("review_and_assign", "approve", "propose_mission", "plan", "dispatch", "assign", "build_brief"):
        assert not any(forbidden in n.lower() for n in names), f"ONA defines {forbidden}"
    assert "command_post" not in src.split('"""', 2)[2]               # no import of / call into the Command Post


def test_ona_does_not_create_missions_from_beacon_data():
    from gateway.ona_gateway import ONAGateway
    from common.protocol import BeaconMessage
    got = []
    ona = ONAGateway(forward_fn=lambda m: got.append(m) or True)
    ona.ingest(BeaconMessage(1, "FIRE", 1.0, 1.0, 1_790_000_000, 3, 90, 100).encode())
    assert len(got) == 1 and not hasattr(ona, "missions") and not hasattr(ona, "_planner")
