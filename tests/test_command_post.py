import time

from common.enums import MemoryState
from common.protocol import BeaconMessage
from command_post.map_state import LivingMap
from command_post.mission_planner import MissionPlanner


def make_beacon(**overrides):
    defaults = dict(beacon_id=1, event_type="FIRE", x_local=2.0, y_local=3.0,
                     timestamp=int(time.time()), severity=3, initial_confidence=90, battery_pct=80)
    defaults.update(overrides)
    return BeaconMessage(**defaults)


def test_ingest_creates_unverified_record():
    lm = LivingMap()
    rec = lm.ingest(make_beacon())
    assert rec.state == MemoryState.UNVERIFIED
    assert len(lm) == 1


def test_mark_verified_updates_state():
    lm = LivingMap()
    lm.ingest(make_beacon(beacon_id=1))
    lm.mark_verified(1, at_tick=5)
    assert lm.get(1).state == MemoryState.VERIFIED


def test_mark_contradicted_updates_state():
    lm = LivingMap()
    lm.ingest(make_beacon(beacon_id=1))
    lm.mark_contradicted(1, at_tick=5, reason="mismatch")
    assert lm.get(1).state == MemoryState.CONTRADICTED


def test_mission_candidates_excludes_contradicted():
    lm = LivingMap()
    lm.ingest(make_beacon(beacon_id=1))
    lm.ingest(make_beacon(beacon_id=2))
    lm.mark_contradicted(2, at_tick=1)
    candidates = lm.mission_candidates()
    ids = {r.beacon_id for r in candidates}
    assert ids == {1}


def test_mission_candidates_excludes_stale():
    lm = LivingMap()
    lm.ingest(make_beacon(beacon_id=1, timestamp=int(time.time()) - 5_000_000))
    assert lm.mission_candidates() == []


def test_mission_candidates_sorted_by_severity_desc():
    lm = LivingMap()
    lm.ingest(make_beacon(beacon_id=1, severity=1))
    lm.ingest(make_beacon(beacon_id=2, severity=4))
    candidates = lm.mission_candidates()
    assert candidates[0].beacon_id == 2


def test_mission_planner_builds_targets_from_candidates():
    lm = LivingMap()
    lm.ingest(make_beacon(beacon_id=1, event_type="FIRE"))
    planner = MissionPlanner()
    mission = planner.review_and_assign(lm.mission_candidates())
    assert len(mission.targets) == 1
    assert mission.targets[0].event_type == "FIRE"
    assert mission.is_empty is False


def test_mission_planner_approve_fn_can_filter():
    lm = LivingMap()
    lm.ingest(make_beacon(beacon_id=1, severity=1))
    lm.ingest(make_beacon(beacon_id=2, severity=4))
    planner = MissionPlanner(approve_fn=lambda cands: [c for c in cands if c.severity >= 3])
    mission = planner.review_and_assign(lm.mission_candidates())
    assert len(mission.targets) == 1
    assert mission.targets[0].beacon_id == 2


def test_empty_candidates_yields_empty_mission():
    planner = MissionPlanner()
    mission = planner.review_and_assign([])
    assert mission.is_empty is True
