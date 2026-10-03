"""Living Map memory lifecycle, versioning, ageing, heartbeats — at the Command Post level."""
import pytest

from command_post.map_state import LivingMap
from common.clock import SimClock
from common.enums import MemoryState, PassageState
from common.memory import MemoryRecord
from common.protocol import BeaconMessage, Heartbeat, MemoryExt, executor_node_id, writer_node_id

TS = 1_790_000_000


def msg(mem=1, host=1, etype="GAS", x=1.0, y=2.0, sev=2, conf=80, ts=TS):
    return BeaconMessage(host, etype, x, y, ts, sev, conf, 100)


def ext(mem=1, ver=1, st=MemoryState.UNVERIFIED, passage=None, ts=TS, src=None):
    return MemoryExt(mem, ver, st, passage, ts, src if src is not None else writer_node_id(1))


def test_new_information_is_unverified():
    lm = LivingMap()
    rec = lm.ingest(msg(), ext())
    assert rec.state == MemoryState.UNVERIFIED and rec.version == 1 and rec.source == "W1"


@pytest.mark.parametrize("final", [MemoryState.VERIFIED, MemoryState.ACTIVE, MemoryState.ESCALATED,
                                    MemoryState.CLEARED, MemoryState.CONTRADICTED])
def test_every_lifecycle_state_can_be_reached_by_an_executor_update(final):
    lm = LivingMap()
    lm.ingest(msg(), ext())
    rec = lm.ingest(msg(), ext(ver=2, st=final, src=executor_node_id(0)))
    assert rec.state == final and rec.version == 2 and rec.source == "E1"
    assert rec.is_open == (final not in (MemoryState.CLEARED, MemoryState.CONTRADICTED))


def test_verified_then_active_then_cleared_chain_keeps_history():
    lm = LivingMap()
    lm.ingest(msg(), ext())
    for v, st in ((2, MemoryState.VERIFIED), (3, MemoryState.ACTIVE), (4, MemoryState.CLEARED)):
        lm.ingest(msg(), ext(ver=v, st=st, src=executor_node_id(1)))
    rec = lm.get(1)
    assert rec.state == MemoryState.CLEARED and rec.version == 4
    assert len(rec.history) >= 4 and "UNVERIFIED->VERIFIED" in " ".join(rec.history)


def test_old_information_never_overwrites_new_information():
    lm = LivingMap()
    lm.ingest(msg(), ext(ver=3, st=MemoryState.CLEARED))
    rec = lm.ingest(msg(), ext(ver=2, st=MemoryState.VERIFIED))
    assert rec.state == MemoryState.CLEARED and lm.stale_discarded == 1
    lm.ingest(msg(), ext(ver=3, st=MemoryState.CLEARED))
    assert lm.duplicates_discarded == 1


def test_blockage_record_exposes_passage_state_and_becomes_a_hazard_hint():
    lm = LivingMap()
    lm.ingest(msg(etype="BLOCKAGE", x=3.0, y=-1.0), ext(mem=9, passage=PassageState.BLOCKED))
    hints = lm.hazard_hints()
    assert len(hints) == 1 and hints[0].passage_state == "BLOCKED" and (hints[0].x_local, hints[0].y_local) == (3.0, -1.0)
    lm.ingest(msg(etype="BLOCKAGE", x=3.0, y=-1.0), ext(mem=9, ver=2, st=MemoryState.CLEARED, passage=PassageState.OPEN))
    assert lm.hazard_hints() == []                                 # cleared: no longer inherited as a hazard
    assert lm.mission_candidates() == []                           # blockages are navigation memory, not response missions


def test_adjacent_cells_are_distinct_but_a_re_report_of_the_same_place_is_merged():
    lm = LivingMap()
    lm.ingest(msg(etype="BLOCKAGE", x=1.0, y=1.0), ext(mem=1, passage=PassageState.BLOCKED))
    lm.ingest(msg(etype="BLOCKAGE", x=1.5, y=0.5), ext(mem=2, passage=PassageState.BLOCKED))      # diagonal neighbour
    assert len(lm) == 2
    twin = lm.ingest(msg(etype="BLOCKAGE", x=1.0, y=1.0), ext(mem=3, passage=PassageState.BLOCKED))
    assert len(lm) == 2 and twin.corroborations == 1


# ── ageing and confidence ─────────────────────────────────────────────────────
def test_age_and_confidence_follow_simulated_time_not_wall_clock():
    clock = SimClock()
    lm = LivingMap(clock=clock.now)
    rec = lm.ingest(msg(ts=clock.timestamp(), conf=100), ext(ts=clock.timestamp()))
    c0 = rec.live_confidence
    clock.set_tick(6000)                                   # 600 simulated seconds, ~0 real seconds
    assert rec.age_seconds == pytest.approx(600, abs=1)
    assert rec.live_confidence < c0 and rec.live_confidence == pytest.approx(1.0 * 2.718281828 ** (-0.002 * 600), abs=0.02)
    clock.set_tick(60_000)
    assert rec.is_stale


def test_a_new_observation_restarts_ageing_and_raises_mission_relevance():
    clock = SimClock()
    lm = LivingMap(clock=clock.now)
    rec = lm.ingest(msg(ts=clock.timestamp(), conf=90), ext(ts=clock.timestamp()))
    clock.set_tick(6000)
    old = rec.live_confidence
    now_ts = clock.timestamp()
    rec = lm.ingest(msg(ts=now_ts, conf=95), ext(ver=2, st=MemoryState.VERIFIED, ts=now_ts, src=executor_node_id(0)))
    assert rec.age_seconds < 2 and rec.live_confidence > old
    assert rec.record_age_seconds == pytest.approx(600, abs=1)       # the record itself is still old
    assert rec.mission_relevance > 0


def test_cleared_and_contradicted_records_have_zero_mission_relevance():
    for st in (MemoryState.CLEARED, MemoryState.CONTRADICTED):
        rec = MemoryRecord.from_beacon_message(msg())
        rec.state = st
        assert rec.mission_relevance == 0.0


def test_observe_bumps_version_and_records_who_saw_it():
    rec = MemoryRecord.from_beacon_message(msg())
    rec.observe(MemoryState.VERIFIED, by="E3", at_tick=50, now_ts=TS + 10)
    assert rec.version == 2 and rec.source == "E3" and rec.verified_by == "E3" and rec.observed_ts == TS + 10


# ── heartbeats ────────────────────────────────────────────────────────────────
def test_heartbeat_updates_beacon_health_without_touching_memory():
    lm = LivingMap()
    lm.ingest(msg(), ext())
    lm.heartbeat(Heartbeat(1, 2.0, 1.0, 2, 0, 88, TS + 5, 1, 1), (36.8, 10.1), 1, 77)
    h = lm.beacons[1]
    assert (h.hops_to_ona, h.battery_pct, h.n_records, h.last_seen_tick) == (2, 88, 1, 77)
    assert lm.get(1).version == 1 and lm.get(1).state == MemoryState.UNVERIFIED
