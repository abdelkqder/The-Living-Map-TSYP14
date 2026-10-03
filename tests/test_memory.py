import time

from common.enums import MemoryState
from common.memory import MemoryRecord
from common.protocol import BeaconMessage


def make_beacon(**overrides):
    defaults = dict(beacon_id=3, event_type="FIRE", x_local=1.0, y_local=2.0,
                     timestamp=int(time.time()), severity=3, initial_confidence=85, battery_pct=70)
    defaults.update(overrides)
    return BeaconMessage(**defaults)


def test_starts_unverified():
    rec = MemoryRecord.from_beacon_message(make_beacon())
    assert rec.state == MemoryState.UNVERIFIED
    assert len(rec.history) == 1


def test_mark_verified():
    rec = MemoryRecord.from_beacon_message(make_beacon())
    rec.mark_verified(by="executor", at_tick=10)
    assert rec.state == MemoryState.VERIFIED
    assert rec.verified_by == "executor"
    assert "VERIFIED" in rec.history[-1]


def test_mark_contradicted():
    rec = MemoryRecord.from_beacon_message(make_beacon())
    rec.mark_contradicted(by="executor", at_tick=10, reason="no match")
    assert rec.state == MemoryState.CONTRADICTED
    assert "CONTRADICTED" in rec.history[-1]


def test_going_stale_does_not_erase_state():
    rec = MemoryRecord.from_beacon_message(make_beacon(timestamp=int(time.time())))
    rec.mark_verified(at_tick=1)
    rec.timestamp -= 5_000_000  # force staleness by aging it artificially
    assert rec.is_stale is True
    assert rec.state == MemoryState.VERIFIED  # state must survive staleness
    assert len(rec.history) == 2  # nothing erased


def test_history_accumulates():
    rec = MemoryRecord.from_beacon_message(make_beacon())
    rec.mark_verified(at_tick=1)
    assert len(rec.history) == 2
