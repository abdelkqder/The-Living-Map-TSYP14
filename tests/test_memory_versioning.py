"""
tests/test_memory_versioning.py
================================
PATCH 9 tests: BeaconMessage version field and LivingMap version guard.

Rules (PATCH 7):
    - BeaconMessage has version: int = 1 (Python-level, not wire-encoded)
    - MemoryRecord stores version from the message
    - LivingMap.ingest(): if existing.version >= msg.version → discard update
    - A higher-version packet DOES overwrite a lower-version record
    - Verification/contradiction state is unaffected by version (separate flow)
"""
from __future__ import annotations
import time, pytest
from command_post.map_state import LivingMap
from common.enums import MemoryState
from common.protocol import BeaconMessage


def _msg(beacon_id=1, event_type="FIRE", severity=3, conf=88, version=1):
    return BeaconMessage(
        beacon_id=beacon_id, event_type=event_type,
        x_local=5.0, y_local=5.0,
        timestamp=int(time.time()),
        severity=severity, initial_confidence=conf,
        battery_pct=90, version=version,
    )


class TestBeaconMessageVersion:
    def test_default_version_is_one(self):
        msg = _msg()
        assert msg.version == 1

    def test_explicit_version_stored(self):
        msg = _msg(version=3)
        assert msg.version == 3

    def test_version_not_in_wire_format(self):
        """Version is Python-level only in Phase 1 — decode_packet returns version=1."""
        from common.protocol import decode_packet
        msg     = _msg(version=5)
        raw     = msg.encode()
        decoded = decode_packet(raw)
        # Version is not encoded in the 18-byte wire format yet
        assert decoded.version == 1, (
            "wire format does not carry version in Phase 1; "
            "decoded message should have default version=1"
        )


class TestMemoryRecordVersion:
    def test_record_stores_version_from_message(self):
        lm  = LivingMap()
        msg = _msg(version=2)
        rec = lm.ingest(msg)
        assert rec.version == 2

    def test_record_default_version_one(self):
        lm  = LivingMap()
        rec = lm.ingest(_msg(version=1))
        assert rec.version == 1


class TestLivingMapVersionGuard:
    def test_same_version_does_not_overwrite(self):
        """Second packet with same version must be discarded."""
        lm   = LivingMap()
        msg1 = _msg(beacon_id=1, severity=3, version=1)
        msg2 = _msg(beacon_id=1, severity=1, version=1)  # same version, lower severity
        lm.ingest(msg1)
        rec2 = lm.ingest(msg2)
        # The existing record must be returned unchanged
        assert rec2.severity == 3, "same-version update must not overwrite"

    def test_older_version_does_not_overwrite(self):
        """A lower-version packet arriving after a higher one is discarded."""
        lm   = LivingMap()
        msg2 = _msg(beacon_id=1, event_type="FIRE", version=2)
        msg1 = _msg(beacon_id=1, event_type="GAS",  version=1)
        lm.ingest(msg2)
        rec = lm.ingest(msg1)
        assert rec.event_type == "FIRE", "older version must not overwrite newer"
        assert rec.version    == 2

    def test_newer_version_does_overwrite(self):
        """A higher-version packet REPLACES the existing record."""
        lm   = LivingMap()
        msg1 = _msg(beacon_id=1, event_type="FIRE", severity=2, version=1)
        msg2 = _msg(beacon_id=1, event_type="FIRE", severity=3, version=2)
        lm.ingest(msg1)
        rec = lm.ingest(msg2)
        assert rec.severity == 3, "higher version must replace existing record"
        assert rec.version  == 2

    def test_first_ingest_always_stored(self):
        lm  = LivingMap()
        rec = lm.ingest(_msg(beacon_id=1, version=1))
        assert rec is not None
        assert len(lm) == 1

    def test_version_guard_does_not_affect_verified_state(self):
        """
        A same-version duplicate arriving after mark_verified must be discarded
        WITHOUT touching the VERIFIED state.
        """
        lm  = LivingMap()
        lm.ingest(_msg(beacon_id=1, version=1))
        lm.mark_verified(1, at_tick=50)
        assert lm.get(1).state == MemoryState.VERIFIED

        # Same-version duplicate — should be discarded
        lm.ingest(_msg(beacon_id=1, version=1))
        assert lm.get(1).state == MemoryState.VERIFIED, \
            "version guard must preserve VERIFIED state"

    def test_different_beacons_are_independent(self):
        lm = LivingMap()
        lm.ingest(_msg(beacon_id=1, event_type="FIRE", version=1))
        lm.ingest(_msg(beacon_id=2, event_type="GAS",  version=1))
        lm.ingest(_msg(beacon_id=1, event_type="GAS",  version=1))  # duplicate b1
        assert lm.get(1).event_type == "FIRE"  # b1 unchanged
        assert lm.get(2).event_type == "GAS"   # b2 unchanged
        assert len(lm) == 2

    def test_version_two_replaces_version_one_contradicted(self):
        """
        If v1 was contradicted but v2 arrives with new observation, v2 wins.
        (The Living Map learns from correction — old contradiction is replaced.)
        """
        lm  = LivingMap()
        lm.ingest(_msg(beacon_id=1, version=1))
        lm.mark_contradicted(1, at_tick=30)
        assert lm.get(1).state == MemoryState.CONTRADICTED

        # Writer re-deploys same beacon with new observation (version 2)
        lm.ingest(_msg(beacon_id=1, event_type="FIRE", version=2))
        rec = lm.get(1)
        assert rec.version == 2
        assert rec.state   == MemoryState.UNVERIFIED, \
            "v2 packet resets state to UNVERIFIED for fresh verification"

    def test_log_contains_version_number(self):
        lm  = LivingMap()
        lm.ingest(_msg(beacon_id=1, version=3))
        assert any("v3" in entry for entry in lm.log)
