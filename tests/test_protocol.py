import time
import pytest

from common.protocol import BeaconMessage, decode_packet, DecodeError, PACKET_SIZE, BeaconRegistry


def make_msg(**overrides):
    defaults = dict(beacon_id=1, event_type="FIRE", x_local=1.5, y_local=2.5,
                     timestamp=int(time.time()), severity=3, initial_confidence=90, battery_pct=80)
    defaults.update(overrides)
    return BeaconMessage(**defaults)


def test_encode_size():
    assert len(make_msg().encode()) == PACKET_SIZE == 18


def test_roundtrip():
    msg = make_msg(event_type="GAS", severity=2)
    decoded = decode_packet(msg.encode())
    assert decoded.event_type == "GAS"
    assert decoded.beacon_id == 1
    assert decoded.severity == 2
    assert abs(decoded.x_local - 1.5) < 1e-3
    assert abs(decoded.y_local - 2.5) < 1e-3


def test_bad_crc_rejected():
    raw = bytearray(make_msg().encode())
    raw[-1] ^= 0xFF  # corrupt the CRC byte
    with pytest.raises(DecodeError):
        decode_packet(bytes(raw))


def test_too_short_rejected():
    with pytest.raises(DecodeError):
        decode_packet(b"\x00\x01\x02")


def test_unknown_event_type_raises_on_encode():
    with pytest.raises(ValueError):
        make_msg(event_type="NOT_A_REAL_TYPE").encode()


def test_confidence_decays_over_time():
    msg = make_msg(initial_confidence=100, timestamp=int(time.time()) - 10_000)
    assert msg.live_confidence < 1.0
    assert msg.confidence_pct < 100


def test_fresh_beacon_is_not_stale():
    assert make_msg(initial_confidence=100, timestamp=int(time.time())).is_stale is False


def test_old_beacon_becomes_stale():
    ancient = make_msg(initial_confidence=100, timestamp=int(time.time()) - 5_000_000)
    assert ancient.is_stale is True


def test_registry_active_excludes_stale():
    reg = BeaconRegistry()
    reg.add(make_msg(beacon_id=1, timestamp=int(time.time())))
    reg.add(make_msg(beacon_id=2, timestamp=int(time.time()) - 5_000_000))
    active_ids = {m.beacon_id for m in reg.active()}
    assert active_ids == {1}


def test_registry_active_sorted_by_severity_desc():
    reg = BeaconRegistry()
    reg.add(make_msg(beacon_id=1, severity=1, timestamp=int(time.time())))
    reg.add(make_msg(beacon_id=2, severity=4, timestamp=int(time.time())))
    ordered = [m.beacon_id for m in reg.active()]
    assert ordered[0] == 2
