import time

from common.protocol import BeaconMessage
from gateway.ona_gateway import ONAGateway


def make_packet(**overrides):
    defaults = dict(beacon_id=1, event_type="FIRE", x_local=2.0, y_local=3.0,
                     timestamp=int(time.time()), severity=3, initial_confidence=90, battery_pct=80)
    defaults.update(overrides)
    return BeaconMessage(**defaults).encode()


def test_ingest_valid_packet_forwards_and_transforms_coords():
    forwarded = []
    ona = ONAGateway(forward_fn=lambda msg: forwarded.append(msg) or True)
    msg = ona.ingest(make_packet())
    assert msg is not None
    assert msg.gps_lat is not None and msg.gps_lon is not None
    assert forwarded == [msg]
    assert ona.stats["forwarded"] == 1
    assert ona.stats["rejected"] == 0


def test_ingest_corrupt_packet_rejected():
    ona = ONAGateway(forward_fn=lambda msg: True)
    raw = bytearray(make_packet())
    raw[-1] ^= 0xFF
    result = ona.ingest(bytes(raw))
    assert result is None
    assert ona.stats["rejected"] == 1
    assert ona.stats["forwarded"] == 0


def test_buffering_when_downstream_unavailable():
    ona = ONAGateway(forward_fn=lambda msg: False)  # downstream always fails
    ona.ingest(make_packet())
    assert ona.stats["buffered"] == 1
    assert len(ona._buffer) == 1


def test_flush_buffer_retries_and_succeeds():
    calls = {"n": 0}

    def flaky_forward(msg):
        calls["n"] += 1
        return calls["n"] > 1  # fail first time, succeed after

    ona = ONAGateway(forward_fn=flaky_forward)
    ona.ingest(make_packet())
    assert ona.stats["buffered"] == 1
    remaining = ona.flush_buffer()
    assert remaining == 0
    assert ona.stats["forwarded"] == 1


def test_flash_ticks_decrease_over_time():
    ona = ONAGateway(forward_fn=lambda msg: True)
    ona.ingest(make_packet())
    assert ona.flash_ticks > 0
    initial = ona.flash_ticks
    ona.tick()
    assert ona.flash_ticks == initial - 1
