import time

from beacon.memory_node import BeaconState, SimulatedBeacon
from common.protocol import BeaconMessage, decode_packet
from communication.mock_transport import MockTransport


def test_beacon_starts_stored():
    msg = BeaconMessage(1, "FIRE", 1.0, 2.0, int(time.time()), 3, 90, 80)
    b = SimulatedBeacon(msg, MockTransport("t"))
    assert b.state == BeaconState.STORED
    assert b.broadcast_count == 0


def test_broadcast_sends_correct_packet():
    msg = BeaconMessage(1, "FIRE", 1.0, 2.0, int(time.time()), 3, 90, 80)
    transport = MockTransport("t")
    received = []
    transport.subscribe(lambda raw: received.append(decode_packet(raw)))
    b = SimulatedBeacon(msg, transport)
    ok = b.broadcast()
    assert ok is True
    assert b.state == BeaconState.BROADCASTING
    assert b.broadcast_count == 1
    assert received[0].beacon_id == 1


def test_repeated_broadcast_increments_count():
    msg = BeaconMessage(1, "GAS", 0.0, 0.0, int(time.time()), 2, 70, 60)
    b = SimulatedBeacon(msg, MockTransport("t"))
    for _ in range(3):
        b.broadcast()
    assert b.broadcast_count == 3
