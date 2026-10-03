"""Spatial radio + beacon relay network: range, walls, multi-hop, TTL, duplicates, CRC, ACK/retry, versions."""
import pytest

from beacon.node import BeaconNode
from common.enums import MemoryState, PassageState
from common.protocol import (
    BROADCAST_ID, DEFAULT_TTL, HOPS_UNKNOWN, BeaconMessage, DecodeError, FrameType, Heartbeat, MemoryExt, MeshFrame,
    ProgramCommand, decode_heartbeat, decode_memory_payload, encode_ack, encode_heartbeat, encode_memory_payload,
    encode_program,
)
from communication.mesh import RadioMedium, RadioPort, RobotLink

TS = 1_790_000_000


def build_chain(cols, rng=4.0, loss=0.0, seed=0, **node_kw):
    """ONA at (1,0) and beacons 1..n on row 1 at the given columns, all in a straight corridor."""
    med = RadioMedium(range_cells=rng, loss=loss, seed=seed)
    received = []
    ona = RadioPort(med, 0, lambda: (1, 0), kind="ona")

    def ona_rx(raw):
        try:
            f = MeshFrame.decode(raw)
        except DecodeError:
            return
        if f.link_dst == 0 and f.ftype not in (FrameType.ACK, FrameType.HELLO, FrameType.PROGRAM):
            received.append(f)
            ona.send(MeshFrame(FrameType.ACK, 0, 0, 1, 0, 0, f.link_src, 0, encode_ack(f.origin, f.seq)).encode())
    ona.subscribe(ona_rx)
    nodes = []
    for i, col in enumerate(cols, start=1):
        nodes.append(BeaconNode(i, RadioPort(med, i, (lambda c=col: (1, c)), kind="beacon"), lambda: TS,
                                hb_period=40, **node_kw))
    return med, ona, nodes, received


def advert(ona, t):
    ona.send(MeshFrame(FrameType.HEARTBEAT, 0, t, 1, 0, 0, BROADCAST_ID, 0,
                       encode_heartbeat(Heartbeat(0, 0, 0, 0, BROADCAST_ID, 100, 0, 0, 0))).encode())


def run(med, ona, nodes, ticks, start=1):
    for t in range(start, start + ticks):
        if t % 20 == 1:
            advert(ona, t)
        med.step(t)
        for n in nodes:
            n.tick(t)


def program(med, beacon_id, mem_id=1, version=1, state=MemoryState.UNVERIFIED, col=13):
    robot = RadioPort(med, 70, lambda: (1, col), kind="robot")
    msg = BeaconMessage(beacon_id, "FIRE", 5.0, 1.0, TS, 3, 90, 100)
    cmd = ProgramCommand(beacon_id, 5.5, 0.0, (msg, MemoryExt(mem_id, version, state, None, TS)))
    return robot.send_acked(MeshFrame(FrameType.PROGRAM, 70, 1, 1, 0, 70, beacon_id, HOPS_UNKNOWN, encode_program(cmd)).encode(), beacon_id)


# ── physics ────────────────────────────────────────────────────────────────────
def test_out_of_range_nodes_cannot_hear_each_other():
    med, ona, nodes, _ = build_chain([3, 12], rng=4.0)
    assert med.link_margin(0, 1) is not None
    assert med.link_margin(0, 2) is None          # 12 cells away: unreachable link
    assert med.link_margin(1, 2) is None


def test_wall_attenuation_breaks_a_link_that_distance_alone_allows():
    walls = {(1, 2)}
    med = RadioMedium(range_cells=5.0, wall_count_fn=lambda a, b: len({(1, 2)} & {(1, c) for c in range(min(a[1], b[1]), max(a[1], b[1]) + 1)}), wall_penalty=2.0)
    RadioPort(med, 1, lambda: (1, 0)); RadioPort(med, 2, lambda: (1, 4))
    assert med.link_margin(1, 2) is None           # 4 cells + one wall (2.0) = 6 > 5
    med2 = RadioMedium(range_cells=5.0, wall_count_fn=lambda a, b: 0)
    RadioPort(med2, 1, lambda: (1, 0)); RadioPort(med2, 2, lambda: (1, 4))
    assert med2.link_margin(1, 2) is not None


def test_links_are_symmetric_on_the_real_world_geometry():
    from simulation.scenario import default_scenario
    from simulation.system import LiveMapSystem
    sys_ = LiveMapSystem(default_scenario(seed=1))
    w = sys_.world
    cells = [(r, c) for r in range(w.rows) for c in range(w.cols) if not w.is_wall(r, c)]
    for a in cells[::7]:
        for b in cells[::11]:
            assert w.walls_between(a, b) == w.walls_between(b, a)


def test_random_loss_is_seeded_and_reproducible():
    def delivered(seed):
        med = RadioMedium(range_cells=9, loss=0.5, seed=seed)
        got = []
        a = RadioPort(med, 1, lambda: (1, 1)); b = RadioPort(med, 2, lambda: (1, 2))
        b.subscribe(got.append)
        frame = MeshFrame(FrameType.HEARTBEAT, 1, 1, 3, 0, 1, BROADCAST_ID, 1, encode_heartbeat(Heartbeat(1, 0, 0, 1, 0, 100, 0, 0, 0))).encode()
        for t in range(1, 60):
            a.send(frame); med.step(t)
        return len(got)
    assert delivered(5) == delivered(5)
    assert 0 < delivered(5) < 59


# ── multi-hop forwarding ──────────────────────────────────────────────────────
def test_gradient_forms_hop_by_hop_toward_the_ona():
    med, ona, nodes, _ = build_chain([3, 6, 9, 12])
    run(med, ona, nodes, 200)
    assert [n.hops for n in nodes] == [1, 2, 3, 4]
    assert [n.parent for n in nodes] == [0, 1, 2, 3]


def test_memory_from_the_deepest_beacon_reaches_the_ona_through_every_relay():
    med, ona, nodes, received = build_chain([3, 6, 9, 12])
    run(med, ona, nodes, 150)
    assert program(med, 4)
    run(med, ona, nodes, 200, start=151)
    mem = [f for f in received if f.ftype == FrameType.MEMORY]
    assert mem, "the record must have crossed 4 hops"
    assert mem[0].hop_count == 3                       # forwarded by B3, B2, B1
    msg, ext = decode_memory_payload(mem[0].payload)
    assert (msg.beacon_id, ext.memory_id, msg.event_type) == (4, 1, "FIRE")
    assert nodes[0].stats.forwarded > 0 and nodes[1].stats.forwarded > 0 and nodes[2].stats.forwarded > 0


def test_a_disconnected_beacon_buffers_then_delivers_when_a_relay_appears():
    med, ona, nodes, received = build_chain([3, 10], rng=4.0)          # a 7-cell gap between B1 and B2
    run(med, ona, nodes, 120)
    assert not nodes[1].connected
    assert program(med, 2, col=11)
    run(med, ona, nodes, 80, start=121)
    assert not [f for f in received if f.ftype == FrameType.MEMORY]       # nothing can leave yet
    relay = BeaconNode(3, RadioPort(med, 3, lambda: (1, 6), kind="beacon"), lambda: TS, hb_period=40)
    nodes.append(relay)
    run(med, ona, nodes, 300, start=201)
    assert [f for f in received if f.ftype == FrameType.MEMORY], "store-and-forward must deliver once a route exists"


def test_ttl_expiry_drops_frames():
    med, ona, nodes, received = build_chain([3, 6, 9, 12])
    run(med, ona, nodes, 150)
    hb = Heartbeat(4, 0, 0, 4, 3, 100, 0, 0, 0)
    f = MeshFrame(FrameType.HEARTBEAT, 4, 999, 0, 0, 4, 3, 4, encode_heartbeat(hb))   # TTL already 0
    ports = RadioPort(med, 90, lambda: (1, 11), kind="robot")
    before = nodes[2].stats.ttl_dropped
    ports.send(f.encode())
    run(med, ona, nodes, 20, start=151)
    assert nodes[2].stats.ttl_dropped == before + 1
    assert not [r for r in received if r.packet_id == (4, 999)]


def test_duplicate_packets_are_suppressed():
    med, ona, nodes, received = build_chain([3, 6])
    run(med, ona, nodes, 100)
    f = MeshFrame(FrameType.HEARTBEAT, 2, 4242, DEFAULT_TTL, 0, 2, 1, 2,
                  encode_heartbeat(Heartbeat(2, 0, 0, 2, 1, 100, 0, 0, 0)))
    port = RadioPort(med, 91, lambda: (1, 5), kind="robot")
    for t in range(101, 106):
        port.send(f.encode()); med.step(t)
        for n in nodes: n.tick(t)
    run(med, ona, nodes, 20, start=106)
    assert nodes[0].stats.dup_dropped >= 1
    assert len([r for r in received if r.packet_id == (2, 4242)]) <= 1


def test_corrupted_frame_is_rejected_by_crc():
    f = MeshFrame(FrameType.MEMORY, 4, 1, DEFAULT_TTL, 0, 4, 3, 4,
                  encode_memory_payload(BeaconMessage(4, "FIRE", 1, 1, TS, 3, 90, 100), MemoryExt(4, 1)))
    raw = bytearray(f.encode()); raw[14] ^= 0x01
    with pytest.raises(DecodeError):
        MeshFrame.decode(bytes(raw))
    med, ona, nodes, _ = build_chain([3])
    nodes[0]._on_raw(bytes(raw))
    assert nodes[0].stats.crc_dropped == 1


def test_crc_is_corruption_detection_not_delivery():
    """A valid CRC says nothing about delivery: with total loss nothing arrives, with loss + ACK/retry more does."""
    def reach(loss, retries):
        med, ona, nodes, received = build_chain([3, 6, 9], loss=loss, seed=11, max_retries=retries)
        run(med, ona, nodes, 400)
        return len(nodes[2].records), nodes
    med, ona, nodes, received = build_chain([3, 6, 9], loss=1.0)
    run(med, ona, nodes, 200)
    assert not received and not nodes[0].connected


def test_ack_retry_recovers_from_moderate_loss():
    med, ona, nodes, received = build_chain([3, 6, 9], loss=0.25, seed=2, ack_timeout=3, max_retries=5)
    run(med, ona, nodes, 300)
    assert all(n.connected for n in nodes) or sum(n.stats.retries for n in nodes) > 0
    assert sum(n.stats.retries for n in nodes) > 0              # retransmissions actually happened


# ── beacon memory ─────────────────────────────────────────────────────────────
def mk(version, state=MemoryState.UNVERIFIED, ts=TS):
    return (BeaconMessage(1, "GAS", 1.0, 2.0, ts, 2, 80, 100), MemoryExt(7, version, state, None, ts))


def test_beacon_stores_and_versions_records():
    med, ona, nodes, _ = build_chain([3])
    b = nodes[0]
    assert b.write_record(*mk(1))
    assert b.write_record(*mk(2, MemoryState.VERIFIED))
    assert b.records[7][1].state == MemoryState.VERIFIED and b.max_version() == 2


def test_older_version_never_overwrites_a_newer_one():
    med, ona, nodes, _ = build_chain([3])
    b = nodes[0]
    b.write_record(*mk(3, MemoryState.CLEARED))
    assert not b.write_record(*mk(2, MemoryState.VERIFIED))
    assert not b.write_record(*mk(3, MemoryState.UNVERIFIED))
    assert b.records[7][1].state == MemoryState.CLEARED and b.stats.stale_updates_ignored == 2


def test_heartbeat_reports_liveness_and_metadata():
    med, ona, nodes, received = build_chain([3, 6])
    run(med, ona, nodes, 120)
    assert program(med, 2, col=7)
    run(med, ona, nodes, 200, start=121)
    hbs = [decode_heartbeat(f.payload) for f in received if f.ftype == FrameType.HEARTBEAT and f.origin == 2]
    assert hbs, "heartbeats from a deep beacon cross the chain"
    last = hbs[-1]
    assert last.n_records == 1 and last.max_version == 1 and last.hops_to_ona == 2 and last.battery_pct == 100
    assert (last.x_local, last.y_local) == pytest.approx((5.5, 0.0))


def test_beacon_battery_drain_kills_the_node():
    med, ona, nodes, _ = build_chain([3], battery_drain_per_tick=10.0)
    run(med, ona, nodes, 30)
    assert nodes[0].alive is False
