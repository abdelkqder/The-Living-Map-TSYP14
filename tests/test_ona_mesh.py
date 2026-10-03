"""ONA: validation, coordinate transformation, buffering, forwarding, mailbox."""
import pytest

from common.coordinates import robot_to_gps
from common.enums import MemoryState, PassageState
from common.protocol import (
    BROADCAST_ID, DEFAULT_TTL, EXEC_STATE_CODES, BeaconMessage, ExecStatus, FrameType, Heartbeat, MemoryExt, MeshFrame,
    MissionOutcome, encode_heartbeat, encode_memory_payload, encode_status,
)
from communication.mesh import RadioMedium, RadioPort
from gateway.ona_gateway import ONAGateway

TS = 1_790_000_000
REF = (36.8065, 10.1815)


def frame_for_ona(msg=None, ext=None, seq=1, ttl=DEFAULT_TTL, dst=0):
    msg = msg or BeaconMessage(4, "FIRE", 2.0, -3.0, TS, 3, 80, 100)
    ext = ext or MemoryExt(4, 1, MemoryState.UNVERIFIED, None, TS)
    return MeshFrame(FrameType.MEMORY, 4, seq, ttl, 2, 3, dst, 1, encode_memory_payload(msg, ext))


def test_ona_validates_crc():
    ona = ONAGateway()
    raw = bytearray(frame_for_ona().encode()); raw[12] ^= 0xFF
    assert ona.receive_frame(bytes(raw)) is False
    assert ona.mesh_stats["crc_rejected"] == 1


def test_ona_ignores_frames_not_addressed_to_it():
    got = []
    ona = ONAGateway(memory_fn=lambda u: got.append(u) or True)
    assert ona.receive_frame(frame_for_ona(dst=3).encode()) is False and not got


def test_ona_translates_local_to_global_with_reference_and_heading():
    got = []
    ona = ONAGateway(ref_gps=REF, ref_heading=90.0, memory_fn=lambda u: got.append(u) or True)
    ona.receive_frame(frame_for_ona().encode())
    up = got[0]
    assert (up.msg.gps_lat, up.msg.gps_lon) == pytest.approx(robot_to_gps(2.0, -3.0, *REF, 90.0), abs=1e-9)
    assert (up.msg.gps_lat, up.msg.gps_lon) != robot_to_gps(2.0, -3.0, *REF, 0.0)     # heading is applied
    assert up.hop_count == 2 and up.ext.memory_id == 4 and up.origin == 4


def test_ona_suppresses_duplicates_by_packet_identity():
    got = []
    ona = ONAGateway(memory_fn=lambda u: got.append(u) or True)
    raw = frame_for_ona(seq=7).encode()
    assert ona.receive_frame(raw) is True
    assert ona.receive_frame(raw) is False
    assert len(got) == 1 and ona.mesh_stats["duplicates"] == 1


def test_ona_buffers_when_the_uplink_is_down_and_flushes_later():
    up = {"ok": False}
    got = []
    ona = ONAGateway(memory_fn=lambda u: (got.append(u) or True) if up["ok"] else False)
    ona.receive_frame(frame_for_ona(seq=1).encode())
    ona.receive_frame(frame_for_ona(seq=2).encode())
    assert ona.buffered_count == 2 and not got
    up["ok"] = True
    ona.tick(1)
    assert len(got) == 2 and ona.buffered_count == 0


def test_ona_acknowledges_what_it_accepts():
    med = RadioMedium(range_cells=8)
    port = RadioPort(med, 0, lambda: (1, 1), kind="ona")
    ona = ONAGateway(memory_fn=lambda u: True)
    ona.attach_radio(port)
    acks = []
    other = RadioPort(med, 3, lambda: (1, 3), kind="beacon")
    other.subscribe(lambda raw: acks.append(MeshFrame.decode(raw)) if MeshFrame.decode(raw).ftype == FrameType.ACK else None)
    other.send(frame_for_ona().encode())
    med.step(1); med.step(2)
    assert any(a.ftype == FrameType.ACK and a.link_dst == 3 for a in acks)


def test_ona_decodes_status_and_heartbeat_and_forwards_them():
    st, hb = [], []
    ona = ONAGateway(status_fn=lambda u: st.append(u) or True, heartbeat_fn=lambda u: hb.append(u) or True)
    s = ExecStatus("E01", "WORKING", 80, 0x0101, MissionOutcome.NONE)
    from common.protocol import encode_status
    ona.receive_frame(MeshFrame(FrameType.STATUS, 130, 5, 8, 1, 3, 0, 1, encode_status(s, EXEC_STATE_CODES)).encode())
    ona.receive_frame(MeshFrame(FrameType.HEARTBEAT, 3, 6, 8, 1, 3, 0, 1, encode_heartbeat(Heartbeat(3, 2.0, 1.0, 1, 0, 90, TS, 1, 2))).encode())
    assert st[0].status.executor_id == "E01" and st[0].status.state == "WORKING"
    assert hb[0].hb.beacon_id == 3 and hb[0].gps == pytest.approx(robot_to_gps(2.0, 1.0, *REF, 0.0), abs=1e-6)


def test_ona_mailbox_carries_a_brief_without_looking_inside():
    ona = ONAGateway()
    brief = object()
    ona.queue_brief("E01", brief)
    assert ona.pending_brief_for("E01") and not ona.pending_brief_for("E02")
    assert ona.poll_brief("E02") is None
    assert ona.poll_brief("E01") is brief and ona.poll_brief("E01") is None


def test_legacy_ingest_path_still_works():
    got = []
    ona = ONAGateway(forward_fn=lambda m: got.append(m) or True)
    assert ona.ingest(BeaconMessage(1, "GAS", 1.0, 1.0, TS, 2, 70, 100).encode()) is not None
    assert got and got[0].gps_lat is not None
