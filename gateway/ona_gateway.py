"""
gateway/ona_gateway.py
======================
Outside Network Area gateway. [IMPLEMENTED] logic, [SIMULATED] links.

Responsibilities (and ONLY these — the ONA must never become a second
Command Post):

  RECEIVE    radio frames from the disconnected zone (via the beacon chain)
  VALIDATE   CRC-8, frame type, duplicate suppression, TTL
  TRANSLATE  robot-local (x, y) -> real-world GPS (common/coordinates.py)
  CARRY      buffer, then forward to the Command Post over the uplink callbacks
  BRIEF      hold a mission brief for an Executor until it polls for it

It does NOT decide what a mission is, which Executor gets it, or what the
priority is — that is command_post/ (tests/test_ona_boundary.py enforces the
import boundary). In the Phase-0 prototype the ONA auto-generated and
auto-approved missions; that is gone.

Two entry points:
  ingest(raw18)        legacy: one bare 18-byte BeaconMessage (kept for its tests)
  receive_frame(raw)   the real path: a MeshFrame that arrived over the radio

[PLANNED] persistent on-disk buffer, real LoRa driver, real Wi-Fi/HTTP uplink.
"""
from __future__ import annotations

import logging
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from common.coordinates import robot_to_gps
from common.protocol import (
    BROADCAST_ID, EXEC_STATE_CODES, FrameType, HOPS_UNKNOWN, ONA_NODE_ID, BeaconMessage,
    DecodeError, ExecStatus, Heartbeat, MemoryExt, MeshFrame, decode_ack, decode_heartbeat,
    decode_memory_payload, decode_packet, decode_status, encode_ack, encode_heartbeat,
)

log = logging.getLogger("ona_gateway")

DEFAULT_REF_GPS     = (36.8065, 10.1815)  # [ASSUMPTION: not yet field-measured]
DEFAULT_REF_HEADING = 0.0


@dataclass
class UplinkMemory:
    """A validated, GPS-enriched memory record on its way to the Command Post."""
    msg: BeaconMessage
    ext: Optional[MemoryExt]
    hop_count: int
    origin: int
    received_tick: int
    received_ts: int


@dataclass
class UplinkHeartbeat:
    hb: Heartbeat
    gps: Tuple[float, float]
    hop_count: int
    received_tick: int


@dataclass
class UplinkStatus:
    status: ExecStatus
    hop_count: int
    received_tick: int
    seq: int = 0          # per-origin frame sequence: lets the Command Post drop out-of-order reports


class ONAGateway:
    def __init__(self, ref_gps=DEFAULT_REF_GPS, ref_heading: float = DEFAULT_REF_HEADING,
                 forward_fn: Optional[Callable[[BeaconMessage], bool]] = None,
                 memory_fn: Optional[Callable[[UplinkMemory], bool]] = None,
                 heartbeat_fn: Optional[Callable[[UplinkHeartbeat], bool]] = None,
                 status_fn: Optional[Callable[[UplinkStatus], bool]] = None,
                 now_ts: Optional[Callable[[], int]] = None,
                 advert_period: int = 10):
        self.ref_gps = ref_gps
        self.ref_heading = ref_heading
        self._forward = forward_fn or (lambda _msg: True)
        self._memory_fn = memory_fn
        self._heartbeat_fn = heartbeat_fn
        self._status_fn = status_fn
        self._now_ts = now_ts or (lambda: 0)
        self._buffer: List[BeaconMessage] = []                  # legacy buffer
        self._uplink_buffer: List[Tuple[str, Any]] = []         # mesh buffer
        self.stats = dict(received=0, decoded=0, rejected=0, forwarded=0, buffered=0)
        self.mesh_stats = dict(frames=0, crc_rejected=0, duplicates=0, ttl_expired=0,
                               memory=0, heartbeats=0, status=0, acks_sent=0, adverts=0)
        self.flash_ticks = 0   # renderer hook: >0 for a few ticks after activity
        self.advert_period = advert_period
        self._port = None
        self._seen: "OrderedDict[Tuple[int, int], None]" = OrderedDict()
        self._adv_seq = 0
        self._mailbox: Dict[str, Any] = {}      # executor_id -> brief (opaque to the ONA)
        self.briefs_queued = 0
        self.briefs_delivered = 0
        self.tick_now = 0
        self.recent: List[UplinkMemory] = []    # latest frame per memory id, for the ONA panel
        self.seen_count: Dict[int, int] = {}    # memory id -> how many times it was (re)received
        self.beacon_table: Dict[int, UplinkHeartbeat] = {}

    # ── radio attachment ─────────────────────────────────────────────────────
    def attach_radio(self, port) -> None:
        """Bind to a RadioPort (communication/mesh.py). The ONA node id is 0."""
        self._port = port
        port.subscribe(self._on_raw)

    def _on_raw(self, raw: bytes) -> None:
        self.receive_frame(raw)

    # ── legacy single-packet path ────────────────────────────────────────────
    def ingest(self, raw: bytes) -> Optional[BeaconMessage]:
        """Process one bare 18-byte packet. Returns the decoded message, or None if rejected."""
        self.stats["received"] += 1
        self.flash_ticks = 20
        try:
            msg = decode_packet(raw)
        except DecodeError as exc:
            log.warning("ONA: rejected packet — %s", exc)
            self.stats["rejected"] += 1
            return None
        self.stats["decoded"] += 1
        lat, lon = robot_to_gps(msg.x_local, msg.y_local, self.ref_gps[0], self.ref_gps[1], self.ref_heading)
        msg.gps_lat, msg.gps_lon = lat, lon
        if self._forward(msg):
            self.stats["forwarded"] += 1
        else:
            self._buffer.append(msg)
            self.stats["buffered"] += 1
            log.warning("ONA: downstream unavailable, buffered beacon #%d", msg.beacon_id)
        return msg

    # ── mesh path ────────────────────────────────────────────────────────────
    def to_gps(self, x_local: float, y_local: float) -> Tuple[float, float]:
        return robot_to_gps(x_local, y_local, self.ref_gps[0], self.ref_gps[1], self.ref_heading)

    def receive_frame(self, raw: bytes) -> bool:
        """Validate -> dedupe -> decode -> translate -> forward/buffer. True if accepted."""
        self.mesh_stats["frames"] += 1
        try:
            f = MeshFrame.decode(raw)
        except DecodeError:
            self.mesh_stats["crc_rejected"] += 1
            self.stats["rejected"] += 1
            return False
        if f.ftype == FrameType.HELLO:
            if self._port is not None:
                self._adv_seq = (self._adv_seq + 1) & 0xFFFF
                hb = Heartbeat(ONA_NODE_ID, 0.0, 0.0, 0, BROADCAST_ID, 100, 0, 0, 0)
                self._port.send(MeshFrame(FrameType.HEARTBEAT, ONA_NODE_ID, self._adv_seq, 1, 0, ONA_NODE_ID,
                                          f.link_src, 0, encode_heartbeat(hb)).encode())
            return False
        if f.ftype == FrameType.ACK or f.ftype == FrameType.PROGRAM:
            return False
        if f.link_dst != ONA_NODE_ID:
            return False                         # overheard, not addressed to the ONA
        self.flash_ticks = 20
        self._ack(f)
        if f.packet_id in self._seen:
            self.mesh_stats["duplicates"] += 1
            return False
        self._seen[f.packet_id] = None
        while len(self._seen) > 512:
            self._seen.popitem(last=False)
        if f.ttl < 0:
            self.mesh_stats["ttl_expired"] += 1
            return False
        try:
            if f.ftype == FrameType.MEMORY:
                msg, ext = decode_memory_payload(f.payload)
                msg.gps_lat, msg.gps_lon = self.to_gps(msg.x_local, msg.y_local)
                item = UplinkMemory(msg, ext, f.hop_count, f.origin, self.tick_now, self._now_ts())
                self.mesh_stats["memory"] += 1
                self.stats["decoded"] += 1
                mid = ext.memory_id
                self.seen_count[mid] = self.seen_count.get(mid, 0) + 1
                self.recent = [r for r in self.recent if r.ext is None or r.ext.memory_id != mid] + [item]
                self.recent = self.recent[-24:]
                self._deliver("memory", item)
            elif f.ftype == FrameType.HEARTBEAT:
                hb = decode_heartbeat(f.payload)
                item = UplinkHeartbeat(hb, self.to_gps(hb.x_local, hb.y_local), f.hop_count, self.tick_now)
                self.mesh_stats["heartbeats"] += 1
                self.beacon_table[hb.beacon_id] = item
                self._deliver("heartbeat", item)
            elif f.ftype == FrameType.STATUS:
                st = decode_status(f.payload, EXEC_STATE_CODES)
                self.mesh_stats["status"] += 1
                self._deliver("status", UplinkStatus(st, f.hop_count, self.tick_now, f.seq))
        except DecodeError:
            self.mesh_stats["crc_rejected"] += 1
            self.stats["rejected"] += 1
            return False
        return True

    def _ack(self, f: MeshFrame) -> None:
        if self._port is None:
            return
        ack = MeshFrame(FrameType.ACK, ONA_NODE_ID, 0, 1, 0, ONA_NODE_ID, f.link_src, 0,
                        encode_ack(f.origin, f.seq))
        self._port.send(ack.encode())
        self.mesh_stats["acks_sent"] += 1

    def _deliver(self, kind: str, item: Any) -> None:
        fn = {"memory": self._memory_fn, "heartbeat": self._heartbeat_fn, "status": self._status_fn}[kind]
        if kind == "memory" and fn is None:
            # no mesh-aware consumer: fall back to the legacy BeaconMessage callback
            if self._forward(item.msg):
                self.stats["forwarded"] += 1
            else:
                self._buffer.append(item.msg)
                self.stats["buffered"] += 1
            return
        if fn is None:
            return
        if fn(item):
            if kind == "memory":
                self.stats["forwarded"] += 1
        else:
            self._uplink_buffer.append((kind, item))
            if kind == "memory":
                self.stats["buffered"] += 1

    # ── buffers ──────────────────────────────────────────────────────────────
    def flush_buffer(self) -> int:
        """Retry buffered messages. Returns how many are still buffered."""
        still: List[BeaconMessage] = []
        for msg in self._buffer:
            if self._forward(msg):
                self.stats["forwarded"] += 1
            else:
                still.append(msg)
        flushed = len(self._buffer) - len(still)
        self._buffer = still
        if flushed:
            log.info("ONA: flushed %d buffered message(s)", flushed)
        return len(self._buffer)

    def flush_uplink(self) -> int:
        pending, self._uplink_buffer = self._uplink_buffer, []
        for kind, item in pending:
            self._deliver(kind, item)
        return len(self._uplink_buffer)

    @property
    def buffered_count(self) -> int:
        return len(self._buffer) + len(self._uplink_buffer)

    # ── downlink: mission briefs (carried, never composed, by the ONA) ───────
    def queue_brief(self, executor_id: str, brief: Any) -> None:
        self._mailbox[executor_id] = brief
        self.briefs_queued += 1

    def poll_brief(self, executor_id: str) -> Optional[Any]:
        brief = self._mailbox.pop(executor_id, None)
        if brief is not None:
            self.briefs_delivered += 1
        return brief

    def pending_brief_for(self, executor_id: str) -> bool:
        return executor_id in self._mailbox

    # ── clock ────────────────────────────────────────────────────────────────
    def tick(self, t: Optional[int] = None) -> None:
        if self.flash_ticks > 0:
            self.flash_ticks -= 1
        if t is None:
            return
        self.tick_now = t
        if self._port is not None and self.advert_period and t % self.advert_period == 0:
            self._adv_seq = (self._adv_seq + 1) & 0xFFFF
            hb = Heartbeat(ONA_NODE_ID, 0.0, 0.0, 0, BROADCAST_ID, 100, 0, 0, 0)
            adv = MeshFrame(FrameType.HEARTBEAT, ONA_NODE_ID, self._adv_seq, 1, 0, ONA_NODE_ID,
                            BROADCAST_ID, 0, encode_heartbeat(hb))
            self._port.send(adv.encode())
            self.mesh_stats["adverts"] += 1
        if self._uplink_buffer:
            self.flush_uplink()
        if self._buffer:
            self.flush_buffer()

    def status(self) -> str:
        lines = ["=== ONA Gateway ===", *(f"  {k}: {v}" for k, v in self.stats.items()),
                 *(f"  mesh.{k}: {v}" for k, v in self.mesh_stats.items())]
        return "\n".join(lines)
