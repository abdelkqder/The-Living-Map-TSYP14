"""
common/protocol.py
==================
Single authoritative beacon wire-format implementation.
Every other module imports BeaconMessage from here. Do NOT define another
BeaconMessage, another wire format, or another EVENT_TYPES table elsewhere
in the codebase — that duplication is exactly what broke the Phase-0
prototype (see docs/architecture/system-architecture.md, "known history").

Wire format  (18 bytes, little-endian)
──────────────────────────────────────
  Byte(s)  Type     Field
  0        uint8    beacon_id
  1        uint8    event_type_id  (see EVENT_TYPES)
  2-5      float32  x_local        (metres east of zone entry)
  6-9      float32  y_local        (metres north of zone entry)
  10-13    uint32   timestamp      (unix seconds, deployment time)
  14       uint8    severity       (1=low … 4=critical)
  15       uint8    initial_confidence  (0-100, at deployment)
  16       uint8    battery_pct    (0-100)
  17       uint8    CRC-8/MAXIM    (over bytes 0-16)

Status: [IMPLEMENTED]

Note on scope: this module defines the PHYSICAL packet only — what would
actually cross the radio link. Anything that shouldn't cost radio bytes
(verification state, version history, mission relevance, ...) belongs in
common/memory.py's MemoryRecord instead. Keep that separation; see the
project vision doc, "physical beacon data vs ONA/command-post metadata".
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

from common.enums import EventType, ExecutorState, MemoryState, PassageState

# ── Aging parameters ──────────────────────────────────────────────────────────
DECAY_LAMBDA    = 0.002   # confidence decay rate (per second)   [ASSUMPTION]
MIN_CONFIDENCE  = 0.05    # floor — beacon never fully "expires"
STALE_THRESHOLD = 0.25    # below this live_confidence -> beacon is stale

# ── Event type registry (wire-level uint8 ids) ────────────────────────────────
# NOTE: "VICTIM_PRESENCE" denotes a human-presence/motion detection only.
#       It does NOT imply confirmed victim recognition.
EVENT_TYPES: Dict[int, str] = {
    1: EventType.VICTIM_PRESENCE.value,
    2: EventType.FIRE.value,
    3: EventType.GAS.value,
    4: EventType.STRUCTURAL.value,
    5: EventType.BLOCKAGE.value,     # PHASE-1 PATCH: navigation-relevant memory
}
EVENT_IDS: Dict[str, int] = {v: k for k, v in EVENT_TYPES.items()}

# ── Wire format ───────────────────────────────────────────────────────────────
_PAYLOAD_FMT  = "<BBffIBBB"                          # 17 bytes
PAYLOAD_SIZE  = struct.calcsize(_PAYLOAD_FMT)         # must be 17
PACKET_SIZE   = PAYLOAD_SIZE + 1                      # +1 CRC byte = 18

assert PAYLOAD_SIZE == 17, f"Unexpected payload size: {PAYLOAD_SIZE}"


# ── CRC-8/MAXIM (1-Wire, polynomial 0x31) ────────────────────────────────────
def _crc8(data: bytes) -> int:
    """
    CRC-8/MAXIM, also known as DOW CRC. Polynomial: 0x31 (x^8+x^5+x^4+1).
    Detects all single-byte errors in the packet payload. [IMPLEMENTED]
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# ── BeaconMessage ─────────────────────────────────────────────────────────────
@dataclass
class BeaconMessage:
    """
    PATCH 7 note: `version` is a Python-level sequence counter — it is NOT
    yet wire-encoded (Phase 1 keeps the 18-byte format unchanged). Phase 2
    firmware will insert it as a uint8 before the CRC byte, making 19 bytes.
    Default = 1. Higher version always wins in LivingMap.ingest().
    """
    """
    The exact bytes a physical beacon would broadcast over LoRa.

    Confidence aging
    ────────────────
    `initial_confidence` is the value at deployment (stored in packet, 0-100).
    `live_confidence` is derived from initial_confidence and elapsed time.
    Aging is computed exactly ONCE, via the `live_confidence` property.

    GPS coordinates
    ───────────────
    `gps_lat` / `gps_lon` are NOT in the wire packet. They are added by the
    ONA gateway after coordinate transformation (common/coordinates.py).

    [IMPLEMENTED]
    """

    beacon_id:           int    # uint8, unique per deployment session
    event_type:          str    # must be a key of EVENT_IDS
    x_local:             float  # metres east  of zone entry [SIMULATED position]
    y_local:             float  # metres north of zone entry [SIMULATED position]
    timestamp:           int    # unix seconds, set at deployment time
    severity:            int    # 1=low, 2=medium, 3=high, 4=critical
    initial_confidence:  int    # 0-100 at deployment (stored in packet)
    battery_pct:         int    # 0-100
    version:             int = 1  # PATCH 7 — not wire-encoded in Phase 1

    # Not in wire packet — added by ONA gateway [SIMULATED]
    gps_lat: Optional[float] = None
    gps_lon: Optional[float] = None

    # ── Aging ─────────────────────────────────────────────────────────────────
    @property
    def age_seconds(self) -> float:
        """Seconds since deployment. Always >= 0."""
        return max(0.0, time.time() - self.timestamp)

    @property
    def live_confidence(self) -> float:
        """
        conf(t) = (initial_confidence / 100) * e^(-DECAY_LAMBDA * age_seconds)
        Applied exactly once here. [IMPLEMENTED]
        """
        decayed = (self.initial_confidence / 100.0) * math.exp(
            -DECAY_LAMBDA * self.age_seconds
        )
        return max(MIN_CONFIDENCE, decayed)

    @property
    def confidence_pct(self) -> int:
        return round(self.live_confidence * 100)

    @property
    def is_stale(self) -> bool:
        return self.live_confidence < STALE_THRESHOLD

    # ── Encoding ──────────────────────────────────────────────────────────────
    def encode(self) -> bytes:
        """Encode to PACKET_SIZE-byte wire format with CRC-8 trailer. [IMPLEMENTED]"""
        if self.event_type not in EVENT_IDS:
            raise ValueError(f"Unknown event_type: {self.event_type!r}")

        payload = struct.pack(
            _PAYLOAD_FMT,
            self.beacon_id             & 0xFF,
            EVENT_IDS[self.event_type] & 0xFF,
            float(self.x_local),
            float(self.y_local),
            int(self.timestamp)        & 0xFFFFFFFF,
            max(1, min(4, self.severity))              & 0xFF,
            max(0, min(100, self.initial_confidence))  & 0xFF,
            max(0, min(100, self.battery_pct))         & 0xFF,
        )
        assert len(payload) == PAYLOAD_SIZE
        return payload + bytes([_crc8(payload)])

    # ── Serialisation ─────────────────────────────────────────────────────────
    def to_dict(self) -> dict:
        """JSON-serialisable representation. [IMPLEMENTED]"""
        return {
            "beacon_id":           self.beacon_id,
            "event_type":          self.event_type,
            "x_local":             round(self.x_local, 3),
            "y_local":             round(self.y_local, 3),
            "timestamp":           self.timestamp,
            "age_s":               round(self.age_seconds, 1),
            "severity":            self.severity,
            "initial_confidence":  self.initial_confidence,
            "live_confidence_pct": self.confidence_pct,
            "is_stale":            self.is_stale,
            "battery_pct":         self.battery_pct,
            "gps_lat":             self.gps_lat,
            "gps_lon":             self.gps_lon,
        }

    def __str__(self) -> str:
        return (
            f"Beacon#{self.beacon_id:02d}[{self.event_type}] "
            f"pos=({self.x_local:.2f}m, {self.y_local:.2f}m) "
            f"sev={self.severity} conf={self.confidence_pct}% "
            f"age={self.age_seconds:.0f}s bat={self.battery_pct}%"
        )


# ── Decode ────────────────────────────────────────────────────────────────────
class DecodeError(ValueError):
    """Raised when a raw packet cannot be decoded (wrong length or bad CRC)."""


def decode_packet(raw: bytes) -> BeaconMessage:
    """
    Decode raw bytes into a BeaconMessage. Validates packet length, CRC-8
    integrity, and event-type-id. Raises DecodeError on any failure.
    [IMPLEMENTED]
    """
    if len(raw) < PACKET_SIZE:
        raise DecodeError(f"Packet too short: got {len(raw)}, expected {PACKET_SIZE}")

    payload  = raw[:PAYLOAD_SIZE]
    got_crc  = raw[PAYLOAD_SIZE]
    want_crc = _crc8(payload)

    if got_crc != want_crc:
        raise DecodeError(f"CRC mismatch: got 0x{got_crc:02X}, expected 0x{want_crc:02X}")

    bid, eid, x, y, ts, sev, conf, bat = struct.unpack(_PAYLOAD_FMT, payload)

    if eid not in EVENT_TYPES:
        raise DecodeError(f"Unknown event_type_id: {eid}")

    return BeaconMessage(
        beacon_id=bid, event_type=EVENT_TYPES[eid], x_local=x, y_local=y,
        timestamp=ts, severity=sev, initial_confidence=conf, battery_pct=bat,
    )


# ── Registry ──────────────────────────────────────────────────────────────────
class BeaconRegistry:
    """In-memory store of received beacon messages, keyed by beacon_id. [IMPLEMENTED]"""

    def __init__(self) -> None:
        self._store: Dict[int, BeaconMessage] = {}

    def add(self, msg: BeaconMessage) -> None:
        self._store[msg.beacon_id] = msg

    def get(self, beacon_id: int) -> Optional[BeaconMessage]:
        return self._store.get(beacon_id)

    def all(self) -> List[BeaconMessage]:
        return list(self._store.values())

    def active(self) -> List[BeaconMessage]:
        """Non-stale beacons sorted by severity (desc), then age (asc)."""
        return sorted(
            [m for m in self._store.values() if not m.is_stale],
            key=lambda m: (-m.severity, m.age_seconds),
        )

    def __len__(self) -> int:
        return len(self._store)

    def __contains__(self, beacon_id: int) -> bool:
        return beacon_id in self._store


# ═════════════════════════════════════════════════════════════════════════════
# PHASE-1 PATCH — MESH / RELAY LAYER
# ═════════════════════════════════════════════════════════════════════════════
# The 18-byte BeaconMessage above is UNCHANGED and remains the source of truth
# for "what a beacon says". What the patch adds is an ENVELOPE (MeshFrame) so
# that a message can be relayed hop-by-hop from a deep beacon to the ONA:
#
#   MeshFrame = link header (who sent this hop, who should take it, sender's
#               gradient)  +  end-to-end header (origin, seq, ttl, hop count)
#               +  payload  +  CRC-8
#
#   CRC-8  detects corruption on ONE hop (receiver drops the frame).
#   ACK / retry / forwarding / store-and-forward (in communication/mesh.py and
#   beacon/memory_node.py) are what actually DELIVER data. A valid CRC says
#   nothing about whether a frame arrives. [SIMULATED] — no real RF was measured.
# ═════════════════════════════════════════════════════════════════════════════

ONA_NODE_ID   = 0
BROADCAST_ID  = 0xFF
HOPS_UNKNOWN  = 0xFF
DEFAULT_TTL   = 12


class FrameType(IntEnum):
    MEMORY    = 1   # a memory record (BeaconMessage + MemoryExt), beacon -> ONA
    HEARTBEAT = 2   # beacon liveness + position + gradient, beacon -> ONA
    STATUS    = 3   # executor state / mission outcome, robot -> ONA
    PROGRAM   = 4   # robot -> beacon over the local link (write/update a record)
    ACK       = 5   # link-layer acknowledgement of ONE hop (payload = origin, seq)
    HELLO     = 6   # robot probe: "who can I hear?" — beacons / ONA answer with their gradient


class MissionOutcome(IntEnum):
    NONE = 0
    VERIFIED = 1
    CONTRADICTED = 2
    CLEARED = 3
    ESCALATED = 4
    BLOCKED = 5              # target unreachable
    FAILED = 6               # executor fault
    ABORTED_LOW_BATTERY = 7
    RETURNED_NO_TARGET = 8   # search mission found nothing
    UNREACHABLE = 9          # target unreachable, NO clearance requested


_STATE_ORDER: List[MemoryState] = [
    MemoryState.UNVERIFIED, MemoryState.VERIFIED, MemoryState.CONTRADICTED,
    MemoryState.ACTIVE, MemoryState.ESCALATED, MemoryState.CLEARED,
]
_PASSAGE_ORDER: List[PassageState] = [
    PassageState.OPEN, PassageState.BLOCKED, PassageState.IMPASSABLE,
    PassageState.TEMPORARILY_BLOCKED, PassageState.DETOUR_REQUIRED,
    PassageState.ACCESSIBILITY_UNKNOWN,
]
_NO_PASSAGE = 0xFF

_HDR_FMT   = "<BBHBBBBBB"                 # type origin seq ttl hops link_src link_dst link_hops len
_HDR_SIZE  = struct.calcsize(_HDR_FMT)    # 10
_MEMEXT_FMT  = "<HBBBIB"                  # memory_id version state passage last_update_ts source_node
_HB_FMT      = "<BffBBBIBB"               # id x y hops parent battery last_update_ts n_records max_version
_STATUS_FMT  = "<4sBBHB"                  # executor id, state-code, battery, memory_id, outcome
_PROG_FMT    = "<BffB"                    # beacon_id x y flags
_ACK_FMT     = "<BH"                      # origin, seq of the acknowledged packet


@dataclass
class MeshFrame:
    """One radio frame of the relay layer. [IMPLEMENTED][SIMULATED link]"""
    ftype:     FrameType
    origin:    int            # node that CREATED the frame
    seq:       int            # per-origin sequence -> (origin, seq) is the packet identity
    ttl:       int
    hop_count: int
    link_src:  int            # node that transmitted THIS hop
    link_dst:  int            # intended next hop (BROADCAST_ID = anyone)
    link_hops: int            # link_src's current hops-to-ONA (gradient), HOPS_UNKNOWN if none
    payload:   bytes = b""

    @property
    def packet_id(self) -> Tuple[int, int]:
        return (self.origin, self.seq)

    def encode(self) -> bytes:
        if len(self.payload) > 255:
            raise ValueError("payload too long for one frame")
        head = struct.pack(_HDR_FMT, int(self.ftype), self.origin & 0xFF, self.seq & 0xFFFF,
                           self.ttl & 0xFF, self.hop_count & 0xFF, self.link_src & 0xFF,
                           self.link_dst & 0xFF, self.link_hops & 0xFF, len(self.payload))
        body = head + self.payload
        return body + bytes([_crc8(body)])

    @staticmethod
    def decode(raw: bytes) -> "MeshFrame":
        """Validate length + CRC-8 + frame type. Raises DecodeError on any failure."""
        if len(raw) < _HDR_SIZE + 1:
            raise DecodeError(f"frame too short: {len(raw)}")
        body, got = raw[:-1], raw[-1]
        if _crc8(body) != got:
            raise DecodeError("frame CRC mismatch")
        t, origin, seq, ttl, hops, lsrc, ldst, lhops, plen = struct.unpack(_HDR_FMT, body[:_HDR_SIZE])
        if len(body) - _HDR_SIZE != plen:
            raise DecodeError("frame length field mismatch")
        try:
            ftype = FrameType(t)
        except ValueError as exc:
            raise DecodeError(f"unknown frame type {t}") from exc
        return MeshFrame(ftype, origin, seq, ttl, hops, lsrc, ldst, lhops, body[_HDR_SIZE:])

    def for_next_hop(self, link_src: int, link_dst: int, link_hops: int) -> "MeshFrame":
        """Copy to transmit on the next hop: TTL-1, hop_count+1, new link header."""
        return MeshFrame(self.ftype, self.origin, self.seq, self.ttl - 1, self.hop_count + 1,
                         link_src, link_dst, link_hops, self.payload)


@dataclass
class MemoryExt:
    """Memory-layer fields that ride next to the 18-byte BeaconMessage."""
    memory_id:      int
    version:        int
    state:          MemoryState = MemoryState.UNVERIFIED
    passage:        Optional[PassageState] = None
    last_update_ts: int = 0
    source:         int = 0          # node id of the robot that made this observation


def encode_memory_payload(msg: BeaconMessage, ext: MemoryExt) -> bytes:
    p_code = _NO_PASSAGE if ext.passage is None else _PASSAGE_ORDER.index(ext.passage)
    tail = struct.pack(_MEMEXT_FMT, ext.memory_id & 0xFFFF, ext.version & 0xFF,
                       _STATE_ORDER.index(ext.state), p_code, ext.last_update_ts & 0xFFFFFFFF, ext.source & 0xFF)
    return msg.encode() + tail


MEMORY_PAYLOAD_SIZE = PACKET_SIZE + struct.calcsize(_MEMEXT_FMT)


def decode_memory_payload(raw: bytes) -> Tuple[BeaconMessage, MemoryExt]:
    if len(raw) < MEMORY_PAYLOAD_SIZE:
        raise DecodeError("memory payload too short")
    msg = decode_packet(raw[:PACKET_SIZE])
    mid, ver, st, pc, ts, src = struct.unpack(_MEMEXT_FMT, raw[PACKET_SIZE:MEMORY_PAYLOAD_SIZE])
    if st >= len(_STATE_ORDER):
        raise DecodeError(f"bad state code {st}")
    passage = None if pc == _NO_PASSAGE else (_PASSAGE_ORDER[pc] if pc < len(_PASSAGE_ORDER) else None)
    msg.version = ver
    return msg, MemoryExt(mid, ver, _STATE_ORDER[st], passage, ts, src)


@dataclass
class Heartbeat:
    beacon_id:      int
    x_local:        float      # where this beacon sits (programmed by the robot that dropped it)
    y_local:        float
    hops_to_ona:    int        # HOPS_UNKNOWN when no uplink
    parent_id:      int        # next hop toward the ONA (BROADCAST_ID if none)
    battery_pct:    int
    last_update_ts: int        # last memory update held by the beacon
    n_records:      int
    max_version:    int


def encode_heartbeat(h: Heartbeat) -> bytes:
    return struct.pack(_HB_FMT, h.beacon_id & 0xFF, h.x_local, h.y_local, h.hops_to_ona & 0xFF,
                       h.parent_id & 0xFF, max(0, min(100, h.battery_pct)),
                       h.last_update_ts & 0xFFFFFFFF, h.n_records & 0xFF, h.max_version & 0xFF)


def decode_heartbeat(raw: bytes) -> Heartbeat:
    if len(raw) < struct.calcsize(_HB_FMT):
        raise DecodeError("heartbeat payload too short")
    return Heartbeat(*struct.unpack(_HB_FMT, raw[:struct.calcsize(_HB_FMT)]))


@dataclass
class ExecStatus:
    executor_id: str
    state:       str          # ExecutorState value
    battery_pct: int = 100
    memory_id:   int = 0      # mission memory the status refers to (0 = none)
    outcome:     MissionOutcome = MissionOutcome.NONE


def encode_status(st: ExecStatus, state_codes: List[str]) -> bytes:
    return struct.pack(_STATUS_FMT, st.executor_id.encode()[:4].ljust(4, b"\0"),
                       state_codes.index(st.state), max(0, min(100, st.battery_pct)),
                       st.memory_id & 0xFFFF, int(st.outcome))


def decode_status(raw: bytes, state_codes: List[str]) -> ExecStatus:
    if len(raw) < struct.calcsize(_STATUS_FMT):
        raise DecodeError("status payload too short")
    eid, sc, bat, mid, oc = struct.unpack(_STATUS_FMT, raw[:struct.calcsize(_STATUS_FMT)])
    if sc >= len(state_codes):
        raise DecodeError("bad status state code")
    try:
        outcome = MissionOutcome(oc)
    except ValueError as exc:
        raise DecodeError("bad outcome code") from exc
    return ExecStatus(eid.rstrip(b"\0").decode(), state_codes[sc], bat, mid, outcome)


@dataclass
class ProgramCommand:
    """Robot -> beacon (local link): set the beacon's position and write/update one record."""
    beacon_id: int
    x_local:   float
    y_local:   float
    record:    Optional[Tuple[BeaconMessage, MemoryExt]] = None
    set_position: bool = True        # False when updating an existing beacon whose position is already set


def encode_program(cmd: ProgramCommand) -> bytes:
    flags = (1 if cmd.record is not None else 0) | (2 if cmd.set_position else 0)
    out = struct.pack(_PROG_FMT, cmd.beacon_id & 0xFF, cmd.x_local, cmd.y_local, flags)
    if cmd.record is not None:
        out += encode_memory_payload(*cmd.record)
    return out


def decode_program(raw: bytes) -> ProgramCommand:
    n = struct.calcsize(_PROG_FMT)
    if len(raw) < n:
        raise DecodeError("program payload too short")
    bid, x, y, flags = struct.unpack(_PROG_FMT, raw[:n])
    rec = decode_memory_payload(raw[n:]) if flags & 1 else None
    return ProgramCommand(bid, x, y, rec, bool(flags & 2))


def encode_ack(origin: int, seq: int) -> bytes:
    return struct.pack(_ACK_FMT, origin & 0xFF, seq & 0xFFFF)


def decode_ack(raw: bytes) -> Tuple[int, int]:
    n = struct.calcsize(_ACK_FMT)
    if len(raw) < n:
        raise DecodeError("ack payload too short")
    return struct.unpack(_ACK_FMT, raw[:n])


# Node-id plan (8-bit, so the frame header stays small) [ASSUMPTION]:
#   0 = ONA, 1..63 = beacons, 64..79 = Writers, 128..191 = Executors, 255 = broadcast
def writer_node_id(index: int) -> int:
    return 64 + (index % 16)


def executor_node_id(index: int) -> int:
    return 128 + (index % 64)


def memory_id_for(robot_index: int, counter: int) -> int:
    """16-bit memory id = robot prefix + per-robot counter, so two robots can
    mint ids without coordinating (and a replacement Writer never collides)."""
    return ((robot_index & 0xFF) << 8) | (counter & 0xFF)


EXEC_STATE_CODES: List[str] = [st.value for st in ExecutorState]   # aliases excluded


def node_label(node_id: int) -> str:
    """Human-readable name for a radio node id (logs / UI)."""
    if node_id == ONA_NODE_ID:
        return "ONA"
    if 64 <= node_id < 80:
        return f"W{node_id - 64}"
    if 128 <= node_id < 192:
        return f"E{node_id - 127}"
    return f"B{node_id}"
