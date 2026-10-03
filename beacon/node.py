"""
beacon/node.py
==============
BeaconNode — the deployed physical-memory beacon, as far as Phase 1 needs it.

A beacon is BOTH

  * a memory store   : holds a few MemoryRecords (what / where / when / version)
                       written by robots over the local link (PROGRAM frames), and
  * a relay node     : forwards other nodes' frames toward the ONA (multi-hop).

Routing = gradient (hop-count) routing toward the ONA:
  - the ONA advertises hops=0; every node overhears neighbours' frames, each of
    which carries the SENDER's hops, and takes  hops = min(neighbour hops) + 1,
    parent = that neighbour. No node needs a global map.
  - upstream frames (MEMORY / HEARTBEAT / STATUS) are unicast to the parent and
    forwarded hop by hop.

Delivery mechanisms (kept separate from corruption detection, on purpose):
  CRC-8            frame is corrupt -> receiver drops it         (common/protocol.py)
  hop-by-hop ACK   sender retries up to `max_retries` times      (this file)
  duplicate suppression by packet id (origin, seq)               (this file)
  TTL              a frame is dropped when its TTL hits zero     (this file)
  store-and-forward frames wait in a bounded queue while the
                   node has no parent; stored records are re-sent when a
                   parent appears and re-sent periodically ("refresh")  (this file)

[SIMULATED] radio; [IMPLEMENTED] protocol logic. The ATmega328P/LoRa firmware
that would run this on hardware is [PLANNED].
"""
from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Tuple

from common.enums import MemoryState
from common.protocol import (
    BROADCAST_ID, DEFAULT_TTL, HOPS_UNKNOWN, ONA_NODE_ID, BeaconMessage, DecodeError,
    FrameType, Heartbeat, MemoryExt, MeshFrame, decode_ack, decode_program, encode_ack,
    encode_heartbeat, encode_memory_payload,
)
from communication.mesh import RadioPort

Record = Tuple[BeaconMessage, MemoryExt]


@dataclass
class NodeStats:
    rx: int = 0
    crc_dropped: int = 0
    dup_dropped: int = 0
    ttl_dropped: int = 0
    forwarded: int = 0
    originated: int = 0
    retries: int = 0
    gave_up: int = 0
    queue_overflow: int = 0
    stored_updates: int = 0
    stale_updates_ignored: int = 0


@dataclass
class _Pending:
    frame: MeshFrame
    next_hop: int
    sent_tick: int
    tries: int = 0
    requeues: int = 0


class BeaconNode:
    def __init__(self, beacon_id: int, port: RadioPort, now_ts: Callable[[], int], *,
                 hb_period: int = 60, refresh_period: int = 400, ack_timeout: int = 4,
                 max_retries: int = 3, parent_timeout: int = 240, queue_limit: int = 16,
                 capacity: int = 4, seen_limit: int = 256, battery_pct: float = 100.0,
                 battery_drain_per_tick: float = 0.0) -> None:
        self.beacon_id = beacon_id
        self.port = port
        self._now_ts = now_ts
        self.hb_period = hb_period
        self.refresh_period = refresh_period
        self.ack_timeout = ack_timeout
        self.max_retries = max_retries
        self.parent_timeout = parent_timeout
        self.queue_limit = queue_limit
        self.capacity = capacity
        self.battery_pct = battery_pct
        self.battery_drain_per_tick = battery_drain_per_tick

        self.x: float = 0.0
        self.y: float = 0.0
        self.programmed = False
        self.records: Dict[int, Record] = {}
        self.hops: int = HOPS_UNKNOWN
        self.parent: Optional[int] = None
        self._neigh: Dict[int, Tuple[int, int]] = {}      # node -> (their hops, tick heard)
        self._seen: "OrderedDict[Tuple[int, int], None]" = OrderedDict()
        self._seen_limit = seen_limit
        self._pending: Dict[Tuple[int, int], _Pending] = {}
        self._queue: Deque[MeshFrame] = deque()
        self._seq = 0
        self._unsent: set = set()
        self._last_refresh = 0
        self.last_update_ts: int = 0
        self.alive = True
        self.stats = NodeStats()
        self.tick_now = 0
        port.subscribe(self._on_raw)

    # ── public view ──────────────────────────────────────────────────────────
    @property
    def connected(self) -> bool:
        return self.parent is not None and self.hops != HOPS_UNKNOWN

    def comm_state(self) -> str:
        if not self.alive:
            return "DEAD"
        return f"UPLINK hops={self.hops} via #{self.parent}" if self.connected else "NO UPLINK (buffering)"

    def max_version(self) -> int:
        return max((e.version for _, e in self.records.values()), default=0)

    # ── memory writes ────────────────────────────────────────────────────────
    def write_record(self, msg: BeaconMessage, ext: MemoryExt) -> bool:
        """Store/replace a record. A newer version replaces an older one; an
        equal-or-older version is ignored (it must not undo a fresher update)."""
        cur = self.records.get(ext.memory_id)
        if cur is not None and ext.version <= cur[1].version:
            self.stats.stale_updates_ignored += 1
            return False
        if cur is None and len(self.records) >= self.capacity:
            victim = next((mid for mid, (_, e) in self.records.items()
                           if e.state in (MemoryState.CLEARED, MemoryState.CONTRADICTED)), None)
            if victim is None:
                return False
            del self.records[victim]
        self.records[ext.memory_id] = (msg, ext)
        self.stats.stored_updates += 1
        self.last_update_ts = max(self.last_update_ts, ext.last_update_ts)
        self._unsent.add(ext.memory_id)
        return True

    # ── radio receive ────────────────────────────────────────────────────────
    def _on_raw(self, raw: bytes) -> None:
        if not self.alive:
            return
        try:
            f = MeshFrame.decode(raw)
        except DecodeError:
            self.stats.crc_dropped += 1
            return
        self.stats.rx += 1
        if f.link_src != self.beacon_id and f.link_hops != HOPS_UNKNOWN:
            self._neigh[f.link_src] = (f.link_hops, self.tick_now)
            self._recompute_route()

        if f.ftype == FrameType.ACK:
            if f.link_dst == self.beacon_id:
                self._pending.pop(decode_ack(f.payload), None)
            return
        if f.ftype == FrameType.PROGRAM:
            self._on_program(f)
            return
        if f.ftype == FrameType.HELLO:
            self._answer_hello(f)
            return
        if f.link_dst != self.beacon_id:
            return                                  # overheard: gradient learned above, not mine to forward
        self._ack(f)
        if f.packet_id in self._seen:
            self.stats.dup_dropped += 1
            return
        self._remember(f.packet_id)
        if f.ttl <= 0:
            self.stats.ttl_dropped += 1
            return
        self.stats.forwarded += 1
        self._enqueue_or_send(f.for_next_hop(self.beacon_id, BROADCAST_ID, self.hops))

    def _answer_hello(self, f: MeshFrame) -> None:
        """A robot asked who is around: reply with my gradient (hops to the ONA). One hop, never forwarded."""
        hb = Heartbeat(self.beacon_id, self.x, self.y, self.hops, self.parent if self.parent is not None else BROADCAST_ID,
                       int(self.battery_pct), self.last_update_ts, len(self.records), self.max_version())
        reply = MeshFrame(FrameType.HEARTBEAT, self.beacon_id, self._next_seq(), 1, 0, self.beacon_id,
                          f.link_src, self.hops, encode_heartbeat(hb))
        self.port.send(reply.encode())

    def _on_program(self, f: MeshFrame) -> None:
        try:
            cmd = decode_program(f.payload)
        except DecodeError:
            self.stats.crc_dropped += 1
            return
        if cmd.beacon_id != self.beacon_id:
            return
        if cmd.set_position:
            self.x, self.y = cmd.x_local, cmd.y_local
        self.programmed = True
        if cmd.record is not None:
            self.write_record(*cmd.record)
            self._flush_unsent()
        self._send_heartbeat()

    def _ack(self, f: MeshFrame) -> None:
        ack = MeshFrame(FrameType.ACK, self.beacon_id, 0, 1, 0, self.beacon_id, f.link_src,
                        self.hops, encode_ack(f.origin, f.seq))
        self.port.send(ack.encode())

    def _remember(self, pid: Tuple[int, int]) -> None:
        self._seen[pid] = None
        while len(self._seen) > self._seen_limit:
            self._seen.popitem(last=False)

    # ── routing ──────────────────────────────────────────────────────────────
    def _recompute_route(self) -> None:
        best: Optional[Tuple[int, int]] = None
        for nid, (h, t) in self._neigh.items():
            if self.tick_now - t > self.parent_timeout:
                continue
            if best is None or (h, nid) < best:
                best = (h, nid)
        if best is None:
            self.hops, self.parent = HOPS_UNKNOWN, None
            return
        new_parent = best[1]
        if self.parent in self._neigh and self.tick_now - self._neigh[self.parent][1] <= self.parent_timeout:
            ph = self._neigh[self.parent][0]
            if ph <= best[0]:                       # keep a still-good parent (stability)
                new_parent, best = self.parent, (ph, self.parent)
        gained = self.parent is None
        self.parent, self.hops = new_parent, best[0] + 1
        if gained:
            self._on_uplink_gained()

    def _on_uplink_gained(self) -> None:
        self._send_heartbeat()          # tell deeper nodes right away: "I have a route"
        while self._queue:
            self._send_upstream(self._queue.popleft())
        self._flush_unsent()

    # ── sending ──────────────────────────────────────────────────────────────
    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def _originate(self, ftype: FrameType, payload: bytes) -> None:
        f = MeshFrame(ftype, self.beacon_id, self._next_seq(), DEFAULT_TTL, 0,
                      self.beacon_id, BROADCAST_ID, self.hops, payload)
        self.stats.originated += 1
        self._remember(f.packet_id)
        self._enqueue_or_send(f)

    def _enqueue_or_send(self, f: MeshFrame) -> None:
        if not self.connected:
            if len(self._queue) >= self.queue_limit:
                self._queue.popleft()
                self.stats.queue_overflow += 1
            self._queue.append(f)
            return
        self._send_upstream(f)

    def _send_upstream(self, f: MeshFrame) -> None:
        out = MeshFrame(f.ftype, f.origin, f.seq, f.ttl, f.hop_count, self.beacon_id,
                        self.parent if self.parent is not None else BROADCAST_ID,
                        self.hops, f.payload)
        self._pending[out.packet_id] = _Pending(out, out.link_dst, self.tick_now, 0)
        self.port.send(out.encode())

    def _flush_unsent(self) -> None:
        if not self.connected:
            return
        for mid in sorted(self._unsent):
            rec = self.records.get(mid)
            if rec is not None:
                self._originate(FrameType.MEMORY, encode_memory_payload(*rec))
        self._unsent.clear()

    def _send_heartbeat(self) -> None:
        hb = Heartbeat(self.beacon_id, self.x, self.y, self.hops, self.parent if self.parent is not None else BROADCAST_ID,
                       int(self.battery_pct), self.last_update_ts, len(self.records), self.max_version())
        self._originate(FrameType.HEARTBEAT, encode_heartbeat(hb))

    # ── clock ────────────────────────────────────────────────────────────────
    def tick(self, t: int) -> None:
        if not self.alive:
            return
        self.tick_now = t
        if self.battery_drain_per_tick:
            self.battery_pct -= self.battery_drain_per_tick
            if self.battery_pct <= 0:
                self.alive = False
                self.port.medium.set_alive(self.beacon_id, False)
                return
        # expire a silent parent
        if self.parent is not None and t - self._neigh.get(self.parent, (0, -10**9))[1] > self.parent_timeout:
            self._neigh.pop(self.parent, None)
            self.parent, self.hops = None, HOPS_UNKNOWN
            self._recompute_route()
        # retries / give up (-> store-and-forward)
        for pid, pend in list(self._pending.items()):
            if t - pend.sent_tick < self.ack_timeout:
                continue
            if pend.tries < self.max_retries:
                pend.tries += 1
                pend.sent_tick = t
                self.stats.retries += 1
                self.port.send(pend.frame.encode())
            else:
                del self._pending[pid]
                self.stats.gave_up += 1
                self._neigh.pop(pend.next_hop, None)     # that neighbour looks dead
                self._recompute_route()
                if pend.requeues < 3:
                    f = pend.frame
                    self._enqueue_or_send(MeshFrame(f.ftype, f.origin, f.seq, f.ttl, f.hop_count,
                                                    self.beacon_id, BROADCAST_ID, self.hops, f.payload))
        # heartbeat (phase-staggered by id so beacons do not all talk at once)
        # A relay advertises its gradient as soon as it HAS one, programmed or not,
        # otherwise deeper nodes could never learn a route through it.
        if (self.programmed or self.connected) and (t + self.beacon_id * 7) % self.hb_period == 0:
            self._send_heartbeat()
        # periodic re-broadcast of stored memory ("beacons keep talking")
        if self.records and self.connected and t - self._last_refresh >= self.refresh_period:
            self._last_refresh = t
            self._unsent.update(self.records.keys())
            self._flush_unsent()
