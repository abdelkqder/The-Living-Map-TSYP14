"""
communication/mesh.py
=====================
Simulated SPATIAL radio: the thing that replaces "Writer -> MockTransport ->
ONA" in the main demonstration.

A RadioMedium owns every radio node (ONA, beacons, robots). A frame sent by a
node is delivered ONLY to nodes that are

  * alive, and
  * within radio range once WALLS are accounted for: every permanent wall cell
    on the straight line between two nodes adds `wall_penalty` cells to the
    effective distance (attenuation through walls; debris is transparent).
    effective = distance + wall_penalty x walls_between  must be <= range.
    A deep node behind several walls therefore cannot reach the ONA directly.

after a one-tick latency, and may additionally be dropped with probability
`loss` (seeded RNG -> reproducible). A deep beacon therefore cannot reach the
ONA directly; its data must be forwarded hop by hop (see beacon/node.py).

What is modelled: range, unreachable links, per-hop latency, random loss.
What is NOT modelled: real RF propagation, fading, collisions, duty cycle,
capture effect. The range is a scaled-down [ASSUMPTION] for a 10 x 6 m arena —
it is NOT a measured LoRa figure. [SIMULATED]

RadioPort is the robot-side handle (implements BaseTransport, so the same
robot code can later sit on a real LoRa driver).
"""
from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Tuple

from common.protocol import (
    BROADCAST_ID, FrameType, HOPS_UNKNOWN, DecodeError, MeshFrame, decode_heartbeat,
)
from communication.interface import BaseTransport

Cell = Tuple[int, int]
DEFAULT_RANGE_CELLS = 6.0


@dataclass
class _Node:
    node_id: int
    position_fn: Callable[[], Cell]
    on_receive: Callable[[bytes], None]
    kind: str
    range_cells: float
    alive: bool = True


@dataclass
class MediumStats:
    transmissions: int = 0
    deliveries: int = 0
    lost_random: int = 0
    out_of_range: int = 0       # counted per (tx, candidate) pair that failed range/LOS


class RadioMedium:
    def __init__(self, los_fn: Optional[Callable[[Cell, Cell], bool]] = None,
                 range_cells: float = DEFAULT_RANGE_CELLS, loss: float = 0.0,
                 seed: int = 0, latency: int = 1,
                 wall_count_fn: Optional[Callable[[Cell, Cell], int]] = None,
                 wall_penalty: float = 2.0) -> None:
        # los_fn: hard line-of-sight (legacy); wall_count_fn: attenuation model (preferred)
        self._los = los_fn or (lambda a, b: True)
        self._walls = wall_count_fn
        self.wall_penalty = wall_penalty
        self.range_cells = range_cells
        self.loss = loss
        self.latency = max(1, latency)
        self._rng = random.Random(seed)          # per-instance: no hidden global RNG
        self._nodes: Dict[int, _Node] = {}
        self._queue: Deque[Tuple[int, int, bytes]] = deque()   # (due_tick, dst, raw)
        self.tick = 0
        self.stats = MediumStats()
        self.trace: List[Tuple[int, int, int, int]] = []   # (tick, src, dst, ftype) bounded
        self.trace_limit = 5000
        self.log_hook: Optional[Callable[[int, int, int, bytes], None]] = None

    # ── registry ──────────────────────────────────────────────────────────────
    def register(self, node_id: int, position_fn: Callable[[], Cell],
                 on_receive: Callable[[bytes], None], kind: str = "beacon",
                 range_cells: Optional[float] = None) -> None:
        self._nodes[node_id] = _Node(node_id, position_fn, on_receive, kind,
                                     range_cells if range_cells is not None else self.range_cells)

    def unregister(self, node_id: int) -> None:
        self._nodes.pop(node_id, None)

    def set_alive(self, node_id: int, alive: bool) -> None:
        if node_id in self._nodes:
            self._nodes[node_id].alive = alive

    def node_ids(self, kind: Optional[str] = None) -> List[int]:
        return [n.node_id for n in self._nodes.values() if kind is None or n.kind == kind]

    def position_of(self, node_id: int) -> Optional[Cell]:
        n = self._nodes.get(node_id)
        return n.position_fn() if n else None

    # ── physics ───────────────────────────────────────────────────────────────
    def link_margin(self, a_id: int, b_id: int) -> Optional[float]:
        """None if a and b cannot hear each other; else 0..1 (1 = adjacent,
        ~0 = at the edge of range) — an RSSI-like proxy."""
        a, b = self._nodes.get(a_id), self._nodes.get(b_id)
        if a is None or b is None or not a.alive or not b.alive or a_id == b_id:
            return None
        pa, pb = a.position_fn(), b.position_fn()
        d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
        rng = min(a.range_cells, b.range_cells)
        if self._walls is not None:
            d += self.wall_penalty * self._walls(pa, pb)
        elif not self._los(pa, pb):
            return None
        if d > rng:
            return None
        return max(0.0, 1.0 - d / rng)

    def neighbours(self, node_id: int) -> List[Tuple[int, float]]:
        out = []
        for other in self._nodes:
            m = self.link_margin(node_id, other)
            if m is not None:
                out.append((other, m))
        return out

    def transmit(self, src_id: int, raw: bytes) -> List[int]:
        """Broadcast on the shared medium. Returns node ids that WILL receive it
        (physics-level; real radios only learn this through ACKs)."""
        src = self._nodes.get(src_id)
        if src is None or not src.alive:
            return []
        self.stats.transmissions += 1
        receivers: List[int] = []
        for other in sorted(self._nodes):
            if other == src_id:
                continue
            if self.link_margin(src_id, other) is None:
                self.stats.out_of_range += 1
                continue
            if self.loss > 0.0 and self._rng.random() < self.loss:
                self.stats.lost_random += 1
                continue
            self._queue.append((self.tick + self.latency, other, raw))
            receivers.append(other)
            if len(self.trace) < self.trace_limit:
                ft = raw[0] if raw else 0
                self.trace.append((self.tick, src_id, other, ft))
        return receivers

    def step(self, tick: int) -> int:
        """Deliver every frame that is due. Returns the number delivered."""
        self.tick = tick
        n = 0
        while self._queue and self._queue[0][0] <= tick:
            _, dst, raw = self._queue.popleft()
            node = self._nodes.get(dst)
            if node is not None and node.alive:
                node.on_receive(raw)
                self.stats.deliveries += 1
                n += 1
        return n

    def pending(self) -> int:
        return len(self._queue)

    def connected_to(self, root_id: int, kinds=("ona", "beacon")) -> List[int]:
        """Graph reachability from root over bidirectional links, restricted to
        `kinds` nodes. Simulator-side diagnostic (tests / UI) — robots never call it."""
        seen, stack = {root_id}, [root_id]
        while stack:
            cur = stack.pop()
            for nb, _ in self.neighbours(cur):
                if nb not in seen and self._nodes[nb].kind in kinds:
                    seen.add(nb)
                    stack.append(nb)
        return sorted(seen)


@dataclass
class Uplink:
    node_id: int
    hops: int         # that node's hops-to-ONA
    margin: float


class RadioPort(BaseTransport):
    """
    A robot's (or the ONA's) radio. Implements BaseTransport so the same robot
    code can run on a real LoRa driver later.

      send(raw)             broadcast one frame; True if at least one node heard it
      send_acked(raw, who)  retry up to `retries` times until node `who` heard it
                            (stands in for a link-layer ACK) [SIMULATED]
      best_uplink()         the strongest-link recently heard node that has a route
                            to the ONA — what a robot knows about its own
                            connectivity (no positions, no global topology)
    """

    def __init__(self, medium: RadioMedium, node_id: int, position_fn: Callable[[], Cell],
                 kind: str = "robot", range_cells: Optional[float] = None,
                 heard_ttl: int = 120) -> None:
        self.medium = medium
        self.node_id = node_id
        self.heard_ttl = heard_ttl
        self._handlers: List[Callable[[bytes], None]] = []
        self.heard: Dict[int, Tuple[int, int]] = {}   # node -> (hops, last_heard_tick)
        self.tx_count = 0
        medium.register(node_id, position_fn, self._on_radio, kind, range_cells)

    def subscribe(self, handler: Callable[[bytes], None]) -> None:
        self._handlers.append(handler)

    def send(self, payload: bytes) -> bool:
        self.tx_count += 1
        return bool(self.medium.transmit(self.node_id, payload))

    def send_acked(self, payload: bytes, expect_node: int, retries: int = 3) -> bool:
        for _ in range(retries + 1):
            self.tx_count += 1
            if expect_node in self.medium.transmit(self.node_id, payload):
                return True
        return False

    def _on_radio(self, raw: bytes) -> None:
        try:
            f = MeshFrame.decode(raw)
        except DecodeError:
            return
        if f.link_hops != HOPS_UNKNOWN:
            self.heard[f.link_src] = (f.link_hops, self.medium.tick)
        for h in self._handlers:
            h(raw)

    def best_uplink(self) -> Optional[Uplink]:
        best: Optional[Uplink] = None
        for nid, (hops, t) in list(self.heard.items()):
            if self.medium.tick - t > self.heard_ttl:
                continue
            margin = self.medium.link_margin(self.node_id, nid)
            if margin is None:
                continue
            cand = Uplink(nid, hops, margin)
            # strongest link wins (RSSI-style parent choice); fewer hops breaks ties
            if best is None or (-cand.margin, cand.hops) < (-best.margin, best.hops):
                best = cand
        return best

    def audible(self, kind_ids=range(1, 64)) -> List[Tuple[int, float]]:
        """Beacons this radio currently hears (recent frame AND still in range),
        strongest first: [(beacon_id, margin)]. What a robot uses to pick a host
        beacon to program."""
        out = []
        for nid, (_, t) in self.heard.items():
            if nid in kind_ids and self.medium.tick - t <= self.heard_ttl:
                m = self.medium.link_margin(self.node_id, nid)
                if m is not None:
                    out.append((nid, m))
        out.sort(key=lambda x: -x[1])
        return out

    def margin_to(self, node_id: int) -> Optional[float]:
        """Signal quality to one node (RSSI-like), None if it cannot be heard. A real
        radio gets this from the RSSI of that node's last frame."""
        return self.medium.link_margin(self.node_id, node_id)

    def hears(self, node_id: int) -> bool:
        return self.medium.link_margin(self.node_id, node_id) is not None


class RobotLink:
    """
    A robot's protocol layer on top of its RadioPort — everything a robot is
    allowed to do on the radio:

      program_beacon(...)   write/update a record in a beacon within local range
                            (PROGRAM frame, link-layer ACK, retried)
      send_upstream(...)    send a STATUS/MEMORY frame toward the ONA through the
                            best uplink it currently hears; if it hears none the
                            frame waits in an outbox (store-and-forward) and is
                            sent when an uplink appears (e.g. back at the entry)
      uplink()              (node, hops, margin) of its best connected neighbour

    There is deliberately NO method to reach the Command Post: a robot can only
    talk to beacons and the ONA. [IMPLEMENTED][SIMULATED radio]
    """

    def __init__(self, port: RadioPort, outbox_limit: int = 16) -> None:
        self.port = port
        self.node_id = port.node_id
        self._seq = 0
        self._outbox: List[Tuple[FrameType, bytes]] = []
        self._outbox_limit = outbox_limit
        self.sent_frames = 0
        self.acked_frames = 0

    def _frame(self, ftype: FrameType, dst: int, hops: int, payload: bytes) -> MeshFrame:
        from common.protocol import DEFAULT_TTL
        self._seq = (self._seq + 1) & 0xFFFF
        return MeshFrame(ftype, self.node_id, self._seq, DEFAULT_TTL, 0, self.node_id, dst, hops, payload)

    def uplink(self) -> Optional[Uplink]:
        return self.port.best_uplink()

    def hello(self) -> None:
        """Probe the neighbourhood; beacons and the ONA answer with their hops-to-ONA."""
        self.port.send(self._frame(FrameType.HELLO, BROADCAST_ID, HOPS_UNKNOWN, b"").encode())

    def program_beacon(self, beacon_id: int, cmd_payload: bytes, retries: int = 3) -> bool:
        f = self._frame(FrameType.PROGRAM, beacon_id, HOPS_UNKNOWN, cmd_payload)
        self.sent_frames += 1
        ok = self.port.send_acked(f.encode(), beacon_id, retries)
        self.acked_frames += int(ok)
        return ok

    def send_upstream(self, ftype: FrameType, payload: bytes) -> bool:
        """True if handed to an uplink now; False if queued for later."""
        up = self.uplink()
        if up is None:
            if len(self._outbox) >= self._outbox_limit:
                self._outbox.pop(0)
            self._outbox.append((ftype, payload))
            return False
        f = self._frame(ftype, up.node_id, HOPS_UNKNOWN, payload)
        self.sent_frames += 1
        ok = self.port.send_acked(f.encode(), up.node_id, 3)
        self.acked_frames += int(ok)
        if not ok:
            self._outbox.append((ftype, payload))
        return ok

    def tick(self) -> None:
        """Flush the outbox when an uplink is available."""
        if self._outbox and self.uplink() is not None:
            pending, self._outbox = self._outbox, []
            for ftype, payload in pending:
                self.send_upstream(ftype, payload)

    @property
    def outbox_size(self) -> int:
        return len(self._outbox)
