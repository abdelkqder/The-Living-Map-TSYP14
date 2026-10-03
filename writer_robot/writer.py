"""
writer_robot/writer.py
=======================
WriterRobot: explore an UNKNOWN space with frontier exploration, localise by
dead reckoning, detect events and blockages, decide what must survive the
Writer, leave it in beacons (placed deliberately — see placement.py), and
get back to the entrance without shutting down.

    SENSE -> LOCALISE -> UNDERSTAND -> PRESERVE -> (keep exploring) -> RETURN

What is genuinely autonomous
  * There is no stored route. Targets are chosen from the frontier of the
    Writer's OWN discovered map; a different world (or a blockage) gives a
    different trajectory.
  * The Writer's coordinates come from an odometry model (common/pose.py):
    turn + forward increments integrate into a local pose (x, y, heading).
    Beacon records carry that pose (plus the sensor's relative displacement for
    an event) — they are NOT read off a global map cell.
  * Blockages (debris) it meets are written into its map (CellState.BLOCKED),
    routed around, and — when important enough — preserved as BLOCKAGE memory.
  * It only touches the world through sense() — structurally enforced by
    tests/test_world.py.

What is simulated: positions, sensing, the radio, the beacon drop mechanism.
`max_ticks` bounds the EXPLORATION time. When it runs out the Writer heads home; the
trip back is on top of that budget [ASSUMPTION: battery sized for the round trip].
Legacy mode: constructed with only a transport (no `link`/`dispenser`), the
Writer sends bare 18-byte packets exactly as before — kept for unit tests.

[IMPLEMENTED] decision logic · [SIMULATED] world, radio, odometry (noise-free by default)
"""

from __future__ import annotations

import math
from typing import List, Optional, Set, Tuple

from common.clock import SimClock
from common.enums import CellState, EventType, MemoryState, PassageState, WriterState
from common.models import Observation
from common.pose import LocalPose, OdometryModel
from common.protocol import (
    HOPS_UNKNOWN, BeaconMessage, MemoryExt, ProgramCommand, encode_program, memory_id_for, writer_node_id,
)
from communication.interface import BaseTransport
from communication.mesh import RobotLink
from beacon.deployer import BeaconDispenser
from navigation.engine import MovementController, astar_known, plan_route
from simulation.world import (
    CELL_PX, METRES_PER_CELL, Cell, DiscoveredWorld, EntryFrame, GroundTruthWorld, cell_to_px,
)
from writer_robot.exploration import ExplorationPolicy
from writer_robot.memory_decision import DeployedBeacon, evaluate
from writer_robot.perception import EventDetector, synthesize_reading
from writer_robot.placement import BeaconPlacementPolicy, PlacedBeacon, PlacementKind

WRITER_SPEED_PX = 2.2
SENSOR_RADIUS   = 2   # cells; [ASSUMPTION] not tied to a real sensor's FOV yet
TICKS_PER_CELL  = CELL_PX / WRITER_SPEED_PX
BLOCKAGE_SEVERITY = 2
BLOCKAGE_CONFIDENCE = 90


class WriterRobot:
    COLOR_HEX = "#6D63D6"
    LABEL     = "W"

    def __init__(
        self,
        world_gt: GroundTruthWorld,
        transport: BaseTransport,
        max_ticks: Optional[int] = None,
        sensor_radius: int = SENSOR_RADIUS,
        writer_id: str = "W1",
        sector_hint: Optional[tuple] = None,
        *,
        link: Optional[RobotLink] = None,
        dispenser: Optional[BeaconDispenser] = None,
        clock: Optional[SimClock] = None,
        index: Optional[int] = None,
        policy: Optional[BeaconPlacementPolicy] = None,
        odometry_noise: Tuple[float, float] = (0.0, 0.0),
        seed: int = 0,
        fail_at_tick: Optional[int] = None,
        return_home: bool = True,
        explore_jitter: float = 0.0,
    ):
        self.world_gt      = world_gt
        self.discovered    = DiscoveredWorld(world_gt.rows, world_gt.cols)
        self.transport     = transport
        self.sensor_radius = sensor_radius
        self.writer_id     = writer_id
        self.sector_hint   = sector_hint
        self.link          = link
        self.dispenser     = dispenser
        self.clock         = clock
        self.policy        = policy or BeaconPlacementPolicy()
        self.fail_at_tick  = fail_at_tick
        self.return_home   = return_home
        self.max_ticks     = max_ticks
        self.index         = index
        self.node_id       = writer_node_id(index) if index is not None else 0

        self.exploration = ExplorationPolicy(max_ticks=max_ticks, sector_hint=sector_hint,
                                             jitter=explore_jitter, seed=seed)
        self.detector    = EventDetector()
        self.movement    = MovementController(cell_to_px, speed_px=WRITER_SPEED_PX)
        self.movement.teleport(world_gt.entry)
        self.movement.can_advance = lambda cell: not world_gt.is_blocked(*cell)   # physical collision only

        # ── localisation: a pose integrated from motion, in the entry-anchored local frame
        self.frame = EntryFrame(world_gt.entry)
        self.odometry = OdometryModel(LocalPose(0.0, 0.0, 0.0), dist_noise_std=odometry_noise[0],
                                      turn_noise_std=odometry_noise[1], seed=seed)

        self.deployed:     List[DeployedBeacon] = []
        self.placed:       List[PlacedBeacon]   = []
        self.beacon_count: int  = 0                 # beacons dropped (events + relays)
        self.memory_count: int  = 0                 # memory records written
        self.trail:        Set[Cell] = {world_gt.entry}
        self.route:        List[Cell] = [world_gt.entry]
        self.tick:         int  = 0
        self.log:          List[str] = []
        self._mode: str = "EXPLORE"                 # EXPLORE | BRIDGE | RETURN | HOME | DEAD
        self._bridge_anchor: Optional[int] = None   # disconnected beacon being bridged to the network
        self._bridge_goal: Optional[Cell] = None
        self._listen = 0                            # ticks to stand still and listen for HELLO replies
        self._prev_cell: Optional[Cell] = None
        self._drop_on_arrival = False
        self._last_connected: Optional[Cell] = world_gt.entry
        self._current_target: Optional[Cell] = None
        self._mem_counter = 0
        self._reported_blocked: Set[Cell] = set()
        self._logged_skips: Set[tuple] = set()
        self._pending_bridge: Optional[int] = None
        self._replan_target: Optional[Cell] = None
        self.return_tick: Optional[int] = None
        # metrics (all produced by the run, none invented)
        self.stats = dict(frontiers_selected=0, replans=0, alternative_routes=0, abandoned_targets=0, dead_ends=0,
                          relays_dropped=0, bridges=0, bridge_failures=0, event_beacons=0, memory_updates=0, not_preserved=0,
                          blockages_seen=0, blockages_preserved=0)

        self._sense_and_integrate(world_gt.entry)

    # ── state ────────────────────────────────────────────────────────────────
    @property
    def active(self) -> bool:
        return self._mode not in ("HOME", "DEAD")

    @property
    def dead(self) -> bool:
        """LEGACY NAME: True once the Writer is no longer exploring — because it
        has returned to the entry (RETURNED) or has failed (DEAD). Use `state` to
        tell them apart; this alias exists so older code keeps working."""
        return not self.active

    @property
    def returned(self) -> bool:
        return self._mode == "HOME"

    @property
    def state(self) -> WriterState:
        if self._mode == "DEAD":
            return WriterState.DEAD
        if self._mode == "HOME":
            return WriterState.RETURNED
        if self._mode == "RETURN":
            return WriterState.RETURNING
        if self.movement.is_moving or self.tick > 0:
            return WriterState.EXPLORING
        return WriterState.IDLE

    @property
    def pose(self) -> LocalPose:
        return self.odometry.pose

    def force_fail(self) -> None:
        """Mid-mission hardware/battery failure: the Writer stops where it is.
        Beacons already deployed keep working — that is the point of the project."""
        self._mode = "DEAD"
        self.movement.cancel_path()
        if self.link is not None:
            self.link.port.medium.set_alive(self.node_id, False)
        self.log.append(f"[t={self.tick}] {self.writer_id} OFFLINE — failed at {self.movement.current_cell} (simulated)")

    # ── Public interface ─────────────────────────────────────────────────────
    def start(self) -> None:
        self._plan_next()

    def update(self) -> List[BeaconMessage]:
        """Advance one tick. Returns memory messages written this tick (usually empty)."""
        if not self.active:
            return []
        self.tick += 1
        if self.link is not None:
            self.link.tick()
        if self.fail_at_tick is not None and self.tick >= self.fail_at_tick:
            self.force_fail()
            return []
        if self._listen > 0:                          # standing still, waiting for HELLO replies
            self._listen -= 1
            if self._listen == 0 and self._mode == "BRIDGE":
                self._bridge_decide(self.movement.current_cell)
            return []
        if not self.movement.is_moving:
            self._plan_next()
            if not self.active:
                return []
            if not self.movement.is_moving:
                return []
        reached = self.movement.step()
        if reached is None:
            return []
        return self._arrive(reached)

    @property
    def px(self) -> float:
        return self.movement.px

    @property
    def py(self) -> float:
        return self.movement.py

    @property
    def angle(self) -> float:
        return self.movement.angle

    # ── planning ─────────────────────────────────────────────────────────────
    def _begin_return(self, reason: str) -> None:
        self._bridge_anchor = None
        self.log.append(f"[t={self.tick}] {self.writer_id} exploration over ({reason}); heading back to the entry")
        if not self.return_home:
            self._mode = "HOME"
            return
        self._mode = "RETURN"
        self._current_target = None

    def _plan_next(self) -> None:
        pos = self.movement.current_cell
        if self._mode == "HOME" or self._mode == "DEAD":
            return
        if self._mode == "RETURN":
            self._plan_return(pos)
            return
        if self._mode == "BRIDGE":
            self._plan_bridge(pos)
            return
        outcome = self.exploration.next_target(self.discovered, pos, self.tick)
        if outcome.target is None:
            self._begin_return(outcome.reason)
            if self._mode == "RETURN":
                self._plan_return(pos)
            return
        path = astar_known(self.discovered, pos, outcome.target.cell)
        assert path is not None, "frontier candidate must be reachable by construction"
        if self._replan_target is not None:
            if outcome.target.cell == self._replan_target:
                self.stats["alternative_routes"] += 1       # same goal, new route around the obstruction
            else:
                self.stats["abandoned_targets"] += 1        # goal given up after the obstruction
            self._replan_target = None
        self._current_target = outcome.target.cell
        self.stats["frontiers_selected"] += 1
        self.movement.set_path(path[1:])

    def _plan_return(self, pos: Cell) -> None:
        if pos == self.world_gt.entry:
            self._arrived_home()
            return
        route = plan_route(self.discovered, pos, self.world_gt.entry, optimistic=False)
        if route is None:
            self._mode = "DEAD"
            self.log.append(f"[t={self.tick}] {self.writer_id} LOST: no known route back to the entry")
            return
        self.movement.set_path(route.path[1:])

    def _arrived_home(self) -> None:
        self._mode = "HOME"
        self.return_tick = self.tick
        self.log.append(f"[t={self.tick}] {self.writer_id} back at the entry — still operational, not shut down")

    # ── arrival handling ─────────────────────────────────────────────────────
    def _arrive(self, cell: Cell) -> List[BeaconMessage]:
        prev = self.route[-1]
        self._prev_cell = prev
        dx, dy = self.frame.step_to_local_delta(prev, cell)
        self.odometry.move_by(dx, dy)                       # dead reckoning: turn + forward
        self.trail.add(cell)
        self.route.append(cell)
        newly_known = len(self.discovered.known_free_cells())
        if self.link is not None:
            self.link.hello()                                   # who can I hear from here?
        msgs = self._sense_and_integrate(cell)
        self._note_connectivity(cell)
        if self._mode == "EXPLORE" and self._current_target == cell:
            if len(self.discovered.known_free_cells()) == newly_known:
                self.stats["dead_ends"] += 1                # reached a frontier that revealed nothing
        # a blockage ahead invalidates the current path -> replan around it
        if self.movement.is_moving and self.discovered.changes:
            if any(self.discovered.is_impassable(c) for c in self.movement.remaining_cells()):
                self.movement.cancel_path()
                self.stats["replans"] += 1
                if self._mode == "EXPLORE" and self._current_target is not None:
                    self._replan_target = self._current_target
        if self._pending_bridge is not None and self._mode == "EXPLORE":
            self._begin_bridge(self._pending_bridge)
            self._pending_bridge = None
        if self._mode == "BRIDGE" and self._drop_on_arrival:
            self._place_bridge_relay(cell, "stepped back to the last cell where the anchor was audible", final=False)
        elif self._mode == "BRIDGE":
            self._listen = 3                                    # let the HELLO replies arrive, then decide
        if self._mode == "RETURN" and cell == self.world_gt.entry:
            self._arrived_home()
        return msgs

    def _sense_and_integrate(self, cell: Cell) -> List[BeaconMessage]:
        snapshot = self.world_gt.sense(cell, self.sensor_radius)
        self.discovered.integrate(snapshot, self.tick)
        reading = synthesize_reading(snapshot, self.tick)
        out: List[BeaconMessage] = []

        # ── events (fire / gas / victim ...) ─────────────────────────────────
        event_cell = (snapshot.event_here.row, snapshot.event_here.col) if snapshot.event_here else cell
        off_r, off_c = snapshot.event_offset if snapshot.event_offset else (0, 0)
        for event_type in self.detector.process(reading):
            severity   = EventDetector.severity(event_type)
            confidence = EventDetector.confidence(event_type, reading)
            obs = Observation(event_type=event_type, row=event_cell[0], col=event_cell[1],
                              severity=severity, confidence=confidence, tick=self.tick, source=self.writer_id)
            self.discovered.record_observation(obs)
            # the event's local position = my dead-reckoned pose + the sensor's relative displacement
            ex = self.odometry.pose.x + off_c * METRES_PER_CELL
            ey = self.odometry.pose.y - off_r * METRES_PER_CELL
            out.extend(self._consider(obs, ex, ey, cell))

        # ── navigation-relevant memory: a blocked passage ────────────────────
        for b in sorted(snapshot.blocked):
            if b in self._reported_blocked:
                continue
            self._reported_blocked.add(b)
            self.stats["blockages_seen"] += 1
            obs = Observation(event_type=EventType.BLOCKAGE, row=b[0], col=b[1], severity=BLOCKAGE_SEVERITY,
                              confidence=BLOCKAGE_CONFIDENCE, tick=self.tick, source=self.writer_id)
            self.discovered.record_observation(obs)
            lx, ly = self.odometry.pose.x + (b[1] - cell[1]) * METRES_PER_CELL, \
                     self.odometry.pose.y - (b[0] - cell[0]) * METRES_PER_CELL
            self.log.append(f"[t={self.tick}] {self.writer_id} found a blocked passage at map {b}")
            out.extend(self._consider(obs, lx, ly, cell))
        return out

    def _consider(self, obs: Observation, x: float, y: float, here: Cell) -> List[BeaconMessage]:
        decision = evaluate(obs, self.deployed)
        if not decision.preserve:
            key = (obs.event_type.value, obs.row, obs.col)
            if key not in self._logged_skips:                 # one line per event, not one per tick
                self._logged_skips.add(key)
                self.log.append(f"[t={self.tick}] {self.writer_id} observed {obs.event_type.value} at "
                                f"({obs.row},{obs.col}) — NOT preserved ({decision.reason})")
            return []
        msg = self._preserve(obs, x, y, here)
        if msg is None:
            return []
        self.deployed.append((obs.event_type, obs.row, obs.col))
        if obs.event_type == EventType.BLOCKAGE:
            self.stats["blockages_preserved"] += 1
        return [msg]

    # ── preserving information ───────────────────────────────────────────────
    def _now_ts(self) -> int:
        return self.clock.timestamp() if self.clock else 0

    def _new_memory_id(self, fallback: int) -> int:
        if self.index is None:
            return fallback
        self._mem_counter += 1
        return memory_id_for(self.index, self._mem_counter)

    def _preserve(self, obs: Observation, x: float, y: float, here: Cell) -> Optional[BeaconMessage]:
        if self.link is None or self.dispenser is None:
            return self._deploy_legacy(obs, x, y)
        up = self.link.uplink()
        dec = self.policy.decide_event(
            severity=obs.severity, confidence_pct=obs.confidence, here=here, event_cell=(obs.row, obs.col),
            placed=self.placed, uplink_margin=up.margin if up else None, stock=self.dispenser.stock)
        if dec.kind == PlacementKind.NONE:
            key = (obs.event_type.value, obs.row, obs.col, "stock")
            if key not in self._logged_skips:
                self._logged_skips.add(key)
                self.stats["not_preserved"] += 1
                self.log.append(f"[t={self.tick}] {self.writer_id} {obs.event_type.value} at ({obs.row},{obs.col}) "
                                f"NOT preserved — {dec.reason}")
            return None
        if dec.kind == PlacementKind.EVENT_UPDATE:
            host, set_pos, pose_xy = dec.host_id, False, (0.0, 0.0)
        else:
            host = self.dispenser.drop()
            if host is None:
                self.stats["not_preserved"] += 1
                return None
            self.beacon_count += 1
            self.stats["event_beacons"] += 1
            self.placed.append(PlacedBeacon(host, here))
            set_pos, pose_xy = True, (self.odometry.pose.x, self.odometry.pose.y)
        mid = self._new_memory_id(host)
        ts = self._now_ts()
        msg = BeaconMessage(beacon_id=host, event_type=obs.event_type.value, x_local=x, y_local=y, timestamp=ts,
                            severity=obs.severity, initial_confidence=obs.confidence,
                            battery_pct=max(20, 100 - self.beacon_count * 4))
        passage = PassageState.BLOCKED if obs.event_type == EventType.BLOCKAGE else None
        ext = MemoryExt(mid, 1, MemoryState.UNVERIFIED, passage, ts, self.node_id)
        cmd = ProgramCommand(host, pose_xy[0], pose_xy[1], (msg, ext), set_pos)
        ok = self.link.program_beacon(host, encode_program(cmd))
        for pb in self.placed:
            if pb.beacon_id == host:
                pb.n_records += 1
        self.memory_count += 1
        if up is None and dec.kind == PlacementKind.EVENT_NEW and self._mode == "EXPLORE":
            self._pending_bridge = host          # begin bridging once this observation is fully handled
        self.log.append(f"[t={self.tick}] {self.writer_id} memory #{mid:04X}: {obs.event_type.value} sev={obs.severity} "
                        f"conf={obs.confidence}% at local ({x:.2f},{y:.2f}) m -> beacon #{host} "
                        f"[{dec.kind.value}: {dec.reason}]" + ("" if ok else " [PROGRAM not acknowledged]"))
        return msg

    def _deploy_legacy(self, obs: Observation, x: float, y: float) -> BeaconMessage:
        """Direct-transport path used by unit tests: one bare 18-byte packet per event."""
        self.beacon_count += 1
        msg = BeaconMessage(beacon_id=self.beacon_count, event_type=obs.event_type.value, x_local=x, y_local=y,
                            timestamp=self._now_ts() or 0, severity=obs.severity, initial_confidence=obs.confidence,
                            battery_pct=max(20, 100 - self.beacon_count * 8))
        sent = self.transport.send(msg.encode())
        self.log.append(f"[t={self.tick}] {self.writer_id} beacon #{msg.beacon_id}: {obs.event_type.value} "
                        f"sev={obs.severity} conf={obs.confidence}%" + ("" if sent else " [packet lost — simulated]"))
        return msg

    # ── communication continuity: bridge on demand ───────────────────────────
    def _note_connectivity(self, cell: Cell) -> None:
        if self.link is None:
            return
        up = self.link.uplink()
        if up is not None and up.margin >= 0.15:
            self._last_connected = cell

    def _begin_bridge(self, anchor: int) -> None:
        self._bridge_anchor = anchor
        self._bridge_goal = self._last_connected
        self._mode = "BRIDGE"
        self._drop_on_arrival = False
        self.movement.cancel_path()
        self.stats["bridges"] += 1
        self.log.append(f"[t={self.tick}] {self.writer_id} beacon #{anchor} has no route to the ONA: walking back "
                        f"toward {self._last_connected} and bridging with relays only where needed")

    def _plan_bridge(self, pos: Cell) -> None:
        goal = self._bridge_goal
        if goal is None or pos == goal:
            goal = self._bridge_goal = self.world_gt.entry     # nothing nearer: keep going to the entry
        route = plan_route(self.discovered, pos, goal, optimistic=False) if pos != goal else None
        if route is None:
            self._end_bridge(ok=False, why="walked back to the entry without finding the network")
            return
        self.movement.set_path(route.path[1:])

    def _bridge_decide(self, cell: Cell) -> None:
        """Decide from the Writer's OWN radio measurements (HELLO replies / signal quality)."""
        anchor = self._bridge_anchor
        port = self.link.port
        heard = port.heard.get(anchor)                       # only nodes WITH a route are recorded here
        if heard is not None and self.tick - heard[1] <= 12:
            self._end_bridge(ok=True, why=f"beacon #{anchor} now reports a route to the ONA")
            return
        m_anchor = port.margin_to(anchor)
        up = self.link.uplink()
        other = up if (up is not None and up.node_id != anchor and up.margin >= 0.1) else None
        if m_anchor is None:                                  # lost the anchor: step back to the last good cell
            prev = self._prev_cell
            if prev is None or prev == cell or self._drop_on_arrival:
                self._end_bridge(ok=False, why="lost the anchor and cannot step back")
                return
            self._drop_on_arrival = True
            self.movement.set_path([prev])
            self.log.append(f"[t={self.tick}] {self.writer_id} lost beacon #{anchor} at {cell}: stepping back to {prev}")
            return
        dec = self.policy.decide_bridge(anchor_margin=m_anchor, uplink_margin=other.margin if other else None,
                                        stock=self.dispenser.stock)
        if dec.kind != PlacementKind.RELAY:
            if self.dispenser.stock <= 0:
                self._end_bridge(ok=False, why="no beacon left to bridge with")
            return
        self._place_bridge_relay(cell, dec.reason, final=other is not None)

    def _place_bridge_relay(self, cell: Cell, why: str, final: bool) -> None:
        bid = self._drop_relay(cell, why)
        self._drop_on_arrival = False
        if bid is None:
            self._end_bridge(ok=False, why="drop mechanism empty")
        elif final:
            self._end_bridge(ok=True, why=f"final relay #{bid} placed between the chain and the network")
        else:
            self._bridge_anchor = bid
            self.movement.cancel_path()                       # re-plan the walk from here

    def _end_bridge(self, ok: bool, why: str) -> None:
        self.log.append(f"[t={self.tick}] {self.writer_id} bridge {'complete' if ok else 'FAILED'}: {why}")
        if not ok:
            self.stats["bridge_failures"] += 1
        self._bridge_anchor = None
        self._drop_on_arrival = False
        self._mode = "EXPLORE"
        self.movement.cancel_path()

    def _drop_relay(self, cell: Cell, why: str = "bridge") -> Optional[int]:
        bid = self.dispenser.drop()
        if bid is None:
            return None
        self.beacon_count += 1
        self.stats["relays_dropped"] += 1
        self.placed.append(PlacedBeacon(bid, cell, 0, True))
        cmd = ProgramCommand(bid, self.odometry.pose.x, self.odometry.pose.y, None, True)
        self.link.program_beacon(bid, encode_program(cmd))
        self.log.append(f"[t={self.tick}] {self.writer_id} relay beacon #{bid} dropped at local "
                        f"({self.odometry.pose.x:.2f},{self.odometry.pose.y:.2f}) m — {why}")
        return bid
