"""
simulation/system.py
=====================
LiveMapSystem — wires every package together into the challenge chain and
drives the per-tick update loop.

PATCH 1 rewrite: writers and executors are now stored as lists.  Multiple
actors can be active simultaneously on the same tick.  The sequential
phase string is replaced by a *derived read-only property* — existing code
that reads `system.phase` continues to work unchanged.

Challenge chain (PHASE-1 PATCH: now a real spatial network):
    Writer ──PROGRAM──▶ Beacon ──hop──▶ Beacon ──hop──▶ … ──▶ ONA ──▶ Command Post
    Executor ◀── brief ◀── ONA mailbox ◀── Command Post   (status/memory go back up
    the same beacon chain)

Radio frames only reach nodes that are in range AND in line of sight
(communication/mesh.py). The ONA is the only object that holds callbacks into
the Command Post; robots hold a RobotLink (beacons + ONA) and nothing else.
There is NO direct Writer/Executor ↔ Command Post link anywhere in this file.
`use_mesh=False`-style direct delivery no longer exists in the system; the
MockTransport remains only for unit tests of individual robots.

Pygame-free — importable and testable without a display.

PATCH 1 public API additions:
    deploy_writer(writer_id)       — deploy any time (concurrent-safe)
    dispatch_executor(mission)     — dispatch any time (concurrent-safe)
    writers: List[WriterRobot]     — all writer instances
    executors: List[ExecutorRobot] — all executor instances
    writer_states()                — {writer_id: WriterState}
    executor_states()              — {executor_id: ExecutorState}

Backward-compatible:
    start_writer()    — still works (creates W1, phase must be "idle")
    start_executor()  — still works (phase must be "dead")
    system.writer     — property → writers[0] or None
    system.executor   — property → executors[0] or None
    system.phase      — derived string property, same values as before
                        plus "concurrent" for the new simultaneous case
"""
from __future__ import annotations

import logging
from typing import Dict, List, Optional, Set, Tuple

from beacon.deployer import BeaconDispenser, BeaconFactory
from command_post.command_post import CommandPost, CommandPostConfig
from command_post.mission_dispatcher import MissionDispatcher, MissionRecord, MissionStatus
from command_post.mission_planner import Mission, MissionPlanner, MissionTarget
from common.clock import SimClock
from common.enums import ExecutorState, RobotCapability, WriterState
from common.protocol import BeaconMessage, ONA_NODE_ID, executor_node_id, writer_node_id
from communication.mesh import RadioMedium, RadioPort, RobotLink
from communication.mock_transport import MockTransport
from executor_robot.capabilities import fleet_capabilities
from executor_robot.executor import ExecutorRobot
from gateway.ona_gateway import ONAGateway
from simulation.metrics import MetricsRecorder, RunMetrics
from simulation.scenario import Scenario, default_scenario
from simulation.world import Cell, GroundTruthWorld
from writer_robot.writer import WriterRobot

log = logging.getLogger(__name__)


class LiveMapSystem:
    """[SIMULATED] end-to-end system, driven one tick at a time by update()."""

    def __init__(self, scenario: Optional[Scenario] = None) -> None:
        self.scenario = scenario or default_scenario()
        sc = self.scenario
        self.clock = SimClock()
        self.world = GroundTruthWorld(sc.grid, sc.events, sc.entry)
        for cell in sc.debris_existing:                      # debris present when the Writer enters
            self.world.add_debris(cell, 0, "existing")
        self.total_free_cells = sum(
            1 for r in range(self.world.rows) for c in range(self.world.cols)
            if not self.world.is_wall(r, c)
        )

        # ── spatial radio + beacon network ────────────────────────────────────
        loss = sc.radio_loss if sc.radio_loss is not None else sc.beacon_packet_loss
        self.medium = RadioMedium(wall_count_fn=self.world.walls_between, range_cells=sc.radio_range_cells,
                                  loss=loss, seed=sc.seed)
        self.beacons = BeaconFactory(self.medium, self.clock.timestamp)
        # (kept for old unit tests / renderer; the system itself never sends on it)
        self.beacon_transport = MockTransport("beacon", packet_loss=loss)

        # ── ONA (at the entry) and Command Post ───────────────────────────────
        self.ona = ONAGateway(ref_gps=sc.ref_gps, ref_heading=sc.ref_heading, now_ts=self.clock.timestamp)
        self.ona_port = RadioPort(self.medium, ONA_NODE_ID, lambda: self.world.entry, kind="ona")
        self.ona.attach_radio(self.ona_port)
        self.command_post = CommandPost(self.ona, self.clock, CommandPostConfig(
            auto_dispatch=False, dispatch_threshold=sc.dispatch_threshold, blockage_policy=sc.blockage_policy, inherit_memory=sc.inherit_memory,
            brief_policy=sc.brief_policy))
        self.ona._memory_fn = self.command_post.on_memory
        self.ona._heartbeat_fn = self.command_post.on_heartbeat
        self.ona._status_fn = self.command_post.on_status
        # legacy handles
        self.living_map         = self.command_post.living_map
        self.mission_dispatcher = self.command_post.dispatcher
        self.mission_planner    = MissionPlanner()

        self.writers:   List[WriterRobot]   = []
        self.executors: List[ExecutorRobot] = []
        self._next_writer_id:   int = 1
        self._next_executor_id: int = 1
        self.tick:          int = 0
        self.log:     List[Tuple[str, str]] = []
        self.metrics: MetricsRecorder       = MetricsRecorder()
        self.change_events: List[str] = []
        self._debris_pending = sorted(sc.debris_schedule)
        self._writer_metrics_done: Set[str] = set()
        if sc.fleet:
            for eid, caps in fleet_capabilities(sc.fleet):
                self._make_executor(executor_id=eid, capabilities=caps, quiet=True)

    # ── Backward-compat single-robot properties ───────────────────────────────
    @property
    def writer(self) -> Optional[WriterRobot]:
        return self.writers[0] if self.writers else None

    @property
    def executor(self) -> Optional[ExecutorRobot]:
        return self.executors[0] if self.executors else None

    @property
    def auto_dispatch(self) -> bool:
        return self.command_post.cfg.auto_dispatch

    @auto_dispatch.setter
    def auto_dispatch(self, value: bool) -> None:
        self.command_post.cfg.auto_dispatch = bool(value)

    @property
    def phase(self) -> str:
        """
        Derived string. "idle" · "writer" · "dead" (writers finished/failed, no
        mission done) · "executor" · "concurrent" · "complete".
        An executor counts as active while it has a mission and has not yet
        finished it — which now includes the trip back to the entry.
        """
        active_w    = [w for w in self.writers if not w.dead]
        active_e    = [e for e in self.executors if e.mission is not None and not e.is_complete]
        completed_e = [e for e in self.executors if e.is_complete]
        if active_w and active_e:
            return "concurrent"
        if active_w:
            return "writer"
        if active_e:
            return "executor"
        if completed_e:
            return "complete"
        if self.writers:
            return "dead"
        return "idle"

    # ── Executor management ───────────────────────────────────────────────────
    def _robot_link(self, node_id: int, position_fn, kind: str) -> RobotLink:
        return RobotLink(RadioPort(self.medium, node_id, position_fn, kind=kind))

    def _make_executor(self, executor_id: Optional[str] = None, capabilities: Optional[List] = None,
                       quiet: bool = False) -> ExecutorRobot:
        idx = self._next_executor_id - 1
        eid = executor_id or f"E{self._next_executor_id}"
        caps = capabilities or list(ExecutorRobot.DEFAULT_CAPABILITIES)
        holder: Dict[str, ExecutorRobot] = {}
        pos = lambda: holder["e"].movement.current_cell if "e" in holder else self.world.entry
        node_id = executor_node_id(idx)
        link = self._robot_link(node_id, pos, "robot")
        kwargs = dict(self.scenario.executor_overrides.get(eid, {}))
        executor = ExecutorRobot(
            world_gt=self.world, executor_id=eid, capabilities=caps,
            link=link, dispenser=BeaconDispenser(self.beacons, pos, self.scenario.executor_beacon_stock),
            clock=self.clock, index=idx, briefing_poll=self.ona.poll_brief,
            odometry_noise=self.scenario.odometry_noise, seed=self.scenario.seed + idx,
            on_verified=self._on_verified_local, on_contradicted=self._on_contradicted_local, **kwargs)
        holder["e"] = executor
        self.executors.append(executor)
        self._next_executor_id += 1
        self.command_post.register_executor(eid, caps, node_id)
        if not quiet:
            self._log(f"[t={self.tick}] {eid} pre-deployed [{', '.join(c.value for c in caps)}]", "exec")
        return executor

    def deploy_executor_pool(self, n: int = 3, capabilities=None) -> List[ExecutorRobot]:
        """Pre-deploy N idle executors at the entry. capabilities: None -> GENERAL; a list ->
        same for all; a list of lists -> per executor (cycled). [IMPLEMENTED]"""
        deployed: List[ExecutorRobot] = []
        for i in range(n):
            if capabilities is None:
                caps = [RobotCapability.GENERAL]
            elif capabilities and isinstance(capabilities[0], list):
                caps = list(capabilities[i % len(capabilities)])
            else:
                caps = list(capabilities)
            deployed.append(self._make_executor(capabilities=caps))
        return deployed

    def _on_verified_local(self, beacon_id: int, tick: int) -> None:
        self._log(f"[t={tick}] ✓ memory #{beacon_id:04X} verified by an executor on site", "success")

    def _on_contradicted_local(self, beacon_id: int, tick: int, reason: str) -> None:
        self._log(f"[t={tick}] ✗ memory #{beacon_id:04X} contradicted on site", "alert")

    # ── Writer management ─────────────────────────────────────────────────────
    def deploy_writer(self, writer_id: Optional[str] = None, sector_hint: Optional[tuple] = None) -> WriterRobot:
        """Deploy a WriterRobot at any time. W2 gets a BLANK map: no hand-off from W1."""
        idx = self._next_writer_id
        wid = writer_id or f"W{idx}"
        holder: Dict[str, WriterRobot] = {}
        pos = lambda: holder["w"].movement.current_cell if "w" in holder else self.world.entry
        link = self._robot_link(writer_node_id(idx), pos, "robot")
        writer = WriterRobot(
            world_gt=self.world, transport=self.beacon_transport,
            max_ticks=self.scenario.writer_max_ticks, writer_id=wid, sector_hint=sector_hint,
            link=link, dispenser=BeaconDispenser(self.beacons, pos, self.scenario.writer_beacon_stock),
            clock=self.clock, index=idx, odometry_noise=self.scenario.odometry_noise,
            seed=self.scenario.seed + idx, fail_at_tick=self.scenario.writer_fail_tick,
            return_home=self.scenario.writer_return_home, explore_jitter=self.scenario.explore_jitter)
        holder["w"] = writer
        writer.start()
        self.writers.append(writer)
        self._next_writer_id += 1
        hint = f" hint={sector_hint}" if sector_hint else ""
        self._log(f"[t={self.tick}] {wid} deployed{hint}", "writer")
        return writer

    def deploy_replacement_writer(self, sector_hint: Optional[tuple] = None) -> Optional[WriterRobot]:
        """Deploy a replacement Writer once the previous one is no longer exploring.
        It starts with a completely blank map (challenge constraint). [IMPLEMENTED]"""
        if [w for w in self.writers if not w.dead]:
            self._log("Cannot deploy replacement — a writer is still active", "alert")
            return None
        writer = self.deploy_writer(sector_hint=sector_hint)
        self._log(f"[t={self.tick}] replacement {writer.writer_id} deployed — blank map, "
                  "no inherited knowledge [CHALLENGE CONSTRAINT MET]", "writer")
        if sector_hint:
            r0, c0, r1, c1 = sector_hint
            self._log(f"sector hint: rows {r0}–{r1}, cols {c0}–{c1}", "dim")
        return writer

    def force_writer_fail(self, writer_id: Optional[str] = None) -> bool:
        target = next((w for w in self.writers if not w.dead and (writer_id is None or w.writer_id == writer_id)), None)
        if target is None:
            self._log("force_writer_fail: no active writer to fail", "alert")
            return False
        target.force_fail()
        self._record_writer_metrics(target)
        self._log(f"[t={self.tick}] {target.writer_id} FORCED OFFLINE", "alert")
        return True

    def start_writer(self) -> None:
        if self.phase != "idle":
            return
        self.deploy_writer()
        self._log("━━ PHASE 1: WRITER ACTIVE ━━", "writer")
        self._log("Autonomous frontier exploration — no map given [IMPLEMENTED]", "dim")

    # ── Dispatch (always through the Command Post -> ONA mailbox) ─────────────
    def _brief_executors_at_entry(self) -> None:
        """Executors standing at the entry pick up their brief from the ONA now."""
        for e in self.executors:
            if e.state == ExecutorState.AVAILABLE and e.at_entry and self.ona.pending_brief_for(e.executor_id):
                e.poll_now()

    def _ensure_elastic_executor(self) -> None:
        """Legacy behaviour when no fleet is configured: create an executor on demand."""
        if self.scenario.fleet is not None:
            return
        cp = self.command_post
        for mr, rec, _pr in cp.ranked_missions(auto=cp.cfg.auto_dispatch):
            from command_post.planner import choose_executor
            if choose_executor(mr.required_capability, cp.fleet).executor is None:
                cap = mr.required_capability
                self._make_executor(capabilities=[cap if cap != RobotCapability.GENERAL else RobotCapability.GENERAL])
                break

    def dispatch_executor(self, mission: Optional[Mission] = None,
                          executor_id: Optional[str] = None) -> Optional[ExecutorRobot]:
        """
        Operator action: send the best queued mission now. The Command Post picks
        the executor and composes the brief; the ONA carries it; the executor
        picks it up at the entry. A hand-built `mission` (legacy / tests) is also
        carried through the ONA mailbox, never injected.
        """
        cp = self.command_post
        if mission is None:
            if cp.dispatcher.next_queued() is None:
                self._legacy_refresh_candidates()
            if cp.dispatcher.next_queued() is not None:
                self._ensure_elastic_executor_for_next()
                decision = cp.dispatch_next()
                if decision is not None:
                    self._brief_executors_at_entry()
                    ex = next((e for e in self.executors if e.executor_id == decision["executor"]), None)
                    self._log(f"[t={self.tick}] {decision['executor']} dispatched — mission {decision['mission_id']} "
                              f"({decision['event_type']} #{decision['memory_id']:04X}, priority {decision['priority']})", "exec")
                    return ex
                self._log("No available executor can take the next mission", "alert")
                return None
            candidates = self.living_map.mission_candidates()
            mission = self.mission_planner.review_and_assign(candidates)
        if mission.is_empty:
            self._log("No mission candidates — cannot dispatch executor", "alert")
            return None
        free = [e for e in self.executors if e.state == ExecutorState.AVAILABLE and not self.ona.pending_brief_for(e.executor_id)
                and e.mission is None and (executor_id is None or e.executor_id == executor_id)]
        executor = free[0] if free else self._make_executor(executor_id)
        self.ona.queue_brief(executor.executor_id, mission)
        executor.poll_now()
        self._log(f"[t={self.tick}] {executor.executor_id} dispatched — mission #{mission.mission_id}: "
                  f"{len(mission.targets)} target(s)", "exec")
        return executor

    def _ensure_elastic_executor_for_next(self) -> None:
        if self.scenario.fleet is not None:
            return
        from command_post.planner import choose_executor
        mr = self.command_post.dispatcher.next_queued()
        if mr is not None and choose_executor(mr.required_capability, self.command_post.fleet).executor is None:
            self._make_executor(capabilities=[mr.required_capability])

    def _legacy_refresh_candidates(self) -> None:
        pass

    def start_executor(self) -> None:
        """Legacy entry point (phase must be 'dead'): ONE executor takes ALL candidates as targets."""
        if self.phase != "dead":
            return
        candidates = self.living_map.mission_candidates()
        mission = self.mission_planner.review_and_assign(candidates)
        if mission.is_empty:
            self._log("No mission candidates — Command Post cannot assign a mission", "alert")
            return
        executor = self._make_executor()
        self.ona.queue_brief(executor.executor_id, mission)
        executor.poll_now()
        entry = self.command_post.fleet.get(executor.executor_id)
        for tgt in mission.targets:
            key = tgt.memory_id if tgt.memory_id is not None else tgt.beacon_id
            mr = self.mission_dispatcher.mission_for_memory(key)
            if mr is not None and mr.status == MissionStatus.QUEUED:
                mr.status = MissionStatus.IN_PROGRESS
                mr.assigned_executor_id = executor.executor_id
                mr.dispatched_tick = self.tick
                self.mission_dispatcher._active[f"{executor.executor_id}:{key}"] = mr
        if entry is not None:
            entry.reserved = True
        self._log("━━ PHASE 2: EXECUTOR ACTIVE ━━", "exec")
        self._log(f"Mission #{mission.mission_id} via ONA relay: {len(mission.targets)} target(s)", "dim")

    def _mission_record_to_mission(self, mr: MissionRecord) -> Mission:
        target = MissionTarget(beacon_id=mr.beacon_id, event_type=mr.event_type, x_local=mr.x_local,
                               y_local=mr.y_local, severity=mr.severity, state=mr.status.value)
        return Mission(mission_id=mr.mission_id, targets=[target])

    def reset(self) -> None:
        self.__init__(self.scenario)

    # ── Dynamic environment ───────────────────────────────────────────────────
    def add_debris(self, cell: Cell, origin: str = "manual") -> bool:
        ok = self.world.add_debris(cell, self.tick, origin)
        msg = f"[t={self.tick}] debris {'appeared' if ok else 'REFUSED'} at {cell} ({origin})"
        self.change_events.append(msg)
        self._log(msg, "alert" if ok else "dim")
        return ok

    def skip_time(self, seconds: float) -> None:
        """Let simulated time pass without robot activity (information ages). [SIMULATED]"""
        self.tick += int(seconds / self.clock.seconds_per_tick)
        self.clock.set_tick(self.tick)

    # ── Per-tick update ───────────────────────────────────────────────────────
    def update(self) -> None:
        self.tick += 1
        self.clock.set_tick(self.tick)
        while self._debris_pending and self._debris_pending[0][0] <= self.tick:
            _, cell = self._debris_pending.pop(0)
            self.add_debris(cell, "scheduled")
        self.medium.step(self.tick)
        self.beacons.tick(self.tick)
        self.ona.tick(self.tick)
        if self.command_post.cfg.auto_dispatch:
            self._ensure_elastic_executor()
        self.command_post.tick(self.tick)
        self._brief_executors_at_entry()

        for writer in self.writers:
            if writer.dead:
                continue
            for msg in writer.update():
                self._log(f"[t={self.tick}] {writer.writer_id}: [{msg.event_type}] preserved in beacon #{msg.beacon_id}", "beacon")
            if writer.dead:
                self._record_writer_metrics(writer)
                self._log("━" * 12, "dim")
                if writer.returned:
                    self._log(f"{writer.writer_id} back at the entry — {writer.beacon_count} beacon(s) left behind", "normal")
                else:
                    self._log(f"{writer.writer_id} OFFLINE [SIMULATED failure]", "alert")
                    self._log(f"{len(self.living_map)} memory record(s) already reached the Command Post — knowledge survives the Writer.", "normal")
        for executor in self.executors:
            was_done = executor.is_complete
            executor.update()
            if executor.is_complete and not was_done:
                self._log("━" * 12, "dim")
                self._log(f"{executor.executor_id} back at entry: {executor.state.value}", "success")

    # ── Fleet status helpers (renderer / UI) ──────────────────────────────────
    def writer_states(self) -> Dict[str, WriterState]:
        return {w.writer_id: w.state for w in self.writers}

    def executor_states(self) -> Dict[str, ExecutorState]:
        return {e.executor_id: e.state for e in self.executors}

    def active_writers(self) -> List[WriterRobot]:
        return [w for w in self.writers if not w.dead]

    def active_executors(self) -> List[ExecutorRobot]:
        return [e for e in self.executors if e.mission is not None and not e.is_complete]

    def network_report(self) -> Dict[str, object]:
        """Simulator-side diagnostics of the beacon network (tests / demos)."""
        connected = self.medium.connected_to(ONA_NODE_ID, kinds=("ona", "beacon"))
        beacon_ids = sorted(self.beacons.nodes)
        return {
            "beacons": len(beacon_ids), "beacons_connected_to_ona": len([b for b in connected if b != ONA_NODE_ID]),
            "max_hops": max((n.hops for n in self.beacons.nodes.values() if n.connected), default=0),
            "frames_transmitted": self.medium.stats.transmissions, "frames_delivered": self.medium.stats.deliveries,
            "frames_lost_random": self.medium.stats.lost_random,
            "forwarded_by_beacons": sum(n.stats.forwarded for n in self.beacons.nodes.values()),
            "retries": sum(n.stats.retries for n in self.beacons.nodes.values()),
            "duplicates_suppressed": sum(n.stats.dup_dropped for n in self.beacons.nodes.values()) + self.ona.mesh_stats["duplicates"],
            "ona_memory_frames": self.ona.mesh_stats["memory"], "ona_heartbeats": self.ona.mesh_stats["heartbeats"],
            "ona_crc_rejected": self.ona.mesh_stats["crc_rejected"],
        }

    def _log(self, msg: str, kind: str = "dim") -> None:
        self.log.append((msg, kind))
        if len(self.log) > 20:
            self.log.pop(0)

    # ── Metrics ───────────────────────────────────────────────────────────────
    def _record_writer_metrics(self, writer: WriterRobot) -> None:
        if writer.writer_id in self._writer_metrics_done:
            return
        self._writer_metrics_done.add(writer.writer_id)
        events_found = len({(o.row, o.col) for o in writer.discovered.observations})
        m = RunMetrics(
            scenario_name=self.scenario.name, seed=self.scenario.seed, role="writer", ticks=writer.tick,
            cells_explored=len(writer.discovered.known_free_cells()), total_free_cells=self.total_free_cells,
            coverage_pct=round(100 * writer.discovered.coverage_ratio(self.total_free_cells), 1),
            beacons_deployed=writer.beacon_count, events_total=len(self.world.all_event_ids()),
            events_found=events_found)
        self.metrics.record(m)

    # ── Renderer / UI convenience accessors ───────────────────────────────────
    @property
    def living_map_records(self):
        return self.living_map.all()

    def observed_cells(self) -> Set[Cell]:
        cells: Set[Cell] = set()
        for w in self.writers:
            cells |= {(o.row, o.col) for o in w.discovered.observations}
        for e in self.executors:
            cells |= {(o.row, o.col) for o in e.discovered.observations}
        return cells

    def visible_cells(self) -> dict:
        cells: dict = {}
        for w in self.writers:
            cells.update(w.discovered.all_known_cells())
        for e in self.executors:
            cells.update(e.discovered.all_known_cells())
        return cells
