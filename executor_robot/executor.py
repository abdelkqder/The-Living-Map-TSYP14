"""
executor_robot/executor.py
===========================
ExecutorRobot — receives a mission BRIEF, enters the zone using INHERITED spatial
memory, navigates intelligently (it re-plans; it never replays an old route),
performs the task its capability allows, writes what it observed back into the
spatial memory, goes home, reports, and becomes available again.

Lifecycle (ExecutorState):

  AVAILABLE -> ASSIGNED -> DEPLOYING -> ON_SITE -> WORKING -> VERIFYING
            -> UPDATE_MEMORY -> RETURNING -> AVAILABLE
  branches:  DEPLOYING -> WAITING -> (retry | UPDATE_MEMORY -> RETURNING)   target unreachable
             any working state -> FAILED -> RETURNING -> NEEDS_REPAIR       (or OUT_OF_SERVICE if stranded)
             any working state -> LOW_BATTERY -> RETURNING -> NEEDS_CHARGING

What it inherits (Mission, common/mission.py): the target record, the beacon
positions leading to it (route skeleton), the remembered blockages, and a
blockage POLICY (how long to wait, retry, request a Debris Executor, go home).
What it does NOT inherit: the Writer's map, or the position of debris that
appeared after the Writer left — that is discovered by sensing (case B/C).

It talks only to beacons and the ONA (RobotLink); the brief is picked up from
the ONA mailbox through `briefing_poll`. There is no handle to the Command Post.

Legacy mode (no `link`): runs a loaded Mission through the same state machine
and fires the on_verified / on_contradicted callbacks — kept for unit tests.

[IMPLEMENTED] decision logic · [SIMULATED] world, sensing, radio, actuators
"""
from __future__ import annotations

from typing import Callable, Dict, List, Optional, Set, Tuple

from common.clock import SimClock
from common.enums import CellState, EventType, ExecutorState, MemoryState, NavMode, PassageState, RobotCapability
from common.mission import Mission, MissionTarget
from common.pose import LocalPose, OdometryModel
from common.protocol import (
    BeaconMessage, EXEC_STATE_CODES, ExecStatus, FrameType, MemoryExt, MissionOutcome, ProgramCommand,
    encode_program, encode_status, executor_node_id, memory_id_for,
)
from communication.mesh import RobotLink
from beacon.deployer import BeaconDispenser
from executor_robot.capabilities import CapabilityProfile, profile_for
from navigation.engine import MovementController, plan_route
from simulation.world import (
    METRES_PER_CELL, Cell, DiscoveredWorld, EntryFrame, GroundTruthWorld, cell_to_px,
)
from writer_robot.exploration import ExplorationPolicy
from writer_robot.perception import EventDetector, synthesize_reading

EXECUTOR_SPEED_PX = 3.0
SENSOR_RADIUS     = 2
ON_SITE_TICKS     = 8        # observation time at the target before work starts
VERIFY_TICKS      = 6
OWN_BLOCKAGE_CONFIDENCE = 95
OUTCOME_FOR_STATE = {
    MemoryState.VERIFIED: MissionOutcome.VERIFIED, MemoryState.ACTIVE: MissionOutcome.VERIFIED,
    MemoryState.ESCALATED: MissionOutcome.ESCALATED, MemoryState.CLEARED: MissionOutcome.CLEARED,
    MemoryState.CONTRADICTED: MissionOutcome.CONTRADICTED,
}

VerifiedCallback     = Callable[[int, int], None]         # (beacon_id, tick)
ContradictedCallback = Callable[[int, int, str], None]    # (beacon_id, tick, reason)

_WORKING_STATES = {ExecutorState.ASSIGNED, ExecutorState.DEPLOYING, ExecutorState.ON_SITE,
                   ExecutorState.WORKING, ExecutorState.VERIFYING, ExecutorState.WAITING}


class ExecutorRobot:
    COLOR_HEX = "#2E6EC7"
    LABEL     = "E"
    DEFAULT_CAPABILITIES: List[RobotCapability] = [RobotCapability.GENERAL]

    def __init__(
        self,
        world_gt: GroundTruthWorld,
        sensor_radius: int = SENSOR_RADIUS,
        on_verified: Optional[VerifiedCallback] = None,
        on_contradicted: Optional[ContradictedCallback] = None,
        executor_id: str = "E1",
        capabilities: Optional[List[RobotCapability]] = None,
        *,
        link: Optional[RobotLink] = None,
        dispenser: Optional[BeaconDispenser] = None,
        clock: Optional[SimClock] = None,
        index: Optional[int] = None,
        briefing_poll: Optional[Callable[[str], Optional[Mission]]] = None,
        battery_drain_per_tick: float = 0.0,
        low_battery_pct: float = 25.0,
        fault_at_tick: Optional[int] = None,        # simulated breakdown N MISSION ticks after briefing
        fault_strands: bool = False,
        work_scale: float = 1.0,
        write_policy: str = "auto",          # "auto" | "existing_only" | "new_only"
        odometry_noise: Tuple[float, float] = (0.0, 0.0),
        seed: int = 0,
    ):
        self.world_gt      = world_gt
        self.discovered    = DiscoveredWorld(world_gt.rows, world_gt.cols)
        self.sensor_radius = sensor_radius
        self.detector      = EventDetector()
        self.movement      = MovementController(cell_to_px, speed_px=EXECUTOR_SPEED_PX)
        self.movement.teleport(world_gt.entry)
        self.movement.can_advance = lambda cell: not world_gt.is_blocked(*cell)   # physical collision only
        self.executor_id   = executor_id
        self.capabilities  = capabilities or list(self.DEFAULT_CAPABILITIES)
        self.link, self.dispenser, self.clock = link, dispenser, clock
        self.index         = index
        self.node_id       = executor_node_id(index) if index is not None else 0
        self.briefing_poll = briefing_poll
        self.battery_pct   = 100.0
        self.battery_drain = battery_drain_per_tick
        self.low_battery_pct = low_battery_pct
        self.fault_at_tick = fault_at_tick
        self.fault_strands = fault_strands
        self.work_scale    = work_scale
        self.write_policy  = write_policy
        self.on_verified, self.on_contradicted = on_verified, on_contradicted

        self.frame    = EntryFrame(world_gt.entry)
        self.odometry = OdometryModel(LocalPose(), dist_noise_std=odometry_noise[0],
                                      turn_noise_std=odometry_noise[1], seed=seed)

        self.mission:    Optional[Mission]   = None
        self.targets:    List[MissionTarget] = []
        self.target_idx: int = 0
        self.reached:    List[MissionTarget] = []
        self.trail:      Set[Cell] = {world_gt.entry}
        self.actual_route: List[Cell] = [world_gt.entry]
        self.planned_route: List[Cell] = []
        self.tick: int = 0
        self.log: List[str] = []
        self.events: List[dict] = []                 # structured trace (tick, kind, detail) for demos / tests
        self._state = ExecutorState.AVAILABLE
        self._finished = False
        self._goal_cell: Optional[Cell] = None
        self._goal_cells: List[Cell] = []            # acceptable arrival cells
        self._profile: Optional[CapabilityProfile] = None
        self._timer = 0
        self._matched: Optional[bool] = None
        self._final_state: Optional[MemoryState] = None
        self._effect: Optional[str] = None
        self._hint_mem: Dict[Cell, Tuple[int, int, Optional[int]]] = {}   # cell -> (memory_id, version, host)
        self._own_mem: Dict[Cell, int] = {}                               # cell -> memory id I created
        self._pending_writes: List[dict] = []
        self._mem_counter = 0
        self._wait_left = 0
        self._retries_left = 0
        self._blockers: List[int] = []
        self._returning_reason = ""
        self._fault = False
        self._low_batt = False
        self._last_reported: Optional[ExecutorState] = None
        self._searching = False
        self._explore = ExplorationPolicy(max_ticks=None)
        self._outcome_override: Optional[MissionOutcome] = None
        self._outcome_mem = 0
        self._verify_ticks_left = 0
        self._mission_start_tick = 0
        self._waypoints: List[Cell] = []
        self._wp_idx = 0
        self._parked = False
        self._outbound_mission = 0
        self._route_mark = 0
        self._bumped_before: Set[Cell] = set()
        self._seen_obstructions: Set[Cell] = set()
        self._versions: Dict[int, int] = {}              # memory id -> latest version I wrote
        self._last_snapshot = None
        self.stats = dict(distance_cells=0, replans=0, discrepancies=0, failed_approaches=0,
                          repeated_approaches=0, blockages_recorded=0, memory_updates=0,
                          beacons_dropped=0, replans_blockage=0, search_targets=0, cells_discovered=0, mission_ticks=0, missions=0,
                          wrong_turns=0)
        self._known_blocked_visits: Set[Cell] = set()
        self._sense_and_integrate(world_gt.entry)

    # ── state & properties ───────────────────────────────────────────────────
    @property
    def state(self) -> ExecutorState:
        return self._state

    @property
    def is_complete(self) -> bool:
        """The mission has been fully handled AND the executor is back at the entry
        (or permanently out of service). It does not stop at 'targets processed'."""
        return self.mission is not None and self._finished

    @property
    def px(self) -> float: return self.movement.px
    @property
    def py(self) -> float: return self.movement.py
    @property
    def angle(self) -> float: return self.movement.angle
    @property
    def pose(self) -> LocalPose: return self.odometry.pose
    @property
    def at_entry(self) -> bool: return self.movement.current_cell == self.world_gt.entry

    def can_handle(self, event_type: str) -> bool:
        if RobotCapability.GENERAL in self.capabilities:
            return True
        try:
            return RobotCapability(event_type) in self.capabilities
        except ValueError:
            return profile_for(self.capabilities, event_type) is not None

    # ── logging / reporting ──────────────────────────────────────────────────
    def _ev(self, kind: str, **detail) -> None:
        self.events.append({"tick": self.tick, "kind": kind, **detail})
        text = f"[{self.executor_id}] t={self.tick}: {kind}" + (" " + ", ".join(f"{k}={v}" for k, v in detail.items()) if detail else "")
        self.log.append(text)

    def _set_state(self, st: ExecutorState, outcome: MissionOutcome = MissionOutcome.NONE, memory_id: int = 0) -> None:
        changed = st != self._state
        self._state = st
        if changed or outcome != MissionOutcome.NONE:
            self._report(outcome, memory_id)

    def _report(self, outcome: MissionOutcome = MissionOutcome.NONE, memory_id: int = 0) -> None:
        if self.link is None:
            return
        payload = encode_status(ExecStatus(self.executor_id, self._state.value, int(self.battery_pct), memory_id, outcome),
                                EXEC_STATE_CODES)
        self.link.send_upstream(FrameType.STATUS, payload)

    def _now_ts(self) -> int:
        return self.clock.timestamp() if self.clock else 0

    # ── mission intake ───────────────────────────────────────────────────────
    def load_mission(self, mission: Mission) -> None:
        self.mission = mission
        self.targets = list(mission.targets)
        self.target_idx = 0
        self.reached = []
        self._finished = False
        self._parked = False
        self._searching = not self.targets and bool(mission.search_types)
        self._wait_left, self._retries_left = mission.policy.max_wait_ticks, mission.policy.max_retries
        self._blockers = []
        self._pending_writes = []
        self.stats["missions"] += 1
        self._mission_start_tick = self.tick
        self._route_mark = max(0, len(self.actual_route) - 1)
        # inherited spatial memory ------------------------------------------------
        self._hint_mem.clear()
        self._waypoints = [self.frame.local_to_cell(x, y) for (x, y) in mission.waypoints]
        self._wp_idx = 0
        self.discovered.add_hint_free(self._waypoints)
        for h in mission.hazards:
            cell = self.frame.local_to_cell(h.x_local, h.y_local)
            self.discovered.mark_passage(cell, PassageState(h.passage_state), "inherited", self.tick, h.memory_id)
            self._hint_mem[cell] = (h.memory_id, h.version, h.host_beacon_id)
        self._ev("mission_loaded", mission=mission.mission_id, targets=len(self.targets),
                 hazards=len(mission.hazards), waypoints=len(mission.waypoints), why=mission.reason or "-")
        self._set_state(ExecutorState.ASSIGNED)
        if self.targets:
            self._set_goal(self.targets[0])

    def _set_goal(self, target: MissionTarget) -> None:
        self._goal_cell = self.frame.local_to_cell(target.x_local, target.y_local)
        self._profile = profile_for(self.capabilities, target.event_type)
        if target.event_type == "BLOCKAGE":
            r, c = self._goal_cell        # arrive NEXT to the debris, never on it
            self._goal_cells = [(r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)]
        else:
            self._goal_cells = [self._goal_cell]

    # ── clock ────────────────────────────────────────────────────────────────
    def update(self) -> None:
        self.tick += 1
        if self.link is not None:
            self.link.tick()
        st = self._state
        if st == ExecutorState.AVAILABLE:
            self._poll_for_brief()
            return
        if st in (ExecutorState.NEEDS_REPAIR, ExecutorState.NEEDS_CHARGING, ExecutorState.OUT_OF_SERVICE):
            return
        if self.mission is None or self._finished:
            return
        self.stats["mission_ticks"] += 1
        if st in _WORKING_STATES or st == ExecutorState.RETURNING:
            self.battery_pct = max(0.0, self.battery_pct - self.battery_drain)
            if self._check_failures():
                return
        {ExecutorState.ASSIGNED: self._t_assigned, ExecutorState.DEPLOYING: self._t_deploying,
         ExecutorState.ON_SITE: self._t_on_site, ExecutorState.WORKING: self._t_working,
         ExecutorState.VERIFYING: self._t_verifying, ExecutorState.UPDATE_MEMORY: self._t_update_memory,
         ExecutorState.WAITING: self._t_waiting, ExecutorState.RETURNING: self._t_returning,
         ExecutorState.LOW_BATTERY: self._t_returning, ExecutorState.FAILED: self._t_returning,
         }.get(st, lambda: None)()

    def poll_now(self) -> bool:
        """Pick up a waiting brief immediately (the executor is at the entry, being briefed)."""
        if self._state == ExecutorState.AVAILABLE:
            self._poll_for_brief()
        return self.mission is not None and not self._finished

    def _poll_for_brief(self) -> None:
        if self.briefing_poll is None or not self.at_entry:
            return
        brief = self.briefing_poll(self.executor_id)
        if brief is not None:
            self.load_mission(brief)

    def _check_failures(self) -> bool:
        st = self._state
        if st in (ExecutorState.RETURNING, ExecutorState.LOW_BATTERY, ExecutorState.FAILED):
            return False
        if self.fault_at_tick is not None and self.stats["mission_ticks"] >= self.fault_at_tick and st in (
                ExecutorState.DEPLOYING, ExecutorState.ON_SITE, ExecutorState.WORKING, ExecutorState.WAITING):
            self._fault = True
            self.fault_at_tick = None
            self._ev("fault", state=st.value)
            self._set_state(ExecutorState.FAILED, MissionOutcome.FAILED, self._target_mem_id())
            if self.fault_strands:
                self._set_state(ExecutorState.OUT_OF_SERVICE)
                self._finished = True
                self.movement.cancel_path()
            else:
                self._start_return("fault: limping home")
            return True
        if self.battery_pct <= self.low_battery_pct and st in (
                ExecutorState.DEPLOYING, ExecutorState.ON_SITE, ExecutorState.WORKING, ExecutorState.WAITING):
            self._low_batt = True
            self._ev("low_battery", battery=int(self.battery_pct))
            self._set_state(ExecutorState.LOW_BATTERY, MissionOutcome.ABORTED_LOW_BATTERY, self._target_mem_id())
            self._start_return("battery low: aborting mission")
            return True
        return False

    def _target_mem_id(self) -> int:
        if self.targets and self.target_idx < len(self.targets):
            t = self.targets[self.target_idx]
            return t.memory_id if t.memory_id is not None else t.beacon_id
        return 0

    # ── ASSIGNED / DEPLOYING ─────────────────────────────────────────────────
    def _t_assigned(self) -> None:
        if not self.targets and not self._searching:
            self._ev("empty_mission")
            self._finish_to_home("nothing to do")
            return
        self._set_state(ExecutorState.DEPLOYING)

    def _t_deploying(self) -> None:
        if self._searching:
            self._deploy_search()
            return
        pos = self.movement.current_cell
        if pos in self._goal_cells and not self.movement.is_moving:
            self._arrive_at_target()
            return
        if not self.movement.is_moving:
            self._plan_to_goal()
            if self._state != ExecutorState.DEPLOYING or not self.movement.is_moving:
                return
        self._step_and_handle()

    def _plan_to_goal(self) -> None:
        """Plan toward the next inherited waypoint (a beacon on the way), else the target.
        Waypoints are consecutive relay beacons, i.e. within radio range of each other, so
        each leg is short — the beacon chain is the route skeleton ("beacons are the map")."""
        pos = self.movement.current_cell
        while self._wp_idx < len(self._waypoints) and (
                abs(self._waypoints[self._wp_idx][0] - pos[0]) + abs(self._waypoints[self._wp_idx][1] - pos[1]) <= 1):
            self._wp_idx += 1
        if self._wp_idx < len(self._waypoints):
            wp = self._waypoints[self._wp_idx]
            leg = plan_route(self.discovered, pos, wp, optimistic=True)
            if leg is not None and leg.length > 0:
                self.planned_route = leg.path
                self.movement.set_path(leg.path[1:])
                return
            self._wp_idx += 1                      # that waypoint is not usable now: skip it
            return self._plan_to_goal()
        best = None
        for g in self._goal_cells:
            r = plan_route(self.discovered, pos, g, optimistic=True)
            if r is not None and (best is None or r.cost < best.cost):
                best = r
        if best is None:
            self._handle_unreachable()
            return
        self.planned_route = best.path
        self.movement.set_path(best.path[1:])

    def _step_and_handle(self) -> None:
        reached = self.movement.step()
        if self.movement.bumped_cell is not None:           # physically stopped by something unsensed
            b = self.movement.bumped_cell
            self.movement.bumped_cell = None
            self._ev("bumped", cell=b)
            self.stats["failed_approaches"] += 1
            if b in self._bumped_before:
                self.stats["repeated_approaches"] += 1
            self._bumped_before.add(b)
            self._sense_and_integrate(self.movement.current_cell, force=True)
            self._after_sensing(self.movement.current_cell)
            return
        if reached is not None:
            self._arrive_cell(reached)

    def _arrive_cell(self, cell: Cell) -> None:
        prev = self.actual_route[-1]
        dx, dy = self.frame.step_to_local_delta(prev, cell)
        self.odometry.move_by(dx, dy)
        self.stats["distance_cells"] += 1
        self.trail.add(cell)
        self.actual_route.append(cell)
        self._sense_and_integrate(cell)
        self._after_sensing(cell)

    def _after_sensing(self, cell: Cell) -> None:
        """React to what sensing just changed: replan if the path is no longer valid."""
        remaining = self.movement.remaining_cells()
        bad = [c for c in remaining if self.discovered.is_impassable(c)]
        if bad:
            debris = [c for c in bad if self.discovered.state_of(c) == CellState.BLOCKED
                      or (c in self.discovered.passage_info)]
            self.stats["replans"] += 1
            if debris:
                self.stats["replans_blockage"] += 1
            self._ev("replan", at=cell, cause="BLOCKAGE ahead" if debris else "unexplored cell turned out to be a wall",
                     cells=debris or bad[:1])
            self.movement.cancel_path()

    # ── sensing & memory of what changed ─────────────────────────────────────
    def _expected_route_cells(self, cell: Cell) -> Set[Cell]:
        """Cells the Executor currently EXPECTS to cross: its planned leg plus the optimistic route from here
        to the target under what it believed BEFORE this sensing (inherited memory + own map)."""
        out = set(self.movement.remaining_cells()) | set(self.planned_route)
        if self._state in (ExecutorState.DEPLOYING, ExecutorState.ASSIGNED) and self._goal_cells and not self._searching:
            for g in self._goal_cells:
                r = plan_route(self.discovered, cell, g, optimistic=True)
                if r is not None:
                    out |= set(r.path)
        return out

    def _sense_and_integrate(self, cell: Cell, force: bool = False) -> None:
        before = len(self.discovered.known_free_cells())
        expected = self._expected_route_cells(cell)
        snap = self.world_gt.sense(cell, self.sensor_radius)
        self.discovered.integrate(snap, self.tick)
        self.stats["cells_discovered"] += max(0, len(self.discovered.known_free_cells()) - before)
        self._last_snapshot = snap
        for ch in self.discovered.changes:
            if ch.reason == "remembered blockage is not there" and ch.cell in self._hint_mem:
                mem, ver, host = self._hint_mem.pop(ch.cell)
                self.stats["discrepancies"] += 1
                self._ev("discrepancy", cell=ch.cell, expected="BLOCKED", observed="FREE", memory=f"#{mem:04X}")
                self._queue_write(mem, "BLOCKAGE", ch.cell, MemoryState.CLEARED, PassageState.OPEN, ver, host,
                                  reason="remembered blockage is gone")
            elif ch.new == CellState.BLOCKED:
                expected = ch.old == CellState.FREE or ch.cell in self.discovered.hint_free
                if expected and ch.cell not in self._seen_obstructions:
                    self._seen_obstructions.add(ch.cell)
                    self.stats["discrepancies"] += 1
                    self._ev("discrepancy", cell=ch.cell, expected="FREE", observed="BLOCKED")
                self._record_blockage(ch.cell, cell, on_path=True)
        for b in sorted(snap.blocked):
            if b not in self._hint_mem and b not in self._own_mem and b in expected and b not in self._seen_obstructions:
                self._seen_obstructions.add(b)
                self.stats["discrepancies"] += 1
                self._ev("discrepancy", cell=b, expected="passable (planned route / inherited map)", observed="BLOCKED (debris)")
        for b in snap.blocked:
            if b in self._hint_mem:                              # re-observed a remembered blockage: confirm it
                mem, ver, host = self._hint_mem[b]
                if b not in self._known_blocked_visits:
                    self._known_blocked_visits.add(b)
                    self._queue_write(mem, "BLOCKAGE", b, MemoryState.VERIFIED, PassageState.BLOCKED, ver, host,
                                      reason="re-observed blocked")
            elif b not in self._own_mem:
                self._record_blockage(b, cell, on_path=b in expected)        # NEW information: preserve it
        # an event of a searched type appeared (baseline: no inherited location)
        if self._searching and snap.event_here is not None and snap.event_here.event_type.value in (self.mission.search_types if self.mission else []):
            ev = snap.event_here
            self._goal_cell = (ev.row, ev.col)
            self._goal_cells = [self._goal_cell]
            self._profile = profile_for(self.capabilities, ev.event_type.value)
            ex, ey = self.frame.cell_to_local(self._goal_cell)
            ref = self.mission.memory_ref if self.mission and self.mission.memory_ref is not None else 0
            self.targets = [MissionTarget(ref, ev.event_type.value, ex, ey, ev.severity, "UNVERIFIED", memory_id=ref,
                                          version=self.mission.version_ref if self.mission else 1)]
            self.target_idx = 0
            self._searching = False
            self.movement.cancel_path()
            self._ev("target_found_by_search", cell=self._goal_cell, event=ev.event_type.value)

    def _record_blockage(self, cell: Cell, from_cell: Cell, on_path: bool) -> None:
        """Preserve a blockage this robot discovered (it affected / would affect navigation)."""
        if cell in self._own_mem or cell in self._hint_mem:
            return
        self._mem_counter += 1
        mid = memory_id_for(0x10 + (self.index or 0), self._mem_counter)   # executor prefix 0x10+ (writers use 1..15)
        self._own_mem[cell] = mid
        self.stats["blockages_recorded"] += 1
        self._ev("new_blockage", cell=cell, memory=f"#{mid:04X}")
        self._queue_write(mid, "BLOCKAGE", cell, MemoryState.VERIFIED, PassageState.BLOCKED, 0, None,
                          reason="blocked passage discovered", severity=2)

    def _queue_write(self, mem_id: int, etype: str, at_cell: Optional[Cell], state: MemoryState,
                     passage: Optional[PassageState], prev_version: int, host: Optional[int],
                     reason: str = "", severity: int = 2, xy: Optional[Tuple[float, float]] = None,
                     confidence: int = OWN_BLOCKAGE_CONFIDENCE) -> None:
        for w in self._pending_writes:                         # newer observation of the same memory replaces the queued one
            if w["mem_id"] == mem_id:
                self._pending_writes.remove(w)
                break
        if xy is None and at_cell is not None:
            xy = self.frame.cell_to_local(at_cell)
        self._pending_writes.append(dict(mem_id=mem_id, etype=etype, state=state, passage=passage,
                                         prev_version=prev_version, host=host, xy=xy, reason=reason,
                                         severity=severity, confidence=confidence))
        self._flush_writes(opportunistic=True)

    def _flush_writes(self, opportunistic: bool = False) -> None:
        """Write queued observations into beacons. While travelling only an
        already-audible beacon is used (opportunistic); at UPDATE_MEMORY a new
        beacon is dropped when none is audible (scenario-dependent)."""
        if self.link is None:
            self._pending_writes.clear()
            return
        for w in list(self._pending_writes):
            host, is_new = self._pick_host(w, allow_drop=not opportunistic)
            if host is None:
                continue
            ts = self._now_ts()
            ver = max(w["prev_version"], self._versions.get(w["mem_id"], 0)) + 1     # never reuse a version I already wrote
            x, y = w["xy"]
            msg = BeaconMessage(beacon_id=host, event_type=w["etype"], x_local=x, y_local=y, timestamp=ts,
                                severity=w["severity"], initial_confidence=w["confidence"],
                                battery_pct=max(20, int(self.battery_pct)))
            ext = MemoryExt(w["mem_id"], ver, w["state"], w["passage"], ts, self.node_id)
            pose = (self.odometry.pose.x, self.odometry.pose.y)
            cmd = ProgramCommand(host, pose[0] if is_new else 0.0, pose[1] if is_new else 0.0, (msg, ext), is_new)
            ok = self.link.program_beacon(host, encode_program(cmd))
            if ok:
                self._pending_writes.remove(w)
                self._versions[w["mem_id"]] = ver
                self.stats["memory_updates"] += 1
                self._ev("memory_write", memory=f"#{w['mem_id']:04X}", type=w["etype"], state=w["state"].value,
                         version=ver, beacon=host, new_beacon=is_new, why=w["reason"])

    def _pick_host(self, w: dict, allow_drop: bool) -> Tuple[Optional[int], bool]:
        audible = [bid for bid, _ in self.link.port.audible()]
        if self.write_policy != "new_only":
            if w["host"] is not None and w["host"] in audible:
                return w["host"], False
            if audible:
                return audible[0], False
        if allow_drop and self.write_policy != "existing_only" and self.dispenser is not None:
            bid = self.dispenser.drop()
            if bid is not None:
                self.stats["beacons_dropped"] += 1
                return bid, True
        return None, False

    # ── unreachable target ───────────────────────────────────────────────────
    def _handle_unreachable(self) -> None:
        pos = self.movement.current_cell
        blockers = self._find_blockers(pos)
        self._blockers = blockers
        self._ev("target_unreachable", blockers=[f"#{b:04X}" for b in blockers], wait=self._wait_left, retries=self._retries_left)
        if self._wait_left > 0 and blockers:
            self._set_state(ExecutorState.WAITING)
            return
        self._give_up_target()

    def _find_blockers(self, pos: Cell) -> List[int]:
        """Which remembered/observed blockages cut the way? (route that pretends they are gone)"""
        best = None
        for g in self._goal_cells:
            r = plan_route(self.discovered, pos, g, optimistic=True, relax=True)
            if r is not None and (best is None or r.cost < best.cost):
                best = r
        if best is None:
            return []
        out: List[int] = []
        for c in best.path:
            if not self.discovered.is_impassable(c):
                continue
            mem = self._own_mem.get(c) or (self._hint_mem[c][0] if c in self._hint_mem else None)
            if mem is None and self.discovered.state_of(c) == CellState.BLOCKED:
                self._record_blockage(c, pos, on_path=True)          # seen but not yet preserved
                mem = self._own_mem.get(c)
            if mem is not None and mem not in out:
                out.append(mem)
        return out

    def _t_waiting(self) -> None:
        if self._parked:
            return                                        # holding position by policy
        self._wait_left -= 1
        if self._wait_left % 15 == 0:                       # re-sense: maybe someone cleared it
            self._sense_and_integrate(self.movement.current_cell)
            if self._route_exists():
                self._ev("route_reopened")
                self._set_state(ExecutorState.DEPLOYING)
                return
        if self._wait_left <= 0:
            if self._retries_left > 0:
                self._retries_left -= 1
                self._wait_left = self.mission.policy.max_wait_ticks
                self.stats["replans"] += 1
                self._ev("retry_after_wait", retries_left=self._retries_left)
                if self._route_exists():
                    self._set_state(ExecutorState.DEPLOYING)
                    return
            else:
                self._give_up_target()

    def _route_exists(self) -> bool:
        pos = self.movement.current_cell
        return any(plan_route(self.discovered, pos, g, optimistic=True) is not None for g in self._goal_cells)

    def _give_up_target(self) -> None:
        pol = self.mission.policy
        blockers = self._blockers
        self._outcome_override = (MissionOutcome.BLOCKED if (blockers and pol.request_debris_executor)
                                  else MissionOutcome.UNREACHABLE)
        self._outcome_mem = blockers[0] if (blockers and pol.request_debris_executor) else self._target_mem_id()
        self._final_state = None
        self._set_state(ExecutorState.UPDATE_MEMORY)

    # ── ON_SITE / WORKING / VERIFYING ────────────────────────────────────────
    def _arrive_at_target(self) -> None:
        if "outbound_cells" not in self.stats or self._outbound_mission != self.stats["missions"]:
            self._outbound_mission = self.stats["missions"]
            route = self.actual_route[self._route_mark:]
            self.stats["outbound_cells"] = max(0, len(route) - 1)
            self.stats["outbound_revisits"] = len(route) - len(set(route))      # backtracking = wrong turns
            self.stats["ticks_to_site"] = self.stats["mission_ticks"]
        self._timer = ON_SITE_TICKS
        self._set_state(ExecutorState.ON_SITE)
        self._ev("on_site", cell=self.movement.current_cell)

    def _target_present(self) -> bool:
        snap = self.world_gt.sense(self.movement.current_cell, self.sensor_radius)
        self._last_snapshot = snap
        t = self.targets[self.target_idx]
        if t.event_type == "BLOCKAGE":
            return self._goal_cell in snap.blocked
        return snap.event_here is not None and snap.event_here.event_type.value == t.event_type

    def _t_on_site(self) -> None:
        self._timer -= 1
        if self._timer > 0:
            return
        self._matched = self._target_present()
        t = self.targets[self.target_idx]
        if not self._matched:
            self._ev("not_found", target=t.event_type)
            self._final_state = MemoryState.CONTRADICTED if t.event_type != "BLOCKAGE" else MemoryState.CLEARED
            self._verify_ticks_left = 0
            self._set_state(ExecutorState.VERIFYING)
            return
        prof = self._profile
        self._timer = int((prof.work_ticks if prof else 0) * self.work_scale)
        self._effect = None
        if prof is None or not prof.actuates or self._timer <= 0:
            self._final_state = MemoryState.VERIFIED if prof is None else prof.resolved_state
            self._set_state(ExecutorState.VERIFYING)
            self._verify_ticks_left = VERIFY_TICKS
            return
        self._ev("work_start", task=prof.task)
        self._set_state(ExecutorState.WORKING)

    def _t_working(self) -> None:
        self._timer -= 1
        if self._timer > 0:
            return
        cap = self._profile.capability.value
        self._effect = self.world_gt.actuate(self.movement.current_cell if self.targets[self.target_idx].event_type != "BLOCKAGE"
                                             else self._goal_cell, cap, self.tick)
        if self._effect == "DEBRIS_CLEARED":
            self._hint_mem.pop(self._goal_cell, None)          # I did that: not a discrepancy
            self.discovered.passage_info.pop(self._goal_cell, None)
        self._ev("work_done", task=self._profile.task, effect=self._effect or "no effect")
        self._verify_ticks_left = VERIFY_TICKS
        self._set_state(ExecutorState.VERIFYING)

    def _t_verifying(self) -> None:
        self._verify_ticks_left -= 1
        if self._verify_ticks_left > 0:
            return
        t = self.targets[self.target_idx]
        if self._matched:
            if self._final_state is None or self._profile is not None:
                present = self._target_present()
                prof = self._profile
                if prof is None:
                    self._final_state = MemoryState.VERIFIED
                elif not prof.actuates:
                    self._final_state = MemoryState.VERIFIED
                else:
                    self._final_state = prof.persists_state if present else prof.resolved_state
                if t.event_type == "BLOCKAGE" and not present:
                    self._final_state = MemoryState.CLEARED
        self._ev("verified", target=t.event_type, result=self._final_state.value if self._final_state else "?")
        self._outcome_override = None
        self._set_state(ExecutorState.UPDATE_MEMORY)

    # ── UPDATE_MEMORY ────────────────────────────────────────────────────────
    def _t_update_memory(self) -> None:
        outcome = self._outcome_override
        mem_id = self._outcome_mem
        t = self.targets[self.target_idx] if self.targets and self.target_idx < len(self.targets) else None
        if outcome is None and t is not None and self._final_state is not None:
            fs = self._final_state
            passage = (PassageState.OPEN if fs == MemoryState.CLEARED else PassageState.BLOCKED) if t.event_type == "BLOCKAGE" else None
            mem_id = t.memory_id if t.memory_id is not None else t.beacon_id
            conf = 95 if fs != MemoryState.CONTRADICTED else 90
            self._queue_write(mem_id, t.event_type, None, fs, passage, t.version, t.host_beacon_id,
                              reason=f"{t.event_type} {fs.value.lower()} (executor observation)",
                              severity=t.severity, xy=(t.x_local, t.y_local), confidence=conf)
            outcome = OUTCOME_FOR_STATE.get(fs, MissionOutcome.VERIFIED)
            # legacy callbacks
            if fs == MemoryState.CONTRADICTED:
                if self.on_contradicted:
                    self.on_contradicted(t.beacon_id, self.tick, "no matching event detected at target location")
            elif self.on_verified:
                self.on_verified(t.beacon_id, self.tick)
            self.reached.append(t)
        elif t is not None:
            self.reached.append(t)          # processed, but unreachable: still counts as handled
        self._flush_writes(opportunistic=False)
        if self._pending_writes and self.link is not None:
            # something could not be written yet (no beacon in range, no stock): keep it and
            # try again on the way home; do NOT block the mission on it
            self._ev("memory_write_deferred", pending=len(self._pending_writes))
        self._report_outcome(outcome or MissionOutcome.NONE, mem_id)
        # next target (multi-target legacy brief) or go home
        if outcome in (MissionOutcome.BLOCKED, MissionOutcome.UNREACHABLE) or t is None:
            if outcome in (MissionOutcome.BLOCKED, MissionOutcome.UNREACHABLE) and not self.mission.policy.return_if_unreachable:
                self._parked = True                       # policy: hold position on site (counts as busy)
                self._wait_left = 10 ** 9
                self._ev("holding_position", why="policy: do not return while the target is unreachable")
                self._set_state(ExecutorState.WAITING)
                return
            self._start_return("target unreachable" if outcome else "done")
            return
        self.target_idx += 1
        self._outcome_override = None
        if self.target_idx < len(self.targets):
            self._set_goal(self.targets[self.target_idx])
            self._set_state(ExecutorState.DEPLOYING)
        else:
            self._start_return("mission complete")

    def _report_outcome(self, outcome: MissionOutcome, mem_id: int) -> None:
        self._report(outcome, mem_id)

    # ── RETURNING ────────────────────────────────────────────────────────────
    def _start_return(self, why: str) -> None:
        self._returning_reason = why
        self.movement.cancel_path()
        if self._state not in (ExecutorState.LOW_BATTERY, ExecutorState.FAILED):
            self._set_state(ExecutorState.RETURNING)
        self._ev("returning", why=why)

    def _finish_to_home(self, why: str) -> None:
        self._start_return(why)

    def _t_returning(self) -> None:
        pos = self.movement.current_cell
        if self._pending_writes and self.link is not None and self.tick % 10 == 0:
            self._flush_writes(opportunistic=True)           # try again whenever a beacon is audible
        if pos == self.world_gt.entry and not self.movement.is_moving:
            self._arrived_home()
            return
        if not self.movement.is_moving:
            route = plan_route(self.discovered, pos, self.world_gt.entry, optimistic=False) \
                or plan_route(self.discovered, pos, self.world_gt.entry, optimistic=True)
            if route is None:
                self._ev("stranded", at=pos)
                self._set_state(ExecutorState.OUT_OF_SERVICE)
                self._finished = True
                return
            self.planned_route = route.path
            self.movement.set_path(route.path[1:])
        self._step_and_handle()

    def _arrived_home(self) -> None:
        self.movement.cancel_path()
        if self._fault:
            final = ExecutorState.NEEDS_REPAIR
        elif self._low_batt:
            final = ExecutorState.NEEDS_CHARGING
        else:
            final = ExecutorState.AVAILABLE
        if self._pending_writes and self.link is not None:
            self._flush_writes(opportunistic=False)
        self._finished = True
        self._ev("home", state=final.value, battery=int(self.battery_pct))
        self._state = final
        self._report()
        if final == ExecutorState.AVAILABLE:
            self.link and self.link.tick()

    def service(self) -> bool:
        """Workshop action (operator / maintenance crew): repair + recharge a robot that is
        NEEDS_REPAIR or NEEDS_CHARGING so it becomes AVAILABLE again. [SIMULATED]"""
        if self._state not in (ExecutorState.NEEDS_REPAIR, ExecutorState.NEEDS_CHARGING):
            return False
        self._fault = self._low_batt = False
        self.battery_pct = 100.0
        self._ev("serviced")
        self._set_state(ExecutorState.AVAILABLE)
        return True

    # ── baseline search (no inherited memory) ────────────────────────────────
    def _deploy_search(self) -> None:
        if self.movement.is_moving:
            self._step_and_handle()
            return
        if self._goal_cell is not None and not self._searching:
            return
        pos = self.movement.current_cell
        out = self._explore.next_target(self.discovered, pos, self.tick)
        if out.target is not None:
            self.stats["search_targets"] += 1
        if out.target is None:
            self._ev("search_exhausted")
            self._outcome_override = MissionOutcome.RETURNED_NO_TARGET
            self._outcome_mem = 0
            self._set_state(ExecutorState.UPDATE_MEMORY)
            self.targets = []
            return
        route = plan_route(self.discovered, pos, out.target.cell, optimistic=False)
        if route is None:
            self._ev("search_exhausted")
            self._outcome_override = MissionOutcome.RETURNED_NO_TARGET
            self._set_state(ExecutorState.UPDATE_MEMORY)
            return
        self.planned_route = route.path
        self.movement.set_path(route.path[1:])
