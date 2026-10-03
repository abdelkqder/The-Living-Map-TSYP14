"""
command_post/command_post.py
============================
The Command Post — the ONLY mission planner in the system.

    ONA --(validated, GPS-enriched memory / heartbeats / status)--> Command Post
    Command Post: builds the Living Map, prioritises, selects an executor,
                  builds the mission brief, hands it to the ONA mailbox.

What the Command Post does NOT do: read a robot object, talk to a robot
directly, or see the ground truth. Everything it knows came through the ONA.

Structured, explainable decisions: every assignment appends a dict to
`decisions` (mission, memory record, executor, priority components, selection
reason) — the UI and the demos print these. [IMPLEMENTED]
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from common.clock import SimClock
from common.enums import ExecutorState, MemoryState, MissionStatus, RobotCapability
from common.mission import BlockagePolicy, HazardHint, Mission, MissionTarget
from common.protocol import MissionOutcome, node_label, ONA_NODE_ID, BROADCAST_ID
from command_post.fleet import FleetEntry, FleetRegistry
from command_post.map_state import LivingMap
from command_post.mission_dispatcher import MissionDispatcher, MissionRecord
from command_post.planner import BLOCKER_BOOST, CAPABILITY_FOR_EVENT, choose_executor, net_priority
from gateway.ona_gateway import ONAGateway, UplinkHeartbeat, UplinkMemory, UplinkStatus

_CLOSING = {MissionOutcome.VERIFIED, MissionOutcome.CONTRADICTED, MissionOutcome.CLEARED,
            MissionOutcome.ESCALATED, MissionOutcome.RETURNED_NO_TARGET}


@dataclass
class CommandPostConfig:
    auto_dispatch: bool = True
    dispatch_threshold: Optional[float] = None   # None = the dispatcher's own DISPATCH_THRESHOLD
    blockage_policy: str = "on_demand"        # "on_demand" | "proactive" | "never"
    inherit_memory: bool = True               # False = baseline: brief carries no inherited map
    replan_period: int = 20
    brief_policy: BlockagePolicy = field(default_factory=BlockagePolicy)


class CommandPost:
    def __init__(self, ona: ONAGateway, clock: SimClock, config: Optional[CommandPostConfig] = None) -> None:
        self.ona = ona
        self.clock = clock
        self.cfg = config or CommandPostConfig()
        self.living_map = LivingMap(clock=clock.now)
        self.dispatcher = MissionDispatcher()
        self.fleet = FleetRegistry()
        self.decisions: List[Dict[str, Any]] = []
        self.log: List[str] = []
        self.waiting_on: Dict[int, List[str]] = {}          # blockage memory id -> blocked mission ids
        self._status_seq: Dict[str, int] = {}
        self._dirty = True
        self.tick_now = 0
        self.stats = dict(memory_in=0, heartbeats_in=0, status_in=0, briefs_sent=0, stale_status_dropped=0)

    def register_executor(self, executor_id: str, capabilities, node_id: int = 0) -> FleetEntry:
        return self.fleet.register(executor_id, capabilities, node_id)

    # ── uplink handlers (called by the ONA) ──────────────────────────────────
    def on_memory(self, item: UplinkMemory) -> bool:
        self.stats["memory_in"] += 1
        t = self.tick_now
        rec = self.living_map.ingest(item.msg, item.ext, hop_count=item.hop_count, tick=t)
        mid = rec.memory_id
        mr = self.dispatcher.mission_for_memory(mid)
        if not rec.is_open:
            if mr is not None and mr.status in (MissionStatus.QUEUED, MissionStatus.BLOCKED):
                self.dispatcher.cancel(mr, t, f"memory {rec.state.value}")
            if rec.event_type == "BLOCKAGE" and rec.state == MemoryState.CLEARED:
                for blocked_id in self.waiting_on.pop(mid, []):
                    bm = self.dispatcher._find(blocked_id)
                    if bm is not None:
                        self.dispatcher.unblock(bm, t)
                        self._note(f"blockage #{mid:04X} CLEARED -> mission {blocked_id} re-queued")
            self._dirty = True
            return True
        # Missions that are finished / parked are not re-created by later updates.
        if mr is None or mr.status in (MissionStatus.QUEUED, MissionStatus.ASSIGNED, MissionStatus.IN_PROGRESS):
            self.dispatcher.on_new_record(rec, t)
            mr = self.dispatcher.mission_for_memory(mid)
            if mr is not None and mr.status == MissionStatus.QUEUED:
                self.dispatcher.refresh_priority(mr, rec, t)
        self._dirty = True
        return True

    def on_heartbeat(self, item: UplinkHeartbeat) -> bool:
        self.stats["heartbeats_in"] += 1
        self.living_map.heartbeat(item.hb, item.gps, item.hop_count, self.tick_now)
        return True

    def _mission_of(self, executor_id: str) -> Optional[MissionRecord]:
        for mr in self.dispatcher.all_missions():
            if mr.assigned_executor_id == executor_id and mr.status in (MissionStatus.ASSIGNED, MissionStatus.IN_PROGRESS):
                return mr
        return None

    def on_status(self, item: UplinkStatus) -> bool:
        self.stats["status_in"] += 1
        st, t = item.status, self.tick_now
        entry = self.fleet.get(st.executor_id)
        if entry is None:
            return True
        # An OUTCOME is applied even if a newer status already arrived (it is
        # idempotent); the executor STATE is only taken from the newest report.
        mr = self.dispatcher.mission_for_memory(st.memory_id) if st.memory_id else None
        if mr is None or mr.assigned_executor_id != st.executor_id or mr.status not in (MissionStatus.ASSIGNED, MissionStatus.IN_PROGRESS):
            mr = self._mission_of(st.executor_id)
        if st.outcome != MissionOutcome.NONE and mr is not None:
            self._on_outcome(entry, mr, st.outcome, st.memory_id)
        last = self._status_seq.get(st.executor_id)
        if last is not None and ((item.seq - last) & 0xFFFF) >= 0x8000:
            self.stats["stale_status_dropped"] += 1
            return True
        self._status_seq[st.executor_id] = item.seq
        self.fleet.update_from_status(st.executor_id, st.state, st.battery_pct, t)
        if entry.state == ExecutorState.DEPLOYING:
            self.dispatcher.mark_in_progress(st.executor_id)
        if entry.state == ExecutorState.AVAILABLE:
            self._dirty = True
        return True

    def _on_outcome(self, entry: FleetEntry, mr: MissionRecord, oc: MissionOutcome, mem_id: int) -> None:
        t = self.tick_now
        if oc in _CLOSING:
            self.dispatcher.finish(mr, t, oc.name)
            entry.missions_done += 1
            self._note(f"{entry.executor_id} reported {oc.name} for mission {mr.mission_id} (memory #{mr.beacon_id:04X})")
        elif oc in (MissionOutcome.BLOCKED, MissionOutcome.UNREACHABLE):
            self.dispatcher.block(mr, t, f"{entry.executor_id}: target unreachable")
            if oc == MissionOutcome.BLOCKED and mem_id:
                # the status names the BLOCKAGE memory that cuts the way (mission id comes from the assignment)
                self.waiting_on.setdefault(mem_id, [])
                if mr.mission_id not in self.waiting_on[mem_id]:
                    self.waiting_on[mem_id].append(mr.mission_id)
                self._note(f"{entry.executor_id}: mission {mr.mission_id} BLOCKED by memory #{mem_id:04X} — clearance requested")
            else:
                self._note(f"{entry.executor_id}: mission {mr.mission_id} UNREACHABLE — no clearance requested")
        elif oc in (MissionOutcome.FAILED, MissionOutcome.ABORTED_LOW_BATTERY):
            self.dispatcher.on_mission_failed(entry.executor_id, t)
            self._note(f"{entry.executor_id} {oc.name}: mission {mr.mission_id} re-queued")
        self._dirty = True

    # ── planning ─────────────────────────────────────────────────────────────
    def _boost_for(self, mr: MissionRecord) -> float:
        waiting = self.waiting_on.get(mr.beacon_id)
        if not waiting:
            return 0.0
        best = 0.0
        for mid in waiting:
            bm = self.dispatcher._find(mid)
            if bm is not None:
                best = max(best, bm.priority_score)
        return BLOCKER_BOOST * best

    def _eligible(self, mr: MissionRecord, rec, auto: bool) -> Tuple[bool, str]:
        if rec is None or not rec.is_open:
            return False, "memory no longer open"
        if rec.event_type == "BLOCKAGE":
            demanded = mr.beacon_id in self.waiting_on
            if self.cfg.blockage_policy == "never":
                return False, "blockage missions disabled"
            if self.cfg.blockage_policy == "on_demand" and not demanded:
                return False, "blockage not blocking any mission (on_demand policy)"
        thr = self.cfg.dispatch_threshold if self.cfg.dispatch_threshold is not None else self.dispatcher.DISPATCH_THRESHOLD
        if auto and mr.priority_score < thr and mr.beacon_id not in self.waiting_on:
            return False, f"score {mr.priority_score:.1f} below dispatch threshold {thr}"
        return True, ""

    def ranked_missions(self, auto: bool = True) -> List[Tuple[MissionRecord, Any, Any]]:
        out = []
        for mr in list(self.dispatcher._queue):
            if mr.status != MissionStatus.QUEUED:
                continue
            rec = self.living_map.get(mr.beacon_id)
            ok, _ = self._eligible(mr, rec, auto)
            if not ok:
                continue
            pr = net_priority(mr, rec, self._boost_for(mr))
            out.append((mr, rec, pr))
        out.sort(key=lambda t: (-t[2].score, t[0].mission_id))
        return out

    def plan(self, max_assignments: Optional[int] = None, auto: bool = True) -> List[Dict[str, Any]]:
        """Assign queued missions to AVAILABLE executors, best first. Returns the new decisions."""
        made: List[Dict[str, Any]] = []
        for mr, rec, pr in self.ranked_missions(auto):
            if max_assignments is not None and len(made) >= max_assignments:
                break
            choice = choose_executor(mr.required_capability, self.fleet)
            if choice.executor is None:
                continue
            made.append(self._assign(mr, rec, pr, choice))
        self._dirty = False
        return made

    def _assign(self, mr: MissionRecord, rec, pr, choice) -> Dict[str, Any]:
        entry: FleetEntry = choice.executor
        brief = self.build_brief(mr, rec, entry)
        self.dispatcher.assign(mr, entry.executor_id, self.tick_now)
        entry.reserved = True
        entry.mission_id = mr.mission_id
        self.ona.queue_brief(entry.executor_id, brief)
        self.stats["briefs_sent"] += 1
        decision = {
            "tick": self.tick_now, "mission_id": mr.mission_id, "memory_id": mr.beacon_id,
            "event_type": rec.event_type, "state": rec.state.value, "executor": entry.executor_id,
            "priority": pr.score, "components": pr.components, "selection": choice.reason,
        }
        self.decisions.append(decision)
        self._note(f"ASSIGN {mr.mission_id} ({rec.event_type} #{mr.beacon_id:04X}, priority {pr.score}) -> "
                   f"{entry.executor_id}: {choice.reason['why']}")
        return decision

    def dispatch_next(self) -> Optional[Dict[str, Any]]:
        """Operator action: assign the single best queued mission now (ignores the auto threshold)."""
        made = self.plan(max_assignments=1, auto=False)
        return made[0] if made else None

    def build_brief(self, mr: MissionRecord, rec, entry: FleetEntry) -> Mission:
        """Compose what the Executor inherits. The Command Post composes it, the ONA only carries it."""
        reason = (f"{rec.event_type} #{mr.beacon_id:04X} state={rec.state.value} "
                  f"conf={rec.confidence_pct}% age={rec.age_seconds:.0f}s")
        if not self.cfg.inherit_memory:           # baseline: no inherited spatial memory
            return Mission(mission_id=mr.mission_id, targets=[], search_types=[rec.event_type],
                           memory_ref=mr.beacon_id, version_ref=rec.version,
                           policy=self.cfg.brief_policy, issued_tick=self.tick_now,
                           reason="BASELINE (no inherited map): " + reason)
        target = MissionTarget(beacon_id=mr.beacon_id, event_type=rec.event_type, x_local=rec.x_local,
                               y_local=rec.y_local, severity=rec.severity, state=rec.state.value,
                               memory_id=mr.beacon_id, version=rec.version,
                               host_beacon_id=rec.host_beacon_id)
        waypoints: List[Tuple[float, float]] = []
        b, seen = rec.host_beacon_id, set()
        while b is not None and b in self.living_map.beacons and b not in seen:
            seen.add(b)
            h = self.living_map.beacons[b]
            waypoints.append((h.x_local, h.y_local))
            b = h.parent_id if h.parent_id not in (ONA_NODE_ID, BROADCAST_ID) else None
        waypoints.reverse()
        hazards = [h for h in self.living_map.hazard_hints()]
        return Mission(mission_id=mr.mission_id, targets=[target], waypoints=waypoints, hazards=hazards,
                       policy=self.cfg.brief_policy, issued_tick=self.tick_now, reason=reason)

    # ── clock ────────────────────────────────────────────────────────────────
    def tick(self, t: int) -> None:
        self.tick_now = t
        if self.cfg.auto_dispatch and (self._dirty or t % self.cfg.replan_period == 0):
            self.plan(auto=True)

    def _note(self, text: str) -> None:
        self.log.append(f"[t={self.tick_now}] {text}")
        if len(self.log) > 400:
            self.log.pop(0)

    # ── read-only views for UI / reports ─────────────────────────────────────
    def mission_table(self) -> List[MissionRecord]:
        return self.dispatcher.all_missions()
