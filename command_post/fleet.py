"""
command_post/fleet.py
=====================
The Command Post's view of the Executor fleet: who exists, what they can do,
and what state they are in. State changes arrive ONLY as STATUS frames that
crossed the ONA — the Command Post never reads an Executor object.

An executor is dispatchable only when its last reported state is AVAILABLE
(healthy, at the entry, no mission). NEEDS_REPAIR / NEEDS_CHARGING / FAILED /
OUT_OF_SERVICE executors are never assigned a mission. [IMPLEMENTED]
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from common.enums import ExecutorState, RobotCapability

UNAVAILABLE_STATES = {
    ExecutorState.NEEDS_REPAIR, ExecutorState.NEEDS_CHARGING, ExecutorState.FAILED,
    ExecutorState.OUT_OF_SERVICE, ExecutorState.LOW_BATTERY,
}
BUSY_STATES = {
    ExecutorState.ASSIGNED, ExecutorState.DEPLOYING, ExecutorState.ON_SITE, ExecutorState.WORKING,
    ExecutorState.VERIFYING, ExecutorState.UPDATE_MEMORY, ExecutorState.WAITING,
}


@dataclass
class FleetEntry:
    executor_id: str
    capabilities: List[RobotCapability]
    state: ExecutorState = ExecutorState.AVAILABLE
    battery_pct: int = 100
    mission_id: Optional[str] = None
    last_report_tick: int = 0
    missions_done: int = 0
    reserved: bool = False      # assigned by the CP, brief not yet confirmed by a status report
    node_id: int = 0            # radio node id the executor uses on the mesh

    @property
    def available(self) -> bool:
        return self.state == ExecutorState.AVAILABLE and not self.reserved and self.mission_id is None

    @property
    def category(self) -> str:
        if self.state in UNAVAILABLE_STATES:
            return "unavailable"
        if self.state == ExecutorState.RETURNING:
            return "returning"
        if self.state in BUSY_STATES or self.reserved:
            return "busy"
        return "available"

    def specialist_for(self, cap: RobotCapability) -> bool:
        return cap in self.capabilities

    def general_covers(self, cap: RobotCapability) -> bool:
        return RobotCapability.GENERAL in self.capabilities and cap in (RobotCapability.FIRE, RobotCapability.GAS)


class FleetRegistry:
    def __init__(self) -> None:
        self._fleet: Dict[str, FleetEntry] = {}

    def register(self, executor_id: str, capabilities: List[RobotCapability], node_id: int = 0) -> FleetEntry:
        e = FleetEntry(executor_id, list(capabilities), node_id=node_id)
        self._fleet[executor_id] = e
        return e

    def get(self, executor_id: str) -> Optional[FleetEntry]:
        return self._fleet.get(executor_id)

    def all(self) -> List[FleetEntry]:
        return list(self._fleet.values())

    def available(self) -> List[FleetEntry]:
        return [e for e in self._fleet.values() if e.available]

    def update_from_status(self, executor_id: str, state_value: str, battery: int, tick: int) -> Optional[FleetEntry]:
        e = self._fleet.get(executor_id)
        if e is None:
            return None
        e.state = ExecutorState(state_value)
        e.battery_pct = battery
        e.last_report_tick = tick
        if e.state == ExecutorState.AVAILABLE:
            e.mission_id = None
            e.reserved = False
        return e

    def counts(self) -> Dict[str, int]:
        out = {"available": 0, "busy": 0, "returning": 0, "unavailable": 0}
        for e in self._fleet.values():
            out[e.category] += 1
        return out
