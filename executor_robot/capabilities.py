"""
executor_robot/capabilities.py
==============================
What each Executor CLASS does when it arrives at a target. One reusable
Executor implementation + a capability profile per class — NOT ten robot
implementations. The Command Post reasons about the capability; the Executor
applies the profile.

    capability   handles event   task (WORKING)      resolved ->   persists ->
    FIRE         FIRE            suppress fire       CLEARED       ESCALATED
    GAS          GAS             ventilate / seal    CLEARED       ACTIVE
    VICTIM       VICTIM_PRESENCE assess victim       (n/a)         ESCALATED
    DEBRIS       BLOCKAGE        clear passage       CLEARED       ESCALATED
    GENERAL      FIRE, GAS       (verify only)       --            VERIFIED

[SIMULATED] task durations are scenario knobs, not measured actuator times.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from common.enums import MemoryState, RobotCapability


@dataclass(frozen=True)
class CapabilityProfile:
    capability: RobotCapability
    event_types: Tuple[str, ...]
    task: str
    work_ticks: int
    resolved_state: MemoryState
    persists_state: MemoryState
    actuates: bool = True            # False = verification only


PROFILES: Dict[RobotCapability, CapabilityProfile] = {
    RobotCapability.FIRE: CapabilityProfile(RobotCapability.FIRE, ("FIRE",), "suppress fire", 50,
                                            MemoryState.CLEARED, MemoryState.ESCALATED),
    RobotCapability.GAS: CapabilityProfile(RobotCapability.GAS, ("GAS",), "ventilate / seal", 40,
                                           MemoryState.CLEARED, MemoryState.ACTIVE),
    RobotCapability.VICTIM: CapabilityProfile(RobotCapability.VICTIM, ("VICTIM_PRESENCE",), "assess victim", 30,
                                              MemoryState.CLEARED, MemoryState.ESCALATED),
    RobotCapability.DEBRIS: CapabilityProfile(RobotCapability.DEBRIS, ("BLOCKAGE",), "clear passage", 60,
                                              MemoryState.CLEARED, MemoryState.ESCALATED),
    RobotCapability.GENERAL: CapabilityProfile(RobotCapability.GENERAL, ("FIRE", "GAS"), "verify only", 0,
                                               MemoryState.VERIFIED, MemoryState.VERIFIED, actuates=False),
}


def profile_for(capabilities, event_type: str) -> Optional[CapabilityProfile]:
    """Best profile an executor with `capabilities` has for `event_type`
    (specialist first, GENERAL as fallback), or None."""
    for cap in capabilities:
        pr = PROFILES.get(cap)
        if pr is not None and cap != RobotCapability.GENERAL and event_type in pr.event_types:
            return pr
    if RobotCapability.GENERAL in capabilities and event_type in PROFILES[RobotCapability.GENERAL].event_types:
        return PROFILES[RobotCapability.GENERAL]
    return None


def fleet_capabilities(spec: Dict[str, int]) -> list:
    """{'FIRE': 3, 'VICTIM': 3, ...} -> [('E01', [FIRE]), ('E02', [FIRE]), ...] in order."""
    out, n = [], 0
    for cap_name, count in spec.items():
        for _ in range(count):
            n += 1
            out.append((f"E{n:02d}", [RobotCapability(cap_name)]))
    return out


DEFAULT_FLEET: Dict[str, int] = {"FIRE": 3, "VICTIM": 3, "GAS": 2, "DEBRIS": 2}
