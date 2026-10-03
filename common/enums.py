"""
common/enums.py
================
Shared enumerations — single source of truth for every state string.

PATCH 1 additions: WriterState, ExecutorState, MissionStatus, RobotCapability
All original enums are unchanged.

[IMPLEMENTED]
"""
from __future__ import annotations
from enum import Enum


# ── original enums (unchanged) ─────────────────────────────────────────────────

class EventType(str, Enum):
    """Detectable event types.

    FIRE / GAS are the two TSYP14-required types, backed by physical sensors
    (KY-026, MQ-2) in the hardware plan.

    VICTIM_PRESENCE / STRUCTURAL are [SIMULATION-ONLY] extensions — no physical
    sensor exists for these in Phase 1.  They allow richer scenarios and
    capability-matching demos without implying any hardware commitment.
    """
    FIRE             = "FIRE"
    GAS              = "GAS"
    VICTIM_PRESENCE  = "VICTIM_PRESENCE"  # [SIMULATION-ONLY]
    STRUCTURAL       = "STRUCTURAL"        # [SIMULATION-ONLY]
    # PHASE-1 PATCH: navigation-relevant memory. A BLOCKAGE record says "a
    # passage here is obstructed" (debris / collapse). Its `passage_state`
    # (PassageState) says how — see common/memory.py. [SIMULATED]
    BLOCKAGE         = "BLOCKAGE"


class MemoryState(str, Enum):
    """
    Lifecycle of a memory record in the Living Map.

    UNVERIFIED   → created when the Writer's observation reaches the ONA.
    VERIFIED     → Executor re-observed the same event and it matched.
    CONTRADICTED → Executor re-observed the location and it did NOT match.
    ACTIVE       → PHASE-1 PATCH: verified AND the problem still exists
                   (e.g. victim located, gas still present after response).
    ESCALATED    → PHASE-1 PATCH: problem persists and needs additional action
                   (another executor class, or a human team).
    CLEARED      → PHASE-1 PATCH: problem no longer present (resolved).

    Staleness (live_confidence < threshold) is a *derived flag*, never a state
    transition — it never overwrites VERIFIED/CONTRADICTED. See common/memory.py.
    """
    UNVERIFIED   = "UNVERIFIED"
    VERIFIED     = "VERIFIED"
    CONTRADICTED = "CONTRADICTED"
    ACTIVE       = "ACTIVE"
    ESCALATED    = "ESCALATED"
    CLEARED      = "CLEARED"


class NavMode(str, Enum):
    """Modes for navigation/engine.py (shared by Writer and Executor)."""
    EXPLORE = "EXPLORE"  # Writer: frontier-based autonomous exploration
    EXECUTE = "EXECUTE"  # Executor: travel toward a mission waypoint
    VERIFY  = "VERIFY"   # Executor: re-observe at a waypoint
    RETURN  = "RETURN"   # either robot: return to entry (Phase 2)


class CellState(str, Enum):
    """A cell's state in a robot's DISCOVERED map."""
    UNKNOWN = "UNKNOWN"
    FREE    = "FREE"
    WALL    = "WALL"
    # PHASE-1 PATCH: a cell that is normally passable but currently obstructed
    # (debris). Unlike WALL it can change back to FREE when a later observation
    # (or a Debris Executor) clears it.
    BLOCKED = "BLOCKED"


class PassageState(str, Enum):
    """
    PHASE-1 PATCH: navigation knowledge attached to a place, preserved in
    BLOCKAGE memory records and in a robot's DiscoveredWorld.passage_info.

    OPEN                 observed passable (used when a blockage is cleared)
    BLOCKED              observed obstructed; do not route through it
    IMPASSABLE           obstructed and not clearable by any fleet capability
    TEMPORARILY_BLOCKED  obstructed but expected to clear; route through only
                         as a last resort (heavy cost penalty)
    DETOUR_REQUIRED      passable only with a detour / degraded (cost penalty)
    ACCESSIBILITY_UNKNOWN reported but not verified (mild cost penalty)
    """
    OPEN                  = "OPEN"
    BLOCKED               = "BLOCKED"
    IMPASSABLE            = "IMPASSABLE"
    TEMPORARILY_BLOCKED   = "TEMPORARILY_BLOCKED"
    DETOUR_REQUIRED       = "DETOUR_REQUIRED"
    ACCESSIBILITY_UNKNOWN = "ACCESSIBILITY_UNKNOWN"


# ── PATCH 1 additions ──────────────────────────────────────────────────────────

class WriterState(str, Enum):
    """
    Observable state of a WriterRobot instance.
    Derived in LiveMapSystem from writer.dead — not stored separately.
    """
    IDLE      = "IDLE"       # created but start() not yet called
    EXPLORING = "EXPLORING"  # actively exploring
    DEAD      = "DEAD"       # failed / lost before getting back to the entry
    OFFLINE   = "OFFLINE"    # removed from active fleet (Phase 2: hardware fail)
    RETURNING = "RETURNING"  # PHASE-1 PATCH: exploration over, heading back to entry
    RETURNED  = "RETURNED"   # PHASE-1 PATCH: back at the entry, did NOT shut down


class ExecutorState(str, Enum):
    """
    Observable state of an ExecutorRobot instance (mission lifecycle).

        AVAILABLE -> ASSIGNED -> DEPLOYING -> ON_SITE -> WORKING -> VERIFYING
                  -> UPDATE_MEMORY -> RETURNING -> AVAILABLE

    Failure / service branches:
        WORKING|DEPLOYING -> FAILED -> NEEDS_REPAIR
        DEPLOYING|WORKING -> LOW_BATTERY -> RETURNING -> NEEDS_CHARGING

    Names that existed before the Phase-1 patch are kept as enum ALIASES so
    old code keeps working: IDLE==AVAILABLE, READY==ASSIGNED,
    ACTIVE==DEPLOYING, OFFLINE==OUT_OF_SERVICE.
    COMPLETED is kept for compatibility but is no longer a resting state: an
    executor that finishes its tasks goes on to UPDATE_MEMORY / RETURNING and
    ends AVAILABLE (healthy) or NEEDS_REPAIR / NEEDS_CHARGING.
    """
    AVAILABLE      = "AVAILABLE"        # at the entry, healthy, no mission
    ASSIGNED       = "ASSIGNED"         # brief received from the ONA, not yet moving
    DEPLOYING      = "DEPLOYING"        # travelling to the mission site
    ON_SITE        = "ON_SITE"          # at the target, observing / confirming
    WORKING        = "WORKING"          # performing the capability task
    VERIFYING      = "VERIFYING"        # re-sensing to determine the outcome
    UPDATE_MEMORY  = "UPDATE_MEMORY"    # writing the new observation into a beacon
    RETURNING      = "RETURNING"        # heading back to the entry / ONA
    COMPLETED      = "COMPLETED"        # [LEGACY] transient marker, not a resting state
    LOW_BATTERY    = "LOW_BATTERY"      # battery below the return threshold
    NEEDS_CHARGING = "NEEDS_CHARGING"   # back at entry, unavailable until recharged
    FAILED         = "FAILED"           # fault detected mid-mission
    NEEDS_REPAIR   = "NEEDS_REPAIR"     # back at entry, unavailable until repaired
    OUT_OF_SERVICE = "OUT_OF_SERVICE"   # lost / permanently unavailable
    WAITING        = "WAITING"          # parked near an obstruction, waiting for clearance / retry

    # legacy aliases (same value => same member)
    IDLE    = "AVAILABLE"
    READY   = "ASSIGNED"
    ACTIVE  = "DEPLOYING"
    OFFLINE = "OUT_OF_SERVICE"


class MissionStatus(str, Enum):
    """
    Lifecycle of a dispatched mission.
    Used by MissionDispatcher (PATCH 3) and the mission queue.
    """
    QUEUED      = "QUEUED"      # created, waiting for a free executor
    ASSIGNED    = "ASSIGNED"    # executor selected, mission about to load
    IN_PROGRESS = "IN_PROGRESS" # executor actively working it
    COMPLETED   = "COMPLETED"   # all targets processed
    FAILED      = "FAILED"      # executor went offline mid-mission
    BLOCKED     = "BLOCKED"     # PHASE-1 PATCH: waiting for a passage to be cleared
    CANCELLED   = "CANCELLED"   # PHASE-1 PATCH: memory CLEARED/CONTRADICTED before dispatch


class RobotCapability(str, Enum):
    """
    Lightweight capability tag for capability-matching (PATCH 4).
    An executor with GENERAL can be assigned any mission type.

    FIRE / GAS match the physical sensor suite.
    VICTIM / STRUCTURAL are [SIMULATION-ONLY] for richer scenario demos.
    """
    GENERAL    = "GENERAL"
    FIRE       = "FIRE"
    GAS        = "GAS"
    VICTIM     = "VICTIM"      # [SIMULATION-ONLY]
    STRUCTURAL = "STRUCTURAL"  # [SIMULATION-ONLY]
    DEBRIS     = "DEBRIS"      # PHASE-1 PATCH [SIMULATION-ONLY] clears BLOCKAGE records
