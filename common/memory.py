"""
common/memory.py
=================
The LOGICAL memory model — what the Living Map actually reasons about.
Deliberately separate from common/protocol.py's BeaconMessage (the physical
radio packet). A MemoryRecord is created from a received BeaconMessage but
carries state that never crosses the radio link: verification status,
a short human-readable history, and who last touched it.

Rule from the project vision: "separate physical beacon data from ONA /
command-post metadata." This module is the metadata side.

[IMPLEMENTED]
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from common.enums import MemoryState, PassageState
from common.protocol import BeaconMessage, DECAY_LAMBDA, MIN_CONFIDENCE, STALE_THRESHOLD

# How much a lifecycle state raises mission relevance (see mission_relevance).
_STATE_RELEVANCE = {
    MemoryState.UNVERIFIED: 0.7, MemoryState.VERIFIED: 0.9, MemoryState.ACTIVE: 1.0,
    MemoryState.ESCALATED: 1.0, MemoryState.CONTRADICTED: 0.0, MemoryState.CLEARED: 0.0,
}


@dataclass
class MemoryRecord:
    """
    Command-post-side representation of one piece of spatial memory.

    `state` starts UNVERIFIED the moment the ONA forwards a beacon reading.
    It moves to VERIFIED or CONTRADICTED only when an Executor re-observes
    the same location (executor_robot/executor.py). `is_stale` is a derived
    property from aging, not a state transition — a stale record keeps
    whatever state (VERIFIED/CONTRADICTED/UNVERIFIED) it already had; going
    stale must never erase the historical record (per the vision doc).

    [IMPLEMENTED]
    """

    beacon_id:          int
    event_type:         str
    x_local:            float
    y_local:            float
    timestamp:           int     # deployment time (unix seconds), from the packet
    severity:            int
    initial_confidence:  int
    gps_lat: Optional[float] = None
    gps_lon: Optional[float] = None

    version:      int = 1                 # PATCH 7: message version counter
    state:        MemoryState = MemoryState.UNVERIFIED
    verified_by:  Optional[str] = None   # e.g. "executor"
    history:      List[str] = field(default_factory=list)

    # ── PHASE-1 PATCH: richer record (all optional -> old call sites unchanged) ──
    memory_id:           Optional[int] = None    # event/memory identity (default = beacon_id)
    source:              str = "writer"          # who made the LATEST observation (label)
    source_node:         int = 0                 # radio node id of that observer (0 = unknown)
    last_update_ts:      Optional[int] = None    # time of latest observation (default = timestamp)
    observed_confidence: Optional[int] = None    # sensor confidence at that observation
    passage_state:       Optional[PassageState] = None   # only for BLOCKAGE records
    battery_pct:         Optional[int] = None
    hop_count:           Optional[int] = None    # radio hops the latest packet took
    host_beacon_id:      Optional[int] = None    # beacon currently hosting the record
    corroborations:      int = 0                 # independent re-reports merged into this record
    received_tick:       Optional[int] = None
    # Clock used for ageing. Default = wall clock (legacy); the simulation
    # injects its SimClock.now so ageing follows simulated time.
    clock: Callable[[], float] = field(default=time.time, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.memory_id is None:
            self.memory_id = self.beacon_id
        if self.observed_confidence is None:
            self.observed_confidence = self.initial_confidence
        if self.host_beacon_id is None:
            self.host_beacon_id = self.beacon_id

    # ── Construction ─────────────────────────────────────────────────────────
    @classmethod
    def from_beacon_message(cls, msg: BeaconMessage) -> "MemoryRecord":
        rec = cls(
            beacon_id=msg.beacon_id, event_type=msg.event_type,
            x_local=msg.x_local, y_local=msg.y_local, timestamp=msg.timestamp,
            severity=msg.severity, initial_confidence=msg.initial_confidence,
            gps_lat=msg.gps_lat, gps_lon=msg.gps_lon,
            version=msg.version,
        )
        rec.history.append(f"t={msg.timestamp} UNVERIFIED (created from beacon #{msg.beacon_id})")
        return rec

    # ── Aging — ONE authoritative implementation, reused from common.protocol ──
    @property
    def observed_ts(self) -> int:
        """Time of the latest observation: the last update, else creation."""
        return self.timestamp if self.last_update_ts is None else self.last_update_ts

    @property
    def age_seconds(self) -> float:
        """Seconds since the latest OBSERVATION (creation or executor update)."""
        return max(0.0, self.clock() - self.observed_ts)

    @property
    def record_age_seconds(self) -> float:
        """Seconds since the record was first created."""
        return max(0.0, self.clock() - self.timestamp)

    @property
    def live_confidence(self) -> float:
        decayed = (self.observed_confidence / 100.0) * math.exp(-DECAY_LAMBDA * self.age_seconds)
        return max(MIN_CONFIDENCE, decayed)

    @property
    def mission_relevance(self) -> float:
        """0..1 — how much this record should drive a mission right now.
        severity x live confidence x lifecycle factor (CLEARED/CONTRADICTED -> 0)."""
        if self.state in (MemoryState.CLEARED, MemoryState.CONTRADICTED):
            return 0.0
        return round((self.severity / 4.0) * self.live_confidence * _STATE_RELEVANCE[self.state], 3)

    @property
    def is_open(self) -> bool:
        """True while the record still describes a live problem."""
        return self.state not in (MemoryState.CLEARED, MemoryState.CONTRADICTED)

    @property
    def confidence_pct(self) -> int:
        return round(self.live_confidence * 100)

    @property
    def is_stale(self) -> bool:
        return self.live_confidence < STALE_THRESHOLD

    # ── State transitions ────────────────────────────────────────────────────
    def mark_verified(self, by: str = "executor", at_tick: Optional[int] = None) -> None:
        """An Executor re-observed this location and it matched. [IMPLEMENTED]"""
        self.state = MemoryState.VERIFIED
        self.verified_by = by
        self.history.append(f"tick={at_tick} VERIFIED by {by}")

    def mark_contradicted(self, by: str = "executor", at_tick: Optional[int] = None,
                           reason: str = "") -> None:
        """An Executor re-observed this location and it did NOT match. [IMPLEMENTED]"""
        self.state = MemoryState.CONTRADICTED
        self.verified_by = by
        note = f" ({reason})" if reason else ""
        self.history.append(f"tick={at_tick} CONTRADICTED by {by}{note}")

    def observe(self, state: MemoryState, *, by: str, at_tick: Optional[int] = None,
                confidence: Optional[int] = None, severity: Optional[int] = None,
                passage: Optional[PassageState] = None, reason: str = "",
                now_ts: Optional[int] = None) -> None:
        """
        PHASE-1: a NEW OBSERVATION changes the record. Bumps `version` (so the
        Command Post can tell new from old), refreshes the observation time
        (ageing restarts from this observation) and records who saw it.
        [IMPLEMENTED]
        """
        self.state = state
        self.verified_by = by
        self.source = by
        self.version += 1
        self.last_update_ts = int(self.clock()) if now_ts is None else now_ts
        if confidence is not None:
            self.observed_confidence = confidence
        if severity is not None:
            self.severity = severity
        if passage is not None:
            self.passage_state = passage
        note = f" ({reason})" if reason else ""
        self.history.append(f"tick={at_tick} v{self.version} {state.value} by {by}{note}")

    # ── Serialisation ────────────────────────────────────────────────────────
    def to_dict(self) -> dict:
        return {
            "memory_id": self.memory_id, "host_beacon_id": self.host_beacon_id,
            "version": self.version, "source": self.source,
            "passage_state": self.passage_state.value if self.passage_state else None,
            "mission_relevance": self.mission_relevance,
            "last_update_ts": self.observed_ts, "hop_count": self.hop_count,
            "beacon_id": self.beacon_id, "event_type": self.event_type,
            "x_local": round(self.x_local, 3), "y_local": round(self.y_local, 3),
            "severity": self.severity, "state": self.state.value,
            "verified_by": self.verified_by,
            "live_confidence_pct": self.confidence_pct, "is_stale": self.is_stale,
            "age_s": round(self.age_seconds, 1),
            "gps_lat": self.gps_lat, "gps_lon": self.gps_lon,
            "history": list(self.history),
        }

    def __str__(self) -> str:
        return (f"Mem#{self.beacon_id:02d}[{self.event_type}] state={self.state.value} "
                f"conf={self.confidence_pct}% age={self.age_seconds:.0f}s")
