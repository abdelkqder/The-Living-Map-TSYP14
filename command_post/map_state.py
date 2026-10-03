"""
command_post/map_state.py
==========================
The Living Map itself — an aggregate of MemoryRecords whose state evolves over
time. Records are keyed by MEMORY id (== beacon id for the legacy single-record
beacon), because one physical beacon can now host several records.

Lifecycle (every transition bumps `version`, so old information can never
overwrite newer information):

    UNVERIFIED --Executor confirms--> VERIFIED --still there--> ACTIVE
                                          |--needs more action--> ESCALATED
    any open state --Executor: gone--> CLEARED
    any open state --Executor: not there / different--> CONTRADICTED
    staleness is a DERIVED flag (confidence decay), never a state change

The Command Post learns about changes ONLY from frames that crossed the ONA
(`ingest(..., ext=...)`); there is no other way to mutate a record from the
outside except the legacy `mark_verified` / `mark_contradicted` helpers kept
for existing tests.

[IMPLEMENTED]
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Optional, Tuple

from common.enums import MemoryState, PassageState
from common.memory import MemoryRecord
from common.mission import HazardHint
from common.protocol import BeaconMessage, MemoryExt, node_label


@dataclass
class BeaconHealth:
    """What the Command Post knows about one physical beacon (from heartbeats)."""
    beacon_id:      int
    x_local:        float
    y_local:        float
    gps:            Tuple[float, float]
    hops_to_ona:    int
    parent_id:      int
    battery_pct:    int
    n_records:      int
    max_version:    int
    last_update_ts: int
    last_seen_tick: int
    hop_count:      int


class LivingMap:
    """[IMPLEMENTED] In-memory. [PLANNED] Persistent storage in Phase 2."""

    def __init__(self, clock: Optional[Callable[[], float]] = None) -> None:
        self._records: Dict[int, MemoryRecord] = {}
        self.log: List[str] = []
        self._clock = clock
        self.beacons: Dict[int, BeaconHealth] = {}
        self.updates: Deque[Tuple[int, int, str]] = deque(maxlen=40)   # (tick, memory_id, text)
        self.stale_discarded = 0
        self.duplicates_discarded = 0

    # ── ingest ───────────────────────────────────────────────────────────────
    def ingest(self, msg: BeaconMessage, ext: Optional[MemoryExt] = None, *,
               hop_count: Optional[int] = None, tick: Optional[int] = None) -> MemoryRecord:
        """
        A memory record arrived via the ONA.
          legacy (ext=None): creates an UNVERIFIED record keyed by beacon id.
          mesh   (ext given): creates the record, or applies a NEWER version of it.
        An equal or older version is discarded (it must not overwrite fresher or
        verified state).
        """
        mid = ext.memory_id if ext is not None else msg.beacon_id
        version = ext.version if ext is not None else msg.version
        existing = self._records.get(mid)
        if existing is not None and existing.version >= version:
            if existing.version == version:
                self.duplicates_discarded += 1
            else:
                self.stale_discarded += 1
            return existing

        if ext is None:                                  # legacy path (unchanged behaviour)
            rec = MemoryRecord.from_beacon_message(msg)
            if self._clock:
                rec.clock = self._clock
            self._records[mid] = rec
            self.log.append(f"beacon #{msg.beacon_id} [{msg.event_type}] v{msg.version} received — UNVERIFIED")
            return rec

        src = node_label(ext.source) if ext.source else "writer"
        if existing is None:
            twin = self._find_twin(msg)
            if twin is not None:           # same thing reported again (e.g. by a replacement Writer)
                twin.corroborations += 1
                twin.observed_confidence = max(twin.observed_confidence or 0, msg.initial_confidence)
                twin.last_update_ts = max(twin.observed_ts, ext.last_update_ts or msg.timestamp)
                twin.history.append(f"tick={tick} corroborated by {src} (duplicate report #{mid:04X} merged)")
                self.log.append(f"memory #{mid:04X} [{msg.event_type}] is a duplicate of #{twin.memory_id:04X} — merged")
                self.updates.append((tick or 0, twin.memory_id, f"{msg.event_type} corroborated by {src}"))
                self.duplicates_discarded += 1
                return twin
            rec = MemoryRecord.from_beacon_message(msg)
            rec.memory_id = mid
            if self._clock:
                rec.clock = self._clock
            rec.history.clear()
            rec.history.append(f"tick={tick} v{version} {ext.state.value} (first received, from {src} via beacon #{msg.beacon_id})")
            self._records[mid] = rec
            self._apply_ext(rec, msg, ext, src, hop_count, tick)
            self.log.append(f"memory #{mid:04X} [{msg.event_type}] v{version} received — {ext.state.value}")
            self.updates.append((tick or 0, mid, f"NEW {msg.event_type} v{version} {ext.state.value} by {src}"))
            return rec
        old_state = existing.state
        self._apply_ext(existing, msg, ext, src, hop_count, tick)
        existing.history.append(f"tick={tick} v{version} {old_state.value}->{ext.state.value} by {src}")
        self.log.append(f"memory #{mid:04X} v{version}: {old_state.value} -> {ext.state.value} by {src}")
        self.updates.append((tick or 0, mid, f"{msg.event_type} v{version}: {old_state.value} -> {ext.state.value} by {src}"))
        return existing

    @staticmethod
    def _apply_ext(rec: MemoryRecord, msg: BeaconMessage, ext: MemoryExt, src: str,
                   hop_count: Optional[int], tick: Optional[int]) -> None:
        rec.state = ext.state
        rec.version = ext.version
        rec.passage_state = ext.passage
        rec.last_update_ts = ext.last_update_ts or msg.timestamp
        rec.observed_confidence = msg.initial_confidence
        rec.severity = msg.severity
        rec.x_local, rec.y_local = msg.x_local, msg.y_local
        rec.gps_lat, rec.gps_lon = msg.gps_lat, msg.gps_lon
        rec.battery_pct = msg.battery_pct
        rec.host_beacon_id = msg.beacon_id
        rec.source = src
        rec.source_node = ext.source
        rec.verified_by = src if ext.state != MemoryState.UNVERIFIED else None
        rec.hop_count = hop_count
        rec.received_tick = tick

    MERGE_RADIUS_M = 0.3    # less than ONE grid cell (0.5 m): distinct neighbouring cells are never merged [ASSUMPTION]

    def _find_twin(self, msg: BeaconMessage) -> Optional[MemoryRecord]:
        """An OPEN record of the same type within MERGE_RADIUS_M: the same physical thing."""
        for r in self._records.values():
            if r.event_type == msg.event_type and r.is_open:
                if ((r.x_local - msg.x_local) ** 2 + (r.y_local - msg.y_local) ** 2) ** 0.5 <= self.MERGE_RADIUS_M:
                    return r
        return None

    def heartbeat(self, hb, gps, hop_count: int, tick: int) -> None:
        self.beacons[hb.beacon_id] = BeaconHealth(
            hb.beacon_id, hb.x_local, hb.y_local, gps, hb.hops_to_ona, hb.parent_id,
            hb.battery_pct, hb.n_records, hb.max_version, hb.last_update_ts, tick, hop_count)

    # ── queries ──────────────────────────────────────────────────────────────
    def get(self, memory_id: int) -> Optional[MemoryRecord]:
        return self._records.get(memory_id)

    def all(self) -> List[MemoryRecord]:
        return list(self._records.values())

    def open_records(self) -> List[MemoryRecord]:
        return [r for r in self._records.values() if r.is_open]

    def blockages(self, open_only: bool = True) -> List[MemoryRecord]:
        return [r for r in self._records.values()
                if r.event_type == "BLOCKAGE" and (r.is_open or not open_only)]

    def hazard_hints(self) -> List[HazardHint]:
        """Navigation-relevant memory an Executor must inherit: open blockages."""
        out = []
        for r in self.blockages():
            ps = r.passage_state or PassageState.BLOCKED
            out.append(HazardHint(r.x_local, r.y_local, ps.value, r.memory_id or 0, r.confidence_pct,
                                  r.version, r.host_beacon_id, r.severity))
        return out

    # ── legacy mutation helpers (kept for existing tests) ────────────────────
    def mark_verified(self, beacon_id: int, by: str = "executor", at_tick: Optional[int] = None) -> None:
        rec = self._records.get(beacon_id)
        if rec is None:
            return
        rec.mark_verified(by=by, at_tick=at_tick)
        self.log.append(f"beacon #{beacon_id} VERIFIED by {by}")

    def mark_contradicted(self, beacon_id: int, by: str = "executor",
                           at_tick: Optional[int] = None, reason: str = "") -> None:
        rec = self._records.get(beacon_id)
        if rec is None:
            return
        rec.mark_contradicted(by=by, at_tick=at_tick, reason=reason)
        self.log.append(f"beacon #{beacon_id} CONTRADICTED by {by} ({reason})")

    def mission_candidates(self) -> List[MemoryRecord]:
        """
        Open (not CLEARED / CONTRADICTED), non-stale records, sorted by severity
        (desc) then age (asc). BLOCKAGE records are navigation memory, not
        missions for a general response, so they are excluded here (the planner
        handles them through the DEBRIS capability). [IMPLEMENTED]
        """
        return sorted(
            (r for r in self._records.values()
             if not r.is_stale and r.is_open and r.event_type != "BLOCKAGE"),
            key=lambda r: (-r.severity, r.age_seconds),
        )

    def __len__(self) -> int:
        return len(self._records)
