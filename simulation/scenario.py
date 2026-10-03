"""
simulation/scenario.py
=======================
Reproducible scenario definitions.

PATCH 6 additions
-----------------
difficulty_scenario(difficulty, seed) — returns a named preset:
    EASY   2 events (FIRE + GAS),   0% packet loss,  writer_max_ticks=900
    MEDIUM 4 events (×2 each),      5% packet loss,  writer_max_ticks=700
    HARD   7 events (mixed types), 15% packet loss,  writer_max_ticks=500

Existing default_scenario() is unchanged for backward compat.
Same seed → same event placement for any difficulty. [IMPLEMENTED]
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from common.enums import EventType
from common.mission import BlockagePolicy
from simulation.world import Cell, HiddenEvent

BASE_GRID: List[List[int]] = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1],
    [1,0,1,0,1,0,1,1,1,0,1,1,1,0,1,0,1,0,1,1],
    [1,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,1],
    [1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,0,1,1,0,1],
    [1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,1],
    [1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,0,1,1],
    [1,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,1],
    [1,0,1,1,1,0,1,1,1,0,1,0,1,0,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,0,1,0,1,1,1,0,1,0,1,1,1,0,1,1,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
]
ENTRY: Cell = (1, 1)
DEFAULT_EVENT_TYPES: List[EventType] = [EventType.FIRE, EventType.GAS]

# ── PATCH 6: difficulty presets ───────────────────────────────────────────────
DIFFICULTY_PRESETS: Dict[str, dict] = {
    "EASY": {
        "event_types": [EventType.FIRE, EventType.GAS],
        "beacon_packet_loss": 0.0,
        "writer_max_ticks": 900,
    },
    "MEDIUM": {
        "event_types": [
            EventType.FIRE, EventType.GAS,
            EventType.FIRE, EventType.GAS,
        ],
        "beacon_packet_loss": 0.05,
        "writer_max_ticks": 700,
    },
    "HARD": {
        "event_types": [
            EventType.FIRE, EventType.GAS,
            EventType.FIRE, EventType.GAS,
            EventType.FIRE,
            EventType.VICTIM_PRESENCE,   # [SIMULATION-ONLY]
            EventType.STRUCTURAL,        # [SIMULATION-ONLY]
        ],
        "beacon_packet_loss": 0.15,
        "writer_max_ticks": 500,
    },
}


@dataclass
class Scenario:
    name:               str
    seed:               int
    grid:               List[List[int]]
    entry:              Cell
    events:             List[HiddenEvent]
    writer_max_ticks:   Optional[int]       = None
    ref_gps:            Tuple[float, float] = (36.8065, 10.1815)
    ref_heading:        float               = 0.0
    beacon_packet_loss: float               = 0.0
    difficulty:         str                 = "EASY"  # PATCH 6
    # ── PHASE-1 PATCH (all defaulted: old scenarios behave as before) ──────────
    radio_range_cells:  float               = 8.0     # [ASSUMPTION] scaled for the arena, NOT a measured LoRa range
    writer_beacon_stock: int                = 12
    executor_beacon_stock: int              = 2
    writer_fail_tick:   Optional[int]       = None    # simulate a Writer failure mid-exploration
    writer_return_home: bool                = True
    fleet:              Optional[Dict[str, int]] = None   # e.g. {"FIRE":3,"VICTIM":3,"GAS":2,"DEBRIS":2}
    executor_overrides: Dict[str, dict]     = field(default_factory=dict)  # per-executor kwargs (fault_at_tick, ...)
    debris_existing:    List[Cell]          = field(default_factory=list)  # present when the Writer enters
    debris_schedule:    List[Tuple[int, Cell]] = field(default_factory=list)  # (tick, cell) appearing LATER
    blockage_policy:    str                 = "on_demand"  # CP: "on_demand" | "proactive" | "never"
    brief_policy:       BlockagePolicy      = field(default_factory=BlockagePolicy)
    inherit_memory:     bool                = True    # False = baseline executors get no inherited map
    odometry_noise:     Tuple[float, float] = (0.0, 0.0)   # (distance std m, turn std rad); 0 = ideal
    radio_loss:         Optional[float]     = None    # None -> use beacon_packet_loss
    dispatch_threshold: Optional[float]     = None    # CP auto-dispatch score floor (None = dispatcher default)
    explore_jitter:     float               = 0.0     # seeded tie-break noise in frontier choice (0 = deterministic)


def _free_cells(grid: List[List[int]]) -> List[Cell]:
    return [(r, c) for r, row in enumerate(grid) for c, v in enumerate(row) if v == 0]


def _place_events(
    rng: random.Random,
    event_types: List[EventType],
    min_separation: int = 3,
) -> List[HiddenEvent]:
    excluded   = {ENTRY, (ENTRY[0]+1, ENTRY[1]), (ENTRY[0], ENTRY[1]+1)}
    candidates = [c for c in _free_cells(BASE_GRID) if c not in excluded]
    rng.shuffle(candidates)

    events: List[HiddenEvent] = []
    for i, et in enumerate(event_types):
        if not candidates:
            break
        pick     = candidates[0]
        severity = 3 if et == EventType.FIRE else (
                   2 if et == EventType.GAS  else 1)
        events.append(
            HiddenEvent(id=i+1, event_type=et, row=pick[0], col=pick[1], severity=severity)
        )
        candidates = [
            c for c in candidates
            if abs(c[0]-pick[0]) + abs(c[1]-pick[1]) > min_separation
        ]
    return events


def default_scenario(
    seed: int = 42,
    writer_max_ticks: Optional[int] = 900,
    event_types: Optional[List[EventType]] = None,
    beacon_packet_loss: float = 0.0,
) -> Scenario:
    """
    Deterministic default scenario (unchanged from original). [IMPLEMENTED]
    """
    rng         = random.Random(seed)
    event_types = event_types or DEFAULT_EVENT_TYPES
    events      = _place_events(rng, event_types)
    return Scenario(
        name=f"default-seed{seed}", seed=seed,
        grid=BASE_GRID, entry=ENTRY, events=events,
        writer_max_ticks=writer_max_ticks,
        beacon_packet_loss=beacon_packet_loss,
        difficulty="EASY",
    )


def difficulty_scenario(
    difficulty: str = "MEDIUM",
    seed: int = 42,
) -> Scenario:
    """
    PATCH 6: named difficulty preset. [IMPLEMENTED]

    EASY   — 2 events, no packet loss, relaxed writer budget
    MEDIUM — 4 events, 5% packet loss, moderate budget
    HARD   — 7 mixed events (incl. SIMULATION-ONLY types), 15% packet loss,
             tight budget — writer will likely fail mid-exploration

    Same seed → same event placement. Difficulty only changes:
        event count/types, packet loss, writer budget.
    """
    diff = difficulty.upper()
    if diff not in DIFFICULTY_PRESETS:
        raise ValueError(f"Unknown difficulty '{difficulty}'. Choose EASY / MEDIUM / HARD.")
    cfg  = DIFFICULTY_PRESETS[diff]
    rng  = random.Random(seed)
    evts = _place_events(rng, cfg["event_types"])
    return Scenario(
        name=f"{diff.lower()}-seed{seed}",
        seed=seed,
        grid=BASE_GRID,
        entry=ENTRY,
        events=evts,
        writer_max_ticks=cfg["writer_max_ticks"],
        beacon_packet_loss=cfg["beacon_packet_loss"],
        difficulty=diff,
    )
