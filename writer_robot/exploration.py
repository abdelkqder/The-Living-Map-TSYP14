"""
writer_robot/exploration.py
============================
Writer-specific exploration POLICY — when to keep exploring and when to
stop — sitting on top of the mechanical, reusable navigation/engine.py.

PATCH 5: sector_hint (row_min, col_min, row_max, col_max) directs a
replacement writer toward an under-explored region without handing it
W1's full discovered map. The hint adds W_SECTOR to frontier utility
for cells inside the bounding box. [IMPLEMENTED]
"""
from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Optional, Tuple

from common.enums import NavMode
from navigation.engine import Candidate, choose_next_target
from simulation.world import Cell, DiscoveredWorld


@dataclass
class ExplorationOutcome:
    target: Optional[Candidate]
    reason: str   # "target_selected" | "fully_explored" | "tick_budget_exhausted"


class ExplorationPolicy:
    """
    [IMPLEMENTED]
    max_ticks models a battery budget — exploration stops before full
    coverage so the demo shows "partial knowledge survives," the actual
    point of the Living Map concept. [ASSUMPTION: max_ticks is a scenario
    knob, not a measured battery figure.]

    sector_hint: optional (row_min, col_min, row_max, col_max) bounding
    box. Frontiers inside it receive a W_SECTOR utility bonus, biasing
    exploration toward that quadrant. Used for replacement writer routing.
    """

    def __init__(
        self,
        max_ticks: Optional[int] = None,
        sector_hint: Optional[Tuple[int, int, int, int]] = None,
        jitter: float = 0.0,
        seed: int = 0,
    ):
        self.max_ticks   = max_ticks
        self.sector_hint = sector_hint
        self.jitter      = jitter
        self._rng        = random.Random(seed)       # per-instance: reproducible, no hidden global state

    def next_target(
        self, world: DiscoveredWorld, start: Cell, tick: int
    ) -> ExplorationOutcome:
        if self.max_ticks is not None and tick >= self.max_ticks:
            return ExplorationOutcome(target=None, reason="tick_budget_exhausted")
        cand = choose_next_target(
            world, start, NavMode.EXPLORE, sector=self.sector_hint,
            rng=self._rng, jitter=self.jitter,
        )
        if cand is None:
            return ExplorationOutcome(target=None, reason="fully_explored")
        return ExplorationOutcome(target=cand, reason="target_selected")
