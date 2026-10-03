"""
simulation/metrics.py
======================
Objective, generated (never hand-typed) run statistics — the plumbing for
"baseline vs Living Map" comparison work. Phase 1 records these for a
single Writer run; the actual baseline-vs-Living-Map dual-run comparison
(§27 of the evolution plan) is scaffolded here but the full benchmark
harness is [PLANNED] — deliberately scoped out of the Oct-5 cut, see
docs/architecture/system-architecture.md.

[IMPLEMENTED]
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List


@dataclass
class RunMetrics:
    scenario_name:     str
    seed:               int
    role:               str   # "writer" | "baseline_explorer"
    ticks:              int
    cells_explored:     int
    total_free_cells:   int
    coverage_pct:       float
    beacons_deployed:   int
    events_total:       int
    events_found:       int

    def to_dict(self) -> dict:
        return {
            "scenario": self.scenario_name, "seed": self.seed, "role": self.role,
            "ticks": self.ticks, "cells_explored": self.cells_explored,
            "total_free_cells": self.total_free_cells, "coverage_pct": self.coverage_pct,
            "beacons_deployed": self.beacons_deployed,
            "events_total": self.events_total, "events_found": self.events_found,
        }

    def __str__(self) -> str:
        return (f"[{self.role}] ticks={self.ticks} coverage={self.coverage_pct}% "
                f"beacons={self.beacons_deployed} events={self.events_found}/{self.events_total}")


class MetricsRecorder:
    """[IMPLEMENTED] Minimal in-memory recorder. [PLANNED] CSV/JSON export for the report."""

    def __init__(self) -> None:
        self.runs: List[RunMetrics] = []

    def record(self, metrics: RunMetrics) -> None:
        self.runs.append(metrics)

    def summary(self) -> str:
        if not self.runs:
            return "(no runs recorded yet)"
        return "\n".join(str(r) for r in self.runs)
