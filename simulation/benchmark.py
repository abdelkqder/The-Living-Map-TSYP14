"""
simulation/benchmark.py
========================
PATCH 8: Headless benchmark mode — compares three operational configurations
against the same scenario to demonstrate the Living Map's engineering value.

Configurations
--------------
A  SEQUENTIAL        Writer finishes → one Executor dispatched → completes.
                     The classic sequential baseline.

B  CONCURRENT        Writer + executor pool run simultaneously; auto-dispatch
                     fires as soon as each beacon meets the priority threshold.
                     Demonstrates event-driven Living Map advantage.

C  NO_DISPATCH       Writer runs to completion; no executor is ever dispatched.
                     Represents the system WITHOUT the Living Map benefit — the
                     "what if we skip verification?" baseline.

Key KPIs measured
-----------------
    total_ticks             wall-clock equivalent of mission duration
    writer_coverage_pct     % of free cells the Writer explored
    beacons_deployed        beacons the Writer deployed
    events_total            hidden events in this scenario
    events_verified         beacons confirmed by Executor re-sensing
    events_contradicted     beacons debunked
    missions_completed      dispatcher missions closed
    avg_beacon_to_dispatch  ticks: first beacon → executor assigned
    avg_beacon_to_verify    ticks: first beacon → executor verification
    executor_count          how many executors were active

Usage (programmatic)
--------------------
    from simulation.benchmark import run_benchmark, print_report
    results = run_benchmark(seed=42, difficulty="MEDIUM")
    print_report(results)

Usage (CLI)
-----------
    python run_simulation.py --benchmark --difficulty MEDIUM --seed 42

[IMPLEMENTED]
"""
from __future__ import annotations

from typing import Dict, Optional

from common.enums import RobotCapability
from simulation.scenario import difficulty_scenario, Scenario
from simulation.system import LiveMapSystem

MAX_TICKS = 50_000   # hard guard per config — prevents infinite loops


# ── Individual config runners ─────────────────────────────────────────────────

def _run_config(
    scenario: Scenario,
    *,
    multi_executor: bool = False,
    auto_dispatch:  bool = False,
    dispatch_after_writer: bool = True,
) -> dict:
    """
    Run one headless config and return its KPI dict.

    Parameters
    ----------
    multi_executor      pre-deploy a pool of 3 executors before the writer
    auto_dispatch       enable event-driven dispatch as beacons arrive
    dispatch_after_writer  manually dispatch once writer is done (sequential path)
    """
    sys_ = LiveMapSystem(scenario)

    if multi_executor:
        sys_.deploy_executor_pool(3, [RobotCapability.GENERAL])

    sys_.auto_dispatch = auto_dispatch
    sys_.deploy_writer()

    for tick in range(MAX_TICKS):
        sys_.update()

        # Sequential path: dispatch once writer is done
        if dispatch_after_writer and sys_.phase == "dead":
            guard = 0
            while sys_.mission_dispatcher.next_queued() is not None and guard < 50:
                guard += 1
                if sys_.dispatch_executor() is None:      # nothing could be dispatched: do not spin
                    break

        if sys_.phase == "complete":
            break
        # NO_DISPATCH config: stop when writer is done
        if not dispatch_after_writer and not auto_dispatch and sys_.phase == "dead":
            break
    else:
        tick = MAX_TICKS

    # Collect writer metrics
    writer    = sys_.writers[0] if sys_.writers else None
    coverage  = 0.0
    beacons   = 0
    if writer is not None:
        coverage = round(
            100 * writer.discovered.coverage_ratio(sys_.total_free_cells), 1
        )
        beacons = writer.beacon_count

    # Collect dispatcher metrics
    dm = sys_.mission_dispatcher.metrics()

    # Count verified / contradicted from living map
    from common.enums import MemoryState
    # PHASE-1: "verified" = confirmed on site by an executor. Depending on the task the
    # final state is VERIFIED / ACTIVE / ESCALATED (still there) or CLEARED (resolved).
    verified      = sum(
        1 for r in sys_.living_map_records
        if r.state in (MemoryState.VERIFIED, MemoryState.ACTIVE,
                       MemoryState.ESCALATED, MemoryState.CLEARED)
    )
    contradicted  = sum(
        1 for r in sys_.living_map_records
        if r.state == MemoryState.CONTRADICTED
    )

    return {
        "total_ticks":            tick + 1,
        "writer_coverage_pct":    coverage,
        "beacons_deployed":       beacons,
        "events_total":           len(scenario.events),
        "events_verified":        verified,
        "events_contradicted":    contradicted,
        "missions_completed":     dm["missions_completed"],
        "avg_beacon_to_dispatch": dm["avg_beacon_to_dispatch"],
        "avg_beacon_to_verify":   dm["avg_beacon_to_verify"],
        "executor_count":         len(sys_.executors),
        "scenario_difficulty":    scenario.difficulty,
        "scenario_seed":          scenario.seed,
    }


# ── Public API ────────────────────────────────────────────────────────────────

def run_benchmark(
    seed:       int = 42,
    difficulty: str = "MEDIUM",
) -> Dict[str, dict]:
    """
    Run all three configurations against the same scenario.
    Returns ``{"A": {...}, "B": {...}, "C": {...}}``.

    Same seed + difficulty always produce the same results (deterministic).
    [IMPLEMENTED]
    """
    scenario = difficulty_scenario(difficulty=difficulty, seed=seed)

    return {
        "A_sequential": _run_config(
            scenario,
            multi_executor=False,
            auto_dispatch=False,
            dispatch_after_writer=True,
        ),
        "B_concurrent": _run_config(
            scenario,
            multi_executor=True,
            auto_dispatch=True,
            dispatch_after_writer=False,
        ),
        "C_no_dispatch": _run_config(
            scenario,
            multi_executor=False,
            auto_dispatch=False,
            dispatch_after_writer=False,
        ),
    }


def print_report(results: Dict[str, dict]) -> None:
    """
    Print a human-readable comparison table to stdout.
    """
    configs = [
        ("A_sequential",  "A  Sequential (1 writer → 1 executor, waits for writer)"),
        ("B_concurrent",  "B  Concurrent (pool + auto-dispatch while writer explores)"),
        ("C_no_dispatch",  "C  No dispatch (writer only — no Living Map verification)"),
    ]

    # Header
    seed = next(iter(results.values())).get("scenario_seed", "?")
    diff = next(iter(results.values())).get("scenario_difficulty", "?")
    print(f"\n{'='*68}")
    print(f"  LIVING MAP BENCHMARK — difficulty={diff}  seed={seed}")
    print(f"{'='*68}")
    row_fmt = "  {:<38}  {:>8}  {:>8}  {:>8}"
    print(row_fmt.format("Metric", "A", "B", "C"))
    print("  " + "-" * 64)

    def _fmt(v):
        if v is None:
            return "—"
        if isinstance(v, float):
            return f"{v:.1f}"
        return str(v)

    metrics = [
        ("total_ticks",            "Total ticks"),
        ("writer_coverage_pct",    "Writer coverage (%)"),
        ("beacons_deployed",       "Beacons deployed"),
        ("events_total",           "Events in scenario"),
        ("events_verified",        "Events VERIFIED"),
        ("events_contradicted",    "Events CONTRADICTED"),
        ("missions_completed",     "Missions completed"),
        ("avg_beacon_to_dispatch", "Avg beacon→dispatch (ticks)"),
        ("avg_beacon_to_verify",   "Avg beacon→verify (ticks)"),
        ("executor_count",         "Executors deployed"),
    ]

    for key, label in metrics:
        vals = [_fmt(results.get(cid, {}).get(key)) for cid, _ in configs]
        print(row_fmt.format(label, *vals))

    print(f"{'='*68}")
    print()
    # Highlight key insight
    a = results.get("A_sequential", {})
    b = results.get("B_concurrent", {})
    c = results.get("C_no_dispatch", {})
    av = a.get("avg_beacon_to_verify")
    bv = b.get("avg_beacon_to_verify")
    if av and bv and bv < av:
        speedup = round(av / bv, 1)
        print(f"  ⚡ Config B verified {speedup}× faster than Config A "
              f"({bv:.0f} vs {av:.0f} ticks beacon→verify)")
    b_ver = b.get("events_verified", 0)
    c_ver = c.get("events_verified", 0)
    if b_ver > c_ver:
        print(f"  ✓  Config B verified {b_ver} event(s); "
              f"Config C (no dispatch) verified {c_ver} — "
              f"Living Map verification advantage demonstrated")
    print()
