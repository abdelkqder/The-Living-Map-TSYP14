"""
tests/test_benchmark.py
========================
PATCH 9 tests: headless benchmark mode — verifies KPI structure, determinism,
and that concurrent operation outperforms sequential on latency.
"""
from __future__ import annotations
import pytest
from simulation.benchmark import run_benchmark, print_report


REQUIRED_KEYS = {
    "total_ticks", "writer_coverage_pct", "beacons_deployed",
    "events_total", "events_verified", "events_contradicted",
    "missions_completed", "avg_beacon_to_dispatch", "avg_beacon_to_verify",
    "executor_count",
}


class TestBenchmarkStructure:
    def test_returns_three_configs(self):
        r = run_benchmark(seed=1, difficulty="EASY")
        assert set(r.keys()) == {"A_sequential", "B_concurrent", "C_no_dispatch"}

    def test_each_config_has_required_keys(self):
        r = run_benchmark(seed=1, difficulty="EASY")
        for cfg, data in r.items():
            missing = REQUIRED_KEYS - set(data.keys())
            assert not missing, f"config {cfg} missing keys: {missing}"

    def test_total_ticks_positive(self):
        r = run_benchmark(seed=1, difficulty="EASY")
        for cfg, data in r.items():
            assert data["total_ticks"] > 0, f"{cfg} total_ticks must be positive"

    def test_events_total_matches_difficulty(self):
        r = run_benchmark(seed=1, difficulty="EASY")
        for cfg, data in r.items():
            assert data["events_total"] == 2, "EASY has 2 events"

    def test_medium_has_four_events(self):
        r = run_benchmark(seed=1, difficulty="MEDIUM")
        for cfg, data in r.items():
            assert data["events_total"] == 4


class TestBenchmarkDeterminism:
    def test_same_seed_same_results(self):
        r1 = run_benchmark(seed=7, difficulty="EASY")
        r2 = run_benchmark(seed=7, difficulty="EASY")
        for cfg in r1:
            assert r1[cfg]["events_verified"] == r2[cfg]["events_verified"]
            assert r1[cfg]["beacons_deployed"] == r2[cfg]["beacons_deployed"]

    def test_different_seeds_may_differ(self):
        r1 = run_benchmark(seed=1, difficulty="EASY")
        r2 = run_benchmark(seed=99, difficulty="EASY")
        # At minimum writer coverage may differ due to different wall layouts
        # (both use same BASE_GRID but events are placed differently)
        # Just assert both complete without error
        assert r1["A_sequential"]["total_ticks"] > 0
        assert r2["A_sequential"]["total_ticks"] > 0


class TestBenchmarkKPIs:
    def test_config_c_verifies_zero_events(self):
        """No dispatch = no verification."""
        r = run_benchmark(seed=42, difficulty="EASY")
        assert r["C_no_dispatch"]["events_verified"] == 0

    def test_config_c_has_zero_executors(self):
        r = run_benchmark(seed=42, difficulty="EASY")
        assert r["C_no_dispatch"]["executor_count"] == 0

    def test_config_a_verifies_events(self):
        r = run_benchmark(seed=42, difficulty="EASY")
        assert r["A_sequential"]["events_verified"] >= 1

    def test_config_b_verifies_events(self):
        r = run_benchmark(seed=42, difficulty="EASY")
        assert r["B_concurrent"]["events_verified"] >= 1

    def test_config_b_has_more_executors_than_a(self):
        r = run_benchmark(seed=42, difficulty="EASY")
        assert r["B_concurrent"]["executor_count"] >= r["A_sequential"]["executor_count"]

    def test_config_a_dispatch_latency_positive(self):
        r = run_benchmark(seed=42, difficulty="EASY")
        dl = r["A_sequential"]["avg_beacon_to_dispatch"]
        assert dl is not None and dl > 0

    def test_config_b_dispatch_latency_not_negative(self):
        r = run_benchmark(seed=42, difficulty="EASY")
        dl = r["B_concurrent"]["avg_beacon_to_dispatch"]
        assert dl is not None and dl >= 0

    def test_config_b_faster_dispatch_than_a(self):
        """
        Concurrent (B) must dispatch sooner than sequential (A).
        A waits for writer to finish; B dispatches as beacons arrive.
        """
        r  = run_benchmark(seed=42, difficulty="EASY")
        dl_a = r["A_sequential"]["avg_beacon_to_dispatch"]
        dl_b = r["B_concurrent"]["avg_beacon_to_dispatch"]
        if dl_a is not None and dl_b is not None:
            assert dl_b < dl_a, (
                f"B should dispatch faster than A "
                f"(B={dl_b:.1f} ticks, A={dl_a:.1f} ticks)"
            )

    def test_writer_coverage_positive(self):
        r = run_benchmark(seed=42, difficulty="EASY")
        for cfg, data in r.items():
            assert data["writer_coverage_pct"] > 0, f"{cfg} must have positive coverage"

    def test_beacons_deployed_positive(self):
        r = run_benchmark(seed=42, difficulty="EASY")
        for cfg in ("A_sequential", "B_concurrent", "C_no_dispatch"):
            assert r[cfg]["beacons_deployed"] >= 1

    def test_print_report_does_not_crash(self, capsys):
        r = run_benchmark(seed=42, difficulty="EASY")
        print_report(r)
        captured = capsys.readouterr()
        assert "LIVING MAP BENCHMARK" in captured.out
        assert "total_ticks" in captured.out.lower() or "Total ticks" in captured.out
        assert "verified" in captured.out.lower()

    def test_hard_difficulty_runs_without_error(self):
        r = run_benchmark(seed=1, difficulty="HARD")
        assert r["A_sequential"]["total_ticks"] > 0
        assert r["B_concurrent"]["total_ticks"] > 0
