"""
tests/test_scenario.py
========================
PATCH 9 tests: scenario generation, difficulty presets, determinism.
"""
from __future__ import annotations
import pytest
from common.enums import EventType
from simulation.scenario import (
    default_scenario, difficulty_scenario, DIFFICULTY_PRESETS,
)


class TestDefaultScenario:
    def test_deterministic_same_seed(self):
        s1 = default_scenario(seed=7)
        s2 = default_scenario(seed=7)
        e1 = [(e.row, e.col, e.event_type) for e in s1.events]
        e2 = [(e.row, e.col, e.event_type) for e in s2.events]
        assert e1 == e2

    def test_different_seeds_differ(self):
        s1 = default_scenario(seed=1)
        s2 = default_scenario(seed=99)
        positions1 = {(e.row, e.col) for e in s1.events}
        positions2 = {(e.row, e.col) for e in s2.events}
        assert positions1 != positions2

    def test_default_has_two_events(self):
        s = default_scenario(seed=42)
        assert len(s.events) == 2

    def test_default_event_types_fire_and_gas(self):
        s    = default_scenario(seed=42)
        etys = {e.event_type for e in s.events}
        assert EventType.FIRE in etys
        assert EventType.GAS  in etys

    def test_events_are_on_free_cells(self):
        s = default_scenario(seed=42)
        for ev in s.events:
            assert s.grid[ev.row][ev.col] == 0, \
                f"event at ({ev.row},{ev.col}) is on a wall"

    def test_events_away_from_entry(self):
        s  = default_scenario(seed=42)
        er, ec = s.entry
        for ev in s.events:
            dist = abs(ev.row - er) + abs(ev.col - ec)
            assert dist > 2, "events should not spawn right at the entry"

    def test_no_two_events_on_same_cell(self):
        s = default_scenario(seed=42)
        positions = [(e.row, e.col) for e in s.events]
        assert len(positions) == len(set(positions))

    def test_entry_on_free_cell(self):
        s = default_scenario(seed=42)
        r, c = s.entry
        assert s.grid[r][c] == 0


class TestDifficultyScenario:
    def test_easy_has_two_events(self):
        s = difficulty_scenario("EASY", seed=42)
        assert len(s.events) == 2

    def test_medium_has_four_events(self):
        s = difficulty_scenario("MEDIUM", seed=42)
        assert len(s.events) == 4

    def test_hard_has_seven_events(self):
        s = difficulty_scenario("HARD", seed=42)
        assert len(s.events) == len(DIFFICULTY_PRESETS["HARD"]["event_types"])

    def test_easy_no_packet_loss(self):
        s = difficulty_scenario("EASY", seed=42)
        assert s.beacon_packet_loss == 0.0

    def test_medium_has_packet_loss(self):
        s = difficulty_scenario("MEDIUM", seed=42)
        assert s.beacon_packet_loss > 0.0

    def test_hard_has_more_packet_loss_than_medium(self):
        m = difficulty_scenario("MEDIUM", seed=42)
        h = difficulty_scenario("HARD",   seed=42)
        assert h.beacon_packet_loss > m.beacon_packet_loss

    def test_hard_shorter_writer_budget_than_easy(self):
        e = difficulty_scenario("EASY", seed=42)
        h = difficulty_scenario("HARD", seed=42)
        assert h.writer_max_ticks < e.writer_max_ticks

    def test_hard_contains_simulation_only_types(self):
        s     = difficulty_scenario("HARD", seed=42)
        etypes = {e.event_type for e in s.events}
        assert EventType.VICTIM_PRESENCE in etypes or EventType.STRUCTURAL in etypes

    def test_difficulty_deterministic(self):
        s1 = difficulty_scenario("MEDIUM", seed=13)
        s2 = difficulty_scenario("MEDIUM", seed=13)
        p1 = [(e.row, e.col) for e in s1.events]
        p2 = [(e.row, e.col) for e in s2.events]
        assert p1 == p2

    def test_unknown_difficulty_raises(self):
        with pytest.raises(ValueError):
            difficulty_scenario("EXTREME", seed=1)

    def test_difficulty_name_stored(self):
        s = difficulty_scenario("HARD", seed=1)
        assert s.difficulty == "HARD"

    def test_case_insensitive(self):
        s = difficulty_scenario("medium", seed=5)
        assert s.difficulty == "MEDIUM"
        assert len(s.events) == 4

    def test_events_on_free_cells_hard(self):
        s = difficulty_scenario("HARD", seed=42)
        for ev in s.events:
            assert s.grid[ev.row][ev.col] == 0

    def test_scenario_integrates_with_livemapsystem(self):
        from simulation.system import LiveMapSystem
        s    = difficulty_scenario("MEDIUM", seed=42)
        sys_ = LiveMapSystem(s)
        sys_.deploy_writer()
        for _ in range(50):
            sys_.update()
        assert sys_.tick == 50
