"""
common/clock.py
===============
Simulation clock. Ageing / timestamps in the simulation must follow
SIMULATED time, not the wall clock, otherwise a headless run that takes two
seconds could never show a beacon going stale and two runs of the same seed
could disagree.

One tick == SECONDS_PER_TICK simulated seconds. With the Writer moving one
0.5 m cell per 20 ticks, 0.1 s/tick corresponds to ~0.25 m/s robot speed
[ASSUMPTION: a plausible small differential-drive robot, not measured].

[IMPLEMENTED]
"""
from __future__ import annotations

SIM_EPOCH        = 1_790_000_000   # fixed "unix" start so runs are reproducible
SECONDS_PER_TICK = 0.1


class SimClock:
    """Deterministic tick-driven clock shared by every actor in one simulation."""

    def __init__(self, seconds_per_tick: float = SECONDS_PER_TICK, epoch0: int = SIM_EPOCH) -> None:
        self.seconds_per_tick = seconds_per_tick
        self.epoch0 = epoch0
        self.tick = 0

    def advance(self, n: int = 1) -> None:
        self.tick += n

    def set_tick(self, tick: int) -> None:
        self.tick = tick

    def now(self) -> float:
        """Simulated unix time in seconds."""
        return self.epoch0 + self.tick * self.seconds_per_tick

    def timestamp(self) -> int:
        return int(self.now())

    def ticks_to_seconds(self, ticks: int) -> float:
        return ticks * self.seconds_per_tick
