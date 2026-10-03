"""
writer_robot/perception.py
===========================
Event detection. This is deliberately the ONLY place that bridges a
GroundTruthWorld.sense() snapshot into synthetic sensor readings — every
robot (Writer or Executor) goes through the same detector, so neither can
"see more" than the sensor model allows.

Status per component
─────────────────────
SensorReading / DetectionConfig: [SIMULATED] synthetic ADC-like values,
    thresholds are estimates, not field-calibrated.
EventDetector:                   [IMPLEMENTED] threshold + hysteresis logic
    (unchanged from the working Phase-1 baseline — this part was already
    solid and needed no rework).
synthesize_reading():            [NEW, IMPLEMENTED] bridges a
    simulation.world.SensorSnapshot to a SensorReading. Phase 1's default
    scenario only spawns FIRE and GAS (the two challenge-required event
    types); VICTIM_PRESENCE/STRUCTURAL detection paths are kept for
    extensibility and covered by tests, but are [PLANNED] for the default
    demo scenario.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from common.enums import EventType
from simulation.world import SensorSnapshot


@dataclass
class SensorReading:
    """Snapshot of all sensor values for one tick. [SIMULATED] in Phase 1."""
    tick:        int
    flame_raw:   int   = 900    # 0-1023 ADC value; LOWER = more flame
    gas_raw:     int   = 0      # 0-1023 ADC; proxy for concentration [ASSUMPTION]
    pir_active:  bool  = False  # human-presence digital output
    temp_c:      float = 25.0   # ambient temperature (deg C)


@dataclass
class DetectionConfig:
    """[ASSUMPTION] thresholds — not field-calibrated."""
    flame_adc_threshold: int   = 400
    temp_c_threshold:    float = 45.0
    gas_adc_threshold:   int   = 600
    pir_confirm_ticks:   int   = 2       # PIR must stay active this many ticks
    rearm_ticks:         int   = 5       # hysteresis re-arm delay


def synthesize_reading(snapshot: SensorSnapshot, tick: int) -> SensorReading:
    """
    Turn "is there a hidden event at the robot's current cell" into a
    plausible synthetic sensor reading, so EventDetector never touches
    ground truth directly. [SIMULATED][IMPLEMENTED]
    """
    if snapshot.event_here is None:
        return SensorReading(tick=tick)

    et = snapshot.event_here.event_type
    if et == EventType.FIRE:
        return SensorReading(tick=tick, flame_raw=180, temp_c=62.0)
    if et == EventType.GAS:
        return SensorReading(tick=tick, gas_raw=780)
    if et == EventType.VICTIM_PRESENCE:
        return SensorReading(tick=tick, pir_active=True)
    # STRUCTURAL: [PLANNED] no dedicated sensor/config yet.
    return SensorReading(tick=tick)


class EventDetector:
    """
    Stateful event detector with hysteresis. [IMPLEMENTED]
    Carried over unchanged from the working Phase-1 baseline.
    """

    def __init__(self, config: Optional[DetectionConfig] = None) -> None:
        self.cfg = config or DetectionConfig()
        self._pir_since: Optional[int] = None
        self._last: dict = {}

    def process(self, r: SensorReading) -> List[EventType]:
        events: List[EventType] = []
        t = r.tick

        if r.flame_raw < self.cfg.flame_adc_threshold or r.temp_c > self.cfg.temp_c_threshold:
            if self._arm(EventType.FIRE, t):
                events.append(EventType.FIRE)
                self._last[EventType.FIRE] = t

        if r.gas_raw > self.cfg.gas_adc_threshold:
            if self._arm(EventType.GAS, t):
                events.append(EventType.GAS)
                self._last[EventType.GAS] = t

        if r.pir_active:
            if self._pir_since is None:
                self._pir_since = t
            elif (t - self._pir_since) >= self.cfg.pir_confirm_ticks:
                if self._arm(EventType.VICTIM_PRESENCE, t):
                    events.append(EventType.VICTIM_PRESENCE)
                    self._last[EventType.VICTIM_PRESENCE] = t
        else:
            self._pir_since = None

        return events

    def _arm(self, ev: EventType, now: int) -> bool:
        last = self._last.get(ev)
        return last is None or (now - last) >= self.cfg.rearm_ticks

    @staticmethod
    def severity(event_type: EventType) -> int:
        return {
            EventType.VICTIM_PRESENCE: 4,
            EventType.FIRE: 3,
            EventType.GAS: 2,
            EventType.STRUCTURAL: 3,
        }.get(event_type, 1)

    @staticmethod
    def confidence(event_type: EventType, reading: SensorReading) -> int:
        """[ASSUMPTION] crude margin-based confidence estimate, 50-100."""
        if event_type == EventType.FIRE:
            margin = max(0, 400 - reading.flame_raw) / 400
        elif event_type == EventType.GAS:
            margin = max(0, reading.gas_raw - 600) / 424  # 1023-600 headroom for gas
        else:
            margin = 0.5
        return int(50 + min(1.0, margin) * 50)
