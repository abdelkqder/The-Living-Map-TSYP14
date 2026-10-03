from common.enums import EventType
from simulation.world import GroundTruthWorld, HiddenEvent
from writer_robot.perception import DetectionConfig, EventDetector, SensorReading, synthesize_reading


def test_fire_detected_above_threshold():
    det = EventDetector()
    events = det.process(SensorReading(tick=1, flame_raw=100))
    assert EventType.FIRE in events


def test_no_event_on_baseline_reading():
    det = EventDetector()
    events = det.process(SensorReading(tick=1))
    assert events == []


def test_gas_detected_above_threshold():
    det = EventDetector()
    events = det.process(SensorReading(tick=1, gas_raw=900))
    assert EventType.GAS in events


def test_rearm_delay_suppresses_immediate_repeat():
    det = EventDetector()
    first = det.process(SensorReading(tick=1, flame_raw=100))
    second = det.process(SensorReading(tick=2, flame_raw=100))  # 1 tick later, still burning
    assert EventType.FIRE in first
    assert EventType.FIRE not in second  # within rearm_ticks window


def test_rearm_after_delay_allows_retrigger():
    det = EventDetector(DetectionConfig(rearm_ticks=3))
    det.process(SensorReading(tick=1, flame_raw=100))
    later = det.process(SensorReading(tick=10, flame_raw=100))
    assert EventType.FIRE in later


def test_pir_requires_sustained_ticks():
    det = EventDetector(DetectionConfig(pir_confirm_ticks=2))
    r1 = det.process(SensorReading(tick=1, pir_active=True))
    assert EventType.VICTIM_PRESENCE not in r1  # first tick, not sustained yet
    r2 = det.process(SensorReading(tick=2, pir_active=True))
    assert EventType.VICTIM_PRESENCE not in r2  # only 1 tick of sustain so far
    r3 = det.process(SensorReading(tick=3, pir_active=True))
    assert EventType.VICTIM_PRESENCE in r3  # 2 full ticks sustained -> confirmed


def test_synthesize_reading_matches_hidden_event():
    world = GroundTruthWorld(
        [[1, 1, 1], [1, 0, 1], [1, 1, 1]],
        [HiddenEvent(id=1, event_type=EventType.FIRE, row=1, col=1, severity=3)],
        entry=(1, 1),
    )
    snap = world.sense((1, 1), radius=0)
    reading = synthesize_reading(snap, tick=5)
    assert reading.flame_raw < DetectionConfig().flame_adc_threshold


def test_synthesize_reading_baseline_when_nothing_sensed():
    world = GroundTruthWorld([[1, 1, 1], [1, 0, 1], [1, 1, 1]], [], entry=(1, 1))
    snap = world.sense((1, 1), radius=0)
    reading = synthesize_reading(snap, tick=1)
    assert reading.flame_raw >= DetectionConfig().flame_adc_threshold
    assert reading.gas_raw == 0
    assert reading.pir_active is False
