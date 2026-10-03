from common.enums import EventType
from common.models import Observation
from writer_robot.memory_decision import evaluate


def obs(**overrides):
    defaults = dict(event_type=EventType.FIRE, row=5, col=5, severity=3, confidence=80, tick=1)
    defaults.update(overrides)
    return Observation(**defaults)


def test_preserved_when_above_thresholds():
    decision = evaluate(obs(), already_deployed=[])
    assert decision.preserve is True
    assert decision.priority == 3


def test_rejected_when_severity_too_low():
    decision = evaluate(obs(severity=1), already_deployed=[])
    assert decision.preserve is False
    assert "severity" in decision.reason


def test_rejected_when_confidence_too_low():
    decision = evaluate(obs(confidence=10), already_deployed=[])
    assert decision.preserve is False
    assert "confidence" in decision.reason


def test_rejected_when_redundant_nearby():
    already = [(EventType.FIRE, 5, 6)]  # 1 cell away, within dedup radius
    decision = evaluate(obs(row=5, col=5), already_deployed=already)
    assert decision.preserve is False
    assert "redundant" in decision.reason


def test_preserved_when_far_from_existing_same_type():
    already = [(EventType.FIRE, 0, 0)]  # far away
    decision = evaluate(obs(row=5, col=5), already_deployed=already)
    assert decision.preserve is True


def test_preserved_when_nearby_but_different_type():
    already = [(EventType.GAS, 5, 6)]  # close, but different event type
    decision = evaluate(obs(row=5, col=5, event_type=EventType.FIRE), already_deployed=already)
    assert decision.preserve is True
