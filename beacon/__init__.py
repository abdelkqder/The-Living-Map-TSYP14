"""
beacon/__init__.py
==================
Backward-compatibility re-export. The authoritative wire-format
implementation lives in common/protocol.py — import from there in new code.
This package does NOT define a second BeaconMessage or a second EVENT_TYPES
table anywhere (that duplication is what caused the Phase-0 bug where
beacon/beacon_protocol.py silently drifted out of sync and was never used).
"""

from common.protocol import (   # noqa: F401
    BeaconMessage,
    BeaconRegistry,
    DecodeError,
    decode_packet,
    EVENT_TYPES,
    EVENT_IDS,
    PACKET_SIZE,
    PAYLOAD_SIZE,
    DECAY_LAMBDA,
    MIN_CONFIDENCE,
    STALE_THRESHOLD,
)
from beacon.memory_node import SimulatedBeacon, BeaconState   # noqa: F401
