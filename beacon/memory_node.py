"""
beacon/memory_node.py
======================
A deployed beacon, modelled only as far as the simulation actually needs.

[IMPLEMENTED] STORE (holds one BeaconMessage) + BROADCAST (re-encode and
send via transport, repeatable — models the real beacon's periodic
re-broadcast, useful for packet-loss resilience per failure case FC-02).

LEGACY NOTE (Phase-1 patch): this single-message class is kept for its
existing unit tests. The beacon the SYSTEM actually uses is
`beacon/node.py::BeaconNode`, which adds over-the-air PROGRAM frames, multiple
versioned records, hop-by-hop ACK/retry, gradient routing, TTL, duplicate
suppression, store-and-forward and heartbeats. Do not extend this class.
"""

from __future__ import annotations

from enum import Enum

from common.protocol import BeaconMessage
from communication.interface import BaseTransport


class BeaconState(str, Enum):
    STORED = "STORED"
    BROADCASTING = "BROADCASTING"


class SimulatedBeacon:
    """[IMPLEMENTED]"""

    def __init__(self, message: BeaconMessage, transport: BaseTransport):
        self.message = message
        self.transport = transport
        self.state = BeaconState.STORED
        self.broadcast_count = 0

    def broadcast(self) -> bool:
        """Re-encode and send the stored message. Returns True if accepted by the transport."""
        sent = self.transport.send(self.message.encode())
        self.state = BeaconState.BROADCASTING
        self.broadcast_count += 1
        return sent
