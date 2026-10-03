"""
communication/mock_transport.py
================================
In-memory transport for Phase 1 simulation. Implements BaseTransport so it
is a drop-in swap for a future LoRaTransport (Phase 2) without any change
to the code that uses it.

[IMPLEMENTED]
"""

from __future__ import annotations

import random
from typing import Callable, List

from communication.interface import BaseTransport


class MockTransport(BaseTransport):
    """
    In-process pub/sub channel with optional simulated packet loss.

    name         : label, used only in log messages / repr.
    packet_loss  : probability in [0,1] that send() silently drops a packet,
                   simulating RF interference. [SIMULATED]
    """

    def __init__(self, name: str = "channel", packet_loss: float = 0.0) -> None:
        self.name = name
        self.packet_loss = packet_loss
        self._subscribers: List[Callable[[bytes], None]] = []
        self.sent_count = 0
        self.dropped_count = 0

    def subscribe(self, handler: Callable[[bytes], None]) -> None:
        self._subscribers.append(handler)

    def send(self, payload: bytes) -> bool:
        if self.packet_loss > 0.0 and random.random() < self.packet_loss:
            self.dropped_count += 1
            return False
        self.sent_count += 1
        for handler in self._subscribers:
            handler(payload)
        return True

    def __repr__(self) -> str:  # pragma: no cover - debugging aid only
        return f"MockTransport({self.name!r}, sent={self.sent_count}, dropped={self.dropped_count})"
