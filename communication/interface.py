"""
communication/interface.py
===========================
Abstract transport interface. High-level robot/gateway logic talks to this
interface only — it never knows whether the underlying channel is the
in-memory MockTransport (Phase 1) or a real LoRaTransport (Phase 2).

[IMPLEMENTED]
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable


class BaseTransport(ABC):
    """Minimal publish/subscribe byte-transport interface."""

    @abstractmethod
    def send(self, payload: bytes) -> bool:
        """Attempt to send raw bytes. Returns True if accepted for delivery."""
        raise NotImplementedError

    @abstractmethod
    def subscribe(self, handler: Callable[[bytes], None]) -> None:
        """Register a callback invoked with raw bytes on each delivery."""
        raise NotImplementedError
