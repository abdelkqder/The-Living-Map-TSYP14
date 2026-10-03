"""
communication/lora_transport.py
================================
[PLANNED] Phase 2 hardware transport. Not implemented — no radio hardware
has been selected or tested yet (see hardware/README.md). This stub exists
only to document the interface a real transport must satisfy so that
swapping MockTransport -> LoRaTransport requires no change anywhere else.
"""

from __future__ import annotations

from typing import Callable

from communication.interface import BaseTransport


class LoRaTransport(BaseTransport):
    """[PLANNED] Not implemented. Raises on use so it fails loudly, not silently."""

    def __init__(self, *args, **kwargs) -> None:
        raise NotImplementedError(
            "LoRaTransport is a Phase 2 stub — no radio hardware has been "
            "selected or bench-tested yet. Use communication.mock_transport."
        )

    def send(self, payload: bytes) -> bool:  # pragma: no cover
        raise NotImplementedError

    def subscribe(self, handler: Callable[[bytes], None]) -> None:  # pragma: no cover
        raise NotImplementedError
