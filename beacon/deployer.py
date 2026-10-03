"""
beacon/deployer.py
==================
The beacon DROP MECHANISM and the inventory behind it.

BeaconFactory   simulator-side: turns "a robot dropped a beacon here" into a
                BeaconNode registered on the radio medium at the robot's
                PHYSICAL cell. Allocates globally unique beacon ids (so a
                replacement Writer can never collide with the first one's ids).
BeaconDispenser per-robot handle: `drop()` is the only thing robot code calls —
                the future servo/drop-mechanism driver sits behind this interface.
                The robot never passes a position: where the beacon lands is
                physics, not the robot's knowledge.

[SIMULATED] drop mechanism; [IMPLEMENTED] inventory / id allocation.
"""
from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple

from beacon.node import BeaconNode
from communication.mesh import RadioMedium, RadioPort

Cell = Tuple[int, int]
MAX_BEACON_ID = 63


class BeaconFactory:
    def __init__(self, medium: RadioMedium, now_ts: Callable[[], int], **node_kwargs) -> None:
        self.medium = medium
        self._now_ts = now_ts
        self._node_kwargs = node_kwargs
        self.nodes: Dict[int, BeaconNode] = {}
        self.drop_cells: Dict[int, Cell] = {}      # physics-side truth, for rendering/metrics only
        self._next_id = 1

    def drop_at(self, cell: Cell) -> Optional[BeaconNode]:
        if self._next_id > MAX_BEACON_ID:
            return None
        bid = self._next_id
        self._next_id += 1
        port = RadioPort(self.medium, bid, lambda c=cell: c, kind="beacon")
        node = BeaconNode(bid, port, self._now_ts, **self._node_kwargs)
        self.nodes[bid] = node
        self.drop_cells[bid] = cell
        return node

    def tick(self, t: int) -> None:
        for node in self.nodes.values():
            node.tick(t)


class BeaconDispenser:
    """Robot-side drop mechanism with a finite stock."""

    def __init__(self, factory: BeaconFactory, position_fn: Callable[[], Cell], stock: int) -> None:
        self._factory = factory
        self._position_fn = position_fn
        self.stock = stock
        self.dropped: List[int] = []

    def drop(self) -> Optional[int]:
        if self.stock <= 0:
            return None
        node = self._factory.drop_at(self._position_fn())
        if node is None:
            return None
        self.stock -= 1
        self.dropped.append(node.beacon_id)
        return node.beacon_id
