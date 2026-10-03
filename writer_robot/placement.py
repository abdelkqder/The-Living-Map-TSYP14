"""
writer_robot/placement.py
=========================
WHERE (and whether) to leave a beacon — a deliberate, modular decision, not a
scripted drop sequence. A first-generation heuristic, easy to replace.

Two reasons to drop a beacon:

  EVENT  preserve information that must survive the Writer.
         Considers: importance (severity x confidence), whether an existing
         beacon close by can host the record (update it instead of spending a
         new one), remaining stock, and whether the spot currently has an uplink
         (store-and-forward still works if not, but the Writer is then told to
         bridge the gap).
  RELAY  keep a route from a preserved record to the ONA ("bridge on demand").
         When an event beacon is dropped where the Writer has no uplink, the
         Writer walks back toward the last place it was connected and drops
         relays only as its own signal measurements require (decide_bridge).
         Relays are not scattered along every explored corridor.

When stock is low only high-importance events get a new beacon.

The policy sees only what the robot sees: its own dropped-beacon list (local
coordinates it measured), the uplink margin it hears, and its stock. It never
sees the radio topology. [IMPLEMENTED]
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

Cell = Tuple[int, int]


class PlacementKind(str, Enum):
    NONE = "NONE"
    EVENT_NEW = "EVENT_NEW"          # drop a new beacon to host the event
    EVENT_UPDATE = "EVENT_UPDATE"    # write into an existing nearby beacon
    RELAY = "RELAY"                  # drop a relay here


@dataclass
class PlacedBeacon:
    beacon_id: int
    cell: Cell
    n_records: int = 0
    is_relay: bool = False


@dataclass
class PlacementDecision:
    kind: PlacementKind
    reason: str
    host_id: Optional[int] = None
    importance: float = 0.0
    at_cell: Optional[Cell] = None


class BeaconPlacementPolicy:
    def __init__(self, relay_margin: float = 0.30, min_spacing_cells: float = 2.5,
                 host_radius_cells: float = 2.0, event_reserve: int = 3,
                 low_stock_min_importance: float = 2.0) -> None:
        self.relay_margin = relay_margin
        self.min_spacing = min_spacing_cells
        self.host_radius = host_radius_cells
        self.event_reserve = event_reserve
        self.low_stock_min_importance = low_stock_min_importance

    @staticmethod
    def importance(severity: int, confidence_pct: int) -> float:
        return round(severity * confidence_pct / 100.0, 2)

    @staticmethod
    def _dist(a: Cell, b: Cell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def decide_event(self, *, severity: int, confidence_pct: int, here: Cell, event_cell: Cell,
                     placed: List[PlacedBeacon], uplink_margin: Optional[float], stock: int,
                     capacity: int = 4) -> PlacementDecision:
        imp = self.importance(severity, confidence_pct)
        hosts = sorted((b for b in placed if b.n_records < capacity and self._dist(b.cell, event_cell) <= self.host_radius),
                       key=lambda b: self._dist(b.cell, event_cell))
        if hosts:
            h = hosts[0]
            return PlacementDecision(PlacementKind.EVENT_UPDATE,
                                     f"beacon #{h.beacon_id} is {self._dist(h.cell, event_cell):.1f} cells from the event: "
                                     "write into it instead of spending a new one", h.beacon_id, imp)
        if stock <= 0:
            return PlacementDecision(PlacementKind.NONE, "no beacon left in stock", None, imp)
        if stock <= self.event_reserve and imp < self.low_stock_min_importance:
            return PlacementDecision(PlacementKind.NONE,
                                     f"stock low ({stock}) and importance {imp} < {self.low_stock_min_importance}", None, imp)
        link = "uplink OK" if uplink_margin is not None else "no uplink here (store-and-forward until bridged)"
        return PlacementDecision(PlacementKind.EVENT_NEW,
                                 f"importance {imp}; no hosting beacon within {self.host_radius} cells; {link}", None, imp, here)

    def decide_bridge(self, *, anchor_margin: Optional[float], uplink_margin: Optional[float],
                      stock: int) -> PlacementDecision:
        """
        Bridge a disconnected beacon (the `anchor`) back to the network while the
        Writer walks toward the last place it had an uplink. At every step:
          * the Writer hears BOTH the anchor and a connected node  -> final relay here;
          * the anchor is about to fall out of range               -> relay here, it becomes the new anchor;
          * otherwise keep walking.
        Relays are therefore only spent where a preserved record actually needs
        a route out — not along every corridor that was explored.
        """
        if stock <= 0:
            return PlacementDecision(PlacementKind.NONE, "no beacon left to bridge with")
        if uplink_margin is not None and anchor_margin is not None:
            return PlacementDecision(PlacementKind.RELAY,
                                     f"final relay: anchor ({anchor_margin:.2f}) and network ({uplink_margin:.2f}) both audible here")
        if anchor_margin is None or anchor_margin < self.relay_margin:
            return PlacementDecision(PlacementKind.RELAY,
                                     f"anchor margin {anchor_margin if anchor_margin is not None else 0:.2f} < {self.relay_margin}: extend the relay corridor")
        return PlacementDecision(PlacementKind.NONE, "anchor still strong: keep walking")
