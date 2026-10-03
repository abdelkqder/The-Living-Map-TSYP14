"""
simulation/renderer.py
=======================
PATCH 2: Command Post UI rewrite.

New in this patch:
  * SYSTEM STATUS section  — every writer/executor with live state indicator
  * COMMAND CONTROLS section — 5 clickable buttons
      [DEPLOY WRITER]  [DISPATCH EXECUTOR]  [REPLACEMENT WRITER]
      [AUTO DISPATCH: ON/OFF]  [PAUSE / RESUME]
  * LIVING MAP section — every beacon record with sev/conf/verification state
  * EVENT LOG section at the bottom (replaces old mission log)
  * All writers and executors drawn on the map with distinct colours
  * Keyboard shortcuts: W deploy writer, E dispatch executor, P pause
  * "concurrent" phase fully handled in labels and run-loop

Backward-compatible: SPACE / R / Q shortcuts still work.
[IMPLEMENTED]
"""
from __future__ import annotations

import math
from typing import Callable, List, Tuple

import pygame

from common.enums import CellState, ExecutorState, MemoryState, WriterState
from simulation.system import LiveMapSystem
from common.protocol import node_label
from simulation.world import CELL_PX, EntryFrame, cell_to_px

# ── Palette ───────────────────────────────────────────────────────────────────
BG           = (247, 248, 250)
FLOOR_FILL   = (255, 255, 255)
FLOOR_GRID   = (231, 233, 238)
WALL_FILL    = (191, 197, 209)
WALL_EDGE    = (158, 165, 180)
ENTRY_TINT   = (219, 231, 250)

PANEL_BG     = (255, 255, 255)
PANEL_BORDER = (224, 227, 233)
HEADER_BG    = (236, 239, 246)
DIVIDER_COL  = (214, 218, 226)

TEXT_BRIGHT  = ( 18,  22,  32)
TEXT_MAIN    = ( 46,  53,  68)
TEXT_DIM     = (128, 136, 150)
WHITE        = (255, 255, 255)

ALERT   = (203,  58,  58)
SUCCESS = ( 24, 140,  94)
WARNING = (188, 128,  20)

# Per-robot colour palettes (index = robot slot)
WRITER_COLORS: List[Tuple[int,int,int]] = [
    (104,  84, 219),   # W1 indigo
    (156,  59, 188),   # W2 violet
    (190,  72,  60),   # W3 rust
]
EXECUTOR_COLORS: List[Tuple[int,int,int]] = [
    ( 33, 103, 199),   # E1 blue
    ( 24, 140,  94),   # E2 teal
    (180, 120,  20),   # E3 amber
]

EVENT_COLORS = {
    "FIRE":            (211,  79,  46),
    "GAS":             (183, 128,  18),
    "VICTIM_PRESENCE": (168,  46, 140),
    "STRUCTURAL":      (100, 108, 122),
    "BLOCKAGE":        (203,  58,  58),
}
# PHASE-1: lifecycle ring colour around a record marker
STATE_RING = {
    "UNVERIFIED": (52, 120, 220), "VERIFIED": (34, 160, 94), "ACTIVE": (34, 160, 94),
    "ESCALATED": (230, 150, 20), "CLEARED": (150, 155, 165), "CONTRADICTED": (203, 58, 58),
}
CONTRA_COLOR = (168, 168, 176)

# State indicator dot colours
DOT_GREEN = ( 34, 197,  94)
DOT_AMBER = (245, 158,  11)
DOT_RED   = (239,  68,  68)
DOT_GRAY  = (180, 188, 200)

_W_DOT = {
    WriterState.EXPLORING: DOT_GREEN,
    WriterState.IDLE:      DOT_AMBER,
    WriterState.DEAD:      DOT_RED,
    WriterState.OFFLINE:   DOT_RED,
}
_E_DOT = {
    ExecutorState.ACTIVE:    DOT_GREEN,
    ExecutorState.DEPLOYING: DOT_GREEN,
    ExecutorState.VERIFYING: DOT_GREEN,
    ExecutorState.READY:     DOT_AMBER,
    ExecutorState.RETURNING: DOT_AMBER,
    ExecutorState.IDLE:      DOT_GRAY,
    ExecutorState.COMPLETED: DOT_RED,
    ExecutorState.OFFLINE:   DOT_RED,
}

PHASE_LABEL = {
    "idle":       "IDLE  ·  press W to start",
    "writer":     "WRITER EXPLORING",
    "dead":       "WRITER OFFLINE  ·  press E",
    "executor":   "EXECUTOR ACTIVE",
    "concurrent": "CONCURRENT OPERATION",
    "complete":   "MISSION COMPLETE",
}
PHASE_COLOR = {
    "idle":       TEXT_DIM,
    "writer":     WRITER_COLORS[0],
    "dead":       ALERT,
    "executor":   EXECUTOR_COLORS[0],
    "concurrent": SUCCESS,
    "complete":   SUCCESS,
}

LOG_COLOR = {
    "writer":  WRITER_COLORS[0],
    "exec":    EXECUTOR_COLORS[0],
    "beacon":  WARNING,
    "alert":   ALERT,
    "success": SUCCESS,
    "normal":  TEXT_MAIN,
    "dim":     TEXT_DIM,
}

MEM_SYM = {
    MemoryState.UNVERIFIED.value:   ("?", WARNING),
    MemoryState.VERIFIED.value:     ("✓", SUCCESS),
    MemoryState.CONTRADICTED.value: ("✗", ALERT),
}

BTN_H   = 28
BTN_GAP = 5


class Renderer:
    PANEL_W = 400

    def __init__(self, system: LiveMapSystem) -> None:
        self.sys     = system
        self.map_w   = system.world.cols * CELL_PX
        self.map_h   = system.world.rows * CELL_PX
        self.panel_w = self.PANEL_W
        self.w       = self.map_w + self.panel_w
        self.h       = max(self.map_h, 680)
        self.paused  = False
        self._buttons: List[Tuple[pygame.Rect, Callable]] = []
        self._selected: int | None = None        # selected memory id (click a row in the ONA / Living Map panels)
        self._frame = EntryFrame(system.world.entry)

        pygame.init()
        self.screen = pygame.display.set_mode((self.w, self.h))
        pygame.display.set_caption("THE LIVING MAP  ·  TSYP14")
        self.font_sm  = pygame.font.SysFont("consolas,menlo,monospace", 13)
        self.font_med = pygame.font.SysFont("consolas,menlo,monospace", 16, bold=True)
        self.font_big = pygame.font.SysFont("consolas,menlo,monospace", 20, bold=True)
        self.clock    = pygame.time.Clock()

    # ── Main draw ─────────────────────────────────────────────────────────────

    def draw(self) -> None:
        self._buttons = []
        self.screen.fill(BG)
        self._draw_map()
        self._draw_beacons()
        self._draw_entities()
        self._draw_panel()
        pygame.display.flip()

    def handle_click(self, pos: Tuple[int, int]) -> None:
        for rect, action in self._buttons:
            if rect.collidepoint(pos):
                action()
                return

    # ── Map ───────────────────────────────────────────────────────────────────

    def _draw_map(self) -> None:
        for (r, c), state in self.sys.visible_cells().items():
            rect = pygame.Rect(c * CELL_PX, r * CELL_PX, CELL_PX, CELL_PX)
            if state == CellState.WALL:
                pygame.draw.rect(self.screen, WALL_FILL, rect)
                pygame.draw.rect(self.screen, WALL_EDGE, rect, 1)
            else:
                pygame.draw.rect(self.screen, FLOOR_FILL, rect)
                pygame.draw.rect(self.screen, FLOOR_GRID, rect, 1)
        for (r, c) in self.sys.world.debris_cells():          # PHASE-1: debris (new or existing) is drawn as a red X
            rect = pygame.Rect(c * CELL_PX + 5, r * CELL_PX + 5, CELL_PX - 10, CELL_PX - 10)
            pygame.draw.rect(self.screen, ALERT, rect, border_radius=3)
            pygame.draw.line(self.screen, WHITE, rect.topleft, rect.bottomright, 2)
            pygame.draw.line(self.screen, WHITE, rect.topright, rect.bottomleft, 2)
        er, ec = self.sys.world.entry
        pygame.draw.rect(
            self.screen, ENTRY_TINT,
            pygame.Rect(ec * CELL_PX + 4, er * CELL_PX + 4, CELL_PX - 8, CELL_PX - 8),
            border_radius=4,
        )

    # ── Beacons ───────────────────────────────────────────────────────────────

    def _draw_beacons(self) -> None:
        sys_ = self.sys
        # radio links of the beacon network (beacon -> its parent toward the ONA)
        pos = dict(sys_.beacons.drop_cells)
        pos[0] = sys_.world.entry
        for bid, node in sys_.beacons.nodes.items():
            if node.connected and node.parent in pos:
                a, b = cell_to_px(pos[bid]), cell_to_px(pos[node.parent])
                pygame.draw.line(self.screen, (120, 190, 150), (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), 2)
        # physical beacons: triangles (green = has a route to the ONA, amber = buffering)
        for bid, cell in pos.items():
            if bid == 0:
                continue
            cx, cy = (int(v) for v in cell_to_px(cell))
            col = SUCCESS if sys_.beacons.nodes[bid].connected else WARNING
            pygame.draw.polygon(self.screen, col, [(cx, cy - 9), (cx + 9, cy + 7), (cx - 9, cy + 7)])
            lbl = self.font_sm.render(str(bid), True, TEXT_BRIGHT)
            self.screen.blit(lbl, (cx + 8, cy - 18))
        # ONA marker at the entry
        ox, oy = (int(v) for v in cell_to_px(sys_.world.entry))
        pygame.draw.rect(self.screen, (20, 60, 150), pygame.Rect(ox - 10, oy - 10, 20, 20), 2)
        self.screen.blit(self.font_sm.render("ONA", True, (20, 60, 150)), (ox - 12, oy + 11))
        # memory records: marker by class, ring by lifecycle state
        for rec in sys_.living_map_records:
            cell = self._frame.local_to_cell(rec.x_local, rec.y_local)
            cx, cy = (int(v) for v in cell_to_px(cell))
            state_v = rec.state.value if hasattr(rec.state, "value") else str(rec.state)
            color = CONTRA_COLOR if state_v in ("CLEARED", "CONTRADICTED") else EVENT_COLORS.get(rec.event_type, (110, 110, 118))
            pygame.draw.circle(self.screen, color, (cx, cy), 10)
            pygame.draw.circle(self.screen, WHITE, (cx, cy), 10, 2)
            pygame.draw.circle(self.screen, STATE_RING.get(state_v, TEXT_DIM), (cx, cy), 14, 3 if rec.memory_id == self._selected else 2)
            lbl = self.font_sm.render(rec.event_type[:1], True, WHITE)
            self.screen.blit(lbl, lbl.get_rect(center=(cx, cy)))
        # executors' planned (light) and actual (solid) routes
        for e in sys_.executors:
            if e.mission is not None and not e.is_complete and e.planned_route:
                pts = [tuple(int(v) for v in cell_to_px(c)) for c in e.planned_route]
                if len(pts) > 1:
                    pygame.draw.lines(self.screen, (150, 190, 240), False, pts, 1)
            if e.stats["missions"] > 0 and len(e.actual_route) > 1:
                pts = [tuple(int(v) for v in cell_to_px(c)) for c in e.actual_route]
                pygame.draw.lines(self.screen, (46, 110, 199), False, pts, 2)

    # ── Robots ────────────────────────────────────────────────────────────────

    def _draw_entities(self) -> None:
        for i, w in enumerate(self.sys.writers):
            self._draw_robot(w.px, w.py, w.angle,
                             WRITER_COLORS[i % len(WRITER_COLORS)],
                             w.writer_id, faded=w.dead)
        for i, e in enumerate(self.sys.executors):
            self._draw_robot(e.px, e.py, e.angle,
                             EXECUTOR_COLORS[i % len(EXECUTOR_COLORS)],
                             e.executor_id, faded=e.is_complete)

    def _draw_robot(self, px, py, angle, color, label, faded=False) -> None:
        c = tuple(int(v + (255 - v) * 0.55) for v in color) if faded else color
        pygame.draw.circle(self.screen, c, (int(px), int(py)), 13)
        pygame.draw.circle(self.screen, WHITE, (int(px), int(py)), 13, 2)
        tip = (px + 17 * math.cos(angle), py + 17 * math.sin(angle))
        pygame.draw.line(self.screen, c, (int(px), int(py)),
                         (int(tip[0]), int(tip[1])), 3)
        txt = self.font_sm.render(str(label)[:2], True, WHITE)
        self.screen.blit(txt, txt.get_rect(center=(int(px), int(py))))

    # ── Panel ─────────────────────────────────────────────────────────────────

    def _draw_panel(self) -> None:
        ox = self.map_w
        pygame.draw.rect(self.screen, PANEL_BG,
                         pygame.Rect(ox, 0, self.panel_w, self.h))
        pygame.draw.line(self.screen, PANEL_BORDER, (ox, 0), (ox, self.h), 2)
        x = ox + 16
        y = 12
        y = self._s_header(x, y)
        y = self._s_status(x, y)
        y = self._s_controls(x, y)
        y = self._s_living_map(x, y)
        y = self._s_ona(x, y)
        y = self._s_fleet(x, y)
        y = self._s_mission_queue(x, y)
        self._s_log(x, y)
        self._kbd_hints(x)

    # ── Panel section: header ─────────────────────────────────────────────────

    def _s_header(self, x: int, y: int) -> int:
        self.screen.blit(self.font_big.render("THE LIVING MAP", True, TEXT_BRIGHT), (x, y))
        y += 24
        self.screen.blit(
            self.font_sm.render("TSYP14  ·  IEEE RAS × AESS  ·  ENIB Bizerte",
                                True, TEXT_DIM), (x, y))
        y += 20
        # Phase bar
        phase   = self.sys.phase
        p_label = PHASE_LABEL.get(phase, phase.upper())
        p_color = PHASE_COLOR.get(phase, TEXT_MAIN)
        if self.paused:
            p_label, p_color = "⏸  PAUSED", TEXT_DIM
        pygame.draw.rect(self.screen, HEADER_BG,
                         pygame.Rect(x - 4, y, self.panel_w - 22, 26), border_radius=4)
        self.screen.blit(self.font_med.render(p_label, True, p_color), (x, y + 4))
        y += 32
        # ONA line
        ona       = self.sys.ona
        ona_color = SUCCESS if ona.flash_ticks > 0 else TEXT_DIM
        self.screen.blit(self.font_sm.render(
            self._fit(self.font_sm,
                      f"ONA  fwd={ona.stats['forwarded']}  rej={ona.stats['rejected']}",
                      self.panel_w - 28),
            True, ona_color), (x, y))
        y += 17
        # Stats line
        n = len(list(self.sys.living_map_records))
        auto = "ON" if self.sys.auto_dispatch else "OFF"
        self.screen.blit(self.font_sm.render(
            f"tick={self.sys.tick}   beacons={n}   auto={auto}", True, TEXT_MAIN), (x, y))
        y += 17
        return self._div(x, y + 3)

    # ── Panel section: system status ──────────────────────────────────────────

    def _s_status(self, x: int, y: int) -> int:
        self.screen.blit(self.font_sm.render("SYSTEM STATUS", True, TEXT_DIM), (x, y))
        y += 15
        for i, w in enumerate(self.sys.writers):
            col  = WRITER_COLORS[i % len(WRITER_COLORS)]
            dot  = _W_DOT.get(w.state, DOT_GRAY)
            self._dot(x + 2, y + 6, dot)
            extra = ""
            if not w.dead:
                cov   = round(100 * w.discovered.coverage_ratio(self.sys.total_free_cells))
                extra = f"  cov={cov}%  b={w.beacon_count}"
            self.screen.blit(self.font_sm.render(
                f"  {w.writer_id}  {w.state.value}{extra}", True, col), (x, y))
            y += 16
        for i, e in enumerate(self.sys.executors):
            col  = EXECUTOR_COLORS[i % len(EXECUTOR_COLORS)]
            dot  = _E_DOT.get(e.state, DOT_GRAY)
            self._dot(x + 2, y + 6, dot)
            prog = f"{len(e.reached)}/{len(e.targets)}" if e.mission else "—"
            self.screen.blit(self.font_sm.render(
                f"  {e.executor_id}  {e.state.value}  [{prog}]", True, col), (x, y))
            y += 16
        if not self.sys.writers and not self.sys.executors:
            self.screen.blit(
                self.font_sm.render("  no robots deployed", True, TEXT_DIM), (x, y))
            y += 16
        return self._div(x, y + 3)

    # ── Panel section: controls (buttons) ─────────────────────────────────────

    def _s_controls(self, x: int, y: int) -> int:
        self.screen.blit(self.font_sm.render("COMMAND CONTROLS", True, TEXT_DIM), (x, y))
        y += 14
        bw = self.panel_w - 30

        # 1. Deploy writer
        y = self._btn(x, y, bw, "W  DEPLOY WRITER",
                      WRITER_COLORS[0],
                      lambda: self.sys.deploy_writer(), True)

        # 2. Dispatch executor
        can_exec = bool(self.sys.living_map.mission_candidates())
        y = self._btn(x, y, bw, "E  DISPATCH EXECUTOR",
                      EXECUTOR_COLORS[0],
                      lambda: self.sys.dispatch_executor(), can_exec)

        # 3. Replacement writer
        can_rep = bool(self.sys.writers) and all(w.dead for w in self.sys.writers)
        y = self._btn(x, y, bw, "W2  REPLACEMENT WRITER",
                      WRITER_COLORS[1],
                      lambda: self.sys.deploy_replacement_writer(), can_rep)

        # 4. Auto-dispatch toggle
        a_label = f"AUTO DISPATCH: {'ON  ✓' if self.sys.auto_dispatch else 'OFF'}"
        a_color = SUCCESS if self.sys.auto_dispatch else (100, 108, 124)
        y = self._btn(x, y, bw, a_label, a_color,
                      lambda: self._toggle_auto(), True)

        # 5. Pause / Resume
        p_label = "▶  RESUME" if self.paused else "⏸  PAUSE"
        y = self._btn(x, y, bw, p_label, (74, 82, 100),
                      lambda: self._toggle_pause(), True)

        return self._div(x, y + 4)

    # ── Panel section: living map records ─────────────────────────────────────

    def _s_living_map(self, x: int, y: int) -> int:
        """Command Post view: every memory record with status, version, age, confidence, executor."""
        if y > self.h - 180:
            return y
        records = list(self.sys.living_map_records)
        self.screen.blit(self.font_sm.render(
            f"LIVING MAP  ({len(records)} record{'s' if len(records)!=1 else ''}) — click a row", True, TEXT_DIM), (x, y))
        y += 15
        for rec in records[:7]:
            if y > self.h - 110:
                break
            ev_col = EVENT_COLORS.get(rec.event_type, (110, 110, 118))
            state_v = rec.state.value if hasattr(rec.state, "value") else str(rec.state)
            ring = STATE_RING.get(state_v, TEXT_DIM)
            row = pygame.Rect(x - 4, y - 1, self.panel_w - 24, 15)
            if rec.memory_id == self._selected:
                pygame.draw.rect(self.screen, HEADER_BG, row)
            self._buttons.append((row, lambda m=rec.memory_id: self._select(m)))
            pygame.draw.rect(self.screen, ev_col, pygame.Rect(x, y + 3, 8, 8))
            stale = "~" if rec.is_stale else " "
            mr = self.sys.mission_dispatcher.mission_for_memory(rec.memory_id)
            who = (mr.assigned_executor_id or "-") if mr else "-"
            line = self._fit(self.font_sm, f"  #{rec.memory_id:04X} {rec.event_type[:5]:<5} s{rec.severity} {rec.confidence_pct:>3}%{stale} v{rec.version} {who}",
                             self.panel_w - 130)
            self.screen.blit(self.font_sm.render(line, True, TEXT_MAIN), (x, y))
            self.screen.blit(self.font_sm.render(state_v[:10], True, ring), (x + self.panel_w - 118, y))
            y += 15
        if len(records) > 7:
            self.screen.blit(self.font_sm.render(f"  … {len(records)-7} more", True, TEXT_DIM), (x, y))
            y += 15
        return self._div(x, y + 3)

    def _select(self, memory_id: int) -> None:
        self._selected = None if self._selected == memory_id else memory_id

    # ── Panel section: ONA (beacon records with communication state) ──────────

    def _s_ona(self, x: int, y: int) -> int:
        if y > self.h - 150:
            return y
        ona = self.sys.ona
        self.screen.blit(self.font_sm.render(
            f"ONA  frames {ona.mesh_stats['frames']}  mem {ona.mesh_stats['memory']}  buffered {ona.buffered_count}", True, TEXT_DIM), (x, y))
        y += 15
        sel = next((r for r in self.sys.living_map_records if r.memory_id == self._selected), None)
        if sel is not None:
            gps = f"{sel.gps_lat:.6f},{sel.gps_lon:.6f}" if sel.gps_lat is not None else "n/a"
            host = self.sys.beacons.nodes.get(sel.host_beacon_id)
            lines = [
                f"  beacon B{sel.host_beacon_id}  memory #{sel.memory_id:04X}  {sel.event_type}  [{sel.state.value}]",
                f"  local ({sel.x_local:.2f},{sel.y_local:.2f}) m   GPS {gps}",
                f"  source {sel.source}  v{sel.version}  conf {sel.confidence_pct}%  age {sel.age_seconds:.0f}s  hops {sel.hop_count}",
                f"  comm: {host.comm_state() if host else 'n/a'}" + (f"  passage {sel.passage_state.value}" if sel.passage_state else ""),
            ]
            for ln in lines:
                self.screen.blit(self.font_sm.render(self._fit(self.font_sm, ln, self.panel_w - 30), True, TEXT_MAIN), (x, y))
                y += 14
        else:
            for bid, node in sorted(self.sys.beacons.nodes.items())[:5]:
                ln = f"  B{bid:<2} rec={len(node.records)} hops={node.hops if node.connected else '-'} {node.comm_state()}"
                self.screen.blit(self.font_sm.render(self._fit(self.font_sm, ln, self.panel_w - 30), True, TEXT_MAIN), (x, y))
                y += 14
            if not self.sys.beacons.nodes:
                self.screen.blit(self.font_sm.render("  no beacons deployed yet", True, TEXT_DIM), (x, y))
                y += 14
        return self._div(x, y + 3)

    # ── Panel section: executor fleet ─────────────────────────────────────────

    def _s_fleet(self, x: int, y: int) -> int:
        if y > self.h - 110 or not self.sys.command_post.fleet.all():
            return y
        c = self.sys.command_post.fleet.counts()
        self.screen.blit(self.font_sm.render(
            f"FLEET  avail {c['available']}  busy {c['busy']}  returning {c['returning']}  unavailable {c['unavailable']}", True, TEXT_DIM), (x, y))
        y += 15
        busy = [e for e in self.sys.command_post.fleet.all() if e.category != "available"][:3]
        for e in busy:
            ln = f"  {e.executor_id} {e.capabilities[0].value:<6} {e.state.value} {e.battery_pct}%"
            self.screen.blit(self.font_sm.render(self._fit(self.font_sm, ln, self.panel_w - 30), True, TEXT_MAIN), (x, y))
            y += 14
        return self._div(x, y + 3)

    # ── Panel section: mission queue (PATCH 3) ────────────────────────────────

    def _s_mission_queue(self, x: int, y: int) -> int:
        if y > self.h - 140:
            return y
        from common.enums import MissionStatus
        disp    = self.sys.mission_dispatcher
        all_mis = disp.all_missions()
        live    = [m for m in all_mis
                   if m.status in (MissionStatus.QUEUED,
                                   MissionStatus.ASSIGNED,
                                   MissionStatus.IN_PROGRESS, MissionStatus.BLOCKED)]
        q, a    = disp.queued_count, disp.active_count
        header  = f"MISSION QUEUE  (q={q}  active={a})"
        self.screen.blit(self.font_sm.render(header, True, TEXT_DIM), (x, y))
        y += 15

        STATUS_COLOR = {
            MissionStatus.QUEUED.value:      WARNING,
            MissionStatus.ASSIGNED.value:    EXECUTOR_COLORS[0],
            MissionStatus.IN_PROGRESS.value: SUCCESS,
            MissionStatus.COMPLETED.value:   TEXT_DIM,
            MissionStatus.FAILED.value:      ALERT,
            MissionStatus.BLOCKED.value:     ALERT,
        }
        right_x = self.map_w + self.panel_w - 22
        for mr in live[:4]:
            if y > self.h - 80:
                break
            ev_col = EVENT_COLORS.get(mr.event_type, (110, 110, 118))
            pygame.draw.rect(self.screen, ev_col, pygame.Rect(x, y + 3, 8, 8))
            st_v   = mr.status.value if hasattr(mr.status, "value") else str(mr.status)
            st_col = STATUS_COLOR.get(st_v, TEXT_DIM)
            eid    = mr.assigned_executor_id or "—"
            line   = self._fit(
                self.font_sm,
                f"  {mr.mission_id}  {mr.event_type:<6}  {eid:<3}  P={mr.priority_score:.0f}",
                self.panel_w - 50,
            )
            self.screen.blit(self.font_sm.render(line, True, TEXT_MAIN), (x, y))
            self.screen.blit(
                self.font_sm.render(st_v[:8], True, st_col), (right_x - 52, y)
            )
            y += 15

        if not live:
            self.screen.blit(
                self.font_sm.render("  no active missions", True, TEXT_DIM), (x, y))
            y += 15

        return self._div(x, y + 3)

    # ── Panel section: event log ──────────────────────────────────────────────

    def _s_log(self, x: int, y: int) -> None:
        if y > self.h - 80:
            return
        self.screen.blit(self.font_sm.render("EVENT LOG", True, TEXT_DIM), (x, y))
        y += 15
        tw   = self.panel_w - 28
        rows = max(2, (self.h - 46 - y) // 16)
        for msg, kind in self.sys.log[-rows:]:
            if y > self.h - 44:
                break
            self.screen.blit(
                self.font_sm.render(self._fit(self.font_sm, msg, tw),
                                    True, LOG_COLOR.get(kind, TEXT_MAIN)), (x, y))
            y += 16

    def _kbd_hints(self, x: int) -> None:
        self.screen.blit(
            self.font_sm.render("W writer  E executor  P pause  R reset  Q quit",
                                True, TEXT_DIM), (x, self.h - 20))

    # ── Drawing primitives ────────────────────────────────────────────────────

    def _div(self, x: int, y: int) -> int:
        pygame.draw.line(self.screen, DIVIDER_COL,
                         (x - 4, y), (self.map_w + self.panel_w - 14, y), 1)
        return y + 9

    def _dot(self, x: int, y: int, color: Tuple) -> None:
        pygame.draw.circle(self.screen, color, (x, y), 5)

    def _btn(self, x: int, y: int, w: int, text: str,
             color: Tuple, action: Callable, enabled: bool) -> int:
        rect   = pygame.Rect(x, y, w, BTN_H)
        bcolor = color if enabled else (185, 192, 205)
        pygame.draw.rect(self.screen, bcolor, rect, border_radius=4)
        pygame.draw.rect(self.screen, PANEL_BORDER, rect, 1, border_radius=4)
        lbl = self.font_sm.render(text, True, WHITE if enabled else TEXT_DIM)
        self.screen.blit(lbl, lbl.get_rect(center=rect.center))
        if enabled:
            self._buttons.append((rect, action))
        return y + BTN_H + BTN_GAP

    def _fit(self, font, text: str, max_w: int) -> str:
        if font.size(text)[0] <= max_w:
            return text
        while text and font.size(text + "…")[0] > max_w:
            text = text[:-1]
        return (text + "…") if text else ""

    # ── Toggles ───────────────────────────────────────────────────────────────

    def _toggle_auto(self) -> None:
        self.sys.auto_dispatch = not self.sys.auto_dispatch
        self.sys._log(
            f"[t={self.sys.tick}] auto-dispatch {'ON' if self.sys.auto_dispatch else 'OFF'}",
            "normal")

    def drop_random_debris(self) -> None:
        """Key X: debris appears at a random reachable corridor cell (the environment changes)."""
        import random
        w = self.sys.world
        events = set(w.event_cells_view())
        cands = sorted(c for c in w.reachable_from_entry() if c != w.entry and c not in events
                       and abs(c[0] - w.entry[0]) + abs(c[1] - w.entry[1]) > 2)
        if cands:
            self.sys.add_debris(random.Random(self.sys.tick).choice(cands), "operator")

    def _toggle_pause(self) -> None:
        self.paused = not self.paused
        self.sys._log(
            f"[t={self.sys.tick}] {'PAUSED' if self.paused else 'RESUMED'}", "dim")


# ── Entry-point run loop ──────────────────────────────────────────────────────

def run(seed: int = 42, difficulty: str = "MEDIUM") -> None:
    """
    Interactive demo.  [IMPLEMENTED]

    Keyboard shortcuts
    ------------------
    W       deploy writer (any time)
    E       dispatch executor (needs mission candidates)
    D       deploy replacement writer (once primary writer is done)
    X       drop debris at a random reachable cell (dynamic environment)
    P       pause / resume
    SPACE   smart: start writer → dispatch executor
    R       reset
    Q/Esc   quit
    """
    from simulation.scenario import default_scenario
    system   = LiveMapSystem(default_scenario(seed=seed))
    renderer = Renderer(system)
    running  = True

    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False

            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                renderer.handle_click(ev.pos)

            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif k == pygame.K_r:
                    system.reset()
                    renderer.paused = False
                elif k == pygame.K_p:
                    renderer._toggle_pause()
                elif k == pygame.K_w:
                    system.deploy_writer()
                elif k == pygame.K_e:
                    system.dispatch_executor()
                elif k == pygame.K_d:
                    system.deploy_replacement_writer()
                elif k == pygame.K_x:
                    renderer.drop_random_debris()
                elif k == pygame.K_SPACE:
                    if system.phase == "idle":
                        system.start_writer()
                    elif system.phase == "dead":
                        system.start_executor()

        if not renderer.paused and system.phase not in ("idle", "complete"):
            system.update()

        renderer.draw()
        renderer.clock.tick(60)

    pygame.quit()
