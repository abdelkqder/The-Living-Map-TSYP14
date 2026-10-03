"""
simulation/snapshot.py
======================
Headless PNG picture of a running/finished simulation (works with SDL_VIDEODRIVER=dummy,
so it is usable in CI and for the report). Draws what the interactive renderer draws, plus the
elements Phase 1 needs to be *visible*:

  environment grid, walls · unknown (fog) vs explored cells · the Writer and its ACTUAL route ·
  beacons with their radio links · the ONA · events and their lifecycle state · Executors and
  their ACTUAL route · the planned route · newly appearing debris · blocked paths ·
  the current mission target

Colours: green=VERIFIED/ACTIVE, amber=ESCALATED, blue=UNVERIFIED, grey=CLEARED/CONTRADICTED,
red=debris/blocked. Simulation logic never imports this module. [IMPLEMENTED]
"""
from __future__ import annotations

import os
from typing import Optional, Sequence

CELL = 34
PAD = 10
STATE_COL = {"UNVERIFIED": (52, 120, 220), "VERIFIED": (30, 160, 90), "ACTIVE": (30, 160, 90),
             "ESCALATED": (230, 150, 20), "CLEARED": (150, 155, 165), "CONTRADICTED": (150, 155, 165)}
EVENT_LETTER = {"FIRE": "F", "GAS": "G", "VICTIM_PRESENCE": "V", "STRUCTURAL": "S", "BLOCKAGE": "X"}


def render_surface(system, *, writer=None, executor=None, show_links: bool = True):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    pygame.font.init()
    w = system.world
    surf = pygame.Surface((w.cols * CELL + 2 * PAD, w.rows * CELL + 2 * PAD + 26))
    surf.fill((247, 248, 250))
    font = pygame.font.SysFont("dejavusans", 14, bold=True)
    small = pygame.font.SysFont("dejavusans", 11)

    def centre(cell):
        return (PAD + cell[1] * CELL + CELL // 2, PAD + cell[0] * CELL + CELL // 2)

    known = {}
    for r in system.writers:
        known.update(r.discovered.all_known_cells())
    for e in system.executors:
        known.update(e.discovered.all_known_cells())
    for r in range(w.rows):
        for c in range(w.cols):
            rect = pygame.Rect(PAD + c * CELL, PAD + r * CELL, CELL - 1, CELL - 1)
            if w.is_wall(r, c):
                pygame.draw.rect(surf, (191, 197, 209), rect)
            elif (r, c) in known:
                pygame.draw.rect(surf, (255, 255, 255), rect)
            else:
                pygame.draw.rect(surf, (226, 229, 236), rect)               # unknown / fog
    pygame.draw.rect(surf, (219, 231, 250), pygame.Rect(PAD + w.entry[1] * CELL, PAD + w.entry[0] * CELL, CELL - 1, CELL - 1))

    for who, col, wid in ([(writer, (109, 99, 214), 3)] if writer else []) + ([(executor, (46, 110, 199), 3)] if executor else []):
        pts = [centre(cc) for cc in getattr(who, "route", None) or getattr(who, "actual_route", [])]
        if len(pts) > 1:
            pygame.draw.lines(surf, col, False, pts, wid)
    for e in system.executors:
        pts = [centre(cc) for cc in e.actual_route]
        if len(pts) > 1 and e.stats["missions"] > 0:
            pygame.draw.lines(surf, (46, 110, 199), False, pts, 2)
        if e.planned_route and not e.is_complete and e.mission is not None:
            pp = [centre(cc) for cc in e.planned_route]
            for a, b in zip(pp, pp[1:]):
                pygame.draw.line(surf, (120, 170, 235), a, b, 1)

    if show_links:                                                   # radio links between beacons / ONA
        pos = {bid: cell for bid, cell in system.beacons.drop_cells.items()}
        pos[0] = w.entry
        for bid, node in system.beacons.nodes.items():
            if node.connected and node.parent in pos:
                pygame.draw.line(surf, (120, 190, 150), centre(pos[bid]), centre(pos[node.parent]), 2)

    for cell, info in w.debris_cells().items():
        r = pygame.Rect(PAD + cell[1] * CELL + 3, PAD + cell[0] * CELL + 3, CELL - 7, CELL - 7)
        pygame.draw.rect(surf, (203, 58, 58), r)
        pygame.draw.line(surf, (255, 255, 255), r.topleft, r.bottomright, 2)
        pygame.draw.line(surf, (255, 255, 255), r.topright, r.bottomleft, 2)
    for rec in system.living_map.all():
        cell = system.world.entry
        from simulation.world import EntryFrame
        cell = EntryFrame(w.entry).local_to_cell(rec.x_local, rec.y_local)
        col = STATE_COL.get(rec.state.value, (100, 100, 100))
        pygame.draw.circle(surf, col, centre(cell), CELL // 3 + 1)
        t = font.render(EVENT_LETTER.get(rec.event_type, "?"), True, (255, 255, 255))
        surf.blit(t, t.get_rect(center=centre(cell)))
    for bid, cell in system.beacons.drop_cells.items():
        node = system.beacons.nodes[bid]
        col = (30, 130, 80) if node.connected else (200, 120, 20)
        pygame.draw.polygon(surf, col, [(centre(cell)[0], centre(cell)[1] - 7), (centre(cell)[0] + 7, centre(cell)[1] + 5), (centre(cell)[0] - 7, centre(cell)[1] + 5)])
        surf.blit(small.render(str(bid), True, (30, 30, 30)), (centre(cell)[0] + 5, centre(cell)[1] - 14))
    o = centre(w.entry)
    pygame.draw.rect(surf, (20, 60, 150), pygame.Rect(o[0] - 8, o[1] - 8, 16, 16), 2)
    surf.blit(small.render("ONA", True, (20, 60, 150)), (o[0] - 10, o[1] + 8))
    for who in list(system.writers) + list(system.executors):
        px, py = (who.px, who.py) if hasattr(who, "px") else (None, None)
        if px is None:
            continue
        col = (109, 99, 214) if who in system.writers else (46, 110, 199)
        pygame.draw.circle(surf, col, (int(PAD + px * CELL / 44), int(PAD + py * CELL / 44)), 7)
        lab = (getattr(who, "writer_id", None) or getattr(who, "executor_id", "?"))
        surf.blit(small.render(lab, True, col), (int(PAD + px * CELL / 44) + 8, int(PAD + py * CELL / 44) - 6))
    legend = "walls grey · fog = unexplored · triangles = beacons (green: has route to ONA) · circles = memory records · red X = debris · blue line = route"
    surf.blit(small.render(legend[:150], True, (90, 96, 110)), (PAD, w.rows * CELL + PAD + 6))
    return surf


def save_png(system, path: str, *, writer=None, executor=None) -> str:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    surf = render_surface(system, writer=writer, executor=executor)
    pygame.image.save(surf, path)
    print(f"  snapshot written: {path}")
    return path
