# Phase 1 demonstration guide

Everything below is headless (no display needed), seeded and reproducible. Same command + same seed = the same output.
Numbers are produced by the simulation; **this guide does not quote performance figures** - run the commands.

```bash
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m pytest -q                                     # the full suite
```

| # | Simulation | Command | What you should see |
|---|---|---|---|
| A | Global Living Map (main showcase) | `python -m simulation.global_demo --seed 42` | Writer map + route, beacon table with hops/parent, ONA table with local **and** GPS coordinates, Command Post decisions with reasons, executor timelines, Living Map before/after (UNVERIFIED -> CLEARED/ACTIVE) |
| A' | same, deeper chain | `python -m simulation.global_demo --seed 5` | a multi-hop beacon chain (several relays between the deepest beacon and the ONA) |
| A'' | Writer fails mid-exploration | `python -m simulation.global_demo --seed 42 --writer-fail 1500` | the Writer ends DEAD, not at the entry; what it already preserved still reaches the Command Post |
| B | Writer exploration / coverage | `python -m simulation.writer_demo --seed 42` | ASCII map of the discovered arena, coverage, distance, frontiers, dead ends, blocked paths, alternative routes, return success. `--debris 2` adds debris present at entry; `--compare 5` runs 5 seeds and counts distinct trajectories |
| C | Executor fleet + mission planning | `python -m simulation.fleet_demo --seed 42` | priority table with components, why each executor was chosen, mission history, executor timelines, memory updates, fleet status; one executor is re-assigned after returning. `--fault E02` breaks one executor: it ends NEEDS_REPAIR and gets no new mission |
| D | Dynamic debris | `python -m simulation.dynamic_debris --seed 42` | debris appears after the Writer left; Executor detects the discrepancy, re-plans, preserves the BLOCKAGE; Command Post updates the Living Map |
| D' | debris that seals the target | `python -m simulation.dynamic_debris --seed 42 --placement seal` | executor waits/retries, reports BLOCKED, Command Post dispatches a DEBRIS executor, memory becomes CLEARED, the original mission is re-queued and completes |
| D'' | other placements / policies | `--placement random`, `--placement chokepoint`, `--max-wait 20 --retries 0`, `--no-request-debris`, `--hold`, `--compare-seeds 6` | different seeds -> different debris cells and, potentially, different responses |
| E | With vs without inherited memory | `python -m simulation.compare_memory --seeds 6` | per-seed tables: travel distance, mission time, wrong turns, cells explored, replans, failed/repeated approaches, events recovered. Same arena, seed and debris in both modes; the Living-Map run does **not** know the new debris in advance |
| UI | Interactive pygame | `python run_simulation.py --seed 42` | keys: `W` writer, `E` executor, `X` drop debris, `D` replacement writer, `P` pause; click a record row to open its ONA detail |
| bench | Existing A/B/C benchmark | `python run_simulation.py --benchmark --seed 42 --difficulty MEDIUM` | unchanged from the previous revision, now over the beacon mesh |

Common options: `--seed N`, `--json` (machine-readable), `--png FILE` (headless snapshot with fog, routes, beacon links, debris, records).

Interpreting results honestly:

* A demo prints what happened in that run. If an effect is absent for some seeds (the comparison has seeds where memory gives no
  advantage because the target is near the entry), that is the result.
* The radio range, wall attenuation, task durations, priority weights and the GPS reference point are **assumptions** in code
  (see the traceability table). Nothing here is a hardware measurement.
