# THE LIVING MAP — Spatial Memory for Emergency Robots

**TSYP14 Technical Challenge** — IEEE RAS × IEEE AESS Tunisia Section Chapters
Team **Living Map** — ENIB Bizerte, Tunisia

> Give the environment a memory that survives the robot.

A Writer robot explores a GPS-denied, partially-observable environment,
decides what's worth preserving, and deploys small radio beacons —
persistent spatial memory that survives even if the Writer itself fails.
An Executor robot later inherits that memory, navigates to it without ever
having seen the tunnel layout itself, and **verifies or contradicts** what
it finds — so the Living Map evolves as new information arrives, rather
than being a static log of what one robot once saw.

**Adaptive Mission-Aware Spatial Memory.** The system preserves information that stays useful after the
robot that found it is gone: *selective preservation* (the Writer decides what must survive and where),
*spatial memory* (the knowledge lives in beacons placed in the environment), *temporal ageing* (every
record carries its own time and loses confidence), *verification* (executors confirm, clear or contradict
what they find), *dynamic updates* (a blockage that appears later is discovered, preserved and routed
around), *communication-aware placement* (beacons also form the relay chain that gets the memory out) and
*mission-aware use* (the Command Post turns the Living Map into prioritised missions for specialised
executors). This is a software/simulation proof of concept, not a revolutionary AI system.

This is the **Phase 1 software foundation**: a seeded, headless-capable simulation of the required chain

```
Writer -> beacon memory / relay network -> ONA -> Command Post -> Executor fleet -> updated memory
```

with **no direct robot <-> Command Post link** (enforced by tests). Everything physical (radio, world,
sensors, actuators) is **simulated**; nothing in this repository is a hardware measurement.

## Quick start

```bash
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt

python -m pytest -q                                     # full test suite (headless)
python -m simulation.global_demo --seed 42              # main showcase
python -m simulation.writer_demo --seed 42              # exploration / coverage
python -m simulation.fleet_demo --seed 42               # executor fleet + mission planning
python -m simulation.dynamic_debris --seed 42           # debris appears after the Writer left
python -m simulation.compare_memory --seeds 6           # with vs without inherited memory
python run_simulation.py --seed 42                      # interactive pygame window
```

Full guide: [`docs/phase1_demo.md`](docs/phase1_demo.md). Requirement table:
[`docs/phase1_traceability.md`](docs/phase1_traceability.md).

## The problem

A Writer can discover critical information (fire, gas, victims, blocked passages) in a GPS-denied space and
then fail, run out of power, or leave. Without persistence the next robot starts from zero. The environment
therefore needs a memory.

## What was added in the Phase-1 patch

| Area | Before | Now |
|---|---|---|
| Communication | Writer -> `MockTransport` -> ONA (direct delivery) | simulated spatial radio: range + walls, multi-hop beacon chain, ACK/retry, TTL, duplicate suppression, store-and-forward, heartbeats |
| Writer | frontier exploration, coordinates read from the map cell, stopped when the budget ran out | odometry pose (turn + forward), seeded frontier choice, blockage memory, bridge-on-demand relays, **returns to the entry without shutting down** |
| Memory | event + coordinates | versioned record: id, type, local + GPS coords, severity, confidence, source, times, age, status (UNVERIFIED / VERIFIED / ACTIVE / ESCALATED / CLEARED / CONTRADICTED), passage state, hop count |
| Environment | static | debris can appear after the Writer left and be cleared |
| Command Post | existing dispatcher | Living Map keyed by memory id, priority with explainable components, capability matching, fleet registry, brief composer |
| Executor | one capability-agnostic run | specialised (FIRE / GAS / VICTIM / DEBRIS) state machine: ASSIGNED -> DEPLOYING -> ON_SITE -> WORKING -> VERIFYING -> UPDATE_MEMORY -> RETURNING -> AVAILABLE, plus FAILED / LOW_BATTERY / NEEDS_REPAIR / NEEDS_CHARGING / OUT_OF_SERVICE |
| ONA | could hand data on | receives, validates, translates local -> GPS, buffers, carries; **never plans** |

Preserved from the previous revision: the 18-byte wire protocol and CRC-8, the coordinate-transform source of
truth, the shared A* / frontier engine, the structural "no ground-truth cheating" test, the Living Map version
guard, the dispatcher and benchmark, writer replacement, and all existing unit tests (a few were updated
where the semantics legitimately changed - see below).

## Interactive controls

| Key / Button | Action |
|---|---|
| `W` / [DEPLOY WRITER] | Deploy a writer robot (works any time) |
| `E` / [DISPATCH EXECUTOR] | Dispatch an executor (greyed-out when no candidates) |
| `D` / [REPLACEMENT WRITER] | Deploy replacement writer (once W1 is offline) |
| `P` / [PAUSE] | Pause / resume simulation |
| [AUTO DISPATCH: ON/OFF] | Toggle event-driven executor dispatch |
| `X` | Drop debris at a random reachable cell (dynamic environment) |
| `SPACE` | Smart start: writer → executor (sequential path) |
| `R` | Reset same scenario |
| `Q` / Esc | Quit |

## What you're watching (interactive UI)

1. **Writer explores** - no map given. It senses a local radius, builds its own discovered map, picks frontier
   targets, tracks a dead-reckoned pose, and decides what to preserve. Beacons appear as triangles (green = has a
   route to the ONA, amber = buffering). Each record travels hop by hop to the ONA and then to the Command Post
   as `UNVERIFIED`. The Writer then walks back to the entry; it does not shut down.

2. **Concurrent operation** - with AUTO DISPATCH on, the Command Post ranks queued missions (class, status,
   confidence, age, distance) and assigns an AVAILABLE executor of the right capability, even while the Writer is
   still exploring. The brief reaches the executor only through the ONA mailbox.

3. **Writer failure** - `force_writer_fail()` or `writer_fail_tick`: the Writer stops where it is; records already
   delivered to the Command Post are unaffected.

4. **Executor feedback loop** - it navigates with inherited memory, re-plans around anything that differs from it,
   performs its task, writes its observation into a beacon (`VERIFIED / ACTIVE / ESCALATED / CLEARED /
   CONTRADICTED`, version bumped), returns to the entry and becomes AVAILABLE (or NEEDS_REPAIR / NEEDS_CHARGING).

5. **Dynamic environment** - press `X`: debris appears at a random reachable cell. Executors discover it by
   sensing, preserve it as a BLOCKAGE record, and the Command Post decides what to do next.

6. **Replacement writer** - `[REPLACEMENT WRITER]` deploys W2 with a blank discovered map and fresh beacon ids; its
   reports of the same event are merged by the Command Post as corroboration.

Click a row in the LIVING MAP panel to see that record's ONA detail (beacon, memory id, local and GPS coordinates,
source, version, confidence, age, hops, communication state).

## Benchmark mode

Three configurations run against the same scenario for direct comparison:

| Config | Description |
|---|---|
| **A Sequential** | Writer finishes → one executor dispatched (classic baseline) |
| **B Concurrent** | Pool of 3 executors + auto-dispatch as beacons arrive |
| **C No dispatch** | Writer only — no executor, no Living Map verification |

```
python run_simulation.py --benchmark --difficulty MEDIUM --seed 42
```

Config B demonstrates the core engineering claim: **the Living Map lets
an executor respond to discoveries while the Writer is still exploring**,
not after it finishes.

## Status

Labels: **IMPLEMENTED** (logic + tests), **SIMULATED** (modelled, not measured), **PLANNED**, **ASSUMED**, **MEASURED**.

| Feature | Status |
|---|---|
| Writer frontier exploration, odometry pose, return to entry | IMPLEMENTED / SIMULATED world |
| Selective preservation + placement policy (event, relay, host reuse) | IMPLEMENTED |
| Multi-hop beacon communication (range, walls, ACK/retry, TTL, dedupe, buffering) | IMPLEMENTED logic / SIMULATED radio |
| Physical SX1278 LoRa radio, beacon firmware, drop mechanism | PLANNED (the LoRa transport is a stub) |
| 18-byte beacon wire format, CRC-8/MAXIM, mesh frame, versioning | IMPLEMENTED |
| ONA: validate, local -> GPS, buffer, forward, mailbox | IMPLEMENTED / SIMULATED links |
| Local -> global coordinate transform (heading-aware) | IMPLEMENTED (mathematics) |
| Real-world GPS / localisation accuracy | **NOT MEASURED** (no accuracy figure is claimed) |
| Living Map lifecycle + ageing, Command Post planner, fleet registry | IMPLEMENTED |
| Specialised executors + full state machine, failure branches | IMPLEMENTED / SIMULATED actuators |
| Dynamic debris, blockage memory, policy-driven response | IMPLEMENTED / SIMULATED |
| With-vs-without-memory comparison | IMPLEMENTED (simulation only, one arena) |
| Physical prototype (ESP32 + sensors), battery life, sensor calibration | PLANNED (Phase 2) |

## Honest notes on changed behaviour

* `WriterRobot.dead` is now a **legacy alias for "no longer exploring"**. The accurate field is `state`: `RETURNED`
  (got home, still operational) or `DEAD` (failed). `max_ticks` bounds *exploration*; the way home is on top.
* `ExecutorState.COMPLETED` is no longer a resting state: a healthy executor ends `AVAILABLE` at the entry.
* Tests updated for those semantics: `test_writer_stops_within_tick_budget`, `test_writer_state_returned_after_exhausted`,
  `test_executor_state_available_after_mission`, `test_living_map_accumulates_from_both_writers` (see the commit/patch diff).
* The old README claimed 269 tests. The suite that was actually in the repository ran **224**; the number now is printed by `pytest -q`.
* `common/coordinates.py` documented `heading` as a compass bearing; the code (and every test) implements
  counter-clockwise-from-east. Only the documentation was wrong and has been corrected.

## Known limitations

Radio range and wall attenuation, task durations, priority weights, odometry (ideal by default), the GPS reference
point and the beacon battery model are assumptions. Not validated physically: RF propagation, LoRa hardware,
GPS/localisation accuracy, sensor calibration, battery performance, the beacon drop mechanism. The brief carries
beacon positions and blockage records, not a corridor-level map: an obstruction between two beacons is
discovered by sensing, not inherited.

## Layout

```
common/           protocol (18-byte beacon + mesh frames), memory record, pose/odometry, clock, coordinates, enums, mission brief
communication/    mesh.py (simulated spatial radio, RobotLink) / mock_transport.py (unit tests) / lora_transport.py (stub)
beacon/           node.py (memory + relay), deployer.py (drop mechanism + stock), memory_node.py (legacy)
gateway/          ONA gateway (receive, validate, translate, buffer, mailbox)
writer_robot/     perception, exploration, memory_decision, placement, writer
executor_robot/   capabilities, executor (state machine)
command_post/     map_state (Living Map), planner, fleet, mission_dispatcher, command_post
navigation/       A* + frontier engine + weighted re-planning
simulation/       world, scenario, system (wiring), phase1 helpers, the demos, snapshot, renderer (pygame)
tests/            pytest suite (headless)
docs/             architecture, protocol, failure analysis, phase1_demo, phase1_traceability
```

## Structural guarantees

* No ground-truth cheating: `tests/test_world.py::test_robot_code_never_touches_ground_truth_internals`.
* No direct robot -> Command Post path: `tests/test_architecture.py`.
