# Failure Case Analysis (Phase 1)

Classification: **RECOVERED** (system continues correctly) ·
**DEGRADED** (continues, with reduced capability or accuracy) ·
**UNRECOVERED** (mission-affecting, no automatic mitigation yet).

| # | Failure | Detection | Response | Class | Where in code |
|---|---------|-----------|----------|-------|----------------|
| FC-01 | Writer runs out of its exploration tick budget ("power loss") before finding every event | `ExplorationPolicy.next_target` returns `tick_budget_exhausted` | Writer stops cleanly; whatever it already preserved is kept | **RECOVERED** — this is the central demo, not a bug (see `tests/test_system.py::test_writer_knowledge_survives_writer_death`) | `writer_robot/exploration.py`, `simulation/system.py` |
| FC-02 | A radio frame is lost in transit (seeded random loss in the simulated medium) | no hop-by-hop ACK arrives | **Phase-1 patch:** the sender retries, re-parents, and stores the frame for later; beacons also re-send their records periodically. Retry counters are reported by `network_report()` | **RECOVERED** for moderate loss (tests: `test_ack_retry_recovers_from_moderate_loss`); with total loss nothing is delivered (`test_crc_is_corruption_detection_not_delivery`). Real RF loss is NOT measured | `communication/mesh.py`, `beacon/node.py` |
| FC-03 | A received packet fails CRC (corruption) | `decode_packet()` raises `DecodeError` | `ONAGateway.ingest()` catches it, increments `stats["rejected"]`, does not forward | **RECOVERED** | `common/protocol.py`, `gateway/ona_gateway.py`, `tests/test_gateway.py::test_ingest_corrupt_packet_rejected` |
| FC-04 | Downstream (Command Post) briefly unavailable when a beacon reading arrives | `forward_fn` callback returns `False` | Message buffered in `ONAGateway._buffer`; `flush_buffer()` retries later | **RECOVERED** | `gateway/ona_gateway.py`, `tests/test_gateway.py::test_flush_buffer_retries_and_succeeds` |
| FC-05 | Executor cannot find a reachable frontier toward a mission target (a gap in what it's discovered that it can't yet route through) | `choose_next_target(..., NavMode.EXECUTE, goal=...)` returns `None` | Executor logs it and skips to the next target rather than stalling forever | **DEGRADED** | `executor_robot/executor.py::_advance_toward_goal` |
| FC-06 | A beacon's claim doesn't hold up when the Executor gets there (stale/wrong information) | Executor re-senses at the target cell; event type doesn't match | `LivingMap.mark_contradicted()`, with a reason string | **RECOVERED** — this is the "verification/contradiction" half of the Living Map concept, not treated as an error | `command_post/map_state.py`, `tests/test_executor.py::test_executor_contradicts_when_nothing_found` |
| FC-07 | Command Post has zero mission candidates (nothing preserved, or everything stale/contradicted) | `MissionPlanner.review_and_assign([])` returns an empty `Mission` | `LiveMapSystem.start_executor()` logs "cannot assign a mission" and does not spawn an Executor | **DEGRADED** — it refuses to send an Executor in with nothing to do. The Phase-1 patch adds a baseline *search* mode (`inherit_memory=False`, used by `simulation/compare_memory.py`) but the Command Post does not yet fall back to it automatically | `command_post/mission_planner.py`, `simulation/system.py` |
| FC-08 | Duplicate beacon deployment for the same real-world event (Writer re-detects something it already preserved) | `memory_decision.evaluate()`'s redundancy check | Second observation is logged as "NOT preserved — redundant", no duplicate beacon | **RECOVERED** | `writer_robot/memory_decision.py`, `tests/test_writer.py::test_writer_does_not_redeploy_for_same_event_twice` |
| FC-09 | Reference GPS/heading measured incorrectly before a mission | No detection mechanism yet — this is an input assumption, not a runtime fault | None | **UNRECOVERED** [ASSUMPTION, not addressed] | `common/coordinates.py` |

| FC-10 | **Phase-1 patch.** A passage the Writer found blocked is remembered, then an Executor is briefed | `BLOCKAGE` memory (`PassageState.BLOCKED/IMPASSABLE/...`) travels in the brief as a hazard | `DiscoveredWorld.mark_passage` -> `plan_route` excludes / penalises it; the Executor takes another way | **RECOVERED** | `common/memory.py`, `simulation/world.py`, `tests/test_executor_inherited.py` |
| FC-11 | New debris appears after the Writer left; the inherited map is wrong | sensing: a cell on the expected route is BLOCKED (`discrepancy` event); or physical bump | re-plan; write a BLOCKAGE record into an audible beacon (or drop one); report to the Command Post | **RECOVERED** when another route exists | `executor_robot/executor.py`, `simulation/dynamic_debris.py` |
| FC-12 | The target is cut off by the new debris | `plan_route` finds no route | wait / retry per `BlockagePolicy`, then report `BLOCKED` naming the blockage; Command Post queues a DEBRIS mission, re-queues the original mission when the blockage is CLEARED | **RECOVERED** if a DEBRIS executor is available and clearance is requested; otherwise the mission stays pending (**UNRECOVERED**) | `command_post/command_post.py`, `tests/test_phase1_system.py` |
| FC-13 | An Executor breaks down / is stranded / runs low on battery | fault / battery threshold | FAILED -> RETURNING -> NEEDS_REPAIR, or LOW_BATTERY -> NEEDS_CHARGING, or OUT_OF_SERVICE; its mission is re-queued; the Command Post never assigns it again until serviced | **RECOVERED** (mission) / **DEGRADED** (fleet) | `executor_robot/executor.py`, `command_post/fleet.py` |
| FC-14 | A beacon dies or the relay chain is broken | missing heartbeats; no ACK -> re-parent | frames are queued (store-and-forward); an alternative parent is used if one is in range | **DEGRADED** - a severed chain stays severed until a robot bridges it | `beacon/node.py` |
| FC-15 | Odometry drift | none in Phase 1 (noise knob exists, default 0) | none | **UNRECOVERED** [ASSUMPTION: ideal odometry by default] | `common/pose.py` |

## Explicitly not yet handled (Phase 2 candidates)

- Beacon battery depletion mid-mission (battery_pct is carried in the
  packet and decremented per deployment as a rough proxy, but nothing
  currently *acts* on a low reading).
- Executor failure/low battery are now simulated (FC-13), but the fault model is a single scripted trigger, not measured.
- Physical radio interference specifics (channel collision, duty cycle) —
  everything here is `MockTransport`, not a radio.
