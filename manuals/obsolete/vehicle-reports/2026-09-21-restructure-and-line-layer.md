# Vehicle acceptance report, 2026-09-21: agv_core restructure + LINE layer Increment 1

Tested on agv-01 at git `305e38b` (branch slam-roadmap) with the uncommitted changes
listed at the end. Operator at the E-stop, wheels on blocks for every armed step.
The legacy controller was dropped by decision during the session: its sections
(restructure 4.2–4.6, 6) are SKIPPED, not failed.

## VEHICLE-TEST-agv_core-restructure.txt

| Step | Result | Note |
|---|---|---|
| 0.1–0.3 | PASS | restructure commit `dc9f608`; services inactive; `/run/lock/amr` locks free |
| 1.1, 1.2 | PASS | `agv-01`, `/home/gvipc-evo-01/agv_can/profiles` |
| 1.3 | PASS (PYTHONPATH) | `pip3 install -e .` refused (build backend lacks PEP 660); `env/vehicle.sh` PYTHONPATH resolves from `/tmp` |
| 1.4 | PASS | 27 modules under walk_packages (25 modules + 2 subpackages), `FAILED: none` |
| 2.1 | PASS after 2 test fixes | `93 test function(s), 898 check(s)`, all pass. Fixes: stale `core/`, `drivers/` `__pycache__` dirs removed; `tests/test_config.py` pp-refusal test now unsets `pp.expect` itself (agv-01 has had them filled since 2026-09-18) |
| 2.2 | PASS | `layout failures: none` |
| 3.1 | PASS | all six tools print usage (`verify_drivers` has no `--help` and just runs, read-only) |
| 3.2 | PASS | nodes 1, 2, 10 present, no collisions |
| 3.3 | PASS | both BLV-R answer; revision/serial SDO abort 0x06090011 (as before) |
| 3.4 | PASS | IMU stationary, accel 0.9907 g |
| 3.5 | PASS | `verify_bus` refused: "can is owned by agv_controller 17299", bus never opened |
| 4.1 | PASS | `main.py` up, web on 5000 |
| 4.2–4.6 | SKIPPED | legacy controller dropped |
| 5.1 | PASS | clean `colcon build --symlink-install`, 11 packages |
| 5.2 | PASS | `deployment validation passed` |
| 5.3 | PASS | repo-root profiles path from the install space |
| 5.4 | SKIPPED | covered by 5.5 (every base node came up under the service) |
| 5.5 | PASS | first start went FAULT `BASE_NOT_READY`: both drives held error register 0x11 (lost-PC-heartbeat alarm from the earlier stop). Drive power cycle, then IDLE in 8 s. No import errors |
| 5.6 | PASS | profile agv-01, `autopilot` section present, drives armed, 50 Hz feedback |
| 5.7 | covered by LINE 3.2 | both wheels driven under ROS through the same drive path; the manual jog itself was not pressed |
| 6.1, 6.2 | SKIPPED | legacy dropped; 3.5 proved the lock with real fcntl in the direction that matters |

## VEHICLE-TEST-line-layer.txt

**Section 0 (R1)**: `1800h:01 = 0x0000018A` (enabled, COB 0x18A), `1800h:02 = 255`
(event-driven), `1800h:05 = 10 ms`, `1800h:03` not implemented (SDO abort).
**Verdict: control-rate TPDO available. Continue.**

| Step | Result | Note |
|---|---|---|
| 1.1–1.4 | PASS | 53 amr_line tests; `11.3 0.94 1000.0 40.0`; line launch test |
| 2.1 | PASS after env fix | `env/sim.sh` lacked the PYTHONPATH export `vehicle.sh` has; sim base nodes died with `No module named 'agv_core'`. Added. Supervisor in sim needs `AMR_SUPERVISOR_SIM=1` |
| 2.2 | PASS | LINE in 1.6 s, `line_available`, LineState IDLE at the mode's generation |
| 2.3 | PASS, message differs | IDLE while ARMED is refused by the earlier panel rule ("needs a valid panel in MANUAL"); the selector leaving AUTO faults an ARMED follower. "disarm it first" was seen from a RUNNING follower with the selector in MANUAL |
| 2.4 | **FAIL, then PASS** | Follower published on plain `/amr/line_cmd`; the mux subscribes to `/amr/layers/g<N>/amr/line_cmd`. Mux stayed "line: no fresh command" for the whole run. Fixed in `line_layer.launch.py` (remap) + launch test. After: mux LINE, `linear.x` max 0.300, 29 mm → 0 with 14 mm overshoot, DONE "track lost" at 6.05 m, IDLE allowed after DONE |
| 2.5 | PASS | track_hz 99–103, sample_age ≤ 12 ms |
| 3.1 | **FAIL, then PASS** | No `/amr/line_track` at all. The MLS boots Operational; `DriveLink.arm()` broadcast NMT Pre-operational (node 0) silenced it, and the reader's fallback only fired for zero frames ever. Fixed: per-drive NMT in `canopen.py`; `mls_track.py` sends NMT Start to node 10 at start and every 2 s while the stream is missing, falls back to SDO meanwhile. After: **source tpdo, 100.0 Hz, age ≤ 1.4 ms, seq contiguous**, `nmt_starts 1`. Polarity: tape to the vehicle's LEFT reads **positive** (+105 mm), matching the sim plant's 2026-08-31 convention; the acceptance file's sentence has it backwards |
| 3.2 | PASS | RUNNING, mux LINE, `linear.x` 0.25, steering followed the tape (operator) |
| 3.3 | **FAIL, then PASS** | Reset ignored while RUNNING (wheels kept turning; stopped by `/amr/line/clear`). `job.tick` honoured Reset only in HOLD/DONE/FAULT. Fixed (+ test). Retest: Reset ended the run (operator) |
| 3.4, 3.5, 3.7 | SKIPPED | operator decision |
| 3.6 | SKIPPED | known deviation from sim: selector → MANUAL while RUNNING gives `HOLD pending` (resumes on Start once back in AUTO), not `FAULT authority`; the mux drops LINE at once so the vehicle stops. `job.py`'s docstring says authority. Not changed |
| 4, 5 | NOT RUN | no floor run; the three 4.4 numbers were not taken |

Gains did not move.

## Changes on the vehicle (uncommitted)

- `amr_ws/env/sim.sh` — PYTHONPATH export (env)
- `tests/test_config.py` — pp-refusal test unsets `pp.expect` itself (test)
- `amr_ws/src/amr_base/test/test_ownerlock.py` — subprocess imports `agv_core.ownerlock` (restructure leftover, test)
- `amr_ws/src/amr_bringup/launch/line_layer.launch.py` — remap `/amr/line_cmd` to the generation-private topic (production, OK'd)
- `amr_ws/src/amr_bringup/test/test_launch_layers.py` — records node remappings; LINE case asserts the remap
- `amr_ws/src/amr_base/amr_base/canopen.py` — NMT Pre-operational per drive, not broadcast (production, OK'd)
- `amr_ws/src/amr_base/amr_base/mls_track.py` — NMT Start to node 10 at start / every 2 s while the TPDO stream is missing; fallback for a stream that died; `nmt_starts` diagnostic (production, OK'd)
- `amr_ws/src/amr_base/test/test_canopen.py`, `test_mls_track.py` — updated + 2 new tests
- `amr_ws/src/amr_line/amr_line/job.py` — Reset ends a RUNNING run (production, OK'd; not autopilot/branch)
- `amr_ws/src/amr_line/test/test_line_authority.py` — new test
- removed: stale `core/__pycache__`, `drivers/__pycache__`, `drivers/canbus/__pycache__`

Test state: `tests/run_all.py` 898 checks pass; amr_base 206, amr_line 54, amr_bringup launch 12 pass; ruff check clean.

## Open items

- Floor run (LINE section 4) and the other-layers regression (section 5) not done.
- 3.6 behaviour vs the docstring (HOLD pending vs FAULT authority): decide which is wanted.
- The acceptance file's polarity sentence (3.1) should read "positive when the tape is to the LEFT".
- Web UI has no LINE mode button; mode LINE is requested by service.
- Drive alarm 0x11 after stopping the service: worth understanding why the stop leaves the PC-loss alarm latched.

## U11 — legacy retired (same day, after acceptance)

Operator decision: Increment 1 accepted on sections 0–3; Increments 2–4 skipped
for now; U11 executed. Tag `legacy-final` = `aa02ec7`.

Deleted after the import audit (no importer under `amr_ws/src`, `agv_core` or a
bench tool): `canworker.py`, `main.py`, `app/`, `agv_core/health.py`,
`agv_core/runlog.py`, `agv_core/drivers/lidar_scan.py`,
`agv_core/drivers/modbus_io.py`, `amr_ws/src/amr_base/amr_base/legacy_guard.py`
(the owner lock is the exclusion now), `amr_ws/deploy/amr_nav.service`,
`amr_mapping.service`, `amr_legacy.env`, `tests/test_canworker.py`,
`test_web.py`, `test_health.py`. Kept although only legacy used them:
`drivers/rfid.py` (Increment 2), `blindrun.py` (commissioning imports it),
`rpdo.py` (canopen imports it), `lss.py`/`verify_bus.py` (bench tools).
Archived: `logs/00xx-*` → `manuals/obsolete/legacy-runs/`.

`tests/run_all.py` per-module checks, before → after:

| module | before | after | what went |
|---|---|---|---|
| test_config | 82 | 82 | — |
| test_canworker | 70 | — | module |
| test_invariants | 29 | 8 | three Controller cases |
| test_health | 32 | — | module |
| test_canmon | 50 | 50 | guard-bypass scan moved to canopen.py |
| test_lss | 55 | 55 | — |
| test_imu | 46 | 46 | — |
| test_rpdo | 58 | 56 | wiring check moved to canopen.py (4 checks, was 6) |
| test_rfid | 18 | 18 | — |
| test_dio | 65 | 61 | legacy health-monitor case |
| test_lidar | 47 | 41 | lidar_scan bench tool |
| test_panel | 85 | 48 | four Controller-driven cases |
| test_blindrun | 65 | 32 | Controller, app page, runlog cases |
| test_web | 129 | — | module |
| test_layout | 43 | 10 | main.py entry point, app/ assets, runlog anchor |
| test_mls | 24 | 24 | — |
| **total** | **898** | **531** | |

ROS side: `amr_base` + `amr_bringup` 241 tests pass, ruff clean,
`deploy/validate.sh` passes. Docs: README rewritten, RUNBOOK §6/§7,
`hardware-reconciliation.md` note, unified plan U11 note, package docstrings.

## Review fixes (same day, gpt6-analysis-doc.md), not yet on the vehicle

Implemented and unit-tested only — nothing below has run on agv-01. All
workspace suites pass (809), `tests/run_all.py` 531, both ruff commands clean,
`deploy/validate.sh` passes. Vehicle steps that now need re-running: LINE 3.3
(Reset, also from ARMED) and 3.6 (selector → MANUAL is FAULT `authority` at
once, Reset to clear — the acceptance file's wording is now what the code
does).

| ID | Done | Where |
|---|---|---|
| F01 | selector leaving AUTO / LEASE_LINE withdrawn → FAULT `authority` on the first tick (a stale panel still goes through the 0.5 s grace) | `amr_line/job.py` `_taken_over` |
| F02 | a torque loss during a `drives`/`rate`/`track`/`pending` hold takes the safety cause; `estop` then waits for Start | `job.py` `_hold_tick` |
| F03 | per-map writer lock (`map_lock`), unique `mkdtemp` stages, fsync before the rename, publish checks the stage manifest's identity; survey save, edits and the fixture writer all run inside one lock hold | `map_bundle.py`, `mapping_session_node.py`, `fixtures.py` |
| F04 | short IMU PDOs are rejected (no sample, no stamp, no receipt time); mapping width checked at start; `rejected`/`last_reject` in diagnostics | `mls_imu.py` |
| F05 | Reset cancels ARMED; Reset wins over Start in the same tick | `job.py` |
| F06 | the node reads `line_min_track_hz` and `auto_resume_hold_s` from `agv_core.config`; an unmeasured rate (< 3 samples) is not usable | `line_follow_node.py`, `track.py` |
| F07 | mux LINE ceiling `line_v_max_m_s` 0.30 (+ optional `line_w_max_rad_s`), curvature-preserving, applied to the slewed output too | `gating.line_cap`, `cmd_mux_kinematics_node.py` |
| F08 | MLS absent (or 1800h:01 unread) at start is probed every 5 s with one 50 ms SDO and re-acquired; `restarts` in diagnostics; `mode: off` stays off | `mls_track.py`, `mls_imu.py` |
| F09 | `/output_paths` has a receipt time (`field_fresh_s` 0.5); no fresh sample = unknown: refuses to arm, torque loss = `estop`; sim launches `field_source:=assume_clear` via the supervisor's `real` | `line_follow_node.py`, `line_layer.launch.py`, `supervisor_node.py` |
| F10 | store readers validate names/revisions and containment (symlinks too); `load_mission` checks its schema; `load_route` and `map_bundle.load` compare the file's identity with the one asked for | `store.py`, `map_bundle.load_manifest`, `readiness.resolve_map`, `navigation_layer.launch.py` |
| F12 | one `_body()` parser: list/string/number/null/malformed JSON → 400 before any adapter call; initialpose and survey_move reject booleans, non-finite and bad revisions | `amr_web/server.py` |
| F13 | core ruff clean (24 → 0, `zip(strict=False)` keeps behaviour); `pythonpath` ini → `conftest.py` (pytest 6.2 on the image) | `pyproject.toml`, `conftest.py` |

Not done: **F11** (LINE in the web UI — Increment 4, a feature, not a fix),
the executor side of F09 (its `None`-unknown handling and classification window
are the 2026-09-19/20 operator decisions; left as they are), and the §4
investigations. F08's bus-occupancy figure and F09's `/output_paths` rate
(assumed ~34 Hz from the scan datagram) need the vehicle.
