# Codebase bug review and revision plan

Date: 2026-09-17  
Repository: `agv_can`  
Reviewed baseline: commit `55d400a`, plus the working tree present during review  
Deliverable: planning document; application code has not been changed

## Assessment

The existing offline suites pass, but they do not cover several consequential failures at the boundaries between the browser, motion authority, CAN feedback, action cancellation, and process recovery. Address the motion and stop-path findings before relying on the affected workflows on the vehicle.

This document contains **34 actionable findings: 19 P1 and 15 P2**, followed by an implementation sequence and acceptance plan. Findings distinguish **reproduced offline** behavior from **source-traced** failure paths. Priorities describe engineering urgency; they do not assert that a field incident has occurred or assess the independent FX3/STO safety chain.

- **P1:** Fix before using the affected motion workflow, or before running the affected hardware/tooling procedure.
- **P2:** Fix in the next reliability revision; incorrect behavior, data loss, unavailable recovery, or incomplete validation.
- All implementation tasks below are **OPEN**. Passing existing tests does not close a finding; its regression acceptance must pass.

## Scope and verification

The review covered the first-party legacy controller and ROS workspace, including Python control logic, JavaScript operator pages, interfaces, configuration, launch/service scripts, simulation, tests, and operating documentation. Generated build/install/log output, binary assets, and vendor manuals were not treated as application source.

| Area | Review focus |
|---|---|
| `main.py`, `config.py`, `canworker.py`, `core/` | Startup, configuration, queue scheduling, panel authority, motion, blind runs, health, logs, ownership |
| `drivers/` | CAN transactions, RPDO/SDO paths, device telemetry, Modbus/DIO, RFID, lidar decoding, bench utilities |
| `app/` | Legacy API and browser command paths, diagnostics, parameters |
| `amr_base` | Drive lifecycle, command mux, leases, wheel feedback, odometry, panel, IMU, commissioning |
| `amr_localization` | Bias estimation, readiness, scan consistency, freshness and reset behavior |
| `amr_maps`, `amr_navigation` | Coordinate transforms, map parsing, footprint clearance, route schema/compiler, persistence |
| `amr_mission` | Mapping save/recovery, route execution, cancellation, resume, active-map binding |
| `amr_bringup` | Supervisor transactions, process ownership, readiness, fault/recovery, launch composition |
| `amr_web` | API validation, jog sessions, asynchronous browser behavior, live overlays, editor rendering |
| `amr_sim`, `amr_description`, `amr_interfaces` | Simulation assumptions, geometry/frame configuration, message contracts |
| Tests and deployment | Offline coverage, simulation isolation, shell/service validation, rollback compatibility |

Commands below describe what was actually run, not additional vehicle actions to perform as part of this document.

| Verification | Result | Qualification |
|---|---|---|
| Root: `python3 tests/run_all.py` | Exit 0; **81 functions, 785 checks passed** | An uncaught exception in a background arm-test thread was printed despite success; see R34 |
| `amr_ws`: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test` | **204 passed, 8 skipped** | Simulation/launch suites were skipped; this is not vehicle acceptance |
| `amr_ws`: normal pytest invocation with plugin autoload, `AMR_SIM_TESTS=0`, domain 89 | Exit 5; **1 skipped**, without running the intended suite | Environment-dependent collection problem; disabled-autoload run above recovered the offline suite |
| `amr_ws`: `ruff check src` | Passed | Lint does not exercise the identified failure paths |
| `amr_ws`: `./deploy/validate.sh` | Passed | Host systemd warnings included a netplan unit permission error and an unsupported snapd unit setting |
| Temporary Python probes using source imports, fake inputs and fake buses | Confirmed the cases listed below | No CAN/Modbus devices opened; no ROS nodes initialized by these probes |
| JavaScript harness executing the actual `jogpad.js` with deferred API responses and stub DOM/timers | Confirmed delayed-press/release race | No browser or vehicle API contacted |

Selected observed probe results:

| Input/failure injected | Observed result |
|---|---|
| Pointer down, pointer up before `/manual/press` resolves, then successful late response | Nonzero refresh issued (`v=0.2`); another refresh timer scheduled |
| Manual command sequence 7, then same-session zero release with sequence 0 | Mux retained `v=0.2`, sequence 7 |
| NaN on both wheel commands, maximum RPM 4000 | `target_rpm()` returned `(4000, 4000)` |
| Raw counts `2147483647` → `-2147483648` | Correct wrapped delta is 1; direct position conversion produced about **−2248.84 m** for the probe's 1,080,000 counts/rev and 0.09 m radius |
| Both wheel-valid flags false, fresh callback receipt | Route prerequisites returned no error |
| Entire route outside a 2 m × 2 m free map | Clearance validation accepted it |
| Fresh input ages and converged covariance, no scan-match measurement | Localization allowed confirmation with `scan_match=None` |
| Second-node heartbeat-consumer write fails | Guard-tracking flag remained false; subsequent normal disarm made zero cleanup writes |
| First fault-stop RPDO send fails | Only one send attempted for two drives; link entered FAULT |
| Goal acceptance arrives after pause | Accepted action handle retained without cancellation |
| Truncated PGM header ending at `255` with no delimiter | Parser did not terminate within the isolated 0.5 s timeout |
| Configuration watchdog set to positive infinity | Configuration validation accepted it |

No physical motion, live fault injection, service restart, CAN reconfiguration, or full simulation stack was executed. R29 must be fixed before using the affected simulation test on a shared host. Source anchors below refer to the reviewed baseline and may move during implementation.

## Finding index

| ID | Priority | Finding | Evidence |
|---|---|---|---|
| R01 | P1 | Jog can start after input release while press request is pending | Reproduced offline |
| R02 | P1 | Jog release zero is rejected as an old sequence | Reproduced offline |
| R03 | P1 | Nonfinite wheel commands become maximum RPM | Reproduced offline |
| R04 | P1 | Fault-stop send failure skips the other drive | Reproduced offline + loop trace |
| R05 | P1 | Arm rollback and heartbeat-guard retirement are incomplete | Partial-arm failure reproduced; teardown source-traced |
| R06 | P1 | Signed encoder rollover corrupts wheel odometry | Reproduced offline |
| R07 | P1 | Invalid or incomplete wheel feedback can retain motion authority | Prerequisite failure reproduced; producer/consumer trace |
| R08 | P1 | Late action acceptance/results survive cancellation or affect a new attempt | Late acceptance reproduced; result race source-traced |
| R09 | P1 | Route speed limits are not applied to motion output | Source-traced |
| R10 | P1 | Clearance validation accepts space outside the map | Reproduced offline |
| R11 | P1 | Localization confirmation can use missing or obsolete consistency evidence | Missing measurement reproduced; stale evidence source-traced |
| R12 | P1 | Pose seeding and overlays are not bound to the viewed map | Source-traced |
| R13 | P1 | Commissioning jobs are not bound to an authority/encoder-scale epoch | Source-traced |
| R14 | P1 | Legacy manual driving survives loss of panel validity | Source-traced |
| R15 | P1 | Legacy blocking diagnostics and queue draining delay stop processing | Source-traced |
| R16 | P1 | Infinite configuration values can disable watchdogs | Reproduced offline |
| R17 | P2 | ROS publishes physical Reset but no control consumer acts on it | Source-traced; legacy behavior differs |
| R18 | P1 | Turn state leaks across runs and pause omits stopping travel | Source-traced |
| R19 | P2 | Grid algorithms disagree about rotated map origins | Source-traced |
| R20 | P2 | `repeat_count` is validated but not executed | Source-traced |
| R21 | P2 | Malformed route data bypasses structured validation | Source-traced |
| R22 | P2 | Default mission names collide across maps | Source-traced |
| R23 | P2 | Concurrent route saves violate immutable revision semantics | Source-traced |
| R24 | P2 | Truncated PGM data can hang the parser | Reproduced offline |
| R25 | P2 | Supervisor exceptions can leave operations permanently pending | Source-traced |
| R26 | P2 | Recovery cannot rebuild a failed base process group | Source-traced |
| R27 | P2 | Survey calls lack complete deadlines and save-error recovery | Source-traced |
| R28 | P2 | Standalone/rollback launch paths no longer support their web workflow | Source-traced |
| R29 | P1 | Simulation fault injection kills matching processes across the host | Source-traced; deliberately not executed |
| R30 | P2 | Stored route step IDs are inserted as HTML | Source-traced |
| R31 | P2 | Commissioning evidence can escape its directory or lose aborted-run data | Source-traced |
| R32 | P2 | Logging errors retain an unbounded buffer and can skip close | Source-traced |
| R33 | P1 | CAN bench tools bypass the repository's device-owner lock | Source-traced |
| R34 | P2 | Test harnesses can hide thread failure or miss intended collection | Observed during verification |

## Motion, authority, and feedback

### R01 — Pending browser press is not cancelled by release

**Location:** `amr_ws/src/amr_web/amr_web/static/jogpad.js:37` (`press`), `:56` (`release`).

- **Failure:** `held` is assigned only after awaiting `/api/manual/press`. Pointer-up, pointer-cancel, blur, or key-up before that response calls `release()`, which returns because `held` is still null. The late response starts nonzero refreshes and keeps scheduling them after the operator has released the input. The server watchdog stays refreshed, so it does not fix this race. Multiple presses can also be pending simultaneously.
- **Change:** Record physical input and a pending-press token before awaiting. Every release path must invalidate that token. A late successful response must release its returned session without refreshing it. Handle rejected/network-failed press and refresh promises, and prevent overlapping press attempts.
- **Acceptance:** Deferred-response JavaScript tests for release, blur, hidden tab, Stop, and pointer cancellation before acceptance. Assert zero nonzero refreshes after cancellation, no surviving timer, and successful subsequent fresh press.

### R02 — Server release commands lose to the mux sequence filter

**Locations:** `amr_ws/src/amr_web/amr_web/server.py:534`; `amr_ws/src/amr_base/amr_base/cmd_mux_kinematics_node.py:170`.

- **Failure:** `/api/manual/release` sends a same-session zero with `seq=0`. After any accepted refresh, the mux rejects it because its sequence is not greater than the previous command. The previous velocity remains eligible until command expiry, instead of immediate release invalidation. Offline delivery of sequence 7 followed by release retained sequence 7 and `v=0.2`.
- **Change:** Define an explicit revocation contract keyed by instance, generation, and session. A revoke must invalidate the session at the mux even when it has no newer velocity sequence. Retain a bounded tombstone so a delayed refresh cannot revive it. Serialize release/publication consistently with refresh handling.
- **Acceptance:** Exercise the full server-to-mux message path, including reordered refresh/revoke delivery, duplicate release, and a new session. Release must inhibit on the next mux cycle; old-session packets must stay invalid.

### R03 — NaN becomes full-scale wheel RPM

**Locations:** `amr_ws/src/amr_base/amr_base/canopen.py:589`; `amr_ws/src/amr_base/amr_base/drive_node.py:156`; `amr_ws/src/amr_base/amr_base/cmd_mux_kinematics_node.py:141`.

- **Failure:** The drive callback and RPM conversion do not reject nonfinite wheel values. Python's current `min`/`max` expression turns NaN into the positive maximum; the probe returned `(4000, 4000)`. The manual-command callback checks finiteness, but that protection does not cover all teleop/navigation/wheel-command inputs.
- **Change:** Reject nonfinite values at every motion ingress and again at the final drive conversion. Invalid input must replace the current command with zero/inhibit and a diagnostic, rather than retaining an earlier nonzero target. Validate relevant scaling parameters too.
- **Acceptance:** NaN, ±infinity, invalid timestamps/lifetimes and overflow-sized values on each ingress, with a previously nonzero command. No invalid sample may produce or preserve nonzero RPDO output.

### R04 — A failed fault-stop send prevents stopping the second drive

**Locations:** `amr_ws/src/amr_base/amr_base/canopen.py:557`; `amr_ws/src/amr_base/amr_base/drive_node.py:228`.

- **Failure:** `fault()` attempts `send_target(0, 0)` once and suppresses its exception. `send_target()` exits its per-node loop at the first failure, skipping the other node. The link then enters FAULT, where periodic target transmission stops, while PC heartbeat transmission continues. A drive that missed the stop may retain its previous target without losing the PC heartbeat.
- **Change:** Attempt zero independently for both nodes. Give FAULT an explicit stop-confirmation state with bounded zero retries and independent measured standstill/drive-state checks. Define the failure fallback when stop cannot be delivered; do not continue advertising controller health indefinitely after an unconfirmed stop.
- **Acceptance:** Fail each individual RPDO send once and persistently. Verify both drives are attempted, no nonzero refresh follows, and the documented bounded fallback occurs when either stop is unconfirmed.

### R05 — Arming is not fully transactional; disarm removes the guard too early

**Location:** `amr_ws/src/amr_base/amr_base/canopen.py:394`, `:469`, `:523`.

- **Failure:** Guard tracking becomes true only after both 1016h writes succeed. Failure on the second node leaves the first configured but untracked. PDO/guard setup and post-enable scale acquisition sit outside the rollback `try`. Normal disarm also returns immediately if the software state is already DISARMED. Separately, disarm clears the heartbeat consumer before confirming zero or de-energization, then performs potentially failing blocking writes. An interrupted teardown can therefore remove the fallback before stop is established.
- **Change:** Track mutations per node and wrap the whole arm transaction in rollback. Maintain heartbeat scheduling through arm/disarm without concurrent unsynchronized CAN ownership. On deliberate exit, establish and verify stop/de-energization before retiring each guard. Preserve truthful cleanup status if any step fails.
- **Acceptance:** Inject failures at every mutating arm/teardown step, including second-node guard setup and post-enable reads. Assert no enabled-but-untracked node, no skipped cleanup due to an initial software state, and correct watchdog behavior during long transactions. Verify actual 1016h behavior on the bench after offline tests.

### R06 — Encoder wrap is interpreted as kilometres of travel

**Locations:** `amr_ws/src/amr_base/amr_base/canopen.py:145`, `:174`; `amr_ws/src/amr_base/amr_base/drive_node.py:355`; `amr_ws/src/amr_base/amr_base/diff_drive_odom_node.py:76`.

- **Failure:** Signed raw positions are converted directly to absolute wheel radians. Odometry subtracts consecutive radians, so the signed 32-bit boundary becomes a huge displacement. A correct `unwrap_i32()` helper exists but is not used by this odometry path. Simulation's continuous wheel position does not naturally expose the defect.
- **Change:** Accumulate wrapped raw-count deltas per wheel before producing continuous positions. Define baseline resets on invalid feedback, encoder reset, scale change, rearm, and publisher restart; a discontinuity must invalidate/rebaseline rather than move the estimated vehicle.
- **Acceptance:** Forward and reverse rollover on either/both wheels, negative mounting signs, invalid-data gaps, reset and rearm. One count must remain one count of travel; stationary reset must not translate or rotate odometry.

### R07 — Feedback arrival is mistaken for valid feedback

**Locations:** `amr_ws/src/amr_base/amr_base/drive_node.py:282`, `:355`, `:381`; `amr_ws/src/amr_base/amr_base/canopen.py:578`; `amr_ws/src/amr_localization/amr_localization/localization_monitor_node.py:108`; `amr_ws/src/amr_mission/amr_mission/route_executor_node.py:249`, `:372`.

- **Failure:** Drive liveness can be refreshed by heartbeat or diagnostic replies even if a required TPDO stops. The publish condition treats a changed `(status_time, position_time)` tuple as a complete new pair, although just one member may have changed. Invalid WheelStates can consequently keep arriving. Localization touches its wheel-age timer without checking validity, and route prerequisites check receipt age but not validity. DriveStatus also derives `operational` from cached statuswords without requiring fresh feedback.
- **Change:** Separate transport liveness, fresh status, fresh position, encoder-scale validity, and operational readiness. Require each needed TPDO independently. Propagate validity and source age through localization, mission execution, commissioning and mux gating; invalid feedback must inhibit the affected motion class.
- **Acceptance:** Drop TPDO1 or TPDO2 independently while retaining heartbeat and diagnostic replies. Also test stale status with fresh position and invalid scale. Motion must inhibit within the configured feedback budget, and diagnostics must identify the missing evidence.

### R08 — Action callbacks are correlated only to a run, not a goal attempt

**Location:** `amr_ws/src/amr_mission/amr_mission/route_executor_node.py:553`–`:605`.

- **Failure:** Pause/abort cancels only an already-known handle. If acceptance arrives later, a paused run can retain the accepted goal without cancelling it; an aborted run can ignore the response and orphan the accepted goal. Results from an earlier goal can also overwrite `goal_result` or clear the handle for a resumed goal because both use the same run ID. Feedback does not enforce its supplied run identity either. Mux inhibition limits immediate movement during pause, but the action lifecycle remains wrong and can contaminate resumed execution.
- **Change:** Introduce a goal-attempt token containing run, step, attempt and authority identity. Track pending acceptance. Cancel any late accepted obsolete goal, ignore its feedback/results, and require terminal cancellation or a bounded fault before issuing a replacement permit/goal.
- **Acceptance:** Deferred fake ActionClient tests covering pause/abort before acceptance, cancellation after resume preparation, old result after new acceptance, rejected goals, exceptions, and server disappearance. Only the current attempt may change progress or its handle.

### R09 — Route speed settings do not cap executed commands

**Locations:** `amr_ws/src/amr_navigation/amr_navigation/compiler.py:94`, `:109`; `amr_ws/src/amr_mission/amr_mission/route_executor_node.py:510`, `:530`, `:763`; `amr_ws/src/amr_navigation/config/nav2_params.yaml:30`, `:103`.

- **Failure:** Route linear/angular limits influence estimated duration and allowance, but no route-specific cap reaches FollowPath/Spin output or the final mux. Nav2 remains configured with its fixed 0.30 limits. Selecting a slower route therefore does not enforce that route's requested maximum.
- **Change:** Carry validated effective limits with the active mission authority. Apply final caps in the command gate, and configure the controller/behavior consistently so its progress and timeout expectations agree. Use the speed-limit mechanism supported by the installed Nav2 version and require successful application before enabling the run. Publish the effective caps for diagnosis.
- **Acceptance:** Run identical routes with multiple linear/angular caps and assert the actual output stream respects each cap, including corners, spin, resume, and changed missions. Cover minimum-controller-speed conflicts and failure to apply a requested limit.

### R10 — Map boundaries disappear from clearance checks

**Location:** `amr_ws/src/amr_navigation/amr_navigation/footprint.py:57`, `:83`, `:113`; `amr_ws/src/amr_navigation/amr_navigation/validate.py:81`.

- **Failure:** Rasterization clips masks to the map. A footprint or route wholly outside produces an empty mask; zero occupied/unknown cells is then considered clear. Partial overhang is also discarded. `np.roll` dilation additionally wraps occupancy masks onto the opposite map edge.
- **Change:** Explicitly reject swept footprints and margins extending outside known map bounds. Preserve an out-of-bounds result independently of the in-map mask. Replace wrapping dilation with bounded/padded operations and conservatively account for boundary cells and margin rounding.
- **Acceptance:** Fully external routes, every edge/corner, partial footprint overhang, rotations near boundaries, and margins smaller than one cell. A left-edge obstacle must not appear on the right edge through dilation.

### R11 — Localization can confirm without fresh scan-consistency evidence

**Locations:** `amr_ws/src/amr_localization/amr_localization/readiness.py:78`, `:91`, `:152`, `:234`; `amr_ws/src/amr_localization/amr_localization/localization_monitor_node.py:134`.

- **Failure:** CHECKING only evaluates scan consistency when measurements are non-null, so missing measurements allow confirmation. Scan receipt is marked fresh before the timestamped transform succeeds. Repeated TF failures can leave an old successful consistency result active. Initial-pose/reset paths do not clear all covariance and consistency evidence.
- **Change:** Require finite, sufficiently recent scan-consistency evidence and post-seed covariance from the current localization attempt. Timestamp successful comparisons separately from raw scan receipt; expire old results. Reset all readiness proof on new pose/map/generation, and distinguish unavailable evidence from poor alignment.
- **Acceptance:** No initial comparison, TF failure after one good comparison, missing current-map evidence, new seed with old low covariance, and delayed samples from an earlier attempt. None may enable confirmation until fresh matching evidence arrives.

### R12 — The displayed map can differ from the map receiving an initial pose

**Locations:** `amr_ws/src/amr_web/amr_web/static/run.js:7`, `:36`; `amr_ws/src/amr_web/amr_web/static/liveview.js:8`; `amr_ws/src/amr_web/amr_web/server.py:559`; `amr_ws/src/amr_web/amr_web/adapter.py:507`.

- **Failure:** The Run page lets the user view a map other than the active navigation map. Overlays compare the generic frame name `map`, not map identity. Dragging an initial pose sends coordinates without the viewed map ID/revision/hash or expected generation. Coordinates drawn on map B can seed localization on active map A.
- **Change:** Bind overlays and pose requests to `(instance, generation, map_id, revision, sha256)`. Disable pose seeding/confirmation on a mismatched view and reject mismatches server-side immediately before publication. Clear mission previews and asynchronous map loads when their identity becomes obsolete.
- **Acceptance:** Two distinct maps with the same frame name; switch the viewed map, switch the active map during a drag/request, and complete image requests out of order. No pose or preview may be applied to a different map identity.

### R13 — Commissioning jobs can survive a change of authority or encoder scale

**Locations:** `amr_ws/src/amr_base/amr_base/commissioning_node.py:86`, `:91`, `:100`, `:124`; `amr_ws/src/amr_base/amr_base/commissioning.py:74`, `:108`.

- **Failure:** Planning uses the last wheel scale without checking current planning admission. The job stores no supervisor instance/generation and ignores later `counts_per_rev` changes. Lease callbacks overwrite the current generation without monotonicity checks. PREPARED jobs survive authority loss; an old job can become eligible again later. A Start edge is buffered as a boolean without identifying which prepared plan/authority it belongs to.
- **Change:** Admit planning only in the intended stable commissioning context, with valid fresh scale and confirmed stillness. Bind the plan to the authority identity and scale; invalidate it on either change. Filter stale leases, and require a Start edge newer than the accepted plan under that same identity. Unknown stillness must fail admission.
- **Acceptance:** Plan across IDLE/navigation transitions, restart the supervisor, replay old leases, change scale after planning, and deliver Start immediately before plan acceptance. An old plan must never be silently relabelled for new authority.

### R14 — Legacy manual jog remains permitted after the panel disappears

**Locations:** `canworker.py:417`, `:366`, `:968`.

- **Failure:** Invalid panel input aborts a blind run but returns without stopping ordinary manual jog or invalidating the cached manual mode. DIO is not a critical health source. The browser can continue renewing `drive()` because it checks cached mode/armed/fault state, not current valid MANUAL input.
- **Change:** Treat loss of panel validity as loss of manual authority. Zero/inhibit immediately, invalidate outstanding manual input, and require a fresh valid panel and fresh operator input for recovery. Keep this policy explicit rather than relying on the unrelated driver-health fault path.
- **Acceptance:** While fake manual jog is active, lose DIO replies and continue browser keepalives. Targets must zero within the panel freshness budget. Reconnection in either selector position must not replay the held direction.

### R15 — Legacy queued diagnostics can starve watchdog and panel processing

**Locations:** `app/server.py:183`; `canworker.py:409`, `:619`, `:738`, `:767`, `:1452`, `:1468`, `:1562`.

- **Failure:** `_drain_queue()` runs to exhaustion before panel/watchdog processing. Preflight is callable while armed and uses blocking SDO reads; a timed-out HTTP wait does not cancel queued work. Periodic telemetry performs six reads with the default 0.4 s failure timeout, with monitoring adding more. Under missing responses or repeated queued diagnostics, stop processing can be delayed far beyond the intended control period.
- **Change:** Bound queued work per cycle and attach enqueue deadlines/cancellation. Refuse disruptive preflight while moving. Split diagnostics into budgeted operations with short failure deadlines, prioritize zero/authority handling, and retain one CAN owner. Review the legacy controller's PC-loss behavior as part of retaining it as a fallback.
- **Acceptance:** Fake slow/missing SDO responses plus a flood of preflight requests while a stop/panel-loss event arrives. Measure worst-case zero-command latency and require it to meet the chosen control budget; expired requests must never execute later.

### R16 — Numeric configuration accepts infinity

**Locations:** `config.py:463`, `:662`, `:702`, `:875`.

- **Failure:** Float coercion accepts nonfinite numbers. Python's JSON reader also accepts nonstandard `Infinity`/`NaN`, and ordinary finite-looking JSON numbers can overflow to infinity. The positive-only watchdog check accepts infinity, making a manual command deadline never expire. Other positive-only parameters have the same validation pattern.
- **Change:** Reject nonfinite values during numeric coercion and JSON constant parsing, then apply field-specific bounds and representability checks. Include derived values and ROS parameters used in watchdogs, geometry and actuator conversion.
- **Acceptance:** Test NaN, ±infinity, exponent overflow, numeric booleans and boundary values across the schema. Invalid configuration must fail before any driver is opened and identify the offending field.

### R17 — Physical Reset has no ROS control behavior

**Locations:** `amr_ws/src/amr_base/amr_base/panel_node.py:106`; `amr_ws/src/amr_base/amr_base/cmd_mux_kinematics_node.py:159`; `amr_ws/src/amr_mission/amr_mission/route_executor_node.py:243`; `amr_ws/src/amr_base/amr_base/commissioning_node.py:86`; compare `canworker.py:1052`.

- **Failure:** ROS publishes `reset_edge`, but the control consumers do not act on it. The legacy controller has an explicit physical-reset handler that stops/clears control state. This is a panel behavior regression; it is not a claim that this button substitutes for E-stop. The localization page's Reset is a separate API action.
- **Change:** Specify and implement the migrated physical Reset contract. Recommended behavior: inhibit/stop the active software operation, invalidate pending motion and held plans, then require a fresh start where appropriate. Keep drive-alarm power-cycle requirements intact. Update the runbook to distinguish physical Reset, localization reset, and fault acknowledgement.
- **Acceptance:** Exercise Reset in manual jog, prepared/running commissioning, ready/executing/paused mission, and fault. It must not inadvertently start, resume, or remotely clear a prohibited drive alarm.

### R18 — Turn state is not reset consistently and pause loses stopping travel

**Locations:** `amr_ws/src/amr_mission/amr_mission/route_executor_node.py:291`, `:401`, `:465`, `:530`, `:591`, `:715`, `:757`.

- **Failure:** Loading a new run resets phase/travelled distance but not all of `turn_target`, `turn_centre`, and `turn_acc0`. Those fields are cleared on normal step completion, so an aborted/faulted turn can contaminate the next run. Pause accounts travel at the interruption instant, then discards the accumulator baseline; additional rotation while stopping is omitted. A prepared resume also enters EXECUTING on Start without re-running all resume checks at that moment.
- **Change:** Centralize per-run/per-step/attempt state initialization. Account measured rotation through verified standstill, then freeze the resume baseline. Revalidate resume conditions at the physical Start edge, including continuity and position constraints.
- **Acceptance:** Abort halfway through one angle and load a different angle at the same and different centers; pause while angular velocity is nonzero; move the vehicle after Prepare resume; inject an odometry gap. Assert correct remaining angle and no reuse of prior-run geometry.

## Route geometry, schemas, and storage

### R19 — Rotated map origins are handled inconsistently

**Locations:** `amr_ws/src/amr_maps/amr_maps/grid.py:45`, `:156`; `amr_ws/src/amr_maps/amr_maps/raycast.py:57`; `amr_ws/src/amr_navigation/amr_navigation/footprint.py:57`; `amr_ws/src/amr_localization/amr_localization/localization_monitor_node.py:166`.

- **Failure:** Map loading and pixel/world UI helpers support `origin_yaw`, while cell conversion, footprint rasterization, raycasting and scan endpoint indexing assume axis-aligned origins. A rotated map can display correctly while clearance and scan comparisons use different coordinates. Keepout grids are also used without verifying geometric alignment.
- **Change:** Use one tested world/grid transform in all consumers, including runtime obstruction checks. Validate resolution, shape, frame and origin compatibility for keepouts. Until all consumers support yaw, reject nonzero yaw explicitly rather than accepting inconsistent geometry.
- **Acceptance:** Known obstacles and routes on translated and rotated grids, with asymmetric geometry so a sign/axis mistake cannot pass. Compare browser pixel coordinates, clearance, raycasts and scan matching; reject misaligned keepouts.

### R20 — Repeated routes execute only one pass

**Locations:** `amr_ws/src/amr_navigation/amr_navigation/route.py:95`; `amr_ws/src/amr_navigation/amr_navigation/validate.py:63`; `amr_ws/src/amr_mission/amr_mission/route_executor_node.py:325`; `amr_ws/src/amr_mission/amr_mission/run_fsm.py` (`step_done`).

- **Failure:** Validation requires closed geometry for repetitions, but the executor loads the number of steps in one compiled pass and the FSM ends after that pass. No execution path consumes the requested repetition count.
- **Change:** Add bounded pass tracking, include it in RunState/UI and evidence, and transition to DONE only after all requested passes. Keep cancellation tokens and pause progress specific to pass/step/attempt. Do not duplicate unbounded geometry in memory.
- **Acceptance:** One, two and three passes; pause/abort at a pass boundary; fault on a later pass; invalid fractional, zero, negative and excessive counts. Report actual progress and perform exactly the requested passes.

### R21 — Route input errors escape validation or are silently coerced

**Locations:** `amr_ws/src/amr_navigation/amr_navigation/route.py:70`, `:113`; `amr_ws/src/amr_navigation/amr_navigation/validate.py:65`; `amr_ws/src/amr_navigation/amr_navigation/compiler.py:94`; `amr_ws/src/amr_web/amr_web/server.py:221`.

- **Failure:** Raw `int`/`float` conversions and `Limits(**...)` can raise ValueError/TypeError outside the expected RouteError handling. Zero speed reaches duration division. Fractional repeat counts are truncated by `int()`. Missing/defaulted pose and insufficient numeric bounds can hide malformed requests or cause excessive compilation work.
- **Change:** Validate the complete schema before compiling: exact integer fields, finite positive speeds, supported frames/step types, bounded geometry/counts, sane tolerances, and required pose fields. Convert input failures to structured field/step errors and HTTP 400/422; preserve internal exceptions as genuine server errors.
- **Acceptance:** API tests for malformed types, unknown limit keys, missing pose, zero/nonfinite speed and large inputs. Fail predictably without storing a route/mission or allocating unbounded sample arrays.

### R22 — Default mission IDs overwrite missions from other maps

**Locations:** `amr_ws/src/amr_web/amr_web/server.py:323`; `amr_ws/src/amr_navigation/amr_navigation/store.py:97`.

- **Failure:** Default IDs are `<route_id>_rev<route_revision>`, but route IDs/revisions are scoped to each map and missions share one global directory. Two maps with the same route name/revision choose the same file. `save_mission()` replaces it with `os.rename()`.
- **Change:** Include map identity in generated mission IDs, or use a unique immutable ID with a separate display name. Reject conflicting existing IDs; allow an idempotent retry only when the complete intended references match. Handle existing files without silently renaming external references.
- **Acceptance:** Create identical route names on two maps and save both missions. Both must remain available. Explicit duplicate IDs with different references must fail without changing the original bytes.

### R23 — Concurrent route writes are not immutable or reliably atomic

**Location:** `amr_ws/src/amr_navigation/amr_navigation/store.py:53`, `:97`.

- **Failure:** Revision allocation is a scan followed by an existence check and rename, with no lock. Two Flask threads can choose the same revision and share the same PID-derived temporary filename. Writes can race, one request can lose its temp file, or a later rename can replace an already published revision. Atomic rename alone does not provide no-overwrite semantics.
- **Change:** Serialize revision allocation/publication across threads and processes, use unique temporary files, and commit with exclusive/no-replace behavior. Flush file and directory metadata where durability is required. Return the hash of the bytes actually committed, and clean abandoned temporaries without affecting published revisions.
- **Acceptance:** Barrier-synchronized concurrent writers targeting one route and one mission. Every success must identify a distinct immutable revision or an explicitly idempotent result; no acknowledged content/hash may subsequently change.

### R24 — EOF in a PGM header loops forever

**Location:** `amr_ws/src/amr_maps/amr_maps/grid.py:68`; map verification/loading callers.

- **Failure:** The token scanner increments `end` until a whitespace byte appears. At EOF, `raw[end:end+1]` stays empty and `.isspace()` remains false forever. A truncated file can hang loading/verification; supervisor map resolution also performs this work while holding its request lock.
- **Change:** Implement a bounded token scanner with explicit EOF errors and supported maxval/payload handling. Validate dimensions, finite positive resolution, thresholds and resource limits before allocation. Convert malformed-map errors to BundleError/API errors. Avoid unbounded parsing under the supervisor control lock.
- **Acceptance:** Empty files, EOF at every header position, missing comment terminators, short payload, invalid dimensions/maxval, supported whitespace forms and oversized images. Each must finish under a hard test timeout; malformed input must not stall lease publication.

## Supervision, recovery, and deployment

### R25 — Supervisor exception handling does not terminate the active operation

**Locations:** `amr_ws/src/amr_bringup/amr_bringup/supervisor_node.py:412`, `:721`, `:739`, `:753`.

- **Failure:** The loop catch changes mode to FAULT but leaves the transaction, survey operation and pending future intact. Later ticks still prioritize those objects and can continue or repeatedly fail them. Recovery can remain blocked by a pending operation. `_boot()` is outside that catch, so a spawn exception can terminate the worker while the ROS process remains alive. A process exit during a survey can finish a newly created fault operation while leaving the original survey operation pending.
- **Change:** Route startup, transaction, future and process-exit failures through one terminal-failure path. Finish the actual affected operation exactly once, invalidate async callbacks/workers, clear transaction ownership, and inhibit motion. Monitor supervisor-loop health and expose unexpected thread death as an actionable failure. Accepted no-op mode requests must also return a valid completed operation ID instead of an empty ID that the web client polls.
- **Acceptance:** Inject exceptions in boot, map resolution, transaction steps and future results; kill an owned layer during each survey operation. No operation may remain pending after terminal failure or later report success from an obsolete callback. Recovery admission and no-op operation polling must work.

### R26 — Base-process failure cannot be repaired by the recovery transaction

**Location:** `amr_ws/src/amr_bringup/amr_bringup/supervisor_node.py:378`, `:446`, `:721`.

- **Failure:** Recovery stops/replaces the `layer` group, but never rebuilds a failed `base` group. It first waits for a mux generation acknowledgement, which cannot arrive if the base/mux is dead. If a partial base failure leaves the mux alive, committing IDLE still does not establish a newly healthy base.
- **Change:** Give base recovery its own sequence: retain inhibition, reap all owned base descendants, restart the base, await its fresh identity/feedback/mux readiness, then commit IDLE. If a failure requires a whole-service restart, report that explicitly and expose a terminal recovery outcome instead of cycling through impossible acknowledgement waits.
- **Acceptance:** Simulate complete base loss and individual required base-node failure. Recovery must either restore verified IDLE with one owner per device or give a precise terminal restart-required outcome; never publish healthy IDLE with a dead base.

### R27 — Survey deadlines and save rollback are incomplete

**Locations:** `amr_ws/src/amr_bringup/amr_bringup/supervisor_node.py:632`; `amr_ws/src/amr_mission/amr_mission/mapping_session_node.py:261`.

- **Failure:** The supervisor bounds the save future but not returned/abort futures. A live but unresponsive coordinator can leave those operations pending indefinitely. The coordinator enters SAVING before revision/staging creation, which occurs outside the rollback `try`; an early filesystem failure leaves inconsistent save state and an exceptional service result.
- **Change:** Give every survey RPC a deadline and terminal outcome, and ignore late responses for invalidated operation IDs. Include allocation/staging in the save transaction. Preserve the distinction between failed-before-publication and unknown-publication-outcome; reconcile an existing published revision before any retry.
- **Acceptance:** Never-completing returned/abort calls, unavailable services, unwritable map directories, staging failure, serialization failure, timeout after publish and late success. Each operation must terminate with consistent mode/state and no duplicate revision.

### R28 — Standalone launches and rollback units use a supervisor-dependent UI

**Locations:** `amr_ws/src/amr_bringup/launch/nav.launch.py:1`; `amr_ws/src/amr_bringup/launch/mapping.launch.py:1`; `amr_ws/src/amr_web/amr_web/server.py:484`; `amr_ws/src/amr_web/amr_web/adapter.py:415`; `amr_ws/src/amr_web/amr_web/static/run.js:43`; `amr_ws/deploy/amr_nav.service:18`.

- **Failure:** The advertised standalone launches start base/web/layer without a supervisor. The web manual, mode and survey paths require supervisor identity/services, and the mission selector filters against an active map supplied by the supervisor. Thus the documented web workflow and rollback launch composition no longer agree.
- **Change:** Make supported launch modes explicit. Prefer routing supported interactive entry points through the supervisor, while keeping a coherent version-pinned prior release for actual rollback. If standalone bench mode remains supported, implement and test a deliberately bounded adapter/authority contract for it; do not remove identity checks as a shortcut.
- **Acceptance:** From each advertised launch/service entry point, test page availability, manual admission, map/survey flow and mission listing. Update README/RUNBOOK/deployment units together, including the exact compatible fallback release/configuration.

### R29 — Simulation fault injection uses host-wide `pkill`

**Location:** `amr_ws/src/amr_bringup/test/test_unified_sim.py:347`.

- **Failure:** The test runs `pkill -9 -f async_slam_toolbox_node`, matching unrelated instances on the host, including another ROS domain or a hardware stack. ROS_DOMAIN_ID does not isolate process signals. Related host-wide process searches also make isolation assertions ambiguous.
- **Change:** Inject failure only into the PID/process group created and owned by the test fixture. Track descendants and verify ownership before signalling. Scope process assertions to that fixture and ensure teardown cannot target unrelated services.
- **Acceptance:** Run with an unrelated sentinel process whose command line matches the old pattern. The injected test child must die while the sentinel survives. Only then enable the full simulation suite on a shared development host.

## Web rendering, evidence, tooling, and tests

### R30 — Stored step IDs execute as HTML in the route editor

**Locations:** `amr_ws/src/amr_web/amr_web/static/editor.js:82`; `amr_ws/src/amr_navigation/amr_navigation/route.py:70`.

- **Failure:** Step IDs from persisted/API route data are interpolated into `innerHTML`. They are not restricted to safe identifiers, so a crafted saved step can inject active HTML when opened. The resulting script runs in the operator page's origin and can access its APIs.
- **Change:** Build the step list with DOM nodes and `textContent`. Validate identifiers for consistency, but keep output encoding even for validated values. Check other dynamic HTML sites for persisted/user-derived values.
- **Acceptance:** Save/load a valid route whose step ID contains HTML/event-handler text. It must display literally, produce no active injected element, and make no unexpected command/API call.

### R31 — Commissioning evidence paths and lifecycle are unreliable

**Locations:** `amr_ws/src/amr_base/amr_base/commissioning.py:74`, `:88`; `amr_ws/src/amr_base/amr_base/commissioning_node.py:113`, `:170`, `:210`.

- **Failure:** Arbitrary `spec.id` becomes part of the evidence path. Absolute or traversal values can escape the configured directory. Clear/abort destroys the run without the normal tick transition that writes evidence, losing the active segment's terminal record. Shutdown likewise has no evidence-finalization path. `_results_path` can retain an earlier job's filename, and second-resolution names can collide. Gyro values used in evidence also have no sample-age check.
- **Change:** Validate a plain string plan ID and enforce resolved-path containment. Snapshot/finalize active-run evidence before clearing, stopping or replacing it, including an abort reason and valid sample intervals. Reset per-job metadata and use unique exclusive filenames. Report write failure accurately rather than returning an older path.
- **Acceptance:** Traversal/absolute/non-string IDs; clear during a segment; orderly shutdown; two runs in one second; disk-write failure; stale gyro. Evidence must stay contained, identify the correct run, preserve completed/aborted segments, and mark missing sensor intervals.

### R32 — Logging failure repeatedly retries a growing in-memory buffer

**Location:** `core/runlog.py:157`, `:173`, `:179`.

- **Failure:** If `_drain()` fails before clearing `_buf`, each tick adds another row and retries the increasingly large write. A persistent disk error creates unbounded memory growth and added control-loop work. `close()` can also skip the actual handle close when draining raises.
- **Change:** Bound the logging queue and choose an explicit drop/disable policy with counters and one actionable error. Move potentially slow flushing off the control path where practical, preserve row ordering, and close the file in a `finally` block independently of draining.
- **Acceptance:** Simulated ENOSPC, read-only filesystem and stalled writer. Buffer size and control-loop latency stay bounded; dropped data is reported; close is attempted even after drain failure.

### R33 — Hardware bench utilities bypass exclusive ownership

**Locations:** `drivers/canbus/drive_forward.py:215`; `drivers/canbus/lss.py:422`; `drivers/canbus/read_imu.py:397`; compare `core/ownerlock.py` and the ROS/legacy owner acquisition.

- **Failure:** Direct CAN bench commands open the bus without acquiring the shared device lock used by runtime owners. `--go` authorizes a write but does not establish exclusive access. A drive or LSS configuration tool can therefore run alongside the controller. Even active SDO reads can overlap the runtime's request/reply channel.
- **Change:** Apply the same CAN ownership guard to repository utilities that transmit requests or commands, before opening or modifying the device. Report the current owner and exit without bus I/O on contention. Keep genuinely passive observation explicitly separate from active diagnostics.
- **Acceptance:** Hold the owner lock with a fake runtime and invoke every active bench entry point, including `--go`. Assert no open/send/reconfiguration occurs. With the lock free, preserve normal bench behavior and guaranteed release after exceptions.

### R34 — Passing test output can conceal a failed thread or skipped collection

**Locations:** `tests/test_canworker.py:37`; `tests/run_all.py:50`; ROS workspace test invocation/configuration.

- **Failure:** The arm/deadlock test checks that a thread stopped, but does not propagate its exception or assert the intended successful arm result. During this review it passed despite an SDO timeout exception in that thread. Separately, the ordinary ROS pytest invocation exited with no intended tests collected; disabling auto-loaded plugins recovered the 204-test suite. The precise plugin conflict was not isolated in this review.
- **Change:** Capture worker results/exceptions and make unexpected thread failure fail the test. Make successful-arm and failed-arm/no-deadlock scenarios separate explicit cases. Provide a reproducible offline test entry point with required plugin selection, clear simulation exclusions and a nonempty collection assertion. Add the browser/action/failure-injection regressions in this plan instead of relying mainly on source-text checks.
- **Acceptance:** An intentionally failing arm thread must fail the suite. A documented command in the supported environment must collect and run all intended offline packages, report skipped integration suites, and return failure on empty/partial unintended collection.

## Implementation sequence

Keep changes reviewable by failure boundary. Do not combine all findings into a broad refactor before the regressions exist.

| Batch | Findings | Concrete output | Dependencies / completion gate |
|---|---|---|---|
| 0 — Trustworthy verification | R29, R34 | Fixture-owned process signals; deterministic offline runner; thread-exception capture; reusable deferred-response/fake-bus helpers | Required before full simulation/fault injection |
| 1 — Command release and input bounds | R01, R02, R03, R16 | Browser pending-input state; explicit revocation protocol; finite-value boundaries | End-to-end release/reorder tests and nonfinite-input tests pass |
| 2 — Drive stop and feedback | R04, R05, R06, R07 | Transactional drive lifecycle; verified stopping; count continuity; explicit freshness/validity contracts | Fake-bus failure matrix first, then controlled bench evidence |
| 3 — Mission authority and lifecycle | R08, R09, R13, R17, R18, R20 | Goal-attempt tracking; effective limits; epoch-bound jobs; Reset contract; correct turn/resume/repeat state | Uses batches 1–2; exercise output commands, not only FSM labels |
| 4 — Geometry and localization | R10, R11, R12, R19 | Conservative bounds; common coordinate transforms; fresh localization proof; map-bound UI actions | Bad-map and stale-evidence cases fail closed; valid maps still work |
| 5 — Data integrity and schemas | R21, R22, R23, R24, R30, R31, R32 | Strict parsers; contained evidence; immutable storage; safe rendering; bounded logging | Concurrent writers, malformed input and disk-failure tests pass |
| 6 — Supervisor and supported launches | R25, R26, R27, R28 | Terminal operation handling; recoverable base/layer lifecycle; deadlines; coherent deployment paths | No pending-operation leaks; all advertised entry points smoke-tested |
| 7 — Retained legacy/bench paths | R14, R15, R33 | Panel-loss inhibition; bounded CAN scheduling; utility ownership | Required before using legacy fallback or active bench tools |

Batches identify dependencies, not permission to keep using a known-broken affected path until its batch is reached. R14/R15/R33 should move earlier if the legacy controller or bench utilities remain the current operating path.

### Cross-cutting design constraints for implementation

- Keep one CAN owner and one Modbus owner. Solve scheduling inside that ownership model; avoid adding competing stop/diagnostic clients.
- Carry identity through command and async boundaries: supervisor instance, generation, session/run, step/pass and attempt where relevant. Revocation is terminal for that identity.
- Separate packet receipt from usable evidence. Define source timestamp, receipt time, validity, freshness and reset semantics explicitly in interfaces and tests.
- Make physical stopping and cleanup observable: requested zero, delivered zero, measured standstill, de-energization and guard retirement are different states.
- Bound queues, blocking calls, parsing/allocation work, log buffers and async operations. Test deadline violations with fake clocks or controlled failures.
- Preserve published map/route references and hashes. Introduce explicit schema/version migration if message fields or stored formats change; document compatibility with the rollback release.
- Update `amr_ws/README.md`, `amr_ws/RUNBOOK.md`, relevant interface definitions, tests and service/launch composition in the same revision as each public behavior change.

## Acceptance plan

| Layer | Required checks | Evidence to retain |
|---|---|---|
| Pure logic | Nonfinite input, wrapped counts, schema limits, geometry bounds/yaw, readiness proof, route passes | Regression test cases tied to finding IDs |
| Browser/API | Deferred press/release; revoked sessions; stale map requests; literal rendering; structured errors | Deterministic request/output assertions, no vehicle calls |
| CAN simulation/fakes | Every arm/stop write failure; independent TPDO loss; heartbeat retained/lost; teardown interruption | Command order, timestamps, state transitions, per-node results |
| ROS contract/integration | Late action callbacks, stale generations, missing valid wheels, stop/resume, mode changes | Published commands/permits and operation terminal states |
| Filesystem | Concurrent writes, duplicate mission IDs, truncated maps, disk full, evidence containment | Original/committed bytes and hashes, bounded completion time |
| Isolated full simulation | Boot → IDLE; jog; survey/save; activate/localize; run/pause/resume/repeat; fault/recover | Owned PID inventory, state/command trace, no unrelated process termination |
| Controlled vehicle bench | Encoder signs/scale and reset; PC-loss guard timing; stop confirmation; feedback loss; physical panel semantics | Measured timing and actual drive status, with the existing runbook's witnessed procedure |
| Deployment/rollback | Each supported unit/launch mode, compatible environment, single ownership after exit/recovery | Version/configuration identity and operator workflow result |

For timing-related findings, agree the intended timeout/stop budget from the existing controller configuration before implementation, then assert that budget. Do not silently enlarge watchdogs to make a test pass. Software-command latency and measured mechanical stopping behavior must be recorded separately.

### Remaining verification limits

- These findings do not establish physical stopping distance, drive firmware behavior, scanner coverage, mechanical limits, or functional-safety conformity. Those require the existing hardware acceptance process.
- Unit success does not validate the eight skipped simulation suites. Run them only after R29's isolation fix and with the corrected collection setup.
- The signed-wrap, cancellation, parser, configuration and browser probes exercised real source functions with controlled inputs; they were not physical reproductions.
- Rotated-grid, concurrent-store, lifecycle and commissioning findings need the specified regression tests to validate the full proposed fixes. Source inspection identifies the paths but does not measure their field frequency.
- Review of RFID, lidar decoding, IMU helpers, URDF and supporting utilities did not produce an additional confirmed finding beyond the issues above; that is not proof of defect absence.
- User-owned working-tree changes were left untouched. Only this planning document is added by the review.
