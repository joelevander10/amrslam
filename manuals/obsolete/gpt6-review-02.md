# Codebase review 02 — findings and implementation plan

Date: 2026-09-17. Baseline: `96c7a8f` (`amcl`), including the working-tree changes present during this review. Previous review: [gpt6-review.md](manuals/obsolete/gpt6-review.md), based on `55d400a`.

This is a planning deliverable. No application code was changed by this review. Findings concern software behavior; offline results do not establish vehicle stopping performance.

## Assessment

The intervening changes address many first-review defects: nonfinite commands, encoder rollover, incomplete feedback pairs, ordinary jog release races, action-attempt correlation, immutable revision writes, malformed PGM termination, commissioning evidence, and test collection all received substantive fixes.

The principal remaining gaps are interactions between those fixes: unknown drive state is still treated too optimistically; action cancellation can lose its unresolved-goal barrier; route limits and clearance checks do not consistently constrain actual motion; localization and the HMI can preserve apparently fresh state after its underlying evidence stops updating.

This review identifies **21 actionable findings: 10 P1 and 11 P2**. P1 means a control, stopping, or navigation correctness defect that should be resolved before relying on the affected operating path. P2 means a recovery, integrity, interface, or conditional configuration defect. Severity is not a claim that a failure was observed on the vehicle.

## Scope and verification

Reviewed the ROS base/CAN driver, command arbitration, panel and commissioning paths; localization, scan gating and scan consistency; route compilation, geometry, storage and mission execution; mapping, process supervision and launch/deployment paths; web API, adapter and browser workflows; retained legacy controller, configuration, logging and active CAN tools; and relevant regression tests and operating documentation. The review combined current-source inspection, comparison with the original baseline, existing tests, and targeted failure probes. It is not a proof that every possible defect has been found.

The existing uncommitted web changes were included: `adapter.py`, `jog.py`, `server.py`, `web_node.py`, `static/amr.css`, `static/amr.js`, `static/jogpad.js`, `templates/base.html`, `test/test_server.py`, and the new `wifi.py`. Two further edits appeared during review and were inspected: `amr_navigation/config/nav2_params.yaml` changed obstacle-layer input from `/scan` to `/scan_gated`, and `amr_mission/amr_mission/route_executor_node.py` increased turn-travel tolerance from 2 to 4 degrees. The first makes scan-gate availability relevant to obstacle-layer updates as well as localization; the second belongs in the permitted-motion envelope analysis in Q05. Final ROS verification was repeated after these edits.

| Verification | Result | Limits |
|---|---|---|
| Root: `python3 tests/run_all.py` | **86 test functions; 851 checks passed** | Offline legacy tests |
| `amr_ws`: `env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test` | **367 passed in 27.00 s** on final run | Repeated after the late source/config edits; simulation modules excluded by collection configuration |
| `amr_ws`: `ruff check src` | **2 failures** | I001 at `amr_base/test/test_pendant_gating.py:3`; E501 at `amr_web/amr_web/wifi.py:45` |
| `./amr_ws/deploy/validate.sh` | Passed | Host warnings included netplan permissions and the host snapd unit's `RestartMode`; no service activation |
| Additional Python failure probes | Confirmed drive-stop, cleanup reporting, command-age, revocation, readiness, cancellation, clearance, scan-gate, map-integrity, commissioning-admission and live-pose-age defects | Pure helpers or mocked node dependencies; no physical CAN activity |
| Additional JavaScript probes against actual browser source | Confirmed focus-related jog continuation and network-error polling failures | Stub DOM, timers and fetch; not a physical tablet/browser test |

The additional probes were temporary files outside the repository. Their essential setup and observed outcomes are recorded below so implementation agents can turn them into maintained regression tests. Existing-suite success does not cover these missing contracts; one fault-stop test explicitly expects the problematic stale-feedback behavior.

No live CAN commands, motion, service restarts, interface changes, full navigation simulation, or vehicle fault injection were performed. Source anchors refer to this reviewed working tree and may move as implementation proceeds.

## Finding index

| ID | Priority | Finding | Evidence |
|---|---|---|---|
| Q01 | P1 | Fault-stop confirmation ignores missing or stale status | Reproduced; loop traced |
| Q02 | P1 | Incomplete disarm reports success and does not enforce cleanup before rearm | Reproduced; teardown traced |
| Q03 | P1 | Drive command watchdog uses time captured before blocking work | Reproduced |
| Q04 | P1 | Authored route speed limits still do not cap motion output | Source traced; old R09 remains |
| Q05 | P1 | Allowed tracking error exceeds the validated clearance margin | Geometric counterexample reproduced |
| Q06 | P1 | Cancellation timeout forgets a potentially live Nav2 goal | Reproduced; recovery traced |
| Q07 | P1 | READY localization does not expire scan-consistency evidence | Reproduced |
| Q08 | P1 | Obstruction checks project an old scan using the latest transform | Source traced |
| Q09 | P1 | Keyboard jog continues when focus enters a text field; Escape is ignored there | Actual JavaScript reproduced |
| Q10 | P2 | Global manual stop does not revoke the active session identity | Intake reordering reproduced |
| Q11 | P2 | Network rejection stops live polling and strands operation controls | Actual JavaScript reproduced |
| Q12 | P2 | Live pose freshness measures cache reads, not transform age | Mocked adapter reproduced |
| Q13 | P2 | Scan gate releases expired scans and fails across clock rewind | Reproduced |
| Q14 | P2 | A verified map bundle can load image bytes outside its integrity hash | Reproduced |
| Q15 | P2 | Editor async responses and displayed settings are not bound to the current draft | Source traced |
| Q16 | P2 | Commissioning plan admission cannot establish supervisor IDLE | Reproduced |
| Q17 | P2 | Recovery still cannot rebuild a failed base process group | Source traced; old R26 remains |
| Q18 | P2 | Advertised standalone/rollback web workflows lack required supervisor authority | Source traced; old R28 remains |
| Q19 | P2 | Physical Reset is published without a ROS control consumer | Repository search; old R17 remains |
| Q20 | P2 | Custom footprint override does not address the costmap's footprint parameter | Source and launch parameter normalization |
| Q21 | P1 | Legacy periodic SDO polling can still delay control processing for seconds | Source-derived timeout budget |

## Drive lifecycle and motion authority

### Q01 — Require positive evidence that each drive stopped

**Location:** `amr_ws/src/amr_base/amr_base/canopen.py:678` (`fault_tick`); `drive_node.py:243` (`_loop`, heartbeat fallback around line 300). Previous R04: partially resolved.

Independent zero sends and retries are now implemented. However, `fault_tick()` only reports a non-stopped drive when its status is fresh. Missing/stale status produces no unresolved-stop condition. Consequently the loop can continue the PC heartbeat while stop confirmation is absent. The assumption that a drive which cannot be heard must also be unable to hear the controller is invalid for selective TPDO failure or asymmetric communication loss. A successful socket send also does not confirm reception or stopping.

**Reproduction:** fault an armed link, make both status samples pre-fault/stale with nonzero recorded RPM, and advance four seconds. Result: `stop_unconfirmed == []`, `heartbeat_withheld == False`.

**Implementation:** track delivered stop request and fresh, post-fault standstill evidence separately for each node. Missing confirmation must remain unresolved. After a bounded deadline, enter the existing documented fallback policy and expose the reason. Preserve independent attempts to stop both drives.

**Acceptance:** exercise failed send, successful send without feedback, one silent TPDO stream with continuing heartbeats, stale speed-zero status, fresh nonzero status and fresh post-fault zero status. Update `test_r04_fault_stop.py`, whose stale-status case currently asserts no escalation. Bench-check the intended 1016h response separately; do not infer its physical behavior from a mock.

### Q02 — Make disarm completion truthful and enforce outstanding cleanup

**Location:** `canopen.py:602` (`disarm`), `canopen.py:829` (`decide`), `drive_node.py:350` (`_serve_requests`). Previous R05: partially resolved.

`disarm()` now returns a success flag and retains `cleanup_owed`/failure details, but callers ignore the flag. The service reports `disarmed` even when cleanup failed; fault acknowledgement also reports success. Software state becomes DISARMED before hardware cleanup completes. Normal policy does not retry cleanup just because it is owed, and outstanding cleanup does not establish a rearm barrier.

The sequence also clears the PC-loss guard after only a best-effort zero and before confirmed standstill/disable. The source documents a real 8130h issue with a previous ordering. That observation must inform the fix; simply reversing the order without validating heartbeat servicing is not a complete solution.

**Reproduction:** a link whose `disarm()` returns `False` produces service response `{ok: true, msg: "disarmed"}`. DISARMED with no arm request leads to decision `none`, so cleanup is not automatically retried.

**Implementation:** represent incomplete teardown explicitly, keep nonzero commands inhibited, return failure/pending status with node-level reasons, and schedule bounded cleanup retries. Block rearm until a defined hardware-state condition is established. Develop a teardown sequence that preserves a usable stop fallback while maintaining required heartbeats during blocking transactions.

**Acceptance:** inject failure at every zero/guard/controlword/NMT stage, including only one node failing. Assert truthful service results, no automatic rearm through unresolved cleanup, and eventual retry or explicit latched failure. Verify normal and interrupted teardown on the bench, including the previously observed 8130h behavior.

### Q03 — Evaluate command age at the point of transmission

**Location:** `drive_node.py:243` (`_loop`), `drive_node.py:374` (`_target`).

The loop captures `now` before servicing requests and potentially executing blocking arming/SDO work. It later passes that old timestamp into the command watchdog. `_target()` uses a fresh clock for the supervisor lease but the caller's old clock for command age, so a fresh lease can coexist with an incorrectly accepted expired command.

**Reproduction:** command timestamp 10.0 s, watchdog 0.2 s, loop timestamp 10.0 s, actual clock 11.0 s. `_target()` returns `(57, 57)` RPM for the test scale; the same command evaluated at the current clock returns `(0, 0)`.

**Implementation:** sample monotonic time immediately before validating and sending a target; reject invalid/negative ages. Invalidate pre-transition commands across arm/ack/authority changes as appropriate. Bound request work so it cannot indefinitely defer control and heartbeat processing.

**Acceptance:** advance a fake clock during arm/service operations, keep the lease fresh, and assert the actual transmitted target is zero until a new eligible command arrives. Include commands received during the blocking interval and authority changes during it.

### Q10 — Give global stop durable session revocation semantics

**Location:** `amr_ws/src/amr_base/amr_base/gating.py:100` (`ManualIntake.offer`). Previous R02: same-session release fixed; global-stop case remains.

An empty-session zero clears the current command but does not tombstone its session. A later refresh for that old session can become current again.

**Reproduction:** accept session `s`, sequence 1, velocity 0.2; accept empty-session zero; deliver session `s`, sequence 2. Current velocity becomes 0.2 again.

This proves an intake-protocol gap, not that the current single-publisher ordered DDS path necessarily produces that ordering. Retain the already-fixed server locking and ordinary release behavior.

**Implementation:** global stop revokes the current session within its authority identity; retain bounded revocation history for the relevant message lifetime. A subsequent jog requires a new session. Document ordering guarantees and explicitly handle multiple producers if supported.

**Acceptance:** exercise global stop followed by delayed refresh, old/new authority identities, repeated stops, fresh sessions and bounded tombstone retention. Test through the mux boundary, not only the HTTP state object.

### Q16 — Admit commissioning plans using explicit supervisor mode

**Location:** `amr_ws/src/amr_base/amr_base/commissioning.py:94` (`Job.plan`); `amr_ws/src/amr_bringup/amr_bringup/supervisor_node.py:235` (`_allowed`). Previous R13: epoch and encoder binding addressed; admission remains incomplete.

Plan admission checks fresh authority, standstill and encoder scale, then rejects only the AUTONOMOUS permission bit. A fresh inhibited lease with `allowed=0` still passes; MAPPING and IDLE can both have manual permission. Therefore the check cannot establish its stated “commissioning plans need IDLE” condition.

**Reproduction:** otherwise valid plans with `lease_allowed=0` and `lease_allowed=1` both enter PREPARED. This does not itself authorize movement, but admits jobs in inappropriate operational contexts and can complicate subsequent transitions.

**Implementation:** admit through the supervisor or consume a fresh, identity-matched mode snapshot. Require the intended IDLE state explicitly, plus the documented panel/standstill conditions. Preserve epoch, encoder-scale and fresh-Start checks already added.

**Acceptance:** reject FAULT, TRANSITIONING, MAPPING, NAVIGATION and stale/mismatched state; admit IDLE only under the documented conditions. Confirm admission never moves the vehicle and old physical Start edges remain unusable.

### Q19 — Define and implement physical Reset behavior

**Location:** `amr_ws/src/amr_base/amr_base/panel_node.py:113`; panel callbacks in the mux, commissioning and route-executor nodes. Previous R17: open.

`reset_edge` is published but no ROS control consumer acts on it. This differs from retained legacy behavior and leaves the operator's physical Reset without a defined control effect.

**Implementation:** specify which software faults/aborted jobs Reset may acknowledge, required authority and standstill, and whether navigation must be re-localized. Route the edge through one owner and ensure it cannot bypass Q01/Q02/Q06 recovery barriers. Keep fault acknowledgement separate from issuing Start or motion permission.

**Acceptance:** fresh, repeated and stale Reset edges in IDLE, active motion, blocked/aborted mission and hardware fault; no reset-triggered motion, implicit resume or clearing of unresolved hardware state.

### Q21 — Bound legacy telemetry work, not just queued diagnostics

**Location:** root `canworker.py:818`, `canworker.py:1522` (`_poll_monitor`), `canworker.py:1538` (`_poll_telemetry`); `drivers/canbus/verify_drivers.py:27`. Previous R15: queue work fixed; periodic polling remains.

Queue draining is now bounded and inappropriate queued actions are rejected. Periodic telemetry still performs three blocking SDO reads for each of two drives on the control worker. The default no-response timeout is 0.4 s per read: six timeouts can consume roughly 2.4 s before other loop work, with monitoring adding up to two further reads. `fast=True` removes the collision window; it does not shorten the no-response timeout.

**Implementation:** schedule bounded transactions across ticks or use a nonblocking transaction state machine with a single receive owner. Make watchdog/zero/panel handling higher priority than diagnostics. Stop scheduling optional reads when a node is stale and expose deadline overruns.

**Acceptance:** use a fake bus with missing replies during motion; measure maximum delay to command expiry, panel loss and explicit stop handling. Establish and test a total loop deadline rather than merely counting dequeued actions. Prioritize this work immediately if the legacy controller remains the active or rollback operating path.

## Navigation, geometry and localization

### Q04 — Enforce route limits on executed commands

**Location:** `amr_ws/src/amr_mission/amr_mission/route_executor_node.py:549`, `:569`, `:795`; `amr_ws/src/amr_navigation/config/nav2_params.yaml:30`, `:109`; command mux and motion-permit interface. Previous R09: open.

The route model stores linear/angular limits, but FollowPath/Spin dispatch and motion permission do not carry or enforce those limits. Nav2 retains configured speeds, including 0.30 m/s desired linear velocity and 0.30 rad/s maximum rotational velocity. A route authored with a lower speed does not establish that lower bound on output. Compiler timing estimates and editor settings are insufficient enforcement.

**Implementation:** bind effective limits to the loaded immutable route and current motion authority, enforce them at the final software arbitration point, and configure the controller consistently. Define linear/angular and wheel-limit interaction so clipping does not unexpectedly alter path curvature. Remove misleading configuration comments and operator instructions until they match actual behavior.

**Acceptance:** load a route capped at 0.10 or 0.15 m/s, feed a higher Nav2 command, and inspect emitted wheel commands. Test spin caps, lease expiry, replacement routes, missing limits and attempt/generation transitions. Do not accept a test that only checks a stored field or predicted duration.

### Q05 — Make permitted tracking motion fit the validated envelope

**Location:** `amr_ws/src/amr_description/config/footprint.yaml`; `route_executor_node.py:695`; `amr_ws/src/amr_navigation/amr_navigation/validate.py` and `footprint.py`.

The configured clearance margin is now 0.10 m. Execution allows twice the normal 0.10 m cross-track limit during the first `converge_m` (default 1 m). Thus it can permit 0.20 m lateral deviation outside the nominal validated margin. The nearby claim that the grace band remains inside that margin is no longer true. Start/heading tolerances and corrective rotations also need to fit the same geometric contract.

**Reproduction:** on a 0.05 m grid, validate the configured footprint along `(2, 2)` to `(3, 2)`, with occupied cells at row 49, columns 40–89. Nominal validation passes. Shift the actual, uninflated footprint laterally by 0.15 m: its sweep overlaps **42 occupied cells**, while the executor's first-metre allowance is 0.20 m.

**Implementation:** establish one explicit relationship between footprint, margin, start offset, cross-track/heading tolerance and correction motion. Reject incompatible settings or validate the full permitted motion envelope. Account for actual displaced pose in dynamic obstruction checks. The 0.10 m margin was deliberately selected in the repository; do not silently restore the old value as the whole fix.

**Acceptance:** preserve this counterexample; add near-wall starts, entry correction, heading error, turn recovery, both sides of the route and map boundaries. Every permitted recovery/control trajectory must fit the envelope being asserted clear. Record physical footprint and tracking measurements separately from software geometry tests.

### Q06 — Preserve the unresolved-action barrier after cancellation timeout

**Location:** `amr_ws/src/amr_mission/amr_mission/goal_attempts.py:86`, `:104`, `:127`; `route_executor_node.py:653`. Previous R08: normal callback correlation fixed; unknown outcomes remain.

Per-attempt tokens correctly reject obsolete feedback/results and cancel late acceptance. However, the executor calls `forget_obsolete()` after the cancellation deadline, removing outstanding attempts without terminal confirmation, then faults. After acknowledgement and a new run, the old goal is no longer a barrier even though its action server may still execute on the same generation's command topic. Goal-request/result communication exceptions also remove outstanding state without necessarily establishing server-side termination; result-future setup can raise outside the existing handler.

**Reproduction:** send a goal whose acceptance future stays pending, revoke it, then forget obsolete attempts. Outstanding count becomes zero while the request remains pending.

**Implementation:** distinguish terminal, rejected and unknown outcomes. A timeout can fault the run but must retain the unresolved-motion barrier. Permit replacement only after terminal evidence or verified shutdown/replacement of the owned action server and command authority. Catch result-subscription/cancellation failures without treating them as successful cancellation. Operator acknowledgement alone must not erase the barrier.

**Acceptance:** deferred acceptance beyond timeout, ignored cancellation, cancellation exception, result transport failure, result-future setup failure, fault acknowledgement and immediate restart. Assert no new motion permit for the same unresolved controller authority. Preserve the existing successful stale-callback isolation tests.

### Q07 — Expire localization evidence while READY

**Location:** `amr_ws/src/amr_localization/amr_localization/readiness.py:163`; `localization_monitor_node.py` scan/AMCL callbacks. Previous R11: initial confirmation substantially improved.

CHECKING now requires recent scan-consistency evidence. READY returns after raw stream/TF age and covariance checks without checking `scan_match_t` expiry. A raw scanner can keep publishing while gated comparisons stop because of a gate/TF failure, leaving READY intact. `amcl_fresh` is also calculated but unused in READY; covariance retained from an old update can continue to appear acceptable.

**Reproduction:** provide good covariance/scan match, settle and confirm at 2.1 s, then evaluate at 100 s with raw stream ages zero and no new scan comparison. State remains READY.

**Implementation:** define the ongoing evidence needed to retain READY and explicitly expire it. Distinguish an AMCL filter legitimately not updating while stationary from loss of independent scan verification; do not impose an arbitrary update policy that breaks valid stationary operation. Use source timestamps/seed epochs to reject delayed pre-seed evidence, and validate covariance values as finite and nonnegative.

**Acceptance:** raw scans continue but `/scan_gated` stops, scan-transform lookup repeatedly fails, scan verification stops, invalid covariance arrives, old AMCL samples arrive after seeding, and a genuinely stationary healthy filter remains valid under the chosen policy. Evidence loss must revoke autonomous permission through the existing LOST path.

### Q08 — Transform obstruction scans at their acquisition timestamp

**Location:** `route_executor_node.py:506` (`_obstruction`).

The obstruction check calls `lookup_transform("map", scan.header.frame_id, Time())`, selecting the latest transform rather than the scan timestamp. During movement/rotation or delayed scan delivery, it projects measurements from one pose using another. A corridor hit can move outside the checked mask or a clear corridor can appear blocked. The check also lacks its own acquisition-age bound; freshness elsewhere is not a substitute for timestamp-correct projection.

**Implementation:** use the scan timestamp with a bounded, nonblocking availability strategy. Defer/inhibit when required transforms are unavailable, or consume appropriately gated scans with explicit age checks. Keep raw scan time, arrival time and projection time distinct. Coordinate this with Q05 and the costmap's new `/scan_gated` subscription.

**Acceptance:** delayed scans while translating and rotating, out-of-order scans, missing transforms and clock discontinuities. Construct a hit that is inside the corridor at acquisition but outside under the latest pose and verify it still blocks. Test that old data cannot establish a newly clear corridor.

### Q13 — Enforce scan-gate deadlines independently of TF availability

**Location:** `amr_ws/src/amr_localization/amr_localization/scan_gate.py:49`; `scan_gate_node.py`.

The gate checks residence expiry only when the transform is unavailable. If its timer stalls and the transform is ready when polling resumes, an already-expired scan is released. `_last_out` also survives backward source-clock jumps: new scans are thinned until timestamps catch up with the old value. Since the late Nav2 edit also sends obstacle-layer input through this gate, failure affects another consumer.

**Reproduction:** with `hold_max_s=0.5`, queue a scan at time 1 and poll at time 2 with TF available: it is released. Release stamp 100, rewind to stamp 1 and poll again: the new scan is thinned.

**Implementation:** check an independent monotonic residence deadline before publication, detect source-clock resets and reset epoch-dependent state, and bound queue size. Validate gate parameters and use each queued scan's frame when testing transform availability. Update module documentation that still says the costmap consumes raw `/scan`.

**Acceptance:** starve polling beyond the deadline then make TF available; rewind simulation/bag time; deliver out-of-order scans without resetting a healthy epoch; overflow the queue; change scan frame. Assert bounded memory, bounded age and prompt recovery, and smoke-test all gated consumers.

### Q20 — Apply the custom footprint to the costmap node correctly

**Location:** `amr_ws/src/amr_bringup/launch/navigation_layer.launch.py:108`; `amr_ws/src/amr_navigation/config/nav2_params.yaml:65`.

The controller launch passes `{"local_costmap.local_costmap.footprint": footprint}` as an inline parameter dictionary. Launch parameter normalization preserves that as a literal dotted parameter name; it does not turn it into the costmap node's `footprint` parameter. The YAML separately supplies a hardcoded footprint. Editing the shared footprint file/custom launch argument can therefore update route validation without updating the Nav2 costmap as the comments promise. Defaults currently agree, so this is a conditional configuration defect.

**Implementation:** generate/rewrite node-scoped ROS parameter YAML or use the supported node-targeted mechanism for the actual costmap `footprint` parameter. Ensure executor validation, browser geometry and Nav2 consume the same selected configuration. Retain the useful single-source intent.

**Acceptance:** launch an isolated navigation instance with a visibly different custom polygon, query the actual costmap parameter and published footprint, and compare against the validator/browser selection. Source/parameter normalization was checked here; a running Nav2 parameter query was not performed.

## Map integrity and browser workflows

### Q09 — Release keyboard jog when input focus changes

**Location:** `amr_ws/src/amr_web/amr_web/static/jogpad.js`, keyboard handlers and held-input lifecycle.

The initial pending-press/release race is fixed. A different lifecycle gap remains: after holding a jog key, focus can move into INPUT/TEXTAREA/SELECT without releasing the existing key intent. Refresh timers keep sending nonzero jog commands. The keydown handler returns for text fields before handling Escape/Space, so Escape in that field does not release the active jog.

**Reproduction:** actual source in the repository's style of fake DOM/timer harness: hold W, focus an input, advance timers, press Escape in the input. Observed two additional refreshes and no Escape-triggered stop.

**Implementation:** handle stop keys before editable-target filtering and revoke held keyboard intent on entry into editable controls, including contenteditable. Apply the same revocation to an in-flight press; require deliberate fresh input before restart.

**Acceptance:** focus change during accepted and pending presses, keyboard/mouse/touch combinations, Escape inside all editable controls, window blur and hidden page. Assert no further refresh and eventual zero/release, including when the press response arrives late.

### Q11 — Recover browser polling and complete operation UI state on network errors

**Location:** `static/liveview.js:34` (`pollLive`); `static/amr.js:107` (`operation`, `followOperation`); `static/maps.js:4` (`op`), all under `amr_web/amr_web`.

The live poll schedules its next timeout only after awaited requests. A rejected fetch prevents rescheduling, retaining an old pose with its last small `age_s` indefinitely. Operation request/poll rejections escape without invoking the completion handler that releases mapping controls; the operation timeout path also needs explicit UI completion. A completed server operation can consequently leave the browser looking permanently busy.

**Reproduction:** reject a live-pose fetch after one successful sample: no retry timer is scheduled and the retained sample still reports age 0.01 s. Reject an operation request: the completion callback count remains zero.

**Implementation:** reschedule polling in `finally`, add bounded request timeouts/backoff, and advance displayed age using elapsed browser time since receipt. Clear or mark unavailable data appropriately. Retain operation IDs and resume status observation after reconnect without resubmitting a possibly executed command. Explicitly represent unknown outcome/timeout and always unwind the local busy state.

**Acceptance:** failure before operation submission response, failure during polling, server completes while disconnected, reconnect, page refresh and timeout. Live displays must become stale and recover automatically; mutation requests must not be blindly duplicated.

### Q12 — Preserve source transform age in live pose snapshots

**Location:** `amr_ws/src/amr_web/amr_web/adapter.py:224`, `:272`; `live.py:90`.

`_live_pose()` repeatedly reads the latest cached transform and `LiveStore.set_pose()` timestamps each read as new evidence. Although `_world_frame()` checks map-to-odom freshness, fallback odom-to-base data has no equivalent age validation. A frozen cached transform can therefore be advertised as a fresh pose on every web update.

**Reproduction:** a mocked lookup always returns a transform stamped zero. Calling the actual adapter method produces a reported pose age of approximately zero.

**Implementation:** carry source transform time and acquisition/receipt time separately; reject or mark stale according to the source data age, including composed/fallback transforms. Validate frame/authority context when selecting cached data. Preserve the recent fix that avoids destroying a TF listener while another executor thread uses it.

**Acceptance:** freeze odom-to-base while timers/web requests continue; freeze map-to-odom; replace a layer while cached TF remains; use valid fresh transforms. The HMI must not relabel old geometry as fresh merely because it was read again. Combine with Q11 to cover both server-side and browser-side aging.

### Q14 — Bind bundle integrity to every runtime-consumed asset

**Location:** `amr_ws/src/amr_mission/amr_mission/map_bundle.py:155` (`verify`); `amr_ws/src/amr_maps/amr_maps/grid.py` image loading.

Verification hashes the manifest's listed files and requires `map.pgm`/`map.yaml`, then loads the YAML. The YAML image path can point to an external, unlisted file. Thus a verified bundle hash can stay unchanged while the grid actually used by validation/navigation changes. Optional keepouts and operational metadata also need a defined integrity contract; merely hashing a file table does not bind arbitrary manifest metadata to that identity.

**Reproduction:** construct a valid bundle, set its hashed `map.yaml` image field to an external PGM and recompute its listed YAML hash. Modify only the external image from free to occupied. `verify()` still succeeds with the same bundle hash; grid loading now returns occupancy 100.

**Implementation:** enforce contained, normalized asset paths; require the actual resolved image and each consumed optional asset to be covered by the manifest; reject escape via absolute paths, traversal or symlinks. Validate manifest schema and geometry against the loaded grid. Define which metadata participates in identity, version the format if needed, and migrate existing references explicitly.

**Acceptance:** external image, traversal, symlink escape, unlisted keepout, changed operational metadata, malformed file table and metadata/grid mismatch. Changing any navigation-relevant content must fail verification or change the published immutable identity. Preserve valid existing bundles through a deliberate compatibility policy.

### Q15 — Bind editor responses and saved references to map/draft identity

**Location:** `amr_ws/src/amr_web/amr_web/static/editor.js`, validation/save/load handlers, `refresh()`, and map-change handler.

The map view's own image-load token does not protect the editor model. Map selection changes globals before awaited work; old route load/save/validation callbacks can later replace the current route, validation result or `savedRef`. `savedRef` contains only route ID/revision, while mission creation combines it with the current map globals. The speed selector is also not restored from `route.limits` in `refresh()`, so loading a route or changing map can leave the displayed speed inconsistent with the saved model.

**Implementation:** use map-selection and draft-version tokens across every asynchronous editor operation. Apply responses only to their originating context; bind saved references to full map identity and route revision. Synchronize all controls on load/undo/redo/reset and invalidate stale validation on edits. Keep “create mission from saved revision” semantics explicit rather than silently substituting an unsaved draft.

**Acceptance:** defer load/save/validate responses, switch maps or edit the route, then resolve responses in reverse order. Assert correct model, list, result and mission reference. Load a low-speed route after a higher-speed route and verify displayed, submitted and persisted limits agree.

## Supervision and supported deployment paths

### Q17 — Give base failures an honest recovery path

**Location:** `amr_ws/src/amr_bringup/amr_bringup/supervisor_node.py:392`, `:475`. Previous R26: open.

Recovery still initiates the normal inhibited transaction and stops/replaces the layer, not the failed base group. A missing mux prevents the required acknowledgement. Other incomplete-base cases can proceed without a complete new base-readiness barrier. The operator-facing recovery path therefore does not repair the failure category it appears to cover.

**Implementation:** distinguish recoverable layer failure from base failure. Either implement owned base teardown/restart plus full readiness before committing IDLE, or return an explicit restart-required outcome with accurate runbook instructions. Keep hardware motion inhibited throughout and coordinate teardown with Q01/Q02.

**Acceptance:** dead mux, dead drive node, partially started base, base boot exception and ordinary layer failure. Recovery must terminate truthfully, avoid orphan processes and never announce usable IDLE without its required base prerequisites.

### Q18 — Reconcile standalone launches with supervisor-dependent web controls

**Location:** `amr_ws/src/amr_bringup/launch/nav.launch.py`, `mapping.launch.py`, rollback deployment units, `README.md`, and `amr_web` manual/active-map admission.

Standalone wrappers and rollback instructions retain interactive web workflows, but those paths do not start the supervisor that supplies current manual authority and active-map context. The tightened web checks correctly require that context, so the advertised standalone operator workflow no longer works as described.

**Implementation:** select and document supported entry points. Provide supervisor-backed wrappers for supported interactive operation, or explicitly limit standalone paths to diagnostics and remove incompatible operator promises. A rollback must deploy a mutually compatible set of launcher, UI and authority components. Do not solve this by bypassing the authority checks.

**Acceptance:** smoke-test every advertised launch/rollback path for state display, manual admission, map selection, localization and operation completion where supported. Unsupported actions must fail with an actionable explanation.

## Disposition of the first review

“Addressed” means the original defect is addressed in the inspected source/offline scope. It does not assert hardware acceptance or that the surrounding subsystem is defect-free. New adjacent findings are distinguished from the original defect.

| Previous ID | Status | Current assessment |
|---|---|---|
| R01 | Addressed | Pending jog release handled; separate focus defect Q09 |
| R02 | Partial | Same-session release and server publication ordering fixed; global-stop contract Q10 |
| R03 | Addressed | Nonfinite wheel commands rejected |
| R04 | Partial | Per-drive stop attempts/retries added; missing confirmation Q01 |
| R05 | Partial | Broader arm rollback and cleanup bookkeeping added; Q02 |
| R06 | Addressed | Wrap-safe encoder continuity implemented; physical scale validation remains separate |
| R07 | Addressed | Required TPDO freshness, complete feedback pairs and validity gates added |
| R08 | Partial | Attempt identity/late callbacks fixed; unknown action outcomes Q06 |
| R09 | Open | Route output caps Q04 |
| R10 | Addressed | Off-map checks and dilation wrapping corrected; separate envelope mismatch Q05 |
| R11 | Partial | Fresh evidence required for confirmation; continuing READY evidence Q07 |
| R12 | Addressed for original path | Map-context seeding/overlay and image-load checks added; adjacent editor/TF issues Q12/Q15 |
| R13 | Partial | Epoch/scale binding and fresh Start enforced; IDLE admission Q16 |
| R14 | Addressed | Legacy panel-loss inhibition and fresh jog-release behavior added |
| R15 | Partial | Queue work bounded and inappropriate diagnostics rejected; periodic polling Q21 |
| R16 | Addressed | Finite configuration checks added |
| R17 | Open | Physical Reset consumer Q19 |
| R18 | Addressed for original state leak | Turn-state reset, coast accounting and resume checks added; cancellation recovery still Q06 |
| R19 | Addressed by restriction | Unsupported rotated origins rejected; keepout alignment checked |
| R20 | Addressed | Repeated-route execution implemented with bounded repeat count |
| R21 | Addressed for original cases | Structured numeric/schema checks and compilation bounds added |
| R22 | Addressed | Map/revision-aware default mission IDs and conflict handling added |
| R23 | Addressed | Locking, unique staging and exclusive revision publication added |
| R24 | Addressed | Truncated PGM parsing terminates with an error |
| R25 | Addressed for original exception paths | Active operation failures/no-op completion/boot errors handled |
| R26 | Open | Base-group recovery Q17 |
| R27 | Addressed for original cases | Survey deadlines, abort/error results and staging cleanup added |
| R28 | Open | Standalone/rollback workflow Q18 |
| R29 | Addressed in source | Fault injection targets owned processes; full simulation not run here |
| R30 | Addressed | Stored route values escaped in rendering |
| R31 | Addressed | Contained IDs, exclusive evidence files and aborted/cleared-run evidence added |
| R32 | Addressed | Logging buffer/retry bounded and cleanup protected |
| R33 | Addressed in inspected active CAN entry points | Shared bus ownership checks added |
| R34 | Addressed | Thread failures surfaced and simulation collection controlled; ordinary offline pytest now succeeds |

## Coding work packages

Each package should be a reviewable implementation change with the listed regression cases. Resolve interfaces before editing their producers and consumers independently.

| Order | Package | Findings | Required output / completion criterion |
|---|---|---|---|
| 1 | Stop/cleanup state and timing | Q01–Q03 | Explicit unresolved-stop/cleanup contract, truthful services, current-time command validation; fake-bus matrix passes before bench validation |
| 2 | Input release and browser continuity | Q09–Q12, Q15 | Focus-safe jog, durable revocation, recoverable polling and context-bound editor; deferred-response and network-loss JS tests |
| 3 | Mission execution contract | Q04, Q06, Q16, Q19 | Enforced route limits, persistent unresolved-action barrier, mode-correct commissioning admission and defined Reset behavior |
| 4 | Geometry and observation contract | Q05, Q07, Q08, Q13, Q20 | Consistent clearance/tracking envelope, ongoing localization proof, timestamp-correct scans, robust gate and actual costmap footprint verification |
| 5 | Immutable map identity | Q14 | Contained assets and versioned integrity/schema contract, including compatibility tests for existing maps/routes |
| 6 | Recovery and deployment | Q17, Q18 | Truthful base recovery or restart-required state; supported launch/rollback matrix tested and documented |
| 7 | Retained legacy scheduling | Q21 | Bounded control-loop latency under missing replies; move this package earlier if legacy is in service |
| Alongside relevant packages | Hygiene and documentation | Lint failures and stale comments | Fix import ordering/line length; align speed, clearance, scan-topic, freshness and recovery documentation with implemented behavior |

Cross-package constraints:

- Q02 and Q06 recovery barriers must survive operator acknowledgement; Q19 must not bypass either.
- Q04 limit changes must be carried atomically with mission authority and tested at emitted commands.
- Q05/Q20 must use the same selected geometry; runtime footprints must be queried rather than assumed from launch dictionaries.
- Q07/Q08/Q13 must agree on observation source time, residence deadlines, TF availability and clock-reset handling.
- Q11/Q12 must distinguish source age, backend receipt age and browser elapsed age.
- Preserve existing immutable route/map references. Any Q14 format change needs explicit migration and invalidation rules.
- Retain the regression fixes already made. In particular, do not reintroduce broad process killing, unbounded queue draining, old attempt callbacks, or TF listener destruction during concurrent callbacks.

## Acceptance and handoff

1. Convert the targeted reproductions above into maintained tests attached to the relevant package. Test observable commands, operation outcomes and persisted identities; avoid tests that simply repeat internal implementation choices.
2. Run the two existing offline suites and `ruff check src` after the corresponding changes. Require clean test-thread completion and explicitly report simulation exclusion. Re-run affected launch/config checks after the late `/scan_gated` change.
3. In an isolated ROS domain/process group, exercise delayed action acceptance, ignored cancellation, stale localization evidence, gate interruption, custom footprint parameters and base recovery. Verify the already-revised process ownership before fault injection.
4. On a controlled bench, validate per-node stop confirmation, heartbeat fallback and cleanup sequencing against the documented 8130h observation. Record actual timing and drive state, not only API success.
5. Validate route cap enforcement and the geometric envelope using measured vehicle footprint/tracking behavior. Include low-speed routes, entry correction, turns, obstruction and resume.
6. Exercise the supported tablet/browser workflow through focus changes, connection loss, map switches and delayed responses, then check the documented launch/rollback matrix.

For every completed package, record the finding IDs closed, tests added, commands/results, remaining hardware or simulation checks, and any interface/configuration migration. Close a finding only when its behavior is verified at the affected boundary; a green pre-existing suite alone is insufficient.
