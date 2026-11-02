# Codebase review and coding plan

Review date: 2026-09-21  
Repository baseline: `e6eb4cc`  
Scope: entire repository; prioritize defects found in code.  
Deliverable: implementation plan only. Application code and configuration were not changed.

## 1. Assessment

The main architecture is worth retaining: `agv_core` provides a ROS-independent vehicle library; `amr_base` owns hardware and final command arbitration; the supervisor replaces mode layers using generations; mission and localization logic have substantial offline test coverage. The CAN write guard, owner locks, command expiry, immutable route store, and explicit shutdown paths are useful foundations.

The highest-priority repairs are in state transitions and data boundaries:

- LINE can resume after a brief selector interruption without a new Start.
- A torque-loss condition arriving during an existing automatic LINE hold does not upgrade that hold to require Start.
- Concurrent map edits can delete or mix another request's staging files.
- Short IMU frames can become apparently valid zero-rate samples.

The passing suites do not cover these scenarios. Fix the behavior and add regression cases before extending tape missions or raising speed limits. Keep the ported `autopilot.py` and `branch.py` control law unchanged for these repairs; the defects identified here belong in the surrounding acquisition, authority, persistence, and UI layers.

### Evidence labels and priority

- **Reproduced:** exercised the current implementation using pure logic, fake inputs, Flask's test client, or temporary files. This is not vehicle acceptance.
- **Code-confirmed:** the control/data path establishes the defect or missing behavior; no full end-to-end reproduction was performed.
- **Investigation:** a plausible concern that requires additional evidence before treating it as a defect.
- **P1:** repair first because the issue changes motion recovery, control evidence, or stored data integrity.
- **P2:** functional correctness, recovery, or missing integration; schedule after the first repair batch.
- **P3:** maintenance and verification hygiene.

## 2. Verification performed

| Check | Result | Interpretation |
|---|---|---|
| `python3 tests/run_all.py` | 61 functions, 531 checks passed | Core library offline baseline |
| `env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q amr_ws/src/*/test` | 671 passed in 30.63 s | Package tests; opt-in launch workflows disabled |
| `ruff check agv_core tests` | Failed: 24 findings | Existing baseline, not introduced by this review |
| `ruff check src` from `amr_ws` | Passed | Workspace lint baseline |
| `bash amr_ws/deploy/validate.sh` | Passed | Emitted host warnings about `netplan-ovs-cleanup.service` access and snapd's `RestartMode` key |
| Targeted defect probes | Results recorded below | No CAN, Modbus, running-service mutation, or robot motion |

Pytest also emitted `Unknown config option: pythonpath`. The suite still passed in this configured shell; this does not establish clean-environment import behavior.

Not performed: a fresh colcon build, unified launch simulation, browser interaction with a running robot, cable-pull tests, or floor testing. Existing vehicle reports are historical context, not new acceptance evidence. No external standards or certification assessment is included.

### Review coverage

| Area | Review focus |
|---|---|
| `agv_core`, profile, core tests | Configuration boundary, owner locks, CAN guard and monitoring, DIO/RFID recovery, library invariants |
| `amr_base` | Lease intake, mux, drive watchdog and arm/disarm paths, TPDO feedback, MLS acquisition, commissioning interfaces |
| `amr_bringup`, deployment | Lease generation, readiness, layer transitions, process-group cleanup, service wrapper and validator |
| `amr_line` | Node/engine boundary, Start/Reset, holds, track rate, speed cap, product integration |
| `amr_mission`, `amr_navigation`, `amr_maps` | Route admission/execution, survey authority, bundle identity, concurrent revisions, route/mission storage |
| `amr_localization` | Readiness, scan and covariance evidence, torque-loss exceptions, transform freshness |
| `amr_web` | API contracts, adapter state, LINE availability, jog lifecycle, map-edit integration |
| `amr_sim`, description, interfaces | Existing test coverage, message/enum contracts, simulated wheel behavior and launch boundaries |

This is a risk-focused repository review, not a claim that every line or hardware state was exhaustively verified.

## 3. Prioritized repair register

| ID | Priority | Finding | Evidence |
|---|---|---|---|
| F01 | P1 | Brief LINE authority loss does not terminate the running job | Reproduced |
| F02 | P1 | New torque-loss conditions do not upgrade existing automatic LINE holds | Reproduced |
| F03 | P1 | Concurrent map revisions share and delete the same staging directory | Reproduced staging interference; concurrent HTTP outcome inferred |
| F04 | P1 | Truncated IMU frames can publish fabricated zero yaw rate | Reproduced |
| F05 | P2 | Physical Reset is ignored while LINE is ARMED | Reproduced |
| F06 | P2 | LINE track-rate profile setting is silently ignored | Reproduced |
| F07 | P2 | Documented independent LINE speed cap is absent from the mux | Reproduced |
| F08 | P2 | MLS absent during startup cannot recover without restarting the owner | Reproduced for track; code-confirmed for IMU |
| F09 | P2 | Protective-field status has no freshness/unknown contract | Code-confirmed |
| F10 | P2 | Store readers accept escaping paths and inconsistent map identities | Reproduced |
| F11 | P2 | LINE cannot be operated through the current web workflow | Reproduced endpoints; code-confirmed adapter gap |
| F12 | P2 | Several HTTP endpoints crash on valid JSON of the wrong shape | Reproduced |
| F13 | P3 | Documented core lint check fails; pytest configuration is not fully honored | Observed tooling output |

### F01 — End LINE authority immediately when the panel or lease permission changes

**Locations:** `amr_ws/src/amr_line/amr_line/job.py:138`, `:192`, `:248`; `amr_ws/src/amr_base/amr_base/gating.py:268` onward.

**Current behavior:** `FollowJob.tick()` immediately faults when the `(instance, generation)` binding changes, but selector state, panel validity, and the LINE permission bit enter the generic prerequisite debounce. The job continues as RUNNING until that prerequisite has failed continuously for 0.5 s. The mux independently stops motion during MANUAL, but the job remains running and continues publishing commands.

**Reproduction:** arm and start; present `panel_auto=False` at time 102.0; restore valid AUTO at 102.2 without Start. The job remains `RUNNING`, reason `running`. With the same fresh LINE lease, the mux can accept commands again. This is an unintended restart opportunity after a brief selector excursion, not a failure of the mux's instantaneous MANUAL gate.

**Coding plan:**

1. Separate authority failures from recoverable sensor prerequisites in `FollowJob`.
2. On selector leaving AUTO, invalid/stale panel, or removal of `LEASE_LINE`, stop the job on the first observed failure and invalidate its armed/start authorization.
3. Use an explicit IDLE or non-automatic authority state with a stable reason; select one lifecycle policy and test it. Recommended: require re-arm plus a new Start after manual takeover.
4. Preserve the existing immediate generation-change fault. Do not use the track debounce to authorize restart.
5. Verify the node emits a zero on the falling edge and the mux cannot reselect an old LINE command after authority returns.

**Acceptance:** AUTO→MANUAL→AUTO within one debounce window, a short panel outage, and permission removal/restoration in the same generation must never restart a job without the newly required operator action. Test both job state and mux output.

### F02 — Escalate hold causes while already held

**Locations:** `amr_ws/src/amr_line/amr_line/job.py:218`, `:245`, `:256`.

**Current behavior:** torque/field classification only runs in RUNNING. In HOLD, `_hold_tick()` keeps the original `hold_cause`; a new failure only resets the clear timer.

**Reproduction:** start a job; set `drives_fresh=False` to enter automatic `HOLD(drives)`; next provide a fresh `torque_off=True, field_clear=True` input. By the existing classifier this is an E-stop/safety-chain condition, but the cause stays `drives` and `auto_resume()` stays true. Restore prerequisites for more than 2 s: state becomes RUNNING without Start.

**Coding plan:**

1. Evaluate new stop causes in both RUNNING and HOLD.
2. Make a condition requiring human acknowledgment dominate an automatic hold. Once latched, a later clear field or restored feedback must not downgrade it.
3. Track when the manual-resume requirement was established; require a Start edge after that point with current prerequisites satisfied.
4. Reset clear timers and pending Start evidence on escalation.
5. Review route executor transitions against the same precedence table, but retain its existing pending-feedback classification window. Do not assume LINE and routes already behave identically.

**Acceptance:** sequence tests for `drives→torque-off`, `rate→torque-off`, and `track→torque-off`; normal protective-field recovery still auto-resumes when sufficiently supported by fresh evidence. A physically observed E-stop/field combination needs a vehicle test because the current software infers cause from reports rather than receiving an explicit E-stop input.

### F03 — Make map revision allocation and publication a transaction

**Locations:** `amr_ws/src/amr_mission/amr_mission/map_bundle.py:103`, `:257`, `:351`; `amr_ws/src/amr_web/amr_web/server.py:313`; `amr_ws/src/amr_web/amr_web/web_node.py` (`threaded=True`).

**Current behavior:** `next_revision()` is unlocked. Staging is named `.staging-<revision>-<pid>`, and `staging_dir()` deletes an existing directory of that name. Two requests in the same Flask process choosing the same revision therefore share a path; the second can erase the first request's work. `publish()` checks existence separately from rename and does not provide a complete transaction/durability protocol.

**Reproduction:** create a stage, write an `inflight` marker, then request the same map/revision stage again in the same process. Both returned paths are identical and the marker is gone. Concurrent edit requests can consequently fail or interleave their files before verification/publication; the full HTTP interleaving was not executed here.

**Coding plan:**

1. Add a per-map interprocess lock that also excludes threads, following the useful `amr_navigation.store._locked()` pattern.
2. Allocate the final revision under that lock. Give each operation a unique temporary directory independent of PID alone.
3. Never delete another operation's stage. Cleanup must own a specific operation directory.
4. Verify the complete stage and commit without replacing a published revision. Coordinate survey saves, derived revisions, and other writers under the same protocol.
5. Flush files and relevant directories before returning committed success; rename atomicity alone is not crash durability.
6. Turn allocation/publication conflicts and filesystem failures into explicit operation/API outcomes. Preserve the parent revision on every failure.

**Acceptance:** use barriers to run two edits of the same parent concurrently, both from threads and processes. Each success must identify a unique immutable revision containing that request's exact edits and matching hashes. Inject write, verify, and publish failures; no success may refer to an incomplete revision, and one operation's cleanup must not remove another's data.

### F04 — Reject malformed IMU PDOs before publishing freshness

**Locations:** `amr_ws/src/amr_base/amr_base/mls_imu.py:52`, `:173`; `amr_ws/src/amr_base/amr_base/canopen.py:253`.

**Current behavior:** `decode_mapped()` pads a short frame to eight bytes. Missing mapped fields become zero. `_on_tpdo()` can then increment the sample count and publish an `ImuSample` with a fresh receipt time. The router also suppresses decoder exceptions without diagnostic accounting.

**Reproduction:** use a valid mapping containing the 16-bit gyro-Z object and call `_on_tpdo()` with `data=b''`. One sample is published with yaw rate `0.0`. Repeated short frames in a mapping without a stamp can keep refreshing this fabricated measurement.

**Coding plan:**

1. Validate mapping widths, total payload size, required gyro field width, and any timestamp width when acquiring the mapping.
2. Before decoding, require enough actual bytes for every mapped bit; reject malformed frames rather than zero-padding missing data.
3. Count rejected payloads and expose the latest reason through diagnostics without crashing the CAN owner.
4. Do not refresh sample time, sample count, or stamp-tracker state for rejected data. Let downstream IMU freshness expire normally.

**Acceptance:** every truncated length below the mapping requirement is rejected; empty frames never publish; valid signed gyro values and stamp wrap behavior remain unchanged. Include malformed mapping and repeated malformed-frame cases.

### F05 — Make Reset cancel an armed LINE job

**Location:** `amr_ws/src/amr_line/amr_line/job.py:198`.

**Current behavior:** Reset handles RUNNING, HOLD, DONE, and FAULT, but excludes ARMED.

**Reproduction:** arm, press Reset, then press Start without re-arming. State becomes RUNNING.

**Coding plan:** include ARMED in the Reset cancellation contract; clear the binding, accepted time, pending recovery state, distance, and stale diagnostics. Give Reset precedence if Reset and Start are observed together.

**Acceptance:** after Reset from every non-IDLE state, Start alone cannot run the follower. Re-arm plus a subsequent fresh Start still works. This should be a small patch alongside F01, not a control-law refactor.

### F06 — Connect LINE rate configuration to its consumer

**Locations:** `agv_core/config.py:420`; `amr_ws/src/amr_line/amr_line/runtime.py` (`_FROM_PROFILE`); `amr_ws/src/amr_line/amr_line/line_follow_node.py:122`.

**Current behavior:** the profile exposes `LINE_MIN_TRACK_HZ`, but `runtime.load_from_profile()` does not copy it. The node reads `getattr(runtime, "LINE_MIN_TRACK_HZ", 40.0)` and always uses the fallback. The current profile is also 40.0, which hides the defect.

**Reproduction:** after loading runtime from the profile, `hasattr(runtime, "LINE_MIN_TRACK_HZ")` is false and the node's expression returns 40.0.

**Coding plan:** read node/acquisition configuration directly from `agv_core.config`, or introduce an explicit typed node configuration separate from the engine namespace. Remove the silent fallback for required settings. Audit `AUTO_RESUME_HOLD_S` versus the node's fixed resume default and document precedence for profile values and ROS overrides.

**Acceptance:** supply a non-default rate threshold and prove that arming/running decisions use it. The displayed effective value must match the active value. Review the startup behavior where `TrackReader.hz()` is not yet known: decide explicitly how much rate evidence is required before arming, instead of treating unknown rate as measured-good.

### F07 — Enforce the LINE ceiling at the mux too

**Locations:** `amr_ws/src/amr_base/amr_base/gating.py:324`; `amr_ws/src/amr_base/amr_base/cmd_mux_kinematics_node.py` (`_tick`); `amr_ws/src/amr_bringup/launch/line_layer.launch.py:46`; `amr_ws/src/amr_line/amr_line/job.py` (`_cap`).

**Current behavior:** the follower caps its body speed, but the mux's LINE branch forwards `line.v`/`line.w` directly. Its later wheel ceiling is the motor limit, not the Increment 1 LINE limit. Comments in both the follower and launch file incorrectly claim an independent mux ceiling.

**Reproduction:** with fresh AUTO, drive, and LINE lease inputs, `gating.select()` accepts a LINE command of 0.8 m/s unchanged.

**Coding plan:** provide an explicit LINE limit at the final arbitration boundary. Preserve curvature when limiting a body command and define any yaw/wheel limit separately. Check the final slew output too, including entry from a faster source; a target clamp alone may leave the output temporarily above the new ceiling. Validate overrides as finite positive values within the intended product limit.

**Acceptance:** over-limit, reverse, and curved inputs stay within the selected limit; source changes cannot carry a faster slew state into LINE. Ordinary current-profile output remains unchanged. This is a missing independent constraint; it does not show that the current follower normally emits 0.8 m/s.

### F08 — Recover MLS discovery after a transient startup miss

**Locations:** `amr_ws/src/amr_base/amr_base/mls_track.py:114`, `:200`; `amr_ws/src/amr_base/amr_base/mls_imu.py:143`, `:184`; `amr_ws/src/amr_base/amr_base/drive_node.py` (`_run`, `_loop`).

**Current behavior:** a missing identification/configuration SDO at startup sets acquisition to `off`. Track polling does not rediscover an off sensor; IMU polling operates only in SDO mode. The owner calls initialization once. The existing track recovery handles a previously configured TPDO stream disappearing, not a sensor absent at startup.

**Reproduction:** start `MlsTrack` with a fake link that returns no objects; restore all objects; call `poll()` later. Mode stays `off`, with zero samples.

**Coding plan:** distinguish intentionally disabled from temporarily unavailable. Add bounded discovery retries on the existing bus-owner thread. Reacquire variant, mapping, and COB-ID before publishing recovered data; reset sample/rate/stamp evidence and avoid duplicate or obsolete router registrations. Spread discovery work across ticks or otherwise budget it so multiple SDO timeouts do not starve wheel commands and PC heartbeats.

**Acceptance:** sensor absent at boot then present; one failed startup read; sensor reboot with changed mapping; prolonged absence with bounded bus occupancy. A configured `off` mode must remain off. Existing NMT recovery must still address only the MLS node.

### F09 — Give protective-field evidence an age and unknown state

**Locations:** `amr_ws/src/amr_line/amr_line/line_follow_node.py:136`, `:200`, `_inputs`; `amr_ws/src/amr_mission/amr_mission/route_executor_node.py:376`, `:387`, `:981`.

**Current behavior:** LINE initializes the field as clear, and neither LINE nor the executor records a receipt timestamp for the current output-path sample. The executor's `_field_trip_t` records a trip event, not ongoing message freshness. A last clear value can remain clear indefinitely; a last violated value can prevent recovery indefinitely. This affects stop classification and resumption even though the hardware safety chain independently removes torque.

**Coding plan:** represent clear, violated, and unknown/stale evidence explicitly. Validate the configured output index and record local monotonic receipt time. For real operation, missing/stale evidence must not qualify an automatic resume or prove that a stop was only a field interruption. Provide an explicit simulation contract rather than implicitly assuming clear when a driver dependency is absent. Coordinate the cause-precedence policy with F02 and the route executor's classification window.

**Acceptance:** no first sample, stream loss after clear, stream loss after violation, malformed status arrays, recovery of the stream, and field/drive messages arriving in either order. Specify when manual Start is required and ensure it remains required once latched.

### F10 — Validate store identity and containment at every read boundary

**Locations:** `amr_ws/src/amr_navigation/amr_navigation/store.py:110`, `:135`, `:194`; `amr_ws/src/amr_mission/amr_mission/map_bundle.py:364`; `amr_ws/src/amr_bringup/amr_bringup/readiness.py` (`resolve_map`).

**Current behavior:** write-side route/mission names receive some validation, but store readers build paths from caller-supplied identifiers without equivalent checks. `map_bundle.load()` verifies bundle contents but does not require the manifest's map ID and revision to equal the requested directory identity. The supervisor checks containment on its own path; that does not cover all web/store callers.

**Reproductions:**

- With a temporary `maps/missions` directory and a YAML file beside `maps`, `load_mission(maps, "../../outside")` loads that external YAML.
- Publish a valid bundle under `requested/rev1` whose manifest says `line_section`, revision 99. `load(maps, "requested", 1)` returns the inconsistent manifest successfully.

These are read-boundary and identity failures. This review does not claim arbitrary file writing or automatic motion from the path traversal alone; mission execution has additional validation.

**Coding plan:**

1. Centralize plain-name, positive-revision, and canonical-root checks in the lowest shared store layer.
2. Apply checks to reads, lists, writes, derivation, and JSON-supplied references; reject absolute paths, traversal, NULs, and escaping symlinks.
3. Compare manifest ID/revision to the requested identity before returning a bundle or launching navigation.
4. Parse mission/manifest schemas explicitly. Normalize expected YAML/schema/filesystem failures into `StoreError`/`BundleError`, preserving useful diagnostic context.
5. Keep existing route/map hash checks. Validate route identity against its requested store path too.

**Acceptance:** direct library calls and API requests refuse escaping IDs and symlinks; mismatched manifests/routes fail before admission; corrupt or scalar YAML produces a bounded refusal rather than a node exception. Valid historical bundles retain their existing hashes.

### F11 — Complete the LINE operator workflow

**Locations:** `amr_ws/src/amr_web/amr_web/server.py:630`; `amr_ws/src/amr_web/amr_web/adapter.py:460`; `amr_ws/src/amr_line/amr_line/line_follow_node.py` (arm/clear services); `amr_ws/src/amr_bringup/amr_bringup/supervisor_node.py` (`_auto_enter_line`, `_allowed`).

**Current behavior:** the web API accepts only `idle` and `navigation` mode targets. The adapter names LINE but has no LineState subscription or line arm/clear clients. There is no LINE state in `/api/state`. A tracked vehicle can boot into LINE, but arming and clearing require ROS tooling; after leaving LINE through the UI, the UI cannot request re-entry.

**Reproduction:** `/api/mode` with `target: line` returns 400; `/api/line/arm` returns 404. This is a confirmed feature-integration gap, consistent with the repository's Increment 1 status, not a claim that a previously shipped LINE page regressed.

**Coding plan:**

1. Expose LINE mode through the existing asynchronous supervisor request/poll contract.
2. Add generation-aware LineState caching with age/stale fields and clear it on layer replacement.
3. Add arm/clear adapter calls and API endpoints. Arm must only prepare the job; motion still requires physical Start in AUTO.
4. Provide one operator surface for mode, arm, clear, hold cause, recovery requirement, track quality/rate, and effective cap.
5. Reflect the tracked-product capability in navigation and page controls; retain backend refusal for incompatible requests.
6. Update the runbook with the complete enter→arm→Start→hold→Reset/clear→leave sequence.

**Acceptance:** a tracked and untracked profile can reach each supported workflow without a terminal. No HTTP action substitutes for Start. Stale or wrong-generation LineState cannot appear current. Mode exit remains blocked until the active job is cleared according to the supervisor contract.

### F12 — Standardize HTTP payload and failure contracts

**Locations:** `amr_ws/src/amr_web/amr_web/server.py:630`, `:645`, `:692`, `:719`, `:787`, and other `request.get_json(...).get(...)` consumers.

**Current behavior:** several routes assume JSON is a dictionary. `request.get_json(force=True) or {}` does not reject a truthy list, string, or number. The map-edit endpoint already provides a useful explicit object check.

**Reproduction:** POST `[1]` as valid JSON to `/api/mode`: HTTP 500 rather than a structured client error, before any adapter call.

**Coding plan:** add one object-payload parser and consistent JSON errors. Validate field types before coercion, including rejecting booleans/fractions for revision fields and handling numeric overflow. Use 400 for payload shape/type, 422 for domain validation, 409 for state/conflict, and 503 for unavailable dependencies where appropriate. Normalize persistence failures from F03/F10; do not hide unexpected programming errors as successful operations.

**Acceptance:** table-driven endpoint tests for object/list/string/number/null, malformed JSON, missing fields, booleans, nonfinite numbers, and extreme numeric values. Invalid requests must invoke no adapter mutation. Preserve the 202-plus-operation-ID contract for accepted asynchronous work.

### F13 — Restore a trustworthy verification baseline

**Locations:** `pyproject.toml`; `agv_core`; `tests`; test/run instructions in both READMEs.

**Observed:** core Ruff reports 24 findings: 6 UP024, 4 B905, 3 F541, 3 E501, 2 B904, 2 UP041, and one each E741/E731/B007/F841. Workspace Ruff passes. Pytest warns that the root `pythonpath` option is unknown in the installed environment.

**Coding plan:** fix lint findings in a separate maintenance commit, inspecting behavior-sensitive changes such as `zip(strict=...)` rather than applying unsafe fixes blindly. Establish the supported pytest/dev-tool versions or remove reliance on unsupported configuration through an explicit test runner/environment. Add one documented offline verification entry point covering core and workspace tests. Preserve the core runner's deliberate check-count accounting when tests are added.

**Acceptance:** both documented Ruff commands pass; test invocation emits no unknown-option warning and succeeds from a clean supported shell. Runtime defect fixes should remain reviewable separately from formatting changes.

## 4. Follow-up investigations and feature repairs

These are not additional confirmed production failures. Keep them separate from the repair register until reproduced or specified.

| Item | Evidence / question | Next bounded task |
|---|---|---|
| Monotonic watchdog clocks | Mux `_now()` uses the ROS clock; `gating._fresh()` accepts negative ages, while drive intake uses monotonic receipt time | Inject backward/forward clock changes and stopped simulation time. Ensure the mux cannot keep republishing an old command as fresh to the drive; use steady time for local authority expiry and ROS time for headers/TF |
| Lease ordering consistency | LINE rejects lower generations within an instance; mux and drive only reject non-increasing sequence within the same generation | Replay old-generation messages and retired supervisor identities using node callback tests. Establish one explicit intake policy; normal single-publisher DDS ordering alone does not prove all restart/replay cases |
| Track transport age | `TrackReader.age_s()` adds decode-to-publish age and elapsed time since local receipt; it does not measure publish-to-receive delay | Inject transport backlog and bursts, then decide whether source-stamp/sequence evidence is required. Test duplicate and out-of-order samples and unknown startup rate |
| Sensor loss versus tape end | `FollowJob._run()` maps both `line_lost` and `sensor_lost` to DONE with a tape-loss message | Split normal end-of-tape completion from sensor communication failure in the surrounding job, preserving the engine. Specify whether recovery is retry, re-arm, or fault |
| Stop-command semantics | LINE sends explicit zero, but the mux slews a fresh LINE zero until it reaches zero or expires; this differs from losing authority | Measure job-stop→wheel-zero behavior in simulation. If the intended contract is immediate command revocation, add an explicit permit/revocation mechanism instead of depending on a zero Twist |
| Survey authority identity | Survey moves stamp each command using the current lease rather than an immutable move binding; mapping intake keeps only state | Test lease replacement and delayed old MappingState during transition. Bind active moves to their admitted generation if the test shows a gap; retain the supervisor's stop/kill barriers |
| LINE reported distance | `followed_m` integrates engine-requested speed before the job's cap and before mux/drive effects | Define whether this is commanded or measured distance. Rename accordingly or integrate valid measured feedback; reset diagnostics between jobs |
| Packaging and deployment reproducibility | `agv_core` deliberately leaves runtime dependencies empty; ROS and device libraries come from the vehicle image | Capture a tested vehicle-image dependency inventory and import smoke check. Keep offline core tests independent of hardware libraries; do not add arbitrary version pins without validating the installed image |
| RFID/stations/branches | README explicitly identifies these as later LINE increments; existing RFID library is not a complete tape-mission integration | After F01–F11, implement a bounded station-event/mission layer with deduplication, arrival/departure semantics, lost-reader behavior, branch selection, and test traces. Preserve current straight-tape behavior |
| Existing commissioning limits | README records unverified high-speed fields, encoder scaling/backlash, and no floor acceptance for LINE | Carry forward the existing commissioning tasks and collect new evidence. Do not turn these known limitations into an automatic speed increase or claim they were resolved by this review |

Existing documented choices—such as provisional dynamic areas, the relaxed route corridor margin, and the absence of web authentication—are not silently redefined by this plan. Any separate change to those product policies needs its own scoped design and acceptance criteria.

## 5. Implementation sequence

| Batch | Work | Dependency / completion gate |
|---|---|---|
| A — LINE authority | F01, F02, F05; define F09 evidence semantics | New sequence regressions reproduce old failures and pass with fixes; no control-law edits |
| B — sensor evidence | F04, F06, F08, F09 | Reject malformed data, recover discovery without starving the bus, use effective configuration |
| C — final arbitration | F07; clock/lease investigations if reproduced | Node-level mux tests prove bounded outputs and expired authority; retain drive's independent watchdog |
| D — persistent data | F03, F10 | Concurrent writers, rollback, identity, and containment tests pass |
| E — operator integration | F11, F12 | Terminal-free LINE workflow; structured refusal paths; physical Start preserved |
| F — tooling and handoff | F13, README/runbook updates | Supported verification entry point, recorded results, and explicit remaining vehicle checks |

Treat each batch as reviewable changes, not one broad refactor. Data-store work is independent of motion repairs and can be implemented separately. UI work must target the repaired job lifecycle rather than cementing the current restart behavior.

### Suggested regression placement

- Extend `amr_line/test/test_line_authority.py` for selector, Reset, and hold escalation sequences.
- Add node-boundary LINE tests for consumed profile settings, field freshness, generation behavior, and falling-edge output; pure job tests alone cannot catch wiring mistakes.
- Extend `amr_line/test/test_line_mux.py` and relevant `amr_base` node tests for speed limits and source transitions.
- Extend `amr_base/test/test_mls_imu.py` and `test_mls_track.py` for malformed frames and startup recovery, using a fake clock and instrumented bus calls.
- Extend `amr_mission/test/test_map_bundle.py` for concurrent edits and identity mismatch; use thread/process barriers rather than sleep-dependent races.
- Extend navigation store tests for read containment/schema failures and web tests for malformed payloads and LINE endpoints.
- Extend the existing JS harness for LINE UI stale-state/refusal handling where practical; validate the completed workflow in a browser as well.

## 6. Integration and vehicle acceptance plan

1. Run focused regressions for each batch, then core and package offline suites once the combined changes settle. Run both Ruff commands.
2. Build changed ROS packages and re-source the overlay when messages, services, package entries, or launch contracts change. Confirm tests import the intended source/install versions.
3. Run the existing unified simulation workflow sequentially on an isolated non-vehicle ROS domain. Add one LINE workflow that exercises entry, arm, physical Start, short selector change, hold escalation, Reset, and exit. Do not launch all historical full-stack suites together on the N97.
4. For storage, retain hashes of parent revisions and assert they remain unchanged after every injected failure and concurrent write. Restart the reader after publication; do not validate only against its in-memory cache.
5. For sensor recovery, use a test bus/fake link first to measure worst-case scheduling delay with failed SDO requests. Then perform the already-established on-blocks vehicle procedure for delayed MLS availability and recovery.
6. On the vehicle, witness the panel/hold sequences with timestamped LINE, mux, drive, and field evidence. Verify zero command on authority loss and no autonomous restart after Reset/manual takeover or a newly latched manual-resume condition. Start with the existing speed limits.
7. Record the deployed commit/profile, test results, observed stop/recovery timing, and remaining limitations in `manuals/vehicle-reports/`. Only then conduct the outstanding LINE floor run under the repository's existing acceptance procedure.

### Definition of done

- Each confirmed finding has a focused regression and a passing repair, or an explicit documented disposition.
- No repair bypasses the CAN write guard, ownership locks, generation separation, command expiry, or physical Start requirement for LINE/autonomous jobs.
- Failed or stale evidence cannot be mistaken for fresh permission to resume.
- Successful save responses identify complete, verified, immutable data produced by that operation.
- The web workflow describes the actual node state and explains required recovery actions.
- Offline, simulation, and vehicle evidence are recorded separately; passing unit tests are not presented as floor acceptance.
