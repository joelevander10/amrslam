# sol61 coding improvement and reliability audit plan

Audit date: 2026-10-02. Baseline: commit `033194c` (`evot-10`) **plus the existing working-tree changes** inspected during this review. Those changes were not reverted or modified. This document proposes follow-up work; it does not change vehicle behavior.

Deployment assumptions supplied by the owner:

- One AGV per controller IPC.
- Reliability takes priority; retain current behavior and settings by default.
- No reported failures or resource problems so far. Findings below are code-level failure opportunities, not a claim that the deployed vehicle has experienced them.
- Target continuous 24/7 operation, including cable faults, delayed devices, slow storage, repeated mode changes, and operator/browser activity.

## 1. Main assessment

The architecture already contains useful reliability controls: exclusive device ownership, persistent base versus replaceable operating layers, generation-bound authority, an independent drive command watchdog, feedback validity checks, latched faults, verified drive cleanup, bounded sensor retry backoff, process-group teardown, a systemd watchdog, immutable map revisions, and extensive offline tests. Keep these mechanisms and strengthen their failure paths.

The first implementation priorities should be:

1. Stop treating repeated publication of a cached Modbus image as proof that the physical panel input is fresh.
2. Recheck current commissioning authority immediately before starting a profile-position move after its blocking setup.
3. Bound CAN transaction work so diagnostics, sensor recovery, and services cannot monopolize command/stop processing.
4. Remove storage and expensive validation from critical state locks and lease-publication paths.
5. Make retention, recovery accounting, and optional monitoring policy explicit for long-running deployment.

CPU optimization should follow measurement. Blocking SDO timeouts are primarily a **latency** problem, while repeated scan projection, serialization, polling, process discovery, and report compression are CPU/resource candidates. Reducing average CPU alone does not fix delayed control or stale authority.

Priority definitions:

| Priority | Meaning |
|---|---|
| P0 | Motion-authority correctness: address before relying on the affected failure path in vehicle acceptance. |
| P1 | High-value 24/7 reliability: bounded execution, recovery, durability, and truthful health. |
| P2 | Efficiency and operational improvements; implement after profiling or as explicit opt-in features. |
| P3 | Maintainability and reproducible engineering checks. |

Evidence labels used below: **confirmed** means directly visible in the reviewed source; **reproduced offline** means a small pure-Python probe demonstrated the behavior without opening hardware; **validate on IPC** means the magnitude or deployment consequence needs runtime measurement. A source finding does not establish a measured incident rate.

## 2. Review coverage and verification baseline

### Production code reviewed

| Area | Principal source and configuration |
|---|---|
| Shared vehicle library | `agv_core/config.py`, `mission.py`, `blindrun.py`, `kinematics.py`, `canmon.py`, `disk.py`, device drivers and ownership/guard helpers. |
| Hardware/base control | `amr_ws/src/amr_base/amr_base/`: drive/CANopen, MLS IMU and track, panel/DIO integration, RFID, command mux/gating, odometry, commissioning and PP control. |
| Supervision/deployment | `amr_ws/src/amr_bringup/`: supervisor, state machine, readiness, operations, process groups, uptime, logs, notification helpers, launch layers, deployment scripts/unit/environment and runbook. |
| LINE execution | `amr_ws/src/amr_line/amr_line/`: control law, runtime profile adapter, job/mission engine, track/tape handling, route execution and authority. |
| Localization/navigation | `amr_localization`, `amr_navigation`: IMU bias, scan gating/consistency, localization readiness, route parsing/compiler/validation, Nav2/AMCL/EKF configuration. |
| Mapping/mission persistence | `amr_mission`, `amr_maps`: route executor, mapping session, map bundles, revision/store/validation paths. |
| Operator interface | `amr_web`: ROS adapter/spinner, Flask routes, jog authority, live scan/map rendering, events/reports/commissioning history, role guards, network probes and browser polling. |
| Supporting surface | Interfaces, robot description, simulation/fakes, offline tests, operational documentation and historical plans for context. |

This is a static software review with offline verification, not an electrical/hardware validation, live timing profile, or functional-safety assessment. Historical manuals are context; active source/launch/deployment files determine the findings. The deployed workspace is organized around ROS 2 Humble/Ubuntu 22.04; do not combine this reliability work with an incidental distro migration.

### Checks performed

| Check | Result and interpretation |
|---|---|
| `python3 tests/run_all.py` from repository root | 64 test functions, **559 checks passed**. |
| `python3 -m pytest -q amr_ws/src` from repository root | **991 passed**, approximately 48 seconds. ROS simulation suites requiring `AMR_SIM_TESTS=1` were not enabled. |
| `ruff check .` from repository root | **6 findings**: import ordering, line length, one lambda-assignment style issue. |
| `ruff check src/` from `amr_ws` | **12 findings**: import ordering, line length and typing import modernization. |
| `./deploy/validate.sh` from `amr_ws` | Deployment validation passed. Installed watchdog was 30 s; journal usage approximately 288 MB; state storage currently uses the root filesystem. Host unit verification also reported netplan/snapd warnings. These are host observations, not AGV source failures. |
| Offline freshness probes | A disconnected `DioLink` with a one-second-old successful image still reported `comms_ok=True`; two identical acquisition snapshots advanced panel debounce to valid; `_fresh(101, 100, 0.2)` returned true. |

No live CAN/Modbus commands, service restarts, motion tests, or deployment changes were made for this audit. Passing current tests does not cover prolonged storage stalls, real bus timing, power loss, or every stale-input scenario below.

## 3. Motion authority and control-path hardening

### R01 — Physical panel freshness and debounce must follow acquisitions [P0]

**Evidence: confirmed and reproduced offline.** `agv_core/drivers/dio.py:153` exposes `connected`, `rx_age_s`, and `scans`, but `_comms_ok()` at line 199 uses the last-success age against `DIO_SILENT_WARN_S`. The current profile allows two seconds. Disconnect does not immediately clear that successful-image age. `amr_ws/src/amr_base/amr_base/panel_io.py:96` consumes the cached image without using acquisition sequence/age. `panel_node.py:101` feeds it from a 50 Hz timer and publishes a newly stamped `PanelState` each time, while the physical DIO scan period is 50 ms (20 Hz).

Consequences:

- A functioning publisher can repeatedly make old physical inputs appear freshly received to the mux. Its 200 ms message timeout does not bound the physical image age.
- Debounce counts publisher ticks rather than distinct successful acquisitions.
- The configured 1.5 s coincidence hold becomes 30 adapter ticks at 50 Hz, approximately 0.6 s, because its scan count is calculated from the 20 Hz DIO period.

Implementation plan:

- Separate diagnostic silence-warning age from the maximum age allowed to authorize panel motion.
- Immediately invalidate authority on a confirmed connection loss; also expire it when the scan thread stalls without reporting a disconnect.
- Carry source acquisition sequence and age/validity through the adapter. Advance debounce/edge detection only on a distinct, successful physical sample. Repeated publication can continue at 50 Hz with truthful source validity.
- Measure coincidence hold using monotonic elapsed time with fresh source evidence, not a timer-rate assumption.
- Retain the existing reconnect baseline rule: a held Start/Reset must not become a new edge merely because communication recovered.

Acceptance: freeze the DIO acquisition worker while leaving the panel publisher alive; disconnect during a held pendant direction; replay one image repeatedly; reconnect with Start held; vary publisher and scan rates. Authority must expire within the agreed physical-source deadline, and coincidence duration must remain 1.5 s. Choose the deadline from measured IPC jitter and required response, not the two-second diagnostic warning.

### R02 — Recheck PP authority after setup, before the start edge [P0]

**Evidence: confirmed.** `amr_ws/src/amr_base/amr_base/drive_node.py:559` takes the PP request and checks the gate before calling blocking `link.pp_enter()`. After success it sends `CW_START` at line 598 without rereading current request/hold/lease/panel state. `canopen.py:795` performs configuration reads, mode writes/readback and target writes. PP is enabled in the current `profiles/agv-01.json`.

Trigger: the operator releases hold, the selector/lease changes, or a request is replaced while setup is in progress. The next loop can halt, but the initial start command has already been sent using the earlier decision.

Implementation plan:

- Bind preparation to immutable run identity, generation, and move specification; refreshing hold may update its freshness without silently replacing the prepared move.
- Keep Halt asserted during preparation.
- After setup, reread the latest authority, hold freshness, drive fault/feedback state, and prepared request identity immediately before sending the start edge.
- On revocation/replacement, send no start edge and perform bounded Halt/return-to-PV cleanup. An incomplete cleanup must retain the existing fault/cleanup-owed behavior.
- Make setup cancellable between transactions as part of R03, rather than relying only on the final recheck.

Acceptance: inject delayed SDO replies and revoke each authority input during setup. Assert **zero `CW_START` transmissions** for a revoked/replaced move; verify cleanup and explicit terminal reason. Cover both wheels, partial mode changes, and cleanup failure.

### R03 — Introduce an aggregate CAN work budget and nonblocking transactions [P1]

**Evidence: confirmed; timing impact requires IPC validation.** `drive_node.py:352` runs command/heartbeat, MLS polling and monitor work on one bus-owner loop. A diagnostic slot uses a 50 ms read timeout. `mls_imu.py:233` can read gyro and occasionally timestamp with separate 50 ms timeouts; `mls_track.py:226` can add another 50 ms read. If eligible slow polls coincide, their transaction waits can total roughly 200 ms against a nominal 20 ms loop. A completely failed gyro read returns early, so not every outage takes all four waits. Sensor rediscovery can call full startup reads with 400 ms defaults; NMT handling adds a 50 ms pump.

`canopen.py:442` defaults reads to 400 ms, and writes use the SDO helper's timeout. `keepalive()` runs before the blocking transaction, not continuously throughout it. Per-call bounds are useful but do not bound the entire control cycle, arming/setup, or teardown sequence.

Implementation plan:

- Keep exactly one CAN receive/transaction owner. Do not add competing SDO readers or detach an unconditional heartbeat producer.
- Represent SDO work as one in-flight transaction with explicit deadline, object/node identity and completion state. Continue routing PDO/heartbeat/EMCY frames while waiting.
- Prioritize stop/fault handling, target updates, required feedback and health-qualified PC heartbeat; admit optional diagnostic work only within the remaining cycle budget.
- Limit RX drain work by time and frame count so heavy unrelated traffic cannot indefinitely postpone policy evaluation.
- Spread discovery, arming and PP configuration across steps with an overall deadline and cancellation points. Distinguish SDO abort, timeout, transport error and unsupported object.
- Apply per-node/object retry backoff/circuit breaking. Existing MLS missing-sample backoff is a useful starting point; do not repeatedly rediscover a partly responding device in one long burst.
- Verify and bound transport send waits for the actual deployed CAN backend. Record oversubscription and skipped optional slots instead of silently catching up in bursts.

Acceptance: CAN saturation, absent MLS, intermittently responding MLS, delayed/aborted SDO, bus-off/removal and late replies. Measure maximum command gap, heartbeat gap and stop-command latency. Optional monitoring must not consume the critical budget. Preserve guard/denylist behavior and all existing fault-stop/heartbeat-withholding tests.

### R04 — Services must not block the drive executor or outlive their request [P1]

**Evidence: confirmed.** `drive_node.py:268` waits up to 15 s for the bus thread inside a service callback. The node uses a single-threaded spin path, so the wait also delays command/lease/panel callbacks and its bus-thread health timer. `_requests` is an unbounded queue, and `_serve_requests()` drains queued work on the bus thread.

Implementation plan:

- Separate admission from completion: bounded request queue, request ID/deadline, explicit pending/terminal result, and cancellation or expiry before unstarted work acts.
- Give stop/disarm precedence; coalesce only semantically equivalent requests and retain ordering for state transitions.
- Keep command/authority callbacks schedulable while services await results. Merely increasing executor thread count is insufficient without callback-group and shared-state review.
- Report a timeout as an unknown/pending outcome when the operation may still complete; do not encourage blind retries of motion-affecting requests.

Acceptance: delayed bus work plus simultaneous hold release, lease expiry and disarm; flood bounded service admission; expire a queued request before execution. Verify no late arming/start from an expired request and no executor starvation.

### R05 — Detect stalled workers and make teardown results truthful [P1]

**Evidence: confirmed.** Drive health checks use thread `is_alive()`, which does not detect a live but blocked owner. Some startup CAN/IMU work precedes the main cleanup `try/finally`. Drive stop joins for eight seconds while multi-transaction teardown can exceed that. The shutdown log says drives are de-energised even when cleanup was not confirmed.

Implementation plan:

- Track monotonic completion time of the last critical cycle, longest transaction and current phase. Distinguish process alive, worker progressing, feedback fresh, and stop confirmed.
- Extend lifecycle cleanup coverage from ownership acquisition through bus setup and shutdown; release resources on every failed startup path.
- Use a total shutdown budget with bounded steps, final worker status and confirmed/unconfirmed drive state. Avoid destroying ROS publishers while a surviving worker can still use them.
- Preserve the existing rule withholding PC heartbeat when fault-stop delivery/confirmation fails. A new watchdog must not certify health merely because a timer thread is still running.

Acceptance: deliberately stall a fake CAN send/receive, fail initialization at each resource boundary, and exceed teardown deadlines. Every case must terminate or escalate predictably and report actual cleanup evidence.

### R06 — Use steady time for physical freshness and authority [P1]

**Evidence: confirmed and reproduced offline.** `cmd_mux_kinematics_node.py:246` uses the ROS clock for receive ages. `gating.py:254` accepts `now - t <= limit`, including negative ages. Mapping, localization and mission execution also use ROS time for several watchdog/deadline decisions. A backwards clock jump can extend validity; paused simulated time can suspend timers. The drive's monotonic watchdog is useful, but a running mux can keep republishing an old selection as a new drive command after a wall-clock jump.

Implementation plan:

- Use monotonic/steady time for receipt freshness, operator holds, leases, execution budgets and physical progress watchdogs, including the timer source where needed.
- Keep ROS timestamps for TF/sensor synchronization. Do not subtract values from different clock domains.
- Explicitly handle negative/future ages and clock reset; invalidate cached authority and rebaseline edges instead of extending their lifetime.
- Give simulation an explicit clock policy so pause/reset behavior remains testable and intentional.

Acceptance: backwards/forwards wall-clock jumps, frozen/reset simulation clock, future stamps, duplicate messages and delayed delivery. Physical authority must expire by steady time even while ROS time is abnormal.

## 4. Supervision, mission execution and recovery

### R07 — Remove filesystem work from lease publication and supervisor state locks [P1]

**Evidence: confirmed.** `operations.py:110` flushes and `fsync`s journal writes synchronously. State changes write uptime records. `supervisor_node.py:1040` performs deferred work, disk checks and periodic log capping before the tick and lease publication. Slow storage can delay the 10 Hz loop and exceed the 300 ms lease window even when hardware is healthy.

Implementation plan:

- Move file I/O into a bounded, serialized writer/maintenance worker; publish immutable state snapshots using short locks.
- Specify which operation milestones need durable acknowledgement before reporting success, and which diagnostics may be dropped/coalesced. Queue acceptance is not proof of durability.
- Surface write errors, queue depth, oldest job age and maintenance stalls. A full queue must have an explicit policy.
- Keep lease renewal tied to successfully evaluated controller health. Do not add an independent unconditional lease/watchdog feeder that conceals a wedged state machine.

Acceptance: delay `fsync`/`statvfs`, fill the filesystem, make it read-only, and flood events. Lease/state evaluation should remain responsive; durable operations should return truthful pending/failure status.

### R08 — Admit mission work cheaply, validate outside the execution lock [P1]

**Evidence: confirmed.** `route_executor_node.py:468` holds its state lock while reading/hashing bundles, loading grids/masks and validating a route, before checking whether the FSM can accept the mission. `_send()` at line 877 waits up to two seconds for an action server; this path is reached from the locked execution tick.

Implementation plan:

- Reject obviously ineligible requests before expensive I/O.
- Capture immutable map/route identity and generation, validate in a bounded worker, then reacquire briefly and recheck state/identity before commit.
- Cache validation by exact content hash, footprint/config version and revision; never reuse results across changed masks or limits.
- Use asynchronous/nonblocking action-server readiness and bounded retry state. Abort/pause/authority handling must remain responsive during missing-server conditions.

Acceptance: submit a large/bad mission while another run is active, stall map storage, remove an action server, and change generation during validation. No pause/abort starvation, stale commit, or unintended mission replacement.

### R09 — Optional process recovery must clean old groups and reset crash accounting [P1]

**Evidence: confirmed.** `supervisor_node.py:993` removes a dead web/Foxglove group before respawning; unlike critical groups, it does not retain the old group for verified empty-group cleanup. A dead leader can leave descendants. Respawn attempts accumulate for the entire supervisor lifetime, so several widely separated crashes can exhaust the three-attempt allowance.

Implementation plan:

- Stop/verify the previous process group before replacement, including a leader-dead/descendants-live case; retain cleanup evidence on failure.
- Use a bounded rolling crash window or reset attempts after sustained healthy operation. Keep a circuit breaker for repeated immediate crashes.
- Give asynchronous worker results an operation/generation identity; enforce one admitted transition worker or explicitly track all workers instead of relying on a shared completion slot.
- Profile `/proc` group enumeration before optimizing it; safe cleanup is more important than avoiding a small scan. Never replace group ownership with process-name matching.

Acceptance: kill only the Foxglove launch leader while a child remains; leave a report subprocess behind a failed web process; inject occasional versus rapid crashes. No duplicate bridge/server owner, stale completion, or restart storm.

### R10 — Preserve unclean-stop evidence until cleanup is actually confirmed [P1]

**Evidence: confirmed.** `supervisor_node.py:1116` logs failed `Group.stop()` results but still calls `uptime.clear()` after the loop. The next boot can therefore classify a stop as clean despite a nonempty group. Uptime classification uses wall-time/boot-time comparisons, which should be reviewed for clock adjustments. Stored mission identity also needs updating when a mission changes within an already active mode, not only on mode transitions.

Implementation plan:

- Clear the dirty marker only after required groups and drive teardown reach the defined confirmed terminal state. Otherwise persist the reason for incomplete cleanup.
- Include Linux boot identity plus instance/generation and active job identity in recovery evidence; use wall time for human timestamps rather than as the sole reboot classifier.
- Preserve interrupted operations as evidence without replaying their side effects. Keep existing operator acknowledgement and fresh authority requirements for motion.
- Fault-inject at each map-save stage and each cleanup stage; retain the existing staging/immutable-revision approach.

Acceptance: SIGKILL/power-loss simulation during save and teardown, failed group cleanup, mission changes within NAV/LINE, and clock adjustment across restart. No partial map becomes a valid revision, and uncertain shutdown must not be labeled clean.

### R11 — Define device-loss and service restart behavior as an end-to-end contract [P1]

**Evidence: confirmed configuration; recovery outcome requires deployment validation.** `amr_ws/deploy/amr.service` has `BindsTo=...can0.device`, 30 s systemd watchdog, 55 s stop timeout, and a three-start/60 s limit. Scanner and RFID use launch respawn with a five-second delay. These mechanisms cover different failure types and should be exercised together.

Implementation plan:

- Document separate behavior for CAN bus-off, disappearance of the device, boot without the device, and return after removal. Verify whether return requires explicit service start; do not assume `BindsTo` alone restarts it.
- Preserve no-motion-until-valid-authority behavior after recovery; restoring device connectivity is not permission to resume a mission.
- Ensure total supervisor/group teardown fits the service timeout, including failed cleanup. Keep crash-loop protection rather than simply increasing restart limits.
- Test required-node hangs as well as exits. `required()` exit handlers cannot detect a live frozen node; consumer freshness/progress monitoring must cover the relevant dependency.
- For optional/respawned nodes, explain the degraded capability: missing RFID can permit plain line following but cannot satisfy tag-dependent mission transitions.

Acceptance: controlled boot/replug/bus-off/frozen-node scenarios with measured fault detection, stop confirmation, recovery time and operator actions. Use the existing runbook's controlled vehicle procedures.

## 5. Monitoring and CPU efficiency

### R12 — Diagnostics need measurement age, not publication age [P1]

**Evidence: confirmed.** `canopen.py:1000`'s `MonitorCursor` reads one object on one node per slot. Eleven objects across two nodes require 22 slots: approximately **5.5 s per sweep while still** at 250 ms/slot, or **22 s while moving** at one second/slot, before extra delays. This differs from the older faster-sweep commentary in `agv_core/canmon.py`. Monitor values do not carry individual acquisition timestamps. `amr_web/adapter.py:501` derives supply freshness from the diagnostic message receipt time, although the value may be much older.

Implementation plan:

- Store last successful measurement time, last attempt/result, and age per object/node. Expose `fresh`, `stale`, `disabled`, `unsupported`, and `unknown` separately.
- Preserve timestamp when republishing a cache; changing publication rate must not refresh measurement age.
- Define expected age by object priority and effective policy. Optionally schedule voltage/temperatures more often than low-value motion diagnostics, without increasing the aggregate CAN budget.
- Update documentation and parameter/status pages to show actual ROS scheduling rather than legacy polling assumptions.

Acceptance: pause polling while publishing diagnostics, fail one object only, and change policy during motion. Stale or disabled values must not appear current/healthy.

### R13 — Add explicit optional-monitoring policy [P2]

The current `monitor_enabled` ROS parameter is read into node state at startup. A runtime policy needs an implemented parameter callback/service and application at the CAN-owner boundary; an apparently successful `ros2 param set` alone is not evidence that it took effect.

Recommended opt-in presets:

| Preset | Intended behavior |
|---|---|
| `current` (default) | Preserve today's monitor settings and behavior. |
| `quiet` | Suspend optional drive-health SDO slots and inactive viewer work. Continue all authority, drive feedback, watchdog and fault handling. UI explicitly shows suspended measurements. |
| `maintenance` | Allow requested diagnostic objects within the same bounded bus budget, with a time-limited session. Never grant motion authority. |

Policy precedence must be explicit: active functional consumers override optional-work suppression; loss of a browser/session cannot leave a required stream disabled. Clear pending optional reads safely when policy changes, and retain timestamps/failure counters.

### R14 — Demand-driven live visualization and bounded scan projection [P2]

**Evidence: confirmed.** `amr_web/adapter.py:68,383` caps scan projection at 10 Hz but continues processing when no viewer needs it. The adapter also updates live pose periodically. `amr_web/live.py:48` uses `n // MAX_POINTS`; this is not a strict 400-point bound (1,152 beams gives a stride of two and 576 points). PNG creation is already lazy/cached, which should be retained.

Implementation plan:

- Add a short-lived viewer demand token/TTL for live pages and stop expensive projection when no viewer is active. Keep cheap receipt/liveness information available independently.
- Use ceiling-based subsampling and strict output size/beam limits. Cache by source sequence and frame identity.
- Make concurrent PNG generation single-flight per snapshot; cap map size and cache memory.
- When no scan consumer exists in IDLE/LINE, evaluate suspending unnecessary scan-gate polling/processing. Wake it and clear stale queued scans before a mapping/NAV layer becomes ready. Confirm TF-buffer/subscription costs with profiling first.

Acceptance: no browser, one viewer, multiple viewers, hidden tabs and rapidly changing maps/frames. Work should scale with active demand and remain bounded; newly activated consumers must receive fresh data.

### R15 — Reduce repeated event-buffer copies and browser polling [P2]

**Evidence: confirmed candidate; measure savings.** RFID snapshots copy a bounded event buffer at 50 Hz, and the LINE side processes snapshots at control cadence. Browser pages independently poll state, alarms, network information and live data; multiple tabs multiply this work. Existing Wi-Fi TTL and internet-probe caching already reduce external calls.

Implementation plan:

- Consume RFID events by acquisition watermark/sequence with bounded batches and explicit gap/reset handling. Separate retained diagnostic history from delivery of new mission events.
- Keep tag acquisition and tag-dependent mission handling active where needed; do not suppress a station/branch event because the HMI is closed.
- Consolidate/cap browser polling, pause nonessential hidden-tab requests, and reuse cached snapshots. Preserve the existing jog release on visibility loss and held-command TTL.
- Keep asynchronous network-status probes request-driven/cached; optionally disable internet reachability checks on isolated deployments.
- Only consider replacing polling with a push mechanism if measured fanout justifies its extra recovery complexity.

Acceptance: RFID bursts and sequence gaps, long mission runs, several tabs/clients, browser disconnect and visibility changes. No missed/duplicate station action or renewed stale jog hold.

### Toggle decision table

| Work | Can be disabled/reduced? | Conditions and protections |
|---|---|---|
| Drive health SDO diagnostics (voltage, temperatures, load, etc.) | Yes, opt-in | `canmon.py` explicitly describes these as diagnostic. Show stale/disabled state; retain PDO fault/status and mandatory watchdogs. Prefer budgeted polling/backoff over blanket suspension when load is acceptable. |
| Foxglove bridge | Yes, existing startup toggle | `AMR_FOXGLOVE=false` is already supported; current environment is true. Suitable for an explicitly selected production preset. Keep default unchanged until selected. |
| Browser scan projection, PNG/live rendering | Yes, demand-driven | Disable expensive viewer work with no active viewer; never disable localization input merely because the viewer is absent. |
| Internet reachability indicator | Yes, optional | Local operation does not need this display-only request. Wi-Fi state may remain useful; it is already cached. |
| MLS track fallback SDO probing outside LINE | Potentially | Need a consumer-aware policy and warm-up/readiness before entering LINE. MLS gyro shares the node; retain required IMU/TPDO handling and review shared NMT behavior. |
| Scan-gate work with no SLAM/AMCL consumer | Potentially | Verify activation ordering, TF availability and no stale queue on reactivation. Persistent required-node supervision must remain truthful. |
| Mapping/NAV layer in idle use | Already mode-managed | Switching to IDLE is the existing explicit layer-control mechanism. Selecting MANUAL inside NAV does not mean AMCL/costmaps/readiness are unnecessary. |
| DIO acquisition, pendant validity/edges, horn deadline | Keep | Source of manual authority and operator warning. Slow diagnostic image publication separately if necessary. |
| Drive target loop, wheel/status feedback, PC heartbeat, stop/fault path | Keep | Mandatory execution and fault detection. Any rate change requires timing/vehicle validation. |
| Scanner protective-field status used by AUTO gating | Keep | Current mux gates AUTO, including LINE. Do not disable the complete scanner as a manual/LINE optimization; hardware OSSD protection is a separate path. |
| Localization readiness/scan consistency during NAV, EKF/gyro used by active functions | Keep | These participate in readiness/operation, even if their names sound like monitoring. |
| RFID during tag-dependent LINE missions | Keep | Needed for stations, branches and mission progress. A missing reader must cause the defined degraded/refused behavior. |

Do not lower the 50 Hz mux/LINE/feedback/EKF rates or turn off Nav2 collision checks as a default CPU improvement. Preserve existing reduced-rate UI streams: panel UI 10 Hz, IO image 5 Hz, track UI 5 Hz. These reductions already exist.

## 6. Long-running resource and persistence reliability

### R16 — Bound operation journal growth and startup replay [P1]

**Evidence: confirmed.** `operations.py:110` appends to `operations.jsonl` without retention. `_replay()` at line 123 calls `readlines()`, reconstructs the full operation set, and trims only afterward. In-memory recent history is bounded; disk history and replay peak memory/boot time are not.

Implementation plan:

- Stream replay and periodically compact/rotate journal under a single writer.
- Retain pending/interrupted evidence and a documented request-ID deduplication window; do not accidentally replay side effects or forget still-actionable requests.
- Bound both journal bytes and retained record count; use an atomic checkpoint with tested crash recovery at each compaction stage.
- Surface journal write failure. Continuing to control the vehicle may be appropriate, but status must not imply durable history was saved.

Acceptance: replay/compact a representative multi-month journal with a memory ceiling; corrupt/truncate the last record; crash during compaction; retry an ID inside/outside the defined deduplication window.

### R17 — Move web event persistence off the ROS callback and bound request work [P1]

**Evidence: confirmed.** `adapter.py:539` writes events from the ROS event callback; ERROR persistence can `fsync`. The spinner uses a single-threaded executor, so slow disk can delay all adapter callbacks despite writing outside the adapter's data lock. `state()` holds that lock while reading IPC temperature/disk information. `web_node.py:33` runs Flask's development server with `threaded=True`.

Implementation plan:

- Use a bounded event writer with explicit priority, drop/coalesce policy, counters and controlled shutdown flush. Keep callbacks short.
- Sample sysfs/disk status outside the adapter state lock; return a cached immutable snapshot with age/unknown state.
- Serve with an explicit bounded HTTP concurrency model and bounded heavy-job queue. Keep one ROS adapter/manual-session owner; do not add multiple worker processes that duplicate subscribers and authority state.
- Set request-body, map/route item-count and expensive-validation limits with clear error responses. Reject excessive work before allocations.
- Review `_call()` timeouts: late ROS service results require a request/operation ID and explicit unknown outcome. Clean up pending futures safely without assuming cancellation undoes a remote side effect.

Acceptance: event flood with slow disk, many slow clients, oversized payloads and timed-out service calls. ROS receipts stay responsive; threads, futures, queue depth and RSS remain bounded.

### R18 — Report generation must be unique, atomic and resource-limited [P1]

**Evidence: confirmed.** `amr_web/reports.py:68` names reports to one-second resolution and opens the final ZIP path directly. Concurrent requests within one second can overwrite/conflict, and listing can expose an incomplete file. Journal capture uses unbounded captured stdout over a two-hour interval; ZIP inputs include role logs and the unbounded operation journal. Retention is 20 reports by count, without a total-byte ceiling. The included profile path is hardcoded to `~/agv_can/profiles/agv-01.json`.

Implementation plan:

- Admit one bounded report job or a small queue; return job status instead of holding arbitrary request threads.
- Use a unique name and unique staging file, finalize/fsync then atomically rename. Prune only completed artifacts.
- Apply per-input and total-byte/time budgets; stream journal output and ZIP inputs. Include a truncation/unavailability manifest.
- Record actual profile path/hash, repository/build identity, effective parameters and dependency manifest; do not silently package a different profile.
- Limit total report storage as well as count, while preserving selected incident reports.

Acceptance: simultaneous builds with a fixed clock, interrupted writes, huge journal/log inputs and low disk. Each published ZIP must be readable, uniquely identified and within its resource budget.

### R19 — Standardize durable atomic writes and storage health [P1]

**Evidence: confirmed.** `amr_web/commissioning_log.py:122` writes measurements using a PID-based temporary name, which is shared by concurrent threads in the same process; the sidecar lacks durable-write synchronization. Uptime marker rename/unlink durability also deserves directory-sync review. `agv_core/disk.py:34` skips unreadable paths and reports `OK` with `free_mb=-1` if none are readable.

Implementation plan:

- Centralize durable record writing: unique temporary file in the destination filesystem, flush/fsync when required, atomic replacement, parent-directory fsync, and cleanup on exception.
- Serialize conflicting updates or use expected-version checks; state whether a second measurement overwrites the first.
- Expose unknown/unreadable storage separately from healthy. Keep already-running motion independent of a free-space display, but require usable storage for work promising a durable save.
- Check writable/readonly status, I/O errors and inode exhaustion in addition to free bytes.
- Avoid adding synchronous fsync to high-rate control data; durability policy belongs to low-rate evidence/configuration operations.

Acceptance: concurrent saves for the same run, ENOSPC/inode exhaustion, read-only filesystem, failed stat, rename failure and simulated interruption before/after publication.

### R20 — Complete retention and bounded-cache policy [P2]

**Evidence: confirmed candidates.** Role/event logs already have limits. Event-log tail parsing builds records from rotated/current files before selecting the last requested entries. The web diagnostic dictionary accepts arbitrary diagnostic names. Maps, drafts, commissioning evidence and report inputs accumulate for different operational reasons; a single log cap does not bound total storage.

Implementation plan:

- Parse event tails with bounded retained records rather than retaining every decoded object. Whitelist or cap/expire diagnostic sources and cap payload lengths.
- Retain recent fault context when capping logs; blindly truncating an active log can discard the most useful pre-fault evidence. Add reason-based rate limiting with suppressed-count summaries.
- Inventory supervisor role logs, ROS launch/node logs, journal, reports, map revisions/drafts and commissioning evidence. Apply byte/age budgets and deliberate archive rules.
- Never automatically delete map/route revisions referenced by missions or evidence needed for an incident. Use a reference-aware archive/prune process.
- Prefer cheap bounded counters/histograms to verbose per-tick logs.

Acceptance: log flood, many diagnostic names, months of commissioning artifacts and referenced old revisions. Bound transient RSS and total disk use without removing active references.

## 7. Configuration and maintenance improvements

### R21 — Validate effective runtime configuration and data bounds [P1/P2]

**Evidence: confirmed.** The profile loader has substantial validation, but ROS parameters create a second configuration surface with direct divisions and cached fields. Some parameters can appear mutable without changing the cached behavior. `scan_gate.py:51`'s nonnegative/NaN check still accepts positive infinity. The profile contains settings used differently by the ROS path: for example `can.use_rpdo=false` while drive control uses RPDO, and monitor/IMU profile polling periods differ from ROS defaults.

Implementation plan:

- Validate finite positive rates/timeouts, ranges and cross-field relationships at node startup and on every supported update. Explicitly reject updates to startup-only parameters.
- Publish an effective configuration manifest with source/profile hash and override precedence. Separate active settings from retired legacy settings; document rather than silently imply an ignored value controls the ROS path.
- Require consistent profile/generation identity across authority-producing/consuming nodes where a mismatch changes geometry, limits or behavior.
- Bound external scan beam count, grid dimensions/cell count, route length/repetitions and evidence body size before allocating or traversing them. Reject malformed values predictably.
- Keep one-vehicle configuration ownership simple; a wholesale configuration-framework rewrite is unnecessary for this deployment scale.

Acceptance: zero/negative/NaN/infinite parameters, unsupported live changes, mismatched profiles, malformed scans and oversized grids/routes. Fail admission/startup clearly rather than failing deep inside a timer.

### R22 — Make the tested vehicle image reproducible and keep checks green [P3]

**Evidence: confirmed.** Root `pyproject.toml` deliberately keeps runtime dependencies empty for offline development. Runtime libraries come from the vehicle image. Preserve lazy imports/offline testability, but document a separately pinned/tested deployment manifest for Python packages, ROS/apt packages, CAN backend and scanner driver revision.

Implementation plan:

- Add a vehicle-image/deployment dependency manifest and build identity to startup diagnostics and incident reports.
- Make one repeatable check command run the core harness, ROS offline tests, both Ruff configurations and deployment static validation where available. Keep simulation/hardware acceptance explicit separate stages.
- Fix the existing 6 root and 12 workspace lint findings in a small isolated change after current working-tree work is integrated; they are lower priority than authority fixes.
- Add regression cases for the failure scenarios in this plan, avoiding tests that merely restate implementation internals.
- Keep ROS Humble reliability improvements separate from any future Jazzy migration, and verify target Python/ROS dependency versions during upgrades.

Acceptance: clean environment reproduces offline results; deployed manifest identifies exact tested runtime versions; CI distinguishes offline, simulation, hardware and soak coverage.

## 8. Implementation sequence and review boundaries

Implement in small reviewable changes; every source change needs a focused explanation of the failure trigger, resulting behavior and verification. Suggested order:

| Batch | Scope | Size/risk | Exit criteria |
|---|---|---|---|
| A | R01 panel acquisition freshness; R02 PP final authority check; focused regression tests | Medium, authority-sensitive | Stale physical input cannot authorize motion; no PP start after revocation; existing behavior tests pass. |
| B | Low-overhead cycle/source-age/queue metrics, R12 measurement age | Small/medium | Baseline available without per-tick log flood or material timing overhead. |
| C | R03 CAN transaction budget; R04 service admission; R05 lifecycle/progress; R06 clocks | Large, stage across separate PRs | Fault injection bounds control gaps and cleanup; one CAN owner and watchdog semantics preserved. |
| D | R07/R08 critical lock/I/O isolation; R09–R11 process/recovery contracts | Medium/large | Slow storage, dead leaders and absent action servers do not starve authority processing; recovery is truthful. |
| E | R16–R19 journals, web/event/report resource limits and durable writes | Medium | Bounded replay/jobs and atomic outputs survive fault injection. |
| F | R13–R15/R20 opt-in efficiency policy and retention | Medium | Same default behavior; measured lower load in selected quiet mode; consumer warm-up/readiness enforced. |
| G | R21/R22 configuration manifest, cleanup and reproducible deployment | Medium | Clear effective configuration and repeatable checks; controlled IPC acceptance and soak completed. |

Do not combine changes to control gains, ramp limits, safety-field behavior or automatic resume with CPU optimization. Keep existing field-block/release and authority-generation behavior covered by regression tests. Quiet presets must be explicitly selected and visible in status; default deployment must remain equivalent until accepted.

## 9. Measurement and acceptance plan

### Establish a baseline on the real IPC

Proposed measurements, not measurements performed by this audit:

- Per-process and per-thread CPU, RSS/PSS, thread count, file descriptors and context switches; system load, I/O wait, temperature and throttling. Tools such as `pidstat`, `ps`, `/proc` and the existing diagnostics are sufficient to start.
- Control-cycle duration and command/heartbeat inter-send gaps: count, p50/p95/p99, maximum and deadline misses. Measure optional SDO transaction latency, RX backlog, retry/abort counts and dropped/skipped diagnostic slots.
- DIO successful-acquisition intervals, actual source age at consumption, authority expiry, drive feedback age, supervisor lease gaps and state-lock hold time.
- Topic rates/payload sizes and subscriber fanout, especially scanner/TF/web streams. Measure CAN utilization on the real bitrate and device configuration.
- Storage write/fsync latency, bytes/day by artifact class, journal/replay duration, report duration/peak memory, map validation time and bounded-worker queue age.

Compare equal operating conditions: IDLE, manual in IDLE, LINE stopped/moving, NAV stopped/running, mapping, commissioning PV/PP, then zero/one/multiple browser viewers and Foxglove on/off. Include realistic map size, tag rate and production IPC temperature. Record software/profile identity for every run.

Nominal 50 Hz control implies a 20 ms period. Define steady-state jitter and worst-case fault budgets from measured hardware behavior and the required response; an initial review target can be p99 cycle time below 25 ms and no ordinary-load command gap above 50 ms, subject to validation. These are proposed acceptance targets, not existing guarantees. The 500 ms drive PC-loss guard is a fallback, not an acceptable routine scheduling budget. Linux/Python/ROS scheduling should not be described as a hard-real-time guarantee.

### Fault matrix

| Injection | Required observation |
|---|---|
| Freeze DIO worker; replay cached panel while publisher stays alive | Physical-source age invalidates motion, debounce does not advance on duplicates, reconnect creates no held-button edge. |
| Revoke PP hold/lease/selector or replace move during setup | No start edge; bounded, verified Halt/PV cleanup and terminal reason. |
| Missing/intermittent MLS, delayed/aborted SDO and CAN flood/bus-off | Critical work remains prioritized; fresh-feedback rules and fault/heartbeat fallback remain correct. |
| Block a drive service/bus worker | Independent progress detection; authority expiry and disarm remain schedulable; expired requests do not act later. |
| Slow/full/read-only storage; excessive events/reports | No critical lock starvation; bounded queues/storage and truthful durability/unknown status. |
| Kill launch leader but retain children; rapid/rare optional crashes | Cleanup before replacement, bounded crash window and no duplicate resource owner. |
| Missing/frozen action server; expensive invalid mission | Pause/abort responsiveness; no stale-generation validation commit. |
| Clock jump/reset/freeze and future/old input stamps | Steady-time authority expiration and explicit TF/simulation clock policy. |
| Several tabs/slow clients; browser disappears during jog | Bounded HTTP/ROS work, jog expires/releases, viewer work becomes idle. |
| SIGTERM/SIGKILL/interruption at persistence stages | No published partial bundle/report; interrupted operations recorded; unconfirmed cleanup retains dirty evidence. |
| Device absent at boot, cable return and repeated mode changes | Explicit bounded recovery/degraded status, no replayed mission/movement from restored connectivity alone. |

Run pure/fake fault tests first, then opt-in ROS simulation and controlled vehicle tests under the existing runbook. Record detection-to-stop-command and stop-confirmation latencies separately; an application log saying “stopped” is insufficient evidence of physical stopping.

### 24/7 soak acceptance

- Progress from a 24-hour baseline to at least 72 hours with representative operations/fault recovery; use a seven-day run before claiming production 24/7 readiness.
- Exercise repeated IDLE/LINE/NAV/mapping transitions, browser connect/disconnect, periodic reports and applicable commissioning workflows in a controlled setting.
- Require no unexplained authority loss, unexpected movement/resume, orphan processes, duplicate device owners, increasing FD/thread counts, or unbounded storage/queue growth.
- Set numerical RSS/storage/CPU thresholds after warm-up baseline and separate expected caches/artifact retention from leaks. Do not require zero memory fluctuation.
- Preserve timestamped metrics, deployment/profile identity and incident evidence. Compare current versus quiet policy on the same workload; accept an optimization only if it reduces measured resource use while meeting reliability criteria.

## 10. Recommended first implementation brief

Start with R01 and R02, with offline fault regression cases before vehicle tests. Add R12 and low-overhead timing metrics next so later changes can be judged from actual source age and cycle gaps. Then stage the CAN budget work, critical I/O isolation and persistence fixes. Expose the quiet monitoring preset only after those measurements and activation rules are in place.

The intended result is a controller whose motion decisions depend on current physical evidence, whose essential work has bounded progress under faults, and whose optional diagnostics and operator tooling consume predictable resources throughout long service periods.
