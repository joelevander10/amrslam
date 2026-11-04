# GLS label fix + route wait step: coding plan (MCQ4-4)

Branch `kyb-trackless` · 2026-10-30 · **P1 committed (`ae373c8`), vehicle acceptance A pending. P0 committed (`4370b16`), P2 committed (`0322c76`). P3 implemented 2026-10-30, uncommitted, tests being written.** P4–P6 not started. Physical tests (bench §7, acceptance A) deferred to one session.

This plan extends [gls6-system.md](gls6-system.md), the design brief. Read the brief first. Where the two differ, this plan wins: it is built on the code as of `481ffda`, after the emergency suite landed. File:line references are from that commit. Check each one before you edit.

## 1. Goal and scope

A run on MCQ4-4 is **HOME → destination → HOME**, one mission. On the way there are steps with specific motions: straights, reverses, rotations and arcs. At the trolley **hitch and release points** the vehicle stops. The operator hitches or releases the trolley by hand, then presses **PB Continue** (DI0D), and the run carries on.

The plan adds two things:

1. **A route `wait` step and PB Continue.** This is needed even without the camera.
2. **GLS621 label fixes at HOME and at the three destinations.** These give an absolute position check at the points where the vehicle is stationary anyway.

| Label | ID | Role |
|---|---|---|
| HOME | `x0000y0000` | Boot localisation, re-zero at the end of a run |
| Dest 1 UBC | `x0100y0000` | Wait-point fix |
| Dest 2 ITG | `x0200y0000` | Wait-point fix |
| Dest 3 PISTON | `x0300y0000` | Wait-point fix |
| (reserved) | `x0400y0000`, `x0N10`…`x0N40` | Printed but not assigned. Do not lay them. |

A label ID is a **name**. Code never derives a position from its digits.

Out of scope:

- labels between stops
- fixes while moving
- creeping onto a label (brief §3.8)
- feeding labels into slam_toolbox
- anything in the safety path

## 2. Facts this plan depends on

### Routes and executor

- **Step types.** These are `straight`, `rotate`, `reverse` and `arc` (`route.py:18,28`). `Step` has no generic params field (`route.py:139-147`). An unknown type raises `RouteError` (`route.py:205`). `SCHEMA_VERSION = 1` (`route.py:15`).
- **Straight end points.** A straight's `to` is an **absolute map point** (`compiler.py:121-161`). So after a pose correction, the next straight still drives to the right place. Its cross-track starts at the size of the correction.
- **Code that silently treats new step types as lines or rotations:**
  - `validate.py:144` (sweep `else`)
  - `editor.js:54,190-194,250-266`
  - `adapter.py:871-883`
  - the executor's `st.type != ROTATE` tests at `:745, :1132, :1151, :1257, :1301, :1307, :1405`

  A new step type needs an explicit branch at every one of these.
- **Run FSM.** States are IDLE, READY, EXECUTING, PAUSED, BLOCKED, FAULT, DONE (`run_fsm.py:14`), and `ACTIVE = (EXECUTING, PAUSED, BLOCKED)`. `step_done` wraps passes and reaches DONE after the last step (`run_fsm.py:170-176`).
- **Executor flow.**
  - Each step ends in SETTLE: wheels still for 0.3 s, then the end pose is verified (`route_executor_node.py:1244-1287`).
  - Start runs `_start_gate`: within 0.10 m / 5° of `route.start` (`:648`).
  - Then `_prereqs`: localisation READY, plus fresh wheels, panel and scan (`:609-627`).
  - Then the pre-move warning: `premove_s` 2.0 s, and the drives must report torque (`:702-717`).
- **`/initialpose` in the executor** (`:487-496`). While EXECUTING it faults the run (`EXEC_POSE_INJECTED`). While PAUSED or BLOCKED it invalidates the resume.
- **Emergency, owner policy update (2026-10-30), which overrides E3.** An emergency **does not stop the sequence or any program**. It is a physical stop only, like the safety chain: STO from the Flexi Soft, no power, mux inhibited. Runs, waits and other programs keep their state, and that includes WAITING.
- **Emergency, as implemented by the emergency suite.** This was uncommitted in the working tree after `481ffda`; re-check the line numbers once it is committed.
  - `_emergency_abort` became `_emergency_hold`. It is called from `_on_panel` and from the tick whenever a fresh panel says `emergency != EMG_NORMAL`.
  - EXECUTING, pre-move included, goes to `_hold("estop", "emergency stop")`. A BLOCKED run with any other cause is re-labelled `estop`, so it can't auto-resume.
  - READY and PAUSED are left alone. **Do not add WAITING to that condition.** WAITING just stays WAITING.
  - After NORMAL, a physical Start resumes `BLOCKED(estop)` through the pre-move warning, from the same pass and step. It never resumes by itself. Start and Load are still refused while not NORMAL.
  - New alarm row `ESTOP_HOLD`; `HOLD_CODES["estop"]` maps to it. Tests are in `amr_mission/test/test_emergency_hold.py`.
  - Blind survey moves still end on an emergency. The survey session does not end. (Commissioning and pp moves were removed 2026-10-30.)
- **Resuming after an emergency (owner, 2026-10-30).** After release and Reset, the operator presses **PB Continue or PB Start**. Either one resumes, whether the run was stopped while driving or while WAITING. Both buttons need a fresh press after NORMAL: an edge seen during the emergency is dropped. Both go through the same checks and the same pre-move warning. The emergency suite has implemented Start. **Part A adds Continue** to the `BLOCKED(estop)` resume (§A.4). In `agv_core/alarms.py`, change `ESTOP_HOLD`'s action text to "press Start or Continue", then regenerate RUNBOOK §4. Nothing is queued: `_start_edge`'s first check refuses an edge while not NORMAL, so a fresh press after NORMAL happens automatically. Give the Continue handler the same first check.
- **`RunState.msg`** is append-only (`RunState.msg:36`).

### Localisation

- **`on_initialpose`.** It sends the monitor to CHECKING and clears both the confirm and the evidence (`readiness.py:100-106`).
- **Reaching READY.** READY needs 2 s of good checks and an operator confirm (`/amr/localization/confirm`, `localization_monitor_node.py:141`). The confirm is gated by scan match ≥ 0.6.
- **Pose-jump check.** It is ignored for 1.5 s after an initial pose (`readiness.py:185`).
- **Today's initial-pose path.** The web publishes `/initialpose` with σ 0.5 m / 15° (`adapter.py:847-848`). It refuses while EXECUTING (`adapter.py:840`).
- **AMCL.** `set_initial_pose: false`, recovery alphas are 0 (`amcl.yaml:16,51-52`).
- **Monitor parameters.** The monitor takes only declared defaults; no yaml is passed (`navigation_layer.launch.py:184`).

### Map bundles

- **The bundle sha256 does not cover `manifest.yaml`** (`map_bundle.py:161-165,191`). The survey start mark lives in the manifest, so it is outside the bundle identity.
- **Map edits.** A derived revision inherits the manifest, start mark included (`map_bundle.py:301-335`).
- **Routes and missions.** A route binds `{id, revision, sha256}`. A mission binds the map and route refs (`store.py:215-244`). Routes are stored at `<maps>/<map_id>/routes/<route_id>/rev<N>.yaml`, immutable (`store.py:67`).

### Panel and config

- **PB Continue.** DI0D is already named `"Continue Run"` (`profiles/agv-01.json:50`). Nothing reads it.
- **`PanelScan`.** `agv_core/panel.py:58-60` is hard-wired to reset/start/auto. The first sample is a baseline that produces no edges, so a button held at power-on or at reconnect never fires (`:112-114, :81-86`). `tests/test_panel.py:13-20` constructs it positionally.
- **Edge fields.** Edges reach consumers as `PanelState.start_edge`/`reset_edge`. The repeat-frame branch clears them (`panel_io.py:128-139`). The executor reads the edge field, not `seq` (`route_executor_node.py:369-379`).
- **Profile strictness.** It is strict: an unknown or missing key is fatal (`config.py:626-656`). `agv-01.json` is the only profile. No profile values reach xacro: the URDF defaults sit in both `amr.urdf.xacro:15-44` and `description.launch.py:18-33`.
- **Owner locks.** `agv_core/ownerlock.py:27` lists `RESOURCES = ("app","can","dio","scanner")`. Bench tools refuse while the node holds the lock (`dio_probe.py:37-40`).
- **Test counts.** `tests/run_all.py:95` has `EXPECTED_CHECKS = 525` and a dated delta note.
- **Alarm codes.** Every emitted code needs a catalogue row (`tests/test_alarms.py:19-31`). The RUNBOOK block is generated by `python3 -m agv_core.alarms --md`.

### GLS621, read live 2026-10-30

- **Device configuration.**
  - The template outputs `$XMCL/$YMCL`.
  - ISO15415 verification is off.
  - The configuration is saved permanently.
- **Result stream.** Port 2112 streams one telegram per decoded code; every code from one image has the same `$FAMS`. Measured with `gls_probe` on the vehicle, 2026-10-30, label at HOME: **about 14.5 frames/s** (`$FAMS` steps of about 60 ms; the brief's "~33 Hz" was not an image rate), about 70 telegrams/s, 4–5 codes per image, spread about 5 counts and 0.3–0.6° between the codes of one image.
- **One telegram can carry several codes.** About 3 % of telegrams hold two results back to back with no separator (`…;946692806114D2x0000y0000;5;-33;…`). `agv_core.drivers.gls.parse` returns a list for that reason.
- **Command replies.** CoLa `sRN` replies also arrive on 2112, mixed in with the results. On 2111, replies get buried in the diagnostics stream.
- **Axes and units, bench 2026-10-30** (agv-can-a5, `manuals/gls6-qr/bench-results-2026-10-30.md`):
  - The device's **Y is the vehicle's fore-aft axis**: reversing lowers it. So `gls.swap_xy` is true. The lateral sign, the angle sign and `T_BC` wait for the spin test.
  - **Scale cross-check: about 0.95 mm/count**, from a fit against wheel odometry (rms 2.6 counts). It is not in the profile yet.
  - **Label print: tick span 54.5 mm on both axes**, against 54.0 nominal (+0.9 %, isotropic). That is only about 0.2 counts per 18 mm row, so the print does **not** explain the per-code bias (about 2.3 counts per row) or the 4–11 count spread between the codes of one frame. The code-to-code pitch measures 18 mm on both axes (operator, by caliper). The print is therefore right, and the cause lies in the device: its configured code layout or working distance. That is still open.

### Emergency suite: rules from agv-can-14, 2026-10-30

- PanelState changes need a **clean rebuild and a full `amr.service` restart**. A stale install crashed panel_node.
- rclpy forbids one logging call site that switches severity, such as `(log.info if x else log.warn)(msg)`. It raised `ValueError`.
- **Continue is refused while `emergency != EMG_NORMAL`.** A Continue edge seen during an emergency is **dropped**. A fresh press is needed after NORMAL.
- DI08 is **not latched**: it goes HIGH on release. The drives leave STO by themselves about 0.7 s after Reset.

## 3. Owner decisions

Settle these before Part A. Each has a recommendation.

| # | Question | Recommended | Why |
|---|---|---|---|
| W1 | Does WAITING survive an emergency? | **Decided (owner, 2026-10-30): yes.** An emergency is a physical stop only and never ends a run or a wait. After NORMAL, Continue needs a fresh press, READY and the wait pose gate (§A.4). | Covered by the emergency policy update (§2). |
| W2 | Drive power while WAITING | **Decided (owner, 2026-10-30): disarmed**, as in IDLE (`drive_power` false). The wheels do not free-wheel when disarmed. | |
| W3 | PB Start pressed at an ordinary wait, with no emergency involved | **Decided (owner, 2026-10-30): Start also continues**, the same as Continue | |
| W4 | A Continue button in the web UI | **No** | Same reasoning as E4: the person releasing the vehicle must be at the vehicle. |
| W5 | Pre-move warning on Continue | **Yes**, the same 2 s lamp + DO02 as Start | Someone is standing next to the vehicle. |
| W6 | Selector switched to MANUAL while WAITING | **Stays WAITING.** Continue needs AUTO and the pose gate. | Nudging the vehicle by hand is allowed. If it moved too far, Continue is refused with a reason and the operator aborts. |
| W7 | A timeout on a wait | **None** | Hitching takes as long as it takes. |
| G1 | Destination label not seen at a wait, after Part E is on | **Warn, and allow Continue** | Same as brief §3.7: a missing label must not strand the vehicle. |
| G2 | GLS disagrees with AMCL by more than 0.30 m or 10° at a wait | **Refuse Continue, raise an ERROR, the operator aborts** | Never continue autonomously on a disputed pose. |
| G3 | Label ID assignment | UBC `x0100`, ITG `x0200`, PISTON `x0300`; `x0400` reserved | Matches Dest 1–3 on DI0A–DI0C. |

## 4. Part A: the `wait` step and PB Continue

This part does not depend on the GLS. Ship it first.

### A.1 Route schema (`amr_navigation/route.py`, `compiler.py`, `validate.py`)

- **New constant `WAIT = "wait"`.** `Step` gains two optional fields:
  - `name: str`, shown to the operator, e.g. `"UBC hitch"`
  - `label_id: str | None`, used by Part D
- **Serialisation.**
  - `to_dict`/`from_dict` get an explicit `wait` branch.
  - Rotate stays its own branch, **not** the fall-through default, so an unknown type keeps raising.
  - Validation: `label_id` matches `^x\d{4}y\d{4}$`, and `name` is at most 40 characters.
- **`SCHEMA_VERSION` stays 1.** The change is additive: an old reader meeting a `wait` refuses the route loudly with "unknown step type". If the repo rule says otherwise, bump it, together with `editor.js:141-145`.
- **Placement rules** (in `from_dict` or `validate`):
  - a wait is never the first or the last step
  - no two waits in a row
  - the step before a wait must not be an arc, so the stop is tunable by Case B (§8)
- **Compiler.** The compiled step has `start = end = the previous end pose`, no samples, `duration_est_s = 0` and no time allowance. `closes()` and `repeat_count` are unchanged: a wait inside each pass is allowed.
- **Validation.** Add an explicit branch at `validate.py:144` that sweeps no line. The stationary footprint at that pose is checked like the start pose (`validate.py:125-130`).

### A.2 Panel (`agv_core/panel.py`, `amr_base/panel_io.py`, `panel_node.py`, `PanelState.msg`)

- **Profile.** Add `panel.di_continue: 13`. Add it to the `chans` distinctness check (`config.py:842-852`) and to the not-a-pendant-channel check, next to `di_estop_ok`.
- **`PanelScan`.** Add an optional 4th channel, `continue_ch=None`, so Continue gets the same baseline and reconnect protection as Start. Positional callers keep working.
- **`PanelState.msg`.** Append `bool continue_edge` after `emergency`.
  - `PanelFrame` gains the matching field.
  - The repeat-frame branch clears it (`panel_io.py:128-139`).
  - `panel_node` sets it (`:166`) and lists the channel in its startup log line (`:108-121`).
- **Sim.** `fake_panel_node` gets a `/sim/panel/press_continue` service, like `press_start` (`:56-57, :78-97`).
- **I/O mapping.** Change `io-mapping.md:135` from "Feature to add later" to the real function.

### A.3 Run FSM (`run_fsm.py`)

- **New state `WAITING = 7`.** Add it to `NAMES`. **Do not** add it to `ACTIVE`. Every `in ACTIVE` test has to be read and decided one by one; the list is in §A.4.
- **New transitions:**
  - `wait(step_name)`: EXECUTING → WAITING, at a wait step.
  - `continue_()`: WAITING → EXECUTING, then on to the next step.
  - `abort` must also accept WAITING.
- **Pure tests** go in `test_run_fsm.py`.

### A.4 Executor (`route_executor_node.py`)

**Entering a wait.**
- When the step before a wait finishes SETTLE (`:1244-1287`), the executor enters WAITING instead of sending a goal.
- The permit becomes NONE and `drive_power` follows W2.
- A `WAIT` event is logged with the step name.
- `_chainable` (`:866-868`) must be false for the step before a wait.

**`_continue_edge()`**, called from `_on_panel` after the emergency check, under `self._lock`. It acts only in WAITING, and refuses with a reason in `fsm.reason` when any of these fails:
1. `emergency != EMG_NORMAL`. The edge is **dropped**, not queued.
2. AUTO mode.
3. `_prereqs()`, which includes localisation READY.
4. **The wait pose gate.** The vehicle must be within `verify_position_m` (0.15) and `verify_heading_deg` (8°) of the wait step's compiled pose. This catches a vehicle that was nudged (W6) or pushed (W1). A GLS correction of up to the 0.30 m gate can exceed 0.15 m. A correction that large should be rare. If it happens, the refusal says so, and the operator aborts.
5. The label gate from Part E, when it is on (G2).

If all pass, it runs `fsm.continue_()` and `_begin_premove()` (W5), then the next step's INIT.

**Continue outside WAITING.** In a run held by an emergency while driving, a Continue edge does the same as a Start edge. It goes through the same `_start_edge` resume path: resume checks, then the pre-move warning (§2). Route both edges through one handler so their gating can't drift apart.

**In WAITING:**
- PB Start: continues, the same as Continue (W3).
- PB Continue in READY does not start a new run: "Continue ignored: a new run needs Start".
- `/initialpose`: **accepted.** No fault and no `invalidate_resume`: a wait is a step boundary, with nothing half-done. Only an odometry gap is recorded. This is the path for both an operator re-seed and the GLS fix.
- Prerequisite failures **do not fault** a WAITING run, because nothing is moving. They only block Continue, with the reason shown. A LOST localisation can be re-seeded from the web; `adapter.py:840` only refuses while EXECUTING.
- **Emergency: stays WAITING** (W1, policy update in §2). STO cuts power in hardware. The executor only marks the wait as interrupted and keeps the route.
  - Recovery: release the E-stop, press Reset, then press Continue or Start (§2).
  - Either button needs a **fresh press after NORMAL**. An edge seen during the emergency is dropped.
  - Then the pose gate and READY are checked again, because the vehicle may have been pushed while it had no power.
  - **Nothing to change in `_emergency_hold`.** It already leaves any state other than EXECUTING and BLOCKED alone. Keep WAITING out of its condition, and add a test that checks it stays out.
- MANUAL: stays WAITING (W6). Check what `_tick`'s manual hold (`:997-1063`) does today, and exclude WAITING from it.

**Every `st.type != ROTATE` site** (`:745, :1132, :1151, :1257, :1301, :1307, :1405`) and every `in ACTIVE` test needs an explicit `WAIT` decision.
- `_resume_checks` does not apply in WAITING.
- `_publish_permit` gives NONE.
- `drive_power` follows W2.

**`RunState.msg`.** Append the following:
- `uint8 WAITING=7` (a constant: the `state` field already exists)
- `string wait_name`
- `float64 waiting_s`
- `string wait_label_id`
- `string continue_refused` (why the last Continue was refused, "" if none)

Set `step_type` to `"wait"`.

### A.5 Web

- **Editor** (`editor.js`, `editor.html:15-27`):
  - a **Wait** button with a name field and an optional label dropdown, filled from the label set in Part C
  - explicit `wait` branches in `poseAfter` (`:46-57`), the step list (`:190-194`) and the overlays (`:250-266`)
- **Route preview** (`adapter.py:871-883`): show a wait marker.
- **Run page and home headline:** "WAITING at UBC hitch: hitch the trolley, then press Continue", the time waited, and `continue_refused` when it is set. No web Continue button (W4).
- **Alarms:**
  - `RUN_WAITING` (INFO, a new CLEARS value `continue_button`, with a `BUTTON` entry `""`)
  - `CONTINUE_REFUSED` (WARN, `continue_button`)

  Regenerate the RUNBOOK §4 block.

### A.6 Tests for Part A

- **`test_route.py`:**
  - wait round-trip
  - placement rules
  - compiler: the wait pose equals the previous end pose
  - validate takes the explicit branch
  - an unknown type still raises
- **`test_run_fsm.py`:** wait, continue and abort-from-WAITING.
- **`test_route_executor_logic.py`** (node built with `__new__`):
  - enter WAITING after SETTLE
  - permit NONE
  - each refusal reason
  - Start ignored
  - `/initialpose` accepted without a fault
  - the prerequisite failure that does not fault
  - Continue then the pre-move warning
- **`test_emergency_hold.py`:**
  - In a run held by an emergency while driving, Continue resumes through the pre-move warning, the same as Start. Continue pressed during the emergency is dropped.
  - WAITING survives an emergency with the route kept.
  - A Continue edge during the emergency is dropped.
  - Continue is refused until NORMAL.
  - A fresh Continue after NORMAL passes the pose gate and resumes.
  - A vehicle pushed beyond the gate during the emergency is refused.
- **`tests/test_panel.py`** and **`test_panel_io.py`:**
  - the Continue edge
  - the baseline at boot and at reconnect
  - the repeat frame clears the edge
- **`tests/test_config.py`:** `di_continue` is distinct, and is not a pendant channel.
- **Sim:** `test_route_sim.py` gets a route HOME → wait → HOME, `press_continue`, then DONE.
- **`EXPECTED_CHECKS`:** raise it by the new `run_all` checks, with a dated note.

## 5. Part B: the GLS driver and node

This part is read-only. It moves nothing.

### B.1 `agv_core/drivers/gls.py` (no ROS; follow the `dio.py` pattern)

The module has three parts:

**Pure parser**
- **`split(buf) -> (telegrams, rest)`:** splits on STX/ETX. It handles telegrams that arrive in pieces, several per read, and garbage before an STX.
- **`parse(telegram)`** returns one of:
  - `CodeRead(pos, label_id, x_cnt, y_cnt, angle_deg, ts_ms)`, from `A1x0000y0000;11;9;0.593822;946687420249`. `pos` is the first two characters; the rest is the label ID.
  - `Reply(kind, name, value)`, for `sRA`/`sWA`/`sAN`/`sFA`.
  - `Unknown`, which is counted and ignored.
- **`FrameAggregator`.**
  - It groups codes by `ts_ms`. A frame closes when a new timestamp arrives, or when no data has come for longer than `frame_gap_s`.
  - Per frame it reports: median x and y; a circular median angle (values wrap at 0/360); `n_codes`; the spread; how many outliers were rejected; and one `label_id`. Mixed IDs in one frame make the frame invalid.

**Conversion**
- Counts become metres and radians using the profile keys `xy_scale_m_per_count`, the signs and `angle_offset_rad`.
- Nothing is hard-coded; bench test §7 sets these values.

**`GlsLink`**
- It has an injectable `client_factory`, its own daemon thread, and reconnects with backoff.
- `snapshot()` reports `comms_ok`, `rx_age_s`, the frame rate, the last frame and the error count. Health is judged by data arriving, not by socket state, as in `dio.py:40-45`.
- **There is one socket, to 2112.** At connect it sends `sRN` for `OFtplstr1`, `GlsCodeSel` and `DCDMVerISO15415`. `DC2DContrast` is a tuning value with no single right answer, so only `gls_probe --verify` shows it; the link doesn't check it. The replies arrive mixed with the result stream, and `parse` separates them. If a value differs from the expected one in the profile, or no reply arrives within `verify_timeout_s`, the link refuses to report frames and raises `GLS_CONFIG_MISMATCH`.
- **It never writes a device parameter.**
- Update the docstring in `drivers/__init__.py`.

### B.2 Bench tool `agv_core/drivers/gls_probe.py`

Build this first. The §7 bench tests need it.

- It acquires the `"gls"` owner lock. Add `"gls"` to `ownerlock.RESOURCES` (`:27`) and to `test_ownerlock.py`.
- If the lock is taken, it exits with "GLS is in use (...). Stop amr.service first."
- What it does:
  - `--raw FILE`: logs port 2112 verbatim with receive timestamps.
  - `--frames`: prints the aggregated frames.
  - `--verify`: prints the four `sRN` values.

### B.3 Profile section `gls` (`config.py` `_SCHEMA` + `_validate`, `agv-01.json`)

All keys are primitives, and the section is required in every profile.

| Key | Initial value | Note |
|---|---|---|
| `enabled` | `true` | false → no node, no fixes |
| `host`, `port` | `"192.168.4.1"`, `2112` | |
| `connect_timeout_s`, `stale_s`, `frame_gap_s`, `verify_timeout_s` | 2.0, 0.5, 0.05, 3.0 | |
| `expect_template` | the live `OFtplstr1` string | checked at connect |
| `xy_scale_m_per_count`, `x_sign`, `y_sign`, `angle_sign`, `angle_offset_rad` | **from bench §7** | until then `correct_*` must be false (validate rule) |
| `mount_x_m`, `mount_y_m`, `mount_yaw_rad` | from the calibration spin (§7.6) | `T_BC` |
| `min_codes`, `outlier_m` | 2, 0.005 | |
| `fix_frames`, `fix_spread_xy_m`, `fix_spread_yaw_rad`, `stationary_s` | 15, 0.002, 0.009, 1.0 | tune from §7.4 |
| `fix_gate_xy_m`, `fix_gate_yaw_rad` | 0.30, 0.175 | G2 |
| `fix_deadband_xy_m`, `fix_deadband_yaw_rad` | 0.005, 0.005 | |
| `cov_floor_xy_m`, `cov_floor_yaw_rad` | 0.005, 0.009 | |
| `correct_at_home`, `correct_at_waits` | `false`, `false` | roll-out switches, §7 |

Add the notes to the TUNING NOTES docstring (`config.py:1070`). Extend `test_config.py`:
- every key is present
- the ranges are checked
- `correct_*: true` is refused while `xy_scale_m_per_count` is still the placeholder 0

### B.4 `amr_base/gls_node` and `GlsReading.msg`

- **Ownership.** The node takes the `"gls"` lock before any I/O and runs `GlsLink`.
- **`/gls/reading`.** It publishes `amr_interfaces/GlsReading` once per frame, and a `valid=false` reading once nothing has arrived for `stale_s`. The device is silent on a no-read.
- **Message fields:**
  - `header` (receive time)
  - `bool valid`
  - `string label_id`
  - `float64 x_m`, `y_m`, `yaw_rad`, the converted `T_LC`
  - `uint16 n_codes`
  - `float64 spread_xy_m`, `spread_yaw_rad`
  - `uint64 device_ts_ms`
  - `bool comms_ok`
  - `bool config_ok`
- **Launch.** Start it in `base.launch.py`'s `if real:` block (`:41-81`) when `gls.enabled`. Add an entry point in `amr_base/setup.py:21-27`.
- **Alarms:**
  - `GLS_DOWN` (WARN, `auto`): "The floor-label camera is not answering". It never stops a run.
  - `GLS_CONFIG_MISMATCH` (ERROR, `engineer`)
- **Web.** Show the camera's status on the IO or Monitor page (Status was removed 2026-10-30): connected, frame rate, codes per frame, last label.

### B.5 URDF `gls_frame`

- Add xacro args `gls_x`, `gls_y` and `gls_yaw`, and a fixed joint `base_link → gls_frame`. Remove the "not here yet" comment (`amr.urdf.xacro:117`).
- **Keep one source for the values.** `description.launch.py` reads them from `agv_core.config`; nodes already import it directly. Don't add a second copy in `XACRO_ARGS`.
- Update `test_urdf.py:11-12` so `gls_frame` is a required link.

### B.6 Sim

`amr_sim/fake_gls_node` publishes `GlsReading` from the sim's ground-truth pose whenever the camera is within the capture window of a label pose. The label poses come from the active label set (Part C), and the capture window is ±0.035 m sideways, ±0.020 m fore-aft. The parser is covered by unit tests, so the fake works at the ROS level, not the TCP level. Launch it in `base.launch.py`'s `real:=false` branch.

### B.7 Tests for Part B

- **`tests/test_gls.py`**, new and added to `run_all` MODULES:
  - split and joined telegrams
  - replies mixed into the result stream
  - malformed fields
  - mixed IDs in one frame
  - angle wrap at 0/360
  - grouping by timestamp and gap
  - outlier rejection
  - conversion with each sign combination
  - the verify mismatch
- **Per-package pytest for `gls_node`:** reconnection, staleness, the `valid=false` cadence and the lock.

## 6. Part C: label poses bound to the map

The label poses `T_ML` belong to a map revision. They go in a **separate, revisioned label set** rather than the manifest, because the manifest is not in the bundle identity (§2). This also means recording a label pose does not create a new map revision, so it does not invalidate the routes.

### C.1 Store (`amr_navigation/store.py`)

- **Location.** `<maps>/<map_id>/labels/rev<N>.yaml`. It is immutable, written with the same `_commit_new` + flock as routes.
- **Contents:**
  - `map: {id, revision, sha256}`
  - `labels`, a list of entries with these fields:
    - `label_id`
    - `role` (`home` | `dest`)
    - `name` (`"UBC"`)
    - `x_m`, `y_m`, `yaw_rad`
    - `method` (`survey` | `amcl`)
    - `recorded`
    - `n_frames`
    - `spread_xy_m`, `spread_yaw_rad`
    - `note`
- **Rules:**
  - **at most one** `home`, and at least one label. A map surveyed without a HOME label has none (owner, 2026-11-02); destinations are recorded on it all the same.
  - unique IDs and unique names
  - IDs match `^x\d{4}y\d{4}$`
- **Functions:** `save_labels`, `load_labels`, `list_labels`. Tests go in `test_route.py`, or in a new `test_labels.py` there.
- **Missions bind a label set.** `save_mission` gains an optional `labels: {revision, sha256}`. A mission without one gets no fixes, only raw logging. `mission_matches_active_map` also checks that the label set's map ref matches the active map.
- **The editor's label dropdown** (§A.5) lists the active map's latest label set. The validator refuses a `wait.label_id` that is not in the label set the mission binds.

### C.2 Recording a label pose

This is web action "Record label here". It is allowed only when all of these hold:
- NAVIGATION mode
- localisation READY
- the vehicle stationary for `stationary_s`
- `GlsReading` valid with ≥ `min_codes` and the spread within limits

It averages `T_MB` (from AMCL) and `T_LC` over `fix_frames`. Then:

```
T_ML = T_MB ⊕ T_BC ⊕ (T_LC)⁻¹
```

It writes a new label-set revision with `method: amcl`, the spread and the date. This is the Case A/B procedure (§8), and the way HOME is recorded on a map that already exists.

### C.3 HOME at survey start (`mapping_session_node.py`)

- At `_srv_start` (`:263-288`), also average `/gls/reading` over the same stationary window. If HOME is seen, keep `T_LC`.
- At save, after the bundle is published, write label set rev1 for the new map with `HOME`, `method: survey`. `T_MB(start)` is the start mark itself.
- **HOME is optional** (owner, 2026-11-02). With no label under the camera at the start mark (none laid, camera off, or not calibrated), the survey starts and saves as usual, says "mapping without a HOME label (why)", and writes no label set. HOME can be recorded later with "Record label here", or never.
- The closure residual at the end of a survey is later work (brief §3.5) and is not in this plan.

## 7. Parts D–F: label fixes

### D. Pure logic

- **`agv_core/glspose.py`:** SE(2) compose, invert, the `T_MB` formula, a circular mean, and the covariance from spread plus the floors. `tests/test_glspose.py` is added to MODULES and round-trips every one.
- **`amr_localization/label_fix.py`:** pure and clock-fed, like `readiness.py`. Its inputs are:
  - `GlsReading`s
  - the AMCL pose
  - the localisation state
  - whether the vehicle is stationary (odom twist ≈ 0 for `stationary_s`)
  - the run context: state, the wait's `label_id`, and whether the vehicle is at the end of the route at HOME
  - the label set

  It outputs a `LabelFix` decision with this status enum:
  - `NONE`
  - `OBSERVING`
  - `LOGGED`
  - `APPLY`
  - `IN_DEADBAND`
  - `REJECT_GATE`
  - `WRONG_LABEL`
  - `UNKNOWN_LABEL`
  - `NOT_STATIONARY`
  - `FEW_CODES`
  - `SPREAD`

**When a fix may be applied.** In any other state it is only logged.

| Context | Expected label | AMCL prior needed | Applies when |
|---|---|---|---|
| UNLOCALIZED (boot) | HOME | no; the unique ID is the evidence | `correct_at_home` |
| DONE or IDLE, parked at HOME | HOME | yes: within 0.30 m / 10° | `correct_at_home`, outside the deadband |
| WAITING with a `label_id` | that ID | yes: within 0.30 m / 10° | `correct_at_waits`, outside the deadband |
| EXECUTING, PAUSED, BLOCKED, READY | — | — | **never.** The executor would fault, and READY has a start gate. |

**A map without HOME** (owner, 2026-11-02): the two HOME rows never apply. Boot localisation stays the operator-confirm path, and there is no re-zero at DONE; the waits' labels still give fixes under `correct_at_waits`.

**Rules that always apply:**
- An ID that isn't in the mission's label set, or isn't the expected one, is reported and nothing is applied.
- The vehicle must be stationary.
- At least `min_codes` codes must be read, and the spread must be within limits.

### E. `amr_localization/label_fix_node` and the monitor

- **Subscriptions:** `/gls/reading`, `/amcl_pose`, `/amr/localization_state`, `/amr/run_state` and `/odometry/filtered`.
- **Loading the label set.** It loads the label set for the active map. That means the mission's set when a run is loaded, and otherwise the latest one.
- **Launch.** Next to `localization_monitor` in `navigation_layer.launch.py` (around `:188`).
- **Publications:**
  - `/initialpose` on `APPLY`, with the covariance from `glspose`.
  - `/amr/label_fix`, a new `LabelFix.msg` carrying the status, `label_id`, role, the residual xy/yaw before the snap, the spread, the applied pose and `applied: bool`.
- **The drift log.** Every decision except `NONE`/`OBSERVING` is appended to a JSONL file. Use the directory the executor's `_run_summary` writes to; check where that is. Each line holds the timestamp, run id, label, residual, spread and stop error. This log is the Phase-1 data.
- **Monitor change (`readiness.py`, pure):**
  - New input: `on_label_fix(t, agree: bool)`, fed from `/amr/label_fix` by `localization_monitor_node`.
  - Behaviour: in CHECKING, when `can_confirm` holds and an applied fix newer than `_initial_t` still agrees with AMCL within `fix_spread` × 3, it **confirms automatically**. The code is `LOC_CONFIRMED_BY_LABEL`, and an event is logged.
  - Unchanged: every automatic check, including scan match ≥ 0.6. With no fix, the operator-confirm path works exactly as today.
  - Tests in the readiness tests: confirm with a fix, without a fix, with a stale fix and with a disagreeing fix.
- **The executor and the label (G1/G2).** The executor subscribes to `/amr/label_fix`. In WAITING with a `label_id`:
  - `REJECT_GATE`, `WRONG_LABEL` or `UNKNOWN_LABEL` → Continue is refused and the matching alarm raised.
  - No fix within `stationary_s + 2 s` → `LABEL_NOT_SEEN` (WARN). Continue is allowed (G1).
- **Alarms:**
  - `LABEL_DISAGREE` (ERROR, `operator`): "The position check at the label failed". The action is to abort the run and call the engineer.
  - `LABEL_WRONG` (WARN)
  - `LABEL_NOT_SEEN` (WARN, `auto`)
  - `LABEL_FIX_APPLIED` (INFO event)

### F. Roll-out (profile switches; see §10 for the phases)

1. Both `correct_*` false: **log only**, everywhere. Run normal production for a while. Read the residuals and stop errors per label.
2. `correct_at_home: true`. Boot on HOME goes to READY with no operator, and the vehicle re-zeros at the end of each run.
3. `correct_at_waits: true`, only once the logs show drift at the destinations worth correcting.

## 8. Placing the destination labels (Case B)

Hitch and release points are fixed by the trolleys, so this is Case B from the planning session:

1. Lay the label at the hitch point, under the camera's lens position (brief §4), clear of the wheel tracks, with +X along the final heading.
2. Make the route's **last two steps before the wait perpendicular straights**. A reverse straight counts. The last one sets the stop along the approach. The one before it sets the sideways position. Never an arc (§A.1).
3. Run the route. Read the miss on `/gls/reading`, or with `gls_probe --frames`.
4. Correct the two typed lengths and save a new route revision. Expect it to settle in 1–2 tries.
5. Park on the label and use "Record label here" (§C.2).

**Capture window warning.** The datasheet estimate was about ±35 mm sideways and ±20 mm fore-aft. The bench on 2026-10-30 measured the fore-aft window on the vehicle: the label is read within about ±75 device counts (≈ ±72 mm at the odometry cross-check scale, 2 codes at the edges), and with ≥4 codes within about −62..+56 counts (≈ ±55 mm). That is comfortably wider than Nav2's ±50 mm goal tolerance (`nav2_params.yaml:24`). The sideways window is not measured yet. `fake_gls_node` keeps the conservative ±20 / ±35 mm. If stops scatter beyond the window, use a 2×2 patch of labels. That needs the IDs from brief §2 and a decision, not code.

## 9. Open items: not decisions, not in this plan

- **How the executor's resume checks treat a vehicle pushed during an emergency while driving** (emergency-suite work). What resumes such a run is decided: PB Continue or PB Start (§2).
- **Repeated passes.** With `repeat_count > 1`, HOME between passes is passed without a stop. Those passes are logged only; the re-zero happens at DONE. If a mid-cycle re-zero is wanted, add a wait at HOME.
- **Emergency suite bench Q2:** does the scanner OSSD also pull DI08 low? It belongs to the emergency suite, and it affects W1's behaviour after a field stop.
- **GLS bench tests from brief §7:**
  - units, signs and angle
  - repeatability
  - frame rate now that ISO15415 is off
  - the extrinsic `T_BC`
  - the capture window
  - the **4–11 count spread between codes** in a single XMCL frame

  All of them block `correct_*: true`. None of them blocks Parts A–C.
- **Exporting the GLS parameter file** and committing it (brief §5).

## 10. Order of work

Each phase ends green: `tests/run_all.py` with `EXPECTED_CHECKS` raised deliberately, the package pytests, ruff and `test_layout`.

| Phase | Work | Depends on | Ends with |
|---|---|---|---|
| P0 | `gls_probe` + the `"gls"` lock + the `gls.py` parser and tests | — | Bench §7 can be run. Start it in parallel. |
| P1 | Part A: wait step, PB Continue, WAITING, web, alarms | W2–W7 answered; the emergency-hold change is committed | On-vehicle acceptance A (below) |
| P2 | Part B: GlsLink, `gls_node`, `GlsReading`, profile `gls`, URDF frame, fake node | P0 | The camera is visible in the web and in TF, read-only |
| P3 | Part C: label store, mission binding, "Record label here", HOME at survey start (optional) | P2 | The three destinations recorded on the production map, and HOME if one is laid |
| P4 | `glspose`, `label_fix`, `label_fix_node`, `LabelFix`, drift log, executor label gate, both switches off | P3 | Log-only data from production |
| P5 | Readiness auto-confirm, `correct_at_home: true` | P4 + bench §7 | Acceptance B |
| P6 | `correct_at_waits: true` | P5 + the drift log | Acceptance C |

**Interface changes.** P1 changes `PanelState` and `RunState`. P2 adds `GlsReading`, and P4 adds `LabelFix`. Each one needs a **clean rebuild and a full `amr.service` restart** (§2, emergency rules).

## 11. On-vehicle acceptance (witnessed, wheels on the ground, area clear)

### A: wait step (P1)

1. The route HOME → wait "UBC hitch" → HOME stops at the wait and shows WAITING on the Run page and the headline. The drives follow W2.
2. Continue gives the 2 s warning, then the run carries on to HOME and reaches DONE. In a second trial, Start does the same (W3).
3. Continue in MANUAL is refused. Back in AUTO, it works.
4. Jog the vehicle about 0.2 m away in MANUAL. Continue is refused with the pose reason.
5. E-stop during WAITING: it stays WAITING and the route is kept. Continue or Start pressed during the E-stop does nothing. After release and Reset, a fresh press of either one resumes.
6. E-stop while driving: the vehicle stops on STO and the run is kept. After release and Reset, a fresh press of Continue, and in a second trial of Start, gives the 2 s warning and then resumes.

### B: HOME (P5; only on a map with a HOME label)

1. Boot parked on HOME: READY with no operator input, with `LOC_CONFIRMED_BY_LABEL` logged. Cover the label: the operator-confirm path still works.
2. After N runs, the drift log shows a residual at HOME for each run, and the vehicle re-zeros at DONE.
3. Swap in a wrong label (`x0400y0000`): it is reported and no fix is applied.

### C: destinations (P6)

1. At each wait the fix is applied while WAITING with no fault. READY comes back before Continue is pressed.
2. Move the label more than 0.30 m: `LABEL_DISAGREE`, Continue refused, no snap.
3. Cover the label: `LABEL_NOT_SEEN`, and Continue still works (G1).
