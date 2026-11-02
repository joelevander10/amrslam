# AGV stack hardening plan: device dropouts, recovery, long-run durability

Date: 2026-10-23. This plan was built from 8 read-only audits, each checked by a verifier, plus a critic pass. Nothing was run on the vehicle.

It supersedes the first draft (interrupted run). Where the two disagreed, this one follows the verifier's corrected fixes:
- **RFID-1:** the re-baseline gets a `tag_clear_s` window.
- **X10/D12:** the gate's freshness limits move to the profile, with hard refusal ceilings.
- **DIO-3:** `WAITING_FOR_PREREQ` stays as it is, because the executor shares it.
- **MLS-2:** frozen-sensor detection becomes diagnostic-only first.

Refuted parts of audit items are left out. Items the verifier rated "partially" carry that label below.

---

## 1. Summary

### How the stack handles dropouts today
- **Age, not events.** Every consumer stamps receive time and ages it against its own limit. Examples:
  - mux: cmd 0.2 s / lease 0.3 s / panel 0.2 s / field 0.5 s (gating.py:51-56, 254-255)
  - line: panel 0.2, drives 0.5, rfid 1.5, markers 2.5 (line_follow_node.py:124-142, 450)
  - executor: 0.25–2.5 s (route_executor_node.py:189-211)
  - localization: scan 0.15, imu 0.2 (readiness.py:50-60)
  - The limits are ROS-param defaults in code. Only `sensor_timeout_s` and `driver_timeout_s` are profile keys (agv-01.json:37, 247).
- **The gate fails safe.** A stale or invalid input zeroes the affected sources at once, with no ramp: FIELD_UNKNOWN, PANEL_STALE, DRIVES_NOT_READY, NOT_LEASED (gating.py:278-290, 540-543). drive_node keeps its own monotonic watchdog (drive_node.py:616-628). The drives have their own 1016h PC-loss stop (canopen.py:526-556). Hardware (OSSD → FX3 → STO) stays the safety path.
- **One owner per device, one thread per driver.**
  - can0 belongs to drive_node, behind ownerlock (drive_node.py:356). The MLS track, MLS gyro and both drives share that one bus thread.
  - DioLink (dio.py:97-110) and RfidLink (rfid.py:421-426) each run their own thread.
  - The scanner driver is a respawned launch node (scanner.launch.py:30-31).
- **Reconnects are per-driver and ad hoc:**
  - DIO: fixed 0.5 s (dio.py:240)
  - RFID: 2 s plus generation/seq (rfid.py:409-417, 655-670)
  - MLS: NMT Start every 2 s, SDO fallback, 5 s rediscovery (mls_track.py:323-356)
  - IMU: 5 s probe (mls_imu.py:234-240)
  - Drives: latched FAULT until `/drives/ack_fault`, which has no caller (drive_node.py:592-599)
  - Scanner: process respawn only
- **Frozen-but-alive data is mostly undetected.** Nothing compares payloads or checks plausibility: MLS TPDO1 (mls_track.py:285-303), gyro (mls_imu.py:108-109), drive feedback (canopen.py:411-427), scanner `is_valid[]` (no consumer). RFID has no overdue bound (rfid.py:65-67).
- **Recovery semantics are inconsistent.**
  - The line job resumes `field/drives/rate/track` holds by itself (job.py:73), against catalogue rows that say Start or engineer (agv_core/alarms.py:67-87).
  - The hold cause is picked from `track_cause`, not from the prerequisite that failed (job.py:533).
  - The executor faults on losses that LINE auto-resumes.
- **Reporting is fragmented.**
  - Device state travels on six different channels: PanelState.valid, DriveStatus strings, web scan age, `/diagnostics`, the StationDetection heartbeat and the MarkerEvent heartbeat.
  - The web `standing()` ignores `drives.link`, `line_track`, `rfid` and diagnostics (amr_web/alarms.py:49-121).
  - `headline()` ignores LINE (alarms.py:124-160).
  - The IMU diagnostic is always OK (drive_node.py:937).
- **Pure, clock-fed tests are strong; fault injection is thin.** Good unit coverage exists: gating, R04–R07, line_authority, auto_resume, readiness. There is one sim injection, which kills the base (test_fault_injection_sim.py:132). The sim fakes cannot mute or freeze a stream.

### Top 5 risks
1. **Silent missed stops on RFID.** Three issues stack:
   - The reconnect re-baseline swallows the next tag of any identity (rfid.py:615-622).
   - The heartbeat can publish a seq ahead of its encounter, and the cursor then skips it (rfid_node.py:89-91, tape_run.py:225).
   - Generation 0 is reused after a respawn (rfid.py:416-417).

   With no overdue bound, a station or Home is driven past with no hold.
2. **AUTO resumes without a fresh physical Start.**
   - `rate`/`track`/`drives` holds auto-resume (job.py:73).
   - A field-unknown or panel loss coinciding with a bad tape read becomes an auto `track` hold (job.py:533).
   - The executor's `_field_clear` never ages (route_executor_node.py:411-424, 1053).
   - After a drive-fault ack, a line job that is still RUNNING drives off (line_follow_node.py:504-509, job.py:274-278).
3. **The bus thread blocks while the last setpoint stays applied.**
   - MLS/IMU rediscovery calls a blocking `start()` with 0.4 s reads (mls_track.py:323-332, mls_imu.py:175-188, canopen.py:475).
   - The IMU polls inside every SDO, with 50 ms timeouts (mls_imu.py:244-256).

   Both stretch a command past the 0.2 s freshness invariant.
4. **A comms loss is shown as a different event.**
   - A silent MLS ends a run as DONE ("tape out of view") with no alarm, and mission progress is discarded (job.py:770-777).
   - A latched drive fault shows as DRIVES_NOT_READY "press Start" (amr_web/alarms.py:34-46).
   - A LINE hold shows READY (alarms.py:144-150).
5. **Dead ends with no operator recovery.**
   - The drive FAULT has no clear path, and a stale EMCY latch re-faults after a power cycle (canopen.py:434, 1032-1033, 1137-1142).
   - A CAN adapter replug leaves amr.service down (amr.service:6, 48).
   - A supervisor crash or watchdog kill SIGKILLs drive_node, so 8130h latches while the HMI says "no power cycle needed" (amr.service:38; agv_core/alarms.py:330-334).

---

## 2. Per-device matrix (current state)

| Device | Transport | Detection | Timeout → zero | Reaction | Recovery | HMI code | Tests | Verdict |
|---|---|---|---|---|---|---|---|---|
| **nanoScan3** (data link only) | Eth/UDP 6060, CoLa2 TCP 2122, ~17 Hz (nanoscan3.yaml:19-28) | Receive age, no stamp on OutputPaths. `is_valid` / case / contamination ignored | Mux 0.5 s; line ~1.0 s; NAV LOST 0.15 s + 0.5 s grace | Mux zeroes AUTO. LINE unknown → `pending`. Executor uses a never-aging `_field_clear` | Respawn 5 s on process exit only. If the scanner reboots while the driver lives, data may never return (SCN-1, partially) | SCANNER_SILENT/STALE (wrong IP, wrong 34 Hz text), FIELD_UNKNOWN, LOC_STREAM_STALE | Pure gating + line only; sim never publishes `/output_paths` | Gate is good; executor and diagnosis are weak |
| **MLS track** (node 10) | CANopen TPDO1 10 ms on can0 | Sample age 0.1 s; rate ≥ 40 Hz; source == tpdo (track.py:72-88). No heartbeat, no frozen check | ~0.1 s + 20 ms | Engine `sensor_lost` → **job DONE** (job.py:770), U-turn → FAULT | NMT Start every 2 s, SDO fallback, rediscovery 5 s (**blocking**) | None for silence; "stale" tile; diag on Monitor only | test_mls_track good; job stale path untested (fixture age 0.005) | **High risk**: misclassified, blocks the bus |
| **MLS markers** | Same TPDO1 → `/amr/line_marker` + 1 Hz hb | Seq gaps, hb 2.5 s. No time-gap check (marker_events.py:73-79) | 2.5 s | Marker run → `markers` hold (Start); HIGH dropped | Generation/seq | MARKERS_LOST | test_marker_events / test_marker_run | Sub-0.1 s holes lose markers silently (MLS-5) |
| **IMU** (MLS gyro) | SDO 2034h:3 at 50 Hz, 50 ms timeout, via `idle()` | No age watchdog; consecutive-miss backoff; stamp checked 1 in 10 | Readiness 0.2 + 0.6 s grace | EKF falls back to wheels; LINE silently falls back to odom yaw | 5 s probe then **blocking start()**; paused in drive FAULT | None (diag level always OK); LOC_STREAM_STALE in NAV | test_mls_imu good on backoff/short frames | Blocks the bus; frozen/sign undetected |
| **Drives + encoders** (2× BLV-R) | CiA-402 RPDO1 50 Hz, TPDO1/2 20 ms | `silent_nodes` 0.6 s, `missing_feedback`, EMCY latch; operational needs TPDO < 50 ms | Mux 0.3 s via `operational`; latch 0.6 s | Zero per node, R04 standstill confirmation, else heartbeat withheld → 1016h | **None in practice**: no caller of `/drives/ack_fault`; EMCY latch never cleared | DRIVE_* events only; standing row misses link fault | R04–R07 excellent | Detection good; recovery and reporting broken |
| **CAN bus / CANable** | slcan 125 k, txqueuelen 1000, host-only units | None (slcan has no error state) | — | A send exception kills the bus thread → BASE_EXITED | amr.service BindsTo can0 and is not restarted on replug | BASE_EXITED (unnamed) | Only ENOBUFS on the fault-stop path | Brittle |
| **DIO panel** | Modbus TCP 192.168.1.30, 50 ms scan | Age of last good scan ≤ 0.2 s + connected (panel_io.py:103-113) | ~0.22 s | All motion zero; power released at once (arm_policy.py:62-63) | 0.5 s fixed backoff after **any** single failure; edge re-baseline | PANEL_STALE; loss event under INFO code PANEL; LINE shows ESTOP or "wait" | test_panel/panel_io/dio good; sim panel injection exists | Authority is good; reporting is wrong; pendant has no release-first |
| **RFID** (CF821) | TCP 192.168.1.200:2022 **behind the DIO island** | comms_ok = connection only; keepalive ~2-3 s; heartbeat off; no overdue | Line 1.5 s if rfid_node silent | Immediate `rfid` hold (Start) on tag missions; nothing for U-turn-only | 2 s reconnect; re-baseline **swallows any tag** | RFID_LINK_LOST | Decoder good; reconnect untested | **High risk**: silent missed stops |
| **Platform** (supervisor, launch, systemd) | — | systemd watchdog 30 s on the loop thread; `required()` exits | Lease stops ≤ 0.3 s if the loop wedges | BASE/LAYER_EXITED; web respawn ×3 per lifetime | Service restart | BASE_EXITED (no node name), SERVICE_CRASHED (wrong text) | test_hardness, one sim kill | Watchdog kill bypasses teardown; ROS-time clocks |
| **HMI link** | Wi-Fi tablet → werkzeug :5001 | SUPERVISOR_DOWN after 5 s | Jog 0.2 s server window | Jog drops to zero | Web respawn | — | test_operator_ux | Wi-Fi powersave is on (critic M6) |

---

## 3. Cross-cutting design: one device-health model

**Recommendation:** add a pure, clock-fed `agv_core/health.py`. It imports neither ROS nor Flask (same style as `gating.py` and `run_fsm.py`). It is the **reporting and correlation layer only**. Enforcement stays where it is now: the gate, drive_node, job and executor.

### 3.1 States
| State | Meaning |
|---|---|
| `ABSENT` | never seen since start |
| `OK` | fresh and plausible |
| `DEGRADED` | fresh but reduced (SDO fallback, rate below nominal, odom-yaw fallback, TPDO gaps counted) |
| `STALE` | age > `stale_after` (3–4 nominal periods) |
| `FROZEN` | samples arrive but fail a plausibility rule while moving |
| `LOST` | age > `lost_after`, or transport down |
| `RECONNECTING` | re-init in progress (rediscovery job chain, socket reconnect, re-baseline) |

### 3.2 Rules
1. **Monotonic time everywhere.** Use `time.monotonic()` for receive stamps and STEADY_TIME clocks for motion-path timers. An age is valid only when `0 ≤ age ≤ limit`; a negative age counts as stale. Header stamps stay on ROS time.
2. **Codes are message fields.** New `amr_interfaces/DeviceHealth.msg`:
   - Fields: `device` (uint8 enum, append-only), `state` (uint8), `code` (catalogue key), `age_s`, `since_s`, `gaps`, `rejects`, `reconnects`, `timeouts`, `detail`.
   - Published at 2 Hz and on every edge by the owner node: drive_node (drive_left, drive_right, can_bus, mls_track, mls_gyro), panel_node (dio), rfid_node (rfid), and the scanner watch wrapper (scanner).
   - No consumer parses strings. That includes DriveStatus state names: export the constants from `bus_health` (missed gap, X2).
3. **One cause policy.** `CAUSE_POLICY: cause → (code, auto_resumes)` lives in `agv_core/alarms.py`. Both `job._AUTO_CAUSES` and the executor's auto set import it. A test asserts that catalogue `clears_by == 'auto'` holds exactly when `auto_resumes` is true. Proposed policy: **only `field` auto-resumes** (plus `estop` behind its existing flag). Every device-dropout cause needs a fresh physical Start. Open question 1.
4. **Reconnect = backoff + incremental re-init.**
   - A pure `Backoff` (2/4/8…30 s cap, reset on the first good sample).
   - Re-init as **one SDO per poll**, each with timeout ≤ 0.05 s, reusing `mls_track._Job` (mls_track.py:383-425).
   - No multi-read blocking `start()` once the drives can be armed. Only the pre-loop boot start keeps 0.4 s.
5. **Frozen detection only ever stops; it never grants.**
   - MLS: identical-payload run length, diagnostic first.
   - Gyro: disagreement with wheel yaw while turning, plus a sign check.
   - Drives: 6064h delta vs 606Ch.
   - Scanner: `is_valid[]` (phase 2).
   - RFID: tag overdue by wheel travel.
   - DIO: COM0 monitor / FX3 Q3 plausibility.
6. **Common-cause correlator.** A pure `classify(states, window=1 s)` collapses related losses:
   - drives ×2 + MLS → `CAN_BUS_LOST`
   - DIO + RFID → `IO_ISLAND_LOST` (shared enp2s0 cable, dio.py:50-56)
   - MLS track + gyro → one `MLS_OFFLINE`
7. **Web.** `standing()` reads DeviceHealth plus the existing snapshot. Every row checks its own source age. Correlated rows suppress duplicates. `headline()` handles LINE.
8. **Freshness config.**
   - The gate limits come from a profile `freshness` block with hard ceilings: cmd ≤ 0.2, lease/permit ≤ 0.3 (X10).
   - Consumer limits must be ≥ the gate's limits, pinned by a contract test.
   - Health thresholds derive from each device's nominal period, which is already in the profile.

### 3.3 Adoption
1. `CAUSE_POLICY` (P0-2) and the clock rules (P0-7).
2. `health.py` plus the message (P1-1), adopted owner by owner: drive_node → panel_node → rfid_node → scanner wrapper.
3. Web rows (P1-2). Existing per-node checks stay in place as enforcement.

---

## 4. Phased work plan

**Legend**
- **Build:** R = service restart (Python, symlinked). C = `colcon build` (amr_interfaces or a new entry point). D = deploy file + `install.sh` + `daemon-reload`, off-shift.
- **Verify:** U = unit/contract test. S = opt-in sim (sequential, own ROS_DOMAIN_ID). V = operator-witnessed on the vehicle.
- **SAFETY-LISTED** = the item touches `gating.py`, `cmd_mux_kinematics_node.py`, `scanner_fields.*` or `guard.py`. Its failure mode, recovery path and required test are stated with it.

### P0: missed stops, AUTO without Start, invariant breaks, silent dead ends

| ID | Title (sources) | Files | Change | Test | Eff | Build | Verify |
|---|---|---|---|---|---|---|---|
| P0-1 | **RFID encounter integrity** (RFID-1, heartbeat-ahead race [missed, high], RFID-5) | agv_core/drivers/rfid.py:416-417, 545, 615-622, 663-668; amr_base/rfid_node.py:89-91; amr_line/tape_run.py:220-225; tag_table.py:136-140; line_follow_node.py:425-429; new amr_line/rfid_intake.py | (a) Store `_connected_at`. Swallow a read after a drop only if `tag == _encounter_tag and now-_connected_at ≤ RFID_TAG_CLEAR_S`. Always clear `_rebaseline` on the first read. Set `_rebaseline` in `_drop` only if `was` is True. (b) `_heartbeat` calls `_poll()` first, then publishes `encounter_seq=self._sent`. (c) `tape_run.scan` and `TagOdometer` advance the cursor only to the highest n consumed. Status seq > cursor with nothing pending → `_lose_link('encounter missing')`. (d) Seed `_generation = random.getrandbits(31)`. In the intake helper, a seq decrease within one generation counts as a generation change. Move the `_on_rfid` merge into pure `rfid_intake.py`. | test_rfid: A,drop,B → B counted; A,drop,A inside window → none; outside window → one; failed connect then A → one; two links have different generations. test_rfid_intake: hb ahead of encounter; seq regression clears. test_mission_engine: status seq=N with empty buffer, then N delivered → stop applied. Update `EXPECTED_CHECKS`. | M | R | U, V |
| P0-2 | **Single hold-cause policy** (MLS-4, X4, X11 [raised to medium], SCN-6, DIO-3, DIO-7) | job.py:31-35, 73, 367-397, 417-429, 521-583; agv_core/alarms.py:67-87, 408-420; line_follow_node.py:534-551, 706-716; route_executor_node.py:426-430, 985-991; test_arm_policy.py:40; RUNBOOK.md §4 | `CAUSE_POLICY` in agv_core/alarms.py with `_AUTO_CAUSES = ('field',)`. `_prerequisite` returns `(cause, device, text)`: `track/rate` only when the track check fails; panel/lease/drives/field-unknown → `pending`. Split `Inputs.panel_valid` into `panel_fresh` (grace kept) and `panel_image_valid`. A fresh `valid=False` while ARMED/RUNNING holds at once with cause `panel` (non-auto, never relabelled `estop`), new PANEL_LINK_LOST (start_button, IP from `config.DIO_IP`). Line `pending` maps to a new LINE_PREREQ_LOST (start_button). `WAITING_FOR_PREREQ` stays as is: the executor's `pending` really does auto-clear. In `_hold_tick`, reclassify an auto hold to `pending` when `field_clear is None`. Executor: torque loss within the safety window after a panel invalid → `panel`, not `estop`. Add `hold_device` to LineState/RunState. Docstring and RUNBOOK regenerated. **Behaviour change: needs your confirmation (Q1).** | test_alarms: `clears_by=='auto'` ⇔ auto cause, both products. test_line_authority: `rate`/`track` holds stay held 10 s with `wants_power()` False and resume only via start_edge + premove; panel invalid + track bad → `pending`; fresh `valid=False` → immediate `panel`; field None + bad track → `pending`, no auto-resume. test_route_executor_logic: panel blip + torque_off → `panel`, Start needed. | M | C (hold_device) | U |
| P0-3 | **MLS silent → HOLD `sensor`, not DONE** (MLS-1, X1, `sensor_age` None [missed], blind travel [missed]) | job.py:523, 531-534, 675-677, 766-777; line_follow_node.py:548, 694; agv_core/alarms.py | In `_run`, before steering: if `i.sensor_age_s is None or > SENSOR_TIMEOUT_S` → `_hold('sensor', 'line sensor (MLS) silent X s')`, drop_high, output (0,0). Non-auto. `line_lost` stays DONE. Pass `sensor_age_s=self.reader.age_s(now)` with None preserved (Inputs Optional); publish `sample_age_s=-1` when unknown. Add a job-level `_blind_m` accumulator: wheel travel while markers_ok is False or the sensor is stale, including the decel into a hold. It survives HOLD (unlike `tape.marker_lost`, cleared at job.py:523). On a marker run, blind_m > 0 makes the resume Start-gated with WARN "markers may have been missed over X m"; above a bound → FAULT. HOLD_CODES `sensor → MLS_SILENT` (start_button). autopilot.py untouched. | test_line_authority: age 0.2 → HOLD `sensor`, (0,0), same `job.tape` object; 10 s fresh without Start stays HOLD; Start → ARMED. `sensor_age_s=None` while RUNNING → hold. Marker run with 0.15 m blind → Start-gated WARN; > bound → FAULT. Fixture derives the age from `track_cause`. | S | R | U, V |
| P0-4 | **LINE reacts to a drive-link FAULT** (D2, X3) | line_follow_node.py:504-509, 551-567; job.py:90-118, 274-278, 509-536, 612-640, 755; drive_node.py:802-812 | Add `drives_operational` and `drive_link` to Inputs. While RUNNING, ARMED or in a powered HOLD: link ≠ `armed` → immediate cause `drive_fault`. It is non-auto, overrides drives/track/rate holds the same way torque loss does, drops `wants_power`, and needs Reset/Start + premove. Never advance tape distance on commanded travel unless counts are valid AND drives operational (job.py:755). drive_node reports a node with stale TPDO1 as state `stale`; check `readiness.STANDBY_STATES` still behaves. | test_line_authority: link `fault` + cached "Operation enabled" → hold `drive_fault`, (0,0), `wants_power` False; link back `armed` → still held until start_edge. test_mission_engine: counts None + not operational → tape distance constant. test_r07: silent node's published state ≠ "Operation enabled". | M | R | U, V |
| P0-5 | **Drive fault: a working clear and an honest standing row** (D1, EMCY latch [missed, high], D3/X2, release() bypass [missed]) | drive_node.py:253, 592-599, 813, 894; canopen.py:434, 625-682, 715-736, 1032-1033, 1137-1142; supervisor_node.py:489-523; amr_web/alarms.py:34-46; agv_core/alarms.py:96-110; RUNBOOK.md:718-721; DriveStatus.msg; test_demo.py:181 | **After P0-4.** (1) In `prepare()`, after preflight reads 1001h==0 and a non-fault statusword, clear that node's `t.alarm` plus cached statusword/error_register, and emit a DRIVE_ALARM cleared edge. (2) Supervisor Recover calls `/drives/ack_fault` only when LineState is not ARMED/RUNNING/powered-HOLD and RunState is inactive, and returns the Trigger text verbatim (incl. "cleanup owed"). (3) A `release()` failure goes through `fault()` semantics (sets `t_fault`, stop_pending), and `_wait_standstill` returns unconfirmed nodes as failures, so R04 confirmation and heartbeat withholding apply. (4) Web: `drives.link=='fault'` → DRIVE_SILENT/DRIVE_FAULT row that suppresses DRIVES_NOT_READY. Append `fault_code`/`fault_detail` (side-labelled) to DriveStatus. (5) Use the `alarm.get('name')` key. (6) Fix test fixtures to `'FAULT'`. Fix the action text: "power-cycle, wait for boot, then Recover". | test_canopen/test_r05: alarm latched → disarm + prepare with 1001h=0 → decide in STANDBY ≠ fault. release() SDO failure → t_fault set, stop_unconfirmed populated. Supervisor admission refuses while LineState RUNNING. test_operator_ux: link fault + "Operation enabled" → DRIVE_SILENT top row, no DRIVES_NOT_READY. Contract test: state strings built from `decode_state()`. | M | C | U, V |
| P0-6 | **Bound all sensor work on the bus thread** (MLS-3, IMU-1, IMU rediscovery start() [missed, high], nested IMU polls [missed], MLS steady-state stall [missed], A4) | mls_track.py:228-332, 367; mls_imu.py:163-188, 233-261; canopen.py:463-512; drive_node.py:384-386, 432, 533-576 | (a) Add `DriveLink.read_sensor(node, idx, sub, timeout≤0.05)`: no `idle()` call inside, no nested IMU poll. All MLS/IMU reads go through it. (b) MLS and IMU rediscovery become one-read-per-poll `_Job` chains (enable → comm → count → entries / variant → 1800h → snapshot). The blocking `start()` only runs pre-loop. (c) `link.nmt_nowait()` for MLS NMT Start; the drive arm/disarm `nmt()` is unchanged. (d) One sensor-SDO token per loop iteration, shared by MlsTrack and MlsImu. IMU timeout ~0.012 s. Gyro and stamp never read in the same poll. Back off on miss **rate** (≥3 of last 10), not on consecutive misses. (e) Record loop period max/overruns (> cmd_timeout_s) into can/bus diagnostics, report-only. Drive stop/teardown reads keep 0.2–0.4 s. | test_mls_track / test_mls_imu with a fake clock that advances by the timeout: every read issued from `poll()` has timeout ≤ 0.05 and there is ≤ 1 sensor SDO per loop iteration, on the rediscovery path too; 1-of-3 flapping sensor reaches backoff within 10 polls; `nmt_nowait` is not followed by a pump; no nested `idle()`. | M-L | R | U, V (overrun counter over one shift) |
| P0-7 | **Monotonic freshness in the mux** (PL-2) **SAFETY-LISTED** | cmd_mux_kinematics_node.py:271-283; gating.py:254-255; panel_node.py:105; line_follow_node.py:279; route_executor_node.py:344; supervisor_node.py:233; deploy chrony drop-in | `_now()=time.monotonic()`. `_fresh` requires `0 ≤ now-t ≤ limit`. Motion-path timers use `Clock(clock_type=ClockType.STEADY_TIME)`. Version a chrony drop-in in `amr_ws/deploy/` that steps only at start (ExecStartPre `chronyc makestep` before amr.service), checked by validate.sh. **Failure mode:** a backward wall-clock step stalls ROS-time ticks for D s; on resume, a sample stamped just before the step reads fresh for up to its limit, extending a stale command. **After:** ticks are immune to steps and a future stamp is stale, so output goes to zero. **Recovery:** the next fresh sample restores selection; nothing latched; AUTO entry rules unchanged. | test_gating: future-stamped lease/panel/permit/line/teleop → not fresh, NONE. Contract test: mux `_now` uses `time.monotonic`; the listed nodes pass a STEADY clock to `create_timer`. | S | R (D for chrony) | U |
| P0-8 | **Executor field state ages** (SCN-2) | route_executor_node.py:212, 411-430, 578-597, 1053; navigation_layer.launch.py; test_launch_layers.py | Store `(status, t_recv)`. Add `field_fresh_s` 0.5 and a `field_source` param (`scanner` real, `assume_clear` sim, passed by launch). `_field_state(now)` returns True/False/None. `_safety_cause` gives `field` only on a fresh False, otherwise `estop`. Auto-resume requires a fresh True. "protective field state unknown" goes into `_prereqs()` → EXEC_PREREQ_LOST after grace. | test_auto_resume: True, then silence past 0.5 s → torque drop gives `estop`; `field` hold does not resume while unknown; EXECUTING with unknown fields FAULTs after grace; `assume_clear` unaffected. test_launch_layers: real → `scanner`. | S | R | U |
| P0-9 | **Missed-tag backstops** (RFID-3, work_mode mismatch [missed]) | agv_core/mission.py; tape_run.py:139-149, advance(); job.py:212-218; rfid.py:250-263; line_follow_node.py:425-433; StationDetection.msg; agv_core/alarms.py | Mission key `max_tag_gap_m`, with schema, validation and refusal test in tests/test_mission.py. TapeRun integrates encoder travel while RUNNING, reset on any mission-row encounter and on depart. Exceeding the bound → hold `rfid_overdue` (non-auto, drop_high) with RFID_TAG_OVERDUE. Append `bool work_mode_mismatch` (true only when params were read AND work_mode ≠ expected; None does not refuse). `_rfid_check` refuses a needs_link Start: "reader not in active mode". | test_mission_engine: 3.5 m with no tag at max 3 → HOLD `rfid_overdue`; an encounter resets; mismatch → Start refused; unread params → not refused. test_mission: refuse ≤ 0. | M | C | U, V |
| P0-10 | **Pendant release-before-use** (DIO-2) **needs your sign-off (Q2)** | agv_core/panel_io.py:160-206; test_panel_io.py:142-153 | In PanelAdapter only (gating.py and DebouncedLevels unchanged). Each pendant direction must read LOW once after (a) baseline/reconnect, (b) published mode becoming MANUAL (including via the coincidence guard), (c) a Manual Arm rising edge. Until then it is published as idle, with note "pendant FWD held at enable: release it". The pendant has no enabling device and the FX3 does not see it (io-mapping.md:123). | test_panel_io: FWD held at baseline / after reconnect / at MANUAL entry / at Arm rising → idle until released and pressed again; a normal held jog is unaffected; an opposing pair still cancels. Replace test_panel_io.py:142. | S | R | U, V |

### P1: recovery and diagnosability

| ID | Title (sources) | Files | Change | Test | Eff | Build | Verify |
|---|---|---|---|---|---|---|---|
| P1-1 | **`agv_core/health.py` + DeviceHealth** (§3; MLS-6, IMU-4, X7, D3 step 2, A6) | new agv_core/health.py; DeviceHealth.msg; drive_node.py:836-969; panel_node.py; rfid_node.py; agv_core/alarms.py | Pure state machine, Backoff and `classify()`. drive_node: `MlsTrack.diagnostic()` → `_edge('mls', …)`, distinguishing MLS_SILENT from MLS_NO_STREAM; real imu/mls level from (mode, age) with `sample_age_s`, `consecutive_misses`, `frozen`; IMU_STALE edge only when the gyro is stale and the track is alive; supply/temperature warn/trip `_edge` events (A6). `marker_status_text` → "no sensor" when SDO is also silent. New catalogue rows: MLS_SILENT, MLS_NO_STREAM, IMU_STALE, IMU_FROZEN, IMU_UNCALIBRATED, CAN_BUS_LOST, IO_ISLAND_LOST. | Pure transition tables; `classify()` (drives×2 + MLS → CAN_BUS_LOST; DIO+RFID → IO_ISLAND_LOST); catalogue test lists every new code; `EXPECTED_CHECKS`. | L | C | U, V |
| P1-2 | **HMI truthfulness** (X8, PL-7, A1, DIO-4, DIO-6) | amr_web/alarms.py:19-160; readiness.py:222-245; ModeState.msg; LineState.msg; panel_node.py:154-157, 222; io.html:22 | `headline()` for LINE: RUNNING; HOLD by a new `LineState.auto_resume`; FAULT → needs you. ModeState `base_missing` string, published continuously, with one WARN event per falling edge. `base_missing` also requires `panel_valid` and drives operational, not just topic receipt (A1). `standing()` stands DeviceHealth codes and BASE_NOT_READY, and never shows "The vehicle is fine" next to NOT READY. `IoImage.comms_ok = PanelAdapter.source_fresh(snap)`; Scan-age tile bad above `PANEL_SOURCE_MAX_AGE_S`. Structured `DIO_LINK_LOST/OK` code on PanelFrame instead of substring matching. PANEL_STALE IP from config. DIO+RFID shared-cable hint. | test_operator_ux: LINE HOLD rfid → STOPPED (needs you); field hold → WILL_RESUME; `base_missing` named row; IoImage fresh verdict; readiness: DIO dead at boot → base not ready. | M | C | U |
| P1-3 | **CAN TX errors and MLS exceptions don't kill the base** (D4/X5, MLS-8, IMU-9, D10) | canopen.py:250-266, 468-471, 893-902; drive_node.py:412-415, 536-576, 894 | Catch `can.CanOperationError` per tick, classified by errno. ENOBUFS/EAGAIN → count, then `link.fault('CAN TX failed')` once failures last > driver_timeout_s, CAN_TX_FAILED. A dropped setpoint is never retried and the heartbeat is never back-filled, so 1016h still trips if sends keep failing. ENODEV/ENETDOWN still exits. Wrap `track.poll` (count `mls_errors`, rediscover after REDISCOVER_S). `idle_errors` + `last_idle_error` in imu diag. Per-COB decode-error counter. | Fake bus raising ENOBUFS N ticks → no raise, `tx_errors==N`, still ARMED; past timeout → decide 'fault'. nmt raising once → contained, no retry before backoff. Short TPDO → counter. | M | R | U |
| P1-4 | **CAN adapter replug + unclean death** (PL-4/X6/D11, PL-1, OOM [missed], kiosk fallback [missed]) | amr_ws/deploy/amr.service:4-8, 37-48; new amr-orphan-stop.sh; canable@.service + 70-canable.rules copied into deploy; supervisor_node.py shutdown/main; agv_core/alarms.py:330-334; alpha-setup.md:473, 1260 | `WantedBy=sys-subsystem-net-devices-can0.device` (minimal fix; an in-process bus reconnect is a large rework, so ask before doing it). canable@ `Restart=on-failure, RestartSec=2`. `WatchdogSignal=SIGINT`. `faulthandler.dump_traceback` when the shutdown lock is not acquired. `ExecStop=amr-orphan-stop.sh`: no-op while `$MAINPID` is alive, otherwise SIGINT the remaining cgroup pgids (base first, using a new atomic `$AMR_STATE_DIR/groups.json`), wait ≤ 12 s. `OOMPolicy=continue`; `OOMScoreAdjust` protects supervisor and base. `OnFailure=amr-failed.service` writes a static "call engineer" page served by nginx. Reword SERVICE_CRASHED (8130h possible). Watch for StartLimitBurst parking the unit in failed; validate.sh reports `is-failed`. | Contract test parses amr.service (WantedBy, WatchdogSignal, ExecStop, OOMPolicy). Script picks pgids from a fake cgroup. Shutdown with lock held dumps a traceback. validate.sh checks units match the repo. | M | D | U, V (replug only when idle) |
| P1-5 | **Supervisor and launch liveness** (PL-3, PL-5, PL-6 partially, executor-wedge [missed]) | supervisor_node.py:152, 233, 1008, 1027-1033, 1061-1080; launch_helpers.py:22-31; base.launch.py:167-186; description.launch.py:50-61; navigation_layer.launch.py:164-223; imu_bias_node.py | Reset the respawn counter after 120 s of uptime; give-up goes through `notify.status` + error log. `_reap` reads `exits/<role>.json`, written by `required()`'s OnProcessExit (or tails the role log), so BASE_EXITED names the node and its exit code. robot_state_publisher and imu_bias `respawn=True, delay 2`. Lifecycle managers and commissioning_node use `required()`. imu_bias drops non-finite gyro_z. Executor heartbeat timer: if older than 2 s, skip WATCHDOG=1 and publish lease `allowed=0` (removes authority only). | test_hardness: long-lived web respawns forever, 3 quick deaths give up; planted exits file → node name in reason; stale executor heartbeat → no WATCHDOG, lease not allowed. test_launch_layers walk: every Node required or respawn (allow-list). test_gyro_bias: NaN/inf ignored. | S-M | R | U |
| P1-6 | **Drive cleanup and recovery timing** (D7, D9) | canopen.py:655-668, 727-736, 780-829; drive_node.py:502-508 | `_wait_standstill` drops only nodes whose SDO read **timed out** and keeps waiting up to 4 s on nodes that answer (a Shutdown while still turning would coast). Compute `retry_at` from monotonic time **after** the action, with a capped backoff of 2/4/8…30 s. Write 1017h in `prepare()` inside the rollback, non-fatal. | test_r05: node 2 dead → `disarm(force)` ≤ ~1.5 s; an answering moving node still waited on; no cleanup reissued before backoff. test_canopen: prepare writes 1017h, a refusal does not abort. | S | R | U |
| P1-7 | **IMU coupling and plausibility** (IMU-2, IMU-3 partially, gyro_sign [missed], IMU-6) | canopen.py:465; drive_node.py:148, 513-542; mls_imu.py:107-120, 255; new amr_localization/gyro_check.py; imu_bias_node.py; line_follow_node.py:487 | Resume IMU polling in FAULT only once `stop_pending` and `stop_unconfirmed` are empty; until then diag WARN "paused: drive fault". StampTracker latches `frozen` on a duplicate stamp; `stamp_every` 5, within the P0-6 budget. Pure `gyro_check`, active only while |wheel vyaw| > 0.1 rad/s for 1 s: disagreement or opposite sign → `suspect`, and imu_bias withholds `/imu/data`. Never flag a constant reading at rest (the sensor is quantisation-limited at 0.061°/s). Bias bound 0.5°/s. LINE curve guard uses `max(|gyro|,|odom|)`, so it can only drop HIGH earlier. Close the `gyro_sign` VERIFY with an operator-witnessed CCW spin recorded in the profile. | test_mls_imu: duplicate stamp → no samples until it advances. test_gyro_check: gyro 0 vs wheel 0.3 rad/s → suspect; opposite sign → suspect; still + constant → OK. test_gyro_bias: 1°/s window refused. FAULT + stop_pending → no IMU SDO. | M | R (C if new entry point) | U, V |
| P1-8 | **MLS marker gaps + frozen diagnostics** (MLS-5, MLS-2 partially, MLS-10) | mls_track.py:285-303, 339-341; marker_events.py:73-79; marker_reader.py:37-45; MarkerEvent.msg | Read the 1800h:05 event timer once via a queued job. `dt` > 3× period → `tpdo_gaps`, `max_gap_ms`, `gap=True`, and `MarkerDetector.break_continuity()`. Append a `gaps` counter to MarkerEvent so the existing tape_run marker_lost/drop_high handling applies. Disable the gap detector while stationary if transmission is on-change. Phase 1 frozen: run length of byte-identical TPDO1 (`max_identical_s`) in diag + tape_health log only. Phase 2 (after a vehicle log) applies a stop rule. Add `rejected_frames` / `reserved_nlcp` counters. | test_mls_track: 40 ms hole → tpdo_gaps 1, stale_periods unchanged; identical frames → `max_identical_s` grows; 6-byte frame → `rejected_frames` 1. test_marker_events: hole + gap → no event across it. test_marker_run: gaps increase → marker_lost. | M | C | U, V |
| P1-9 | **Bench tool ownership** (MLS-7) | agv_core/drivers/canbus/read_mls.py:946-990 | Take ownerlock `can` for every mode except `calibrate --step report`. The `--nmt` exit restore only runs under the lock. | tests/test_mls.py: OwnerBusy → snapshot/poll/stream return 2, `open_bus` never called. | S | — | U |
| P1-10 | **DIO transport tolerance** ([missed] immediate reconnect) | agv_core/drivers/dio.py:222-247 | After a failure that follows a good scan, try one immediate reconnect + scan on a fresh client. Apply `reconnect_period_s` only if that also fails. Re-baseline is unchanged (any gap > 0.2 s still invalidates). Add a drops-in-10-min counter. | test_dio: fail once then answer → next good scan within ~1 scan period; two failures → 0.5 s wait. | S | R | U, V (errors/shift) |
| P1-11 | **RFID detection and reporting** (RFID-2 partially, RFID-4, RFID-6, RFID-8, RFID-9, RFID-10) | rfid.py:49-52, 507-523, 577-582; rfid_node.py:55-91; StationDetection.msg; profiles/agv-01.json:74; job.py:214 | Fix the docstring for the DIO-chained topology. `except TimeoutError as e: if e.errno is None: continue; raise`. `_comms_ok` also requires the thread to be alive; `_drop` goes inside a try. Edge-triggered rosout WARN/INFO for comms_ok/detail/heartbeat/config mismatch; append `detail`, `heartbeat_state`, `bad_frames`, `silent`. U-turn-only missions: IDLE note + track-hold suffix "RFID link down – U-turn tag not read". **Only after the 0x0050 bench check:** `heartbeat_s 0.5`, `heartbeat_timeout_s 1.5`. Do **not** turn "unanswered" into a Start refusal unless a new `rfid.heartbeat_required` key (with schema + refusal test) is set. | test_rfid: ETIMEDOUT → drop; plain timeout → continue; dead thread → comms_ok False. Edge-logger helper test. test_mission_engine: U-turn-only + comms False → reason mentions RFID. | M | C | U, V |
| P1-12 | **Scanner recovery + diagnosis** (SCN-1 partially, SCN-3 phase 1, SCN-4) | scanner.launch.py:30-31; new amr_bringup scanner_watch wrapper (pure `scanner_watch.py`); amr_web/alarms.py; agv_core/alarms.py:113-165; mapping_session_node.py:140-217 | **After the bench check (Q3)**: a wrapper launches `sick_safetyscanners2_node`, watches `/scan` age, and exits non-zero after N s of silence (profile key with schema + refusal test). respawn re-sends CoLa2. Backoff 5/10/20/60; SCANNER_RESTART event. Do not kill by PID hunting. Web derives SCANNER_CONTAMINATED / SCANNER_ERROR from the driver's `/diagnostics` "State", ranked above FIELD_PROTECTIVE ("clean window / call engineer"). Mapping: pure gap function in `_check_slam` gives "NO SCAN for X s: the map has a gap here" and SURVEY_SCAN_GAP once per gap. | Pure watch: stale → exit, backoff sequence, reset on fresh, no action when the driver is absent. alarms: contamination diag → row above FIELD_PROTECTIVE. Gap fn: 0.1 ok, 2.0 flagged once, fresh clears. | M | C | U, V |
| P1-13 | **Scanner index/polarity single source** ([missed]) **SAFETY-LISTED (scanner_fields.*)** | line_follow_node.py:127; route_executor_node.py:212; line_layer.launch.py:85-95; navigation_layer.launch.py; scanner_fields.py:36-94 | Pass protective path index and polarity from `scanner_fields.load()` into line_follow and the executor the same way as the mux (base.launch.py:147), and apply polarity in both. **Failure mode:** a Safety Designer change updates only the yaml, and line/executor read the wrong path or inverted polarity, so hold causes and auto-resume decisions use the wrong evidence. **Recovery:** after the fix, a yaml change reaches all three consumers; a validation error refuses launch. OSSD unaffected. | test_launch_layers: mux, line_follow and executor get identical index and polarity. scanner_fields validation test unchanged/extended. | S | R | U |
| P1-14 | **Odometry under STO** (ODO-1 partially) | readiness.py:174-232; localization_monitor_node.py:255-269 | **First confirm on the vehicle** whether BLV-R TPDOs stop under STO; an operator observation of `/wheel_states` during a field trip counts. If they stop: while wheels are excused, latch odom_base and go LOST ("odometry extrapolating with wheels silent") on > 0.05 m / 2°. No fabricated zero twist. | test_readiness: excused + drift over limit → LOST; still → READY. | S | R | U, V |
| P1-15 | **Event forensics independent of the web** (critic A2, A3) | adapter.py:291; eventlog.py:36-53; new event recorder in amr_bringup or supervisor | Move EventLog writing into the supervisor process (always up; subscribes to `/amr/events` with depth ≥ 200, transient-local where the publisher allows). The web reads the file. Surface `EventLog.errors` > 0 as an EVENTLOG_WRITE_FAILED row. | Recorder unit test with a fake fs failure → counter + row. | M | R | U |

### P2: durability and long-run

| ID | Title (sources) | Files | Change | Eff | Build | Verify |
|---|---|---|---|---|---|---|
| P2-1 | Freshness + drive-link tunables into profile (X10, D12, IMU-8) **SAFETY-LISTED (gating, cmd_mux)** | config.py; profiles/agv-01.json; tests/test_config.py; cmd_mux_kinematics_node.py:138-163; drive_node.py:134-150; mls_imu.py:163-168 | `freshness` block. Refuse cmd > 0.2, lease/permit > 0.3, `pc_heartbeat ≥ pc_loss/2`, `pc_loss == 0`, `driver_timeout ≤ 3 TPDO periods`. Wire `imu.poll_period_s`, `retry_period_s`, new `sdo_timeout_s` (0.005–0.02). Fix the IMU docstring (config.py:364-382). **No value changes.** **Failure mode:** a future edit widens a gate limit, or tightens a consumer below the gate, unnoticed. **Recovery:** the profile is refused at load. Contract test: gate ≤ consumer limits. Update `EXPECTED_CHECKS`. | M | R | U |
| P2-2 | Drive odometry bridge + plausibility (D6 partially, D8, M9) | canopen.py:153-178, 411-427, 1036-1065; diff_drive_odom_node.py:77-96 | Bridge staleness-only gaps (same epoch/scale, \|Δ\| ≤ MAX_RPM·gap·cpr/60·1.2, gap < 0.5 s). Gap/rebaseline counters. 6064h-delta vs 606Ch check → invalidate that wheel + DRIVE_FEEDBACK_IMPLAUSIBLE. Skip commanded-vs-actual (false trips on ramps). Update the R06 contract test deliberately. | M | R | U |
| P2-3 | MLS reboot/variant + tapeloc (MLS-9 partially, MLS-11 partially) | mls_track.py; line_follow_node.py:358; tape_runtime.py:107-113 | Route 0x70A **read-only**: on boot-up → reboots++, `new_generation()`, queue a 2006h:01 re-read, markers untrusted until done. **No MLS 1017h write.** Pass `nlcp=None` when stale; abort open fork episodes with a WARN. | S | R | U |
| P2-4 | Scanner config hygiene (SCN-5, SCN-7 optional, monitoring case [missed], stop budget [missed]) **SAFETY-LISTED (cmd_mux, scanner_fields)** | scanner.launch.py; nanoscan3.yaml:14-28; agv_core/alarms.py:115-123; line_follow_node.py:133; lidarframe.py:37; validate.sh | Launch builds addresses from `LIDAR_*` profile keys. Fix text (sensor .10, host .2:6060, ~17 Hz). Remove `lidar_scan.py` references. validate.sh checks enp3s0 has LIDAR_HOST_IP. Optional depth=1 on `/output_paths` (it trims superseded samples; it does **not** bound stamp error; contract test on QoS). SCANNER_CASE_MISMATCH reporting row. Measure the `/output_paths` p99 inter-arrival on the vehicle and document it against `stop_m` (0.07 m budget); no speed-cap change. **Failure mode:** stale config/explanation; recovery: config fix. | S | R | U, V |
| P2-5 | CAN host config tracked (D5 partially, A8, critic M4/M5) | deploy/canable@.service, 70-canable.rules, install.sh, validate.sh; amr.service | Measure the TX backlog on the bench first, then txqueuelen ≈ measured peak + margin (~64). Monitor-slot SDO RTT in diagnostics, warn-only. Fail loudly on the netdev timeout. USB power/control=on in udev. `RuntimeWatchdogSec=30` + iTCO/sp5100 wdt module + `kernel.panic=10` (M4). Scheduling: `CPUSchedulingPolicy`/`Nice` for the base group or `os.sched_setscheduler` on the drive bus thread only, measured with the P0-6 overrun counter first (M5). | M | D | V |
| P2-6 | Supply/brownout (critic M3, A5) | config.py:927-929; agv-01.json:179-188; canmon.py:139-160; amr.js:93-97 | `bus_v_trip_low` key (schema + refusal test). Warn/trip `_edge` events (with P1-1). Header chip warn/bad class. Refuse a new run below trip_low (refusal only). Document the 1 s sag-detection latency while moving. | S | R | U |
| P2-7 | Logs, disk, storage (PL-9 partially, PL-8 partially, critic M10) | amr-supervisor.sh; logs.py; disk.py:47; supervisor_node.py:576, 1149-1159 | `ROS_LOG_DIR=$AMR_STATE_DIR/ros_log` + boot sweep. Add `/` to the watch. Check ST_RDONLY via `os.statvfs` → DISK_READONLY row. Stop web/foxglove in parallel after base with 2/1/1 s budgets; unit test that the summed budget < TimeoutStopSec − 10. | S | R/D | U |
| P2-8 | DIO hardware-side (DIO-1 partially, DIO-5, DIO-8, critic M8, M12) | panel_io.py; dio.py:34-35, 124-125; profiles/agv-01.json:80-130, 153; RUNBOOK.md:165-194 | FX3 Q3/DI08 vs DI02 plausibility → PANEL_INPUT_FAULT (removes authority only), or DI07 as COM0 monitor (Q5). Module comm-loss output reset, if the CK-EC5163 supports it, set in module config. Correct the docstrings. RUNBOOK DI03..DI06, 66 %. Name DO0/DO1 horn sounds. DO8 in `motion_on` and `num_do` 16 vs 12 relays: **ask, don't change** (Q5). Optional DO→DI loopback toggled per scan to prove the I/O path end to end. | S-M | R | U, V |
| P2-9 | HMI link (critic M6, A7) | NetworkManager conf (tracked in deploy); web_node.py:33 | `wifi.powersave = 2` (off) on wlp1s0. Socket timeout on the werkzeug server (`request_handler.timeout`) to stop thread pile-up from a roaming tablet. | S | D/R | V |
| P2-10 | Connector-health trend + black box | health.py consumers; new recorder | Daily per-device counters (TPDO gaps, SDO timeouts, Modbus errors, RFID reconnects, scan gaps) → `reports/health/<date>.jsonl` with a weekly delta warning. Low-rate ring of `_ui` copies + DeviceHealth, dumped for 60 s on any ERROR edge. | M | C | V |
| P2-11 | IMU TPDO path (IMU-5 partially) | mls_imu.py:176-241 | **Deferred until the gyro TPDO is commissioned; a precondition of that write.** Frame-age stale after 3 periods → SDO fallback; single NMT owner for node 10 (`mls_track._nmt_start`); re-read mapping on recovery. | S | R | U |

### P3: test and fault-injection infrastructure

| ID | Title | Change | Eff | Verify |
|---|---|---|---|---|
| P3-1 | Pure tests first (MLS-12, RFID-11, X9 pure half, TEST-1 pure half) | Job fixture derives `sensor_age_s` from `track_cause`. Catalogue ↔ CAUSE_POLICY guard. RFID reconnect session test (server closes, re-accepts, new tag counted, same tag not duplicated) + hb-race test. Extract drive_node tick into a pure step for send-failure / FAULT-IMU-poll tests. FakeLink timeout-budget tests (P0-6). DriveStatus state strings built from `decode_state()`. | M | U |
| P3-2 | Sim fault hooks | Per fake, `~/mute` and `~/freeze` (SetBool): fake_base (drive N silent / fault / frozen counts / wheels invalid; `cmd_timeout_s` 0.25 → **0.2**), fake_panel (stop / valid=false), tape_synth (`dropout_at_s`, `dropout_s`, `freeze_at_s`, `rate_hz_after`), fake_imu (`drop_after_s`, `freeze_after_s`, `start_delay_s`), new fake_output_paths (clear / drop / violate / invalid) and fake_rfid/marker publishers. All guarded so they never launch with `real:=true` (asserted in test_launch_layers). | M | S |
| P3-3 | Opt-in sim cases (once per milestone, sequential, own ROS_DOMAIN_ID, never while the vehicle runs) | `test_tape_dropout_holds_sensor` (HOLD sensor, zero, no resume until fake Start). `test_field_stream_drop` (FIELD_UNKNOWN zero ≤ 0.6 s, no AUTO after resume until Start). `test_silent_drive_offers_power_cycle` (DRIVE_SILENT row, no DRIVES_NOT_READY). `test_stale_panel_names_panel`. `test_imu_drop_reaches_operator` (LOST + IMU code, run FAULT without motion). `test_rfid_drop_holds`. Mechanism checks only; no tolerance tuning. | M | S |

---

## 5. Fault-injection and vehicle acceptance (operator-run, after the fixes)

**Rules:**
- The operator runs every step. The service owner approves each step.
- Area is clear, FX3 is live, the E-stop is in reach, and the vehicle is on blocks wherever motion is not the point.
- No CAN frames or bench tools while amr.service holds can0.
- A step the operator saw pass counts as PASS; no re-runs for instrumentation.

| Device | Injection | Expected (post-fix) |
|---|---|---|
| **nanoScan3** | Pull the Ethernet at the scanner in LINE AUTO | AUTO zero ≤ 0.6 s (FIELD_UNKNOWN). LINE hold `pending`/LINE_PREREQ_LOST, needs Start. SCANNER_STALE ≤ 5 s. Replug → data returns, no motion until Start |
| | Power-cycle **only** the scanner, service running, idle | Answers Q3. With P1-12: data returns via SCANNER_RESTART without a service restart |
| | Cover/smudge the window (test target) | SCANNER_CONTAMINATED above FIELD_PROTECTIVE; not "continues by itself" |
| | Person into protective field in NAV, then pull the cable while held | Executor hold becomes `estop` (Start), never auto-resumes on stale True |
| **MLS track** | Unplug MLS connector mid-straight, LINE RUNNING | Zero ≤ ~0.15 s. HOLD `sensor`, MLS_SILENT, mission progress kept. Replug → stays held. Start → premove → continues. Overrun counter shows no tick > 0.2 s |
| | Same, on a marker run's back leg over Home | Blind-travel WARN or FAULT, never a silent pass past Home |
| | Unplug in IDLE, then MANUAL jog for 30 s | MLS_SILENT standing. Jog stops ≤ 0.2 s after release (no stall during rediscovery) |
| | Lift sensor off tape (tape lost, sensor alive) | `line_lost` → DONE / TRACK_LOST, distinct from MLS_SILENT |
| **IMU** | Covered by MLS unplug in NAVIGATION READY | IMU_STALE (not a bare LOC_STREAM_STALE). Run FAULTs, no motion. Re-localize + Start needed |
| | Drive fault (unplug one drive) in NAV | imu diag "paused: drive fault". Gyro resumes after stop confirmation |
| | CCW spin in MANUAL | Gyro sign positive; record it to close `gyro_sign` VERIFY |
| **Drives** | Unplug right drive CAN while ARMED (LINE) | Mux zero ≤ 0.3 s. DRIVE_SILENT naming right. Line hold `drive_fault`. Power-cycle → Recover → STANDBY (no re-fault from EMCY latch). Nothing moves until Start + premove |
| | Power-cycle drives at idle | One DRIVE_SILENT. Recover clears it. No back-to-back 6 s cleanups (bus diag stays live) |
| | Observe `/wheel_states` during a field trip | Answers ODO-1 (TPDOs stop under STO or not) |
| **CAN adapter** | USB replug, vehicle idle, service running | amr.service returns active ≤ 30 s into IDLE. Drives need a power cycle for 8130h. HMI shows reason, not a browser error |
| **DIO** | Pull island Ethernet 1 s during LINE run | Zero ≤ 0.22 s. Hold `panel` / PANEL_LINK_LOST (not ESTOP, not "wait"). Start resumes. IO page shows LOST |
| | ~100 ms blip (port flap) | Reconnect within ~1 scan. Run may still stop (by design); card names DIO |
| | Pendant FWD held while switching Manual Arm on | No motion until FWD is released and pressed again (P0-10) |
| **RFID** | Pull reader cable approaching a stop at NORMAL | Hold within detection latency (~2 s now, ~0.5 s once the heartbeat is enabled). After reconnect + Start, the **next** stop tag is honoured (not swallowed) |
| | Power-cycle reader parked at a station | Same tag not double-counted; a different tag counted |
| | Cover the Home tag | RFID_TAG_OVERDUE hold after `max_tag_gap_m`, not a drive to tape end |
| | Reader in answer mode (bench config) | Start refused: "reader not in active mode" |
| **Platform** | `systemctl kill -s SIGKILL` supervisor main PID (bench, after P1-4) | ExecStop disarms base; no 8130h. Journal has traceback/teardown. SERVICE_CRASHED text correct |
| | Kill web 4× spread over hours | Respawns every time |

---

## 6. What you might have missed in the prompt (from critic)

| # | Item | Why it matters here | Plan |
|---|---|---|---|
| M2 | FX3 / E-stop / STO state not wired to the PC | Software guesses E-stop from torque loss (job.py:398-402); this causes the DIO-3 and executor mislabels. Spare DIs exist | Q5; would replace guessing in P0-2 |
| M3/A5 | Battery/brownout: no `trip_low`, chip has no colouring, no event | Sag lengthens stopping distance (canmon.py:139-141) | P2-6 |
| M4 | No hardware watchdog (`RuntimeWatchdogSec` unset, no wdt module, `kernel.panic=0`) | A kernel hang leaves the vehicle dark with 8130h until a power cycle | P2-5 |
| M5 | No scheduling priority for the motion path on a 4-core N97 shared with slam/Nav2 | Loop already near 20 ms (config.py:169-171) | P2-5 (measure first, P0-6) |
| M6 | Wi-Fi powersave on; jog 100 ms vs 0.2 s window | Fail-safe but stuttering jog | P2-9 |
| M7/A1 | Boot readiness passes with MLS/scanner/RFID absent or DIO dead (readiness.py:233-245) | "Ready" lies at boot | P1-2 |
| M8 | Frozen-but-alive I/O island (stale image, Modbus answering) | Only short images refused (panel.py:90-95) | P2-8 loopback |
| M9 | No command-vs-measured tracking (blocked wheel, being pushed) | Distinct from frozen feedback | P2-2 (warn-only first) |
| M10 | Read-only remount undetected (disk.py:47) | Map saves/events lost silently | P2-7 |
| A2/A3 | Events lost while web is down; EventLog errors never surfaced | Forensics gap exactly when things break | P1-15 |
| A4 | Bus-loop overrun measured and thrown away (drive_node.py:574-575) | The one number proving P0-6 on the vehicle | P0-6(e) |
| A7 | werkzeug dev server, no request timeout | Thread pile-up on a roaming tablet | P2-9 |
| M13 | Common-cause losses (RFID behind DIO; drives+MLS+IMU on can0) | One fault raises 2–4 unrelated alarms | P1-1 `classify()` |
| — | Clean on re-check: bounded deques, sockets closed, fixed threads, counter wrap, NIC carrier stable | No items needed | — |

---

## 7. Open questions (only the ones that change the plan)

1. **Hold-cause policy (P0-2).** Should only `field` (and `estop` behind its flag) auto-resume? Then `track`, `rate`, `drives` and every device-dropout cause would need a fresh physical Start in both products. That reverses the documented design at job.py:31-35 and the RUNBOOK. P0-2 assumes yes.
2. **Pendant release-first (P0-10).** May it override the documented "a pendant held at power-on is meant to drive" rule (panel.py:141-148)? There is no enabling device and the FX3 does not see the pendant.
3. **Bench session, service stopped.** These results gate P1-11, P1-12, P1-14, P2-5 and P0-6 tuning:
   - Does a scanner power cycle keep the CoLa2 UDP output settings?
   - Is the scanner's 24 V switched by the E-stop chain? If yes, SCN-1 becomes high.
   - Does 0x0050 interleave with RFID active streaming?
   - Is MLS 1800h:05 a 10 ms timer or on-change?
   - What is the slcan TX backlog during arm/disarm?
4. **Drive-fault clear act and one-drive loss.** Recover through the supervisor admission (P0-5 assumes this), or the physical Reset, or an engineer PIN? And when only one drive goes silent, should the healthy drive still be forced into 8130h by heartbeat withholding (canopen.py:863-867)?
5. **I/O hardware.**
   - Can spare DIs take FX3 status outputs (E-stop / reset-required)?
   - What does FX3 Q3 → DI08 encode?
   - Is DO8 in `motion_on` a planned beacon or should it be removed?
   - Should `num_do` be 12, to match the CK-EC5163?
   - Does that module have a comm-loss output reset?