# Coding plan: line following (engineering), U11 retirement, plant-layout background

> **Status 2026-09-19:** plan only. Nothing is built, restarted or deleted yet.
> Decisions below were taken with the operator on 2026-09-19.

## Context

- **Line following.** Magnetic-tape following was retired on 2026-09-06
  (`8403dcb`: `core/autopilot.py`, `branch.py`, `read_mls.py`, `/auto`). The
  SICK MLS (node 10) stayed on the vehicle for its gyro. We now want tape
  following back as an **engineering feature**:
  - read the track, run a PID and drive;
  - **not shown in the UI**; the only visible part is the LCP reading on the
    Monitor page.
- **U11** (retire the legacy `canworker`/`app`/`main.py` and old units) is still
  open. It has to be planned so that it does not remove anything the line
  follower needs, and so the line follower does not revive legacy code.
- **Plant-layout background.** Aligning a SLAM map with the plant CAD by eye is
  slow. The route editor should take a JPG/PNG of the layout as a background,
  aligned by hand:
  - rotation as a number;
  - translation by dragging;
  - scale from two clicked points and a real distance;
  - a lock/unlock button.

**Decisions already taken (2026-09-19):**

| Topic | Decision |
|---|---|
| Line-follow authority | Armed from the CLI in IDLE. A new exclusive LINE lease class. Drives only under selector AUTO after a fresh physical Start. |
| Layout storage | Per map revision, outside the hashed bundle, with "copy from rev N" |
| U11 timing | Prepared fully now. Deletion still waits for the U10 witnessed session. |

**Facts this plan relies on (checked in the code):**
- MLS TPDO1 is COB `0x18A`:
  - bytes 0–5 are LCP1–LCP3 (INT16, mm);
  - byte 6 bits 0–2 are `#LCP`, bits 3–7 the marker;
  - byte 7 is the status.
  - Variant `2006h:01` was commissioned to 0 (Standard), so the fields are
    plain INT16.
  - With one tape, **LCP2** is the track.
  - The decoder is in `git show 8403dcb^:drivers/canbus/read_mls.py`.
- The PID with speed-scaled Kp (`Kp = K_RATIO·v`, damping independent of v)
  and its lateral-rate clamp are in `git show 8403dcb^:core/autopilot.py`.
- `amr_base/drive_node.py` owns the bus. `canopen.Router.add(cob, fn)` routes
  frames; `mls_imu.py` already reads node 10 this way.
- Nothing registers `0x18A` today, so track frames are dropped.
- The Monitor page renders `/diagnostics` generically, key by key
  (`templates/monitor.html`). A new `DiagnosticStatus` shows up there with **no
  UI change**.
- Mux authority (`amr_base/gating.py:select`):
  - AUTO is honoured only with `LEASE_AUTONOMOUS`;
  - the supervisor grants that only in NAVIGATION (`supervisor_node._allowed`);
  - commissioning is an exclusive IDLE substate with its own lease class. That
    is the pattern to copy.
- The only root modules the ROS stack still imports:
  - `config`, `kinematics`, `guard`, `rpdo`, `read_imu`, `alarms`, `bus_health`;
  - `drive_forward`, `verify_drivers`, `panel`, `dio`, `ownerlock`, `canmon`,
    `blindrun`.
- `tests/run_all.py` pins 15 modules and `EXPECTED_CHECKS = 869`.

---

## Part 1 — Line following (engineering only)

### 1.1 Sensor reading (no motion). Ship this first; it is useful alone.
- **Restore `drivers/canbus/read_mls.py`** from `8403dcb^`, trimmed:
  - keep the decoders (`decode_tpdo1`, `decode_lcp`, `decode_status`,
    `decode_marker`, `NLCP_MEANING`) and the read-only `snapshot`/`stream` CLI;
  - drop anything tied to the old tape-following runtime.
  - It stays free of `config`, like the rest of `canbus/`.
  - The shape follows `read_imu.py` / `mls_imu.py`.
- **New `amr_base/amr_base/mls_track.py`**, a pure module:
  - imports `read_mls` through `amr_base.agv_repo`;
  - `MlsTrack(link, node)` checks at start that TPDO1 `1800h:01` is enabled and
    reads the variant `2006h:01`;
  - it only reads, never writes;
  - it registers `0x180+node` on the router and keeps the latest decoded sample
    with its receive time;
  - it counts samples, stale periods and variant mismatches.
- **`drive_node.py`**:
  - builds `MlsTrack` next to `MlsImu`;
  - publishes a new `amr_interfaces/msg/LineTrack.msg` on `/amr/line_track`
    (sensor QoS, every sample). Fields: `lcp_mm[3]`, `nlcp`, `valid` (which
    LCPs), `line_good`, `track_level`, `polarity`, `marker`, `stamp`.
  - It adds `DiagnosticStatus(name="mls/track")`. Keys: `nlcp` with its label,
    `lcp1_mm`, `lcp2_mm`, `lcp3_mm`, `line_good`, `track_level`, `polarity`,
    `marker`, `age_ms`, `samples`, `variant`. Level: WARN when there is no
    track, ERROR when stale.
  - This is the **only** UI surface: it appears on Monitor automatically.
- **Vehicle check before any code relies on it:**
  - `candump can0,18A:7FF` shows frames once the drive owner runs;
  - if the sensor is still Pre-operational, TPDOs do not flow. Decide on the
    vehicle between an NMT start addressed to node 10 only (as `read_mls stream`
    did) and SDO polling of the LCP objects. Record the answer in the doc.
  - Gyro TPDO `1806h` plus track TPDO1 is 2 of the MLS's 4 TPDO slots: OK.
- **Vehicle result 2026-09-19 (Gate B1), no NMT sent:**
  - there are no `0x18A` frames and no node-10 heartbeat, so the MLS is in
    Pre-operational;
  - `drive_node` (`mls_track_mode: auto`) fell back to SDO polling:
    `/amr/line_track` runs at 9.8 Hz and `mls/track` shows on Monitor;
  - with two tapes under the sensor it reads LCP1 −61 mm, LCP2 +33 mm, `#LCP 3`
    (diverter, LCP1+LCP2), line good, level 7, north.
  - **Open decision (operator):** NMT start for node 10 only (TPDO1 at control
    rate, needed by the line follower), or stay on SDO (Monitor only).

### 1.2 Controller (pure)
- **New `amr_base/amr_base/line_follow.py`**, ported from the old
  `LineFollower` and cut down to what was asked: PID only.
  - `ω = −(Kp·e + Ki·∫e + Kd·ḋ_filt)` with `Kp = k_ratio·v`;
  - a speed ramp to `v_base`;
  - the derivative is primed from the first sample (no reset kick);
  - `LATERAL_RATE_FACTOR` rate clamp against track swaps;
  - integral clamp;
  - `predicted_zeta()` for commissioning.
- **Out of scope:** junction ladder (`branch.py`), RFID stations, measured
  stops at tags, per-run CSV logging.
- **Track selection:** LCP2 always, per MLS table 17. With `#LCP` 3/6/7
  (diverter) it keeps LCP2 and flags `diverter` in the state.
- **Stop rules** (the output goes to zero, then the mux ramp stops the vehicle):
  - track lost (`#LCP == 0` or `!line_good`) for more than `lost_distance_m`
    (0.05) of travel → DONE "track lost";
  - a sensor sample older than `sample_age_s` (0.1) → FAULT.
- **Parameters** are node parameters in `base.launch.py`, not the profile
  (policy, not hardware):
  - `v_base_mps` 0.20; `v_max_mps` 0.30 (hard cap);
  - `k_ratio`, `ki`, `kd`, `d_filter_hz`;
  - `sensor_lookahead_m` (measure it);
  - `sign` (cable-outlet convention, `2027h`).
  - Starting values come from the last committed profile before `8403dcb`.

### 1.3 Authority (new, the same pattern as commissioning)
- **New node `amr_base/line_follow_node.py`, in the base launch** (it runs in
  IDLE):
  - `/amr/line_follow/arm` and `/amr/line_follow/disarm` (Trigger). There is no
    web endpoint.
  - It publishes `/amr/line_follow_state` (new `LineFollowState.msg`: state
    `IDLE|ARMED|RUNNING|DONE|FAULT`, reason, `e_mm`, `v`, `w`, `nlcp`), latched
    and at 2 Hz.
  - It publishes `/amr/line_cmd` (a stamped twist with the generation) only
    while RUNNING.
- **Arming:**
  - allowed only in mode IDLE, with no commissioning job, and with a fresh track
    sample showing `#LCP ≥ 2` (a track under the sensor);
  - ARMED → RUNNING on a **fresh physical Start edge** under selector AUTO;
  - it drops back to IDLE on the selector going to MANUAL, Reset, disarm, a
    stale panel or lease, or drives that stop being operational.
- **Supervisor (`supervisor_node.py`, `mode_fsm.py`):**
  - new `LEASE_LINE = 8`;
  - while `line_follow_state ∈ {ARMED, RUNNING}` in IDLE, `_allowed()` returns
    `LEASE_LINE` only (exclusive, like `LEASE_COMMISSIONING`);
  - mode requests are refused with BUSY until the line follower is disarmed.
- **Mux (`gating.py`, `cmd_mux_kinematics_node.py`):**
  - new source `LINE = 7`;
  - under AUTO, `lease.allowed & LEASE_LINE` and a fresh `/amr/line_cmd` of the
    lease generation → `Selection(LINE, v, w)`, with v capped at
    `line_v_max_mps` (0.30);
  - LINE is not a manual source, so it gets the autonomous ramp (`slew_asym`);
  - `drive_gate` needs no change (it checks `allowed != 0` and the generation).
- **Safety note (goes in the RUNBOOK):**
  - no software obstacle check runs during line following;
  - only the nanoScan3 protective field and E-stop, the same as the old tape
    AGV;
  - hence the 0.30 m/s cap and "person at the E-stop" for first runs.

### 1.4 Engineering use (RUNBOOK, CLI only)
```bash
ros2 topic echo /amr/line_track --once        # or the Monitor page: "mls/track"
ros2 service call /amr/line_follow/arm std_srvs/srv/Trigger
# selector AUTO -> press Start; stops at end of tape / selector MANUAL / Reset
ros2 service call /amr/line_follow/disarm std_srvs/srv/Trigger
```

---

## Part 2 — U11, with line following in mind

The U10 gate stays: everything below except the **deletions** can be prepared
now. Deletion happens after the witnessed U10 session, and then only K1's
boot/ownership part is repeated.

### 2.1 Keep (the line follower and ROS stack need these)
- MLS hardware and TPDO1 `0x18A`. Update the `read_imu.py` comment that says
  "retiring tape-following frees that slot".
- `drivers/canbus/read_mls.py` (restored in Part 1), `read_imu.py`, `verify_drivers.py`.
- `drive_forward.py`, `guard.py`, `rpdo.py`, `alarms.py`, `bus_health.py`.
- `config.py` and `profiles/`.
- `core/kinematics.py`, `panel.py`, `ownerlock.py`, `canmon.py`, `blindrun.py`.
- `drivers/dio.py`, `drivers/rfid.py` (RFID stays for later station work).
- Import audit rule: every module on this list must be imported by
  `amr_ws/src/**` or a kept tool, or be moved to the delete list with a
  reason.

### 2.2 Delete (after U10), with an audit step for each
- `canworker.py`, `main.py`, `app/` (templates, static, fonts).
- Deploy files: `amr_ws/deploy/amr_nav.service`, `amr_mapping.service`,
  `amr_legacy.env`, their entries in `install.sh`/`validate.sh` `UNITS`, and
  any installed copies on the PC (`systemctl disable`, then remove the unit
  files).
- Root modules left with no importer after the audit (candidates: `core/events.py`, `core/health.py`,
  `core/motion.py`, `core/lidarframe.py`, `core/runlog.py`, `drivers/lidar_scan.py`, `drivers/modbus_io.py`).
  Delete one only after `grep` over `amr_ws/src`, `drivers/` and `tests/`
  finds no importer.
- Archive, don't delete: `logs/00xx-auto_*` → `manuals/obsolete/legacy-runs/`, plus a
  git tag `legacy-final` on the last commit where `canworker` is deployable.
- Documentation:
  - README status banner: tape following is back as an **engineering feature
    in the ROS stack**, not as the legacy `/auto` page;
  - `hardware-reconciliation.md` note;
  - the ownership table in the spec.
- Mention the line follower nowhere in the operator pages.

### 2.3 Legacy test migration (`tests/run_all.py`)
| Module | After U11 |
|---|---|
| `test_canworker`, `test_web`, `test_layout` | Removed along with `canworker`/`app`. Shared-library checks inside them are moved first (see below). |
| `test_lidar` | Keep if `lidarframe` is kept, else remove with it (audit). |
| `test_config`, `test_invariants`, `test_health`, `test_canmon`, `test_lss`, `test_imu`, `test_rpdo`, `test_rfid`, `test_dio`, `test_panel`, `test_blindrun` | Keep. Drop cases that only assert legacy wiring. |
| **new `test_mls`** | Decoders of the restored `read_mls.py`: Standard/Combi LCP, `#LCP` table, status bits, short frame. |
- `EXPECTED_CHECKS`: recompute once, with a comment in `run_all.py` listing
  checks removed (legacy only), checks moved (shared libs) and checks added
  (`test_mls`). Never lower it without that note.
- Before deleting: move any case in `test_canworker`/`test_web` that exercises
  a kept library (`guard`, `kinematics`, `panel`, DIO) into that library's test
  module.

### 2.4 U11 exit gate (from the unified plan, plus line following)
- A clean checkout builds and tests;
- `amr.service` is the only AGV boot service;
- no process, import or device owner comes from legacy;
- `mls/track` is visible on Monitor, and `/amr/line_follow/arm` works on the
  accepted build.

---

## Part 3 — Plant-layout background in the route editor

### 3.1 Storage (robot side, per revision, not in the bundle)
- Path: `maps_dir/<map_id>/layouts/rev<N>/image.png|jpg` + `layout.json`.
  - `list_revisions` only matches `rev\d+`, so the sibling `layouts/` directory
    is invisible to it;
  - bundle `verify` never sees it;
  - saving a layout therefore never creates a revision.
- `layout.json`:
  - `image` (name), `sha256`, `width_px`, `height_px`;
  - `m_per_px`;
  - `yaw_deg` (image rotation in the map frame);
  - `x_m`, `y_m` (the image **centre** in map metres);
  - `opacity`, `locked`;
  - `updated`, `calibration` (`{p1_px, p2_px, meters}`, from the last
    two-point scale).
- **New `amr_navigation/amr_navigation/layout.py`** (pure):
  - load, save and validate the JSON, with bounds: finite values,
    `1e-5 ≤ m_per_px ≤ 1`, `|x|, |y| ≤ 10 km`;
  - `px_to_world` / `world_to_px`, the reference for the JS;
  - image header sniffing without PIL: PNG IHDR / JPEG SOF for the size;
  - limits: 15 MB, 60 MP, PNG/JPEG magic only;
  - atomic writes (tmp + rename).
- **`server.py` endpoints:**
  - `GET/PUT /api/maps/<id>/<rev>/layout` (the JSON; PUT refuses while locked
    unless the body sets `locked:false`);
  - `POST /api/maps/<id>/<rev>/layout/image` (upload; resets the transform to
    "fit to map");
  - `GET …/layout/image`;
  - `POST …/layout/copy {from_rev}`;
  - `DELETE …/layout`.
  - The map revision must exist and verify; nothing else is touched.

### 3.2 Editor UI (`editor.html`, `editor.js`, `mapview.js`)
- A new **Layout** block in the tools column:
  - file input (png/jpg) and **Upload**;
  - **Rotation °** number input;
  - opacity slider (0.2–0.9, default 0.5);
  - **Move** tool (translation by dragging the image);
  - **Scale** tool: click point 1, click point 2 on the image, type metres,
    press **Apply**. The scale becomes `metres / pixel distance`, and point 1
    stays fixed in the world;
  - **Lock / Unlock**; **Copy from rev…**; **Remove**.
  - Locked: all layout inputs are disabled, and drags and clicks go to the
    route tools again.
- **Drawing:** `MapView` draws the layout **after the map image, before the
  dynamic hatch and overlays**, at `opacity`, with the image-to-world transform:
  `world = [x, y] + R(yaw)·m_per_px·[u − w/2, −(v − h/2)]`.
  The route and footprints stay on top.
- **Saving:**
  - each committed change (drag end, rotation change, scale applied, lock
    toggle) PUTs the JSON;
  - the reply is applied only if the map token still matches (the editor's
    `mapToken` pattern);
  - a map or revision change loads that revision's layout, or none.
- The layout is a visual aid only. Validation, the executor and maps ignore
  it, and the editor hint says so.
- `test_server.py::test_every_element_id…` covers the new ids.

---

## Part 4 — Test list updates (everything above)

### 4.1 New and changed automated tests
| Package / file | Tests |
|---|---|
| `tests/test_mls.py` (root, new) | TPDO1 Standard vs Combi decoding; `#LCP` → valid LCPs; status and marker bits; frames under 8 bytes → None |
| `amr_base/test/test_mls_track.py` (new) | Router registration on `0x18A`; latest sample and age; disabled-TPDO1 start reports `off`; diagnostic keys and levels (no track → WARN, stale → ERROR) |
| `amr_base/test/test_line_follow.py` (new) | Kp scales with v (ζ does not depend on v); derivative primed without a kick; a track swap is rate-clamped; integral clamp; track lost over 0.05 m → DONE; stale sample → FAULT; v ≤ `v_max` |
| `amr_base/test/test_line_follow_node.py` (new, logic, `__new__` pattern) | Arm refused outside IDLE, during commissioning and with no track; RUNNING only on a fresh Start under AUTO; MANUAL, Reset, disarm or stale lease → IDLE with zero command |
| `amr_base/test/test_gating.py` (extend) | LINE is selected only with `LEASE_LINE` + AUTO + a fresh `/amr/line_cmd` of the lease generation; capped at `line_v_max`; the LINE lease does not admit MANUAL, FOLLOW or COMMISSIONING, and the other leases do not admit LINE |
| `amr_bringup/test/test_mode_fsm.py`, supervisor tests (extend) | `_allowed()` is `LEASE_LINE` only while ARMED/RUNNING in IDLE; mode requests get BUSY meanwhile |
| `amr_bringup/test/test_launch_layers.py` (extend) | The base launch composes `line_follow_node` once, in both sim and real; the navigation layer does not |
| `amr_navigation/test/test_layout.py` (new) | JSON bounds; `px↔world` round trip with rotation and a non-square image; two-point scale keeps point 1 fixed; PNG/JPEG header sizes; oversize and wrong-magic uploads refused |
| `amr_web/test/test_server.py` (extend) | Layout upload → GET → PUT transform → lock refuses a PUT → unlock → copy to another revision → delete; the map bundle hash is unchanged throughout; `/editor` ids |
| `amr_web/test/layout_harness.js` + `test_layout_js.py` (new, editor-harness style) | Move-drag changes x/y by the world delta; rotation input sets `yaw_deg`; scale from two clicks and metres; locked → drags go to the route tools and no PUT is sent; a stale-token reply is ignored |
| `amr_sim` (only if cheap) | `scan_synth`-style fake MLS publishing `/amr/line_track` along a sine tape, for one closed-loop sim test of the node with the fake base. Optional; add only if the pure tests leave a gap. |
| Root `tests/run_all.py` | Add `test_mls`; remove the legacy modules at U11; recompute `EXPECTED_CHECKS` with the migration note |

### 4.2 Unified-plan verification tables (`unified_amr_service_plan.md` §12)
- **P9 (new core automated scenario):**
  - line-follow authority: pure gating + supervisor fixtures, as above;
  - a LINE command without the lease, under MANUAL, or from another generation
    is zero.
- **K6 (new hardware check, in the U10 session or right after it):**
  - `mls/track` live on Monitor over the tape (LCP2 sign matches the side);
  - arm, then AUTO + Start: follows 5 m of straight tape and one gentle curve
    at 0.20 m/s;
  - lifting the tape end or switching to MANUAL stops the vehicle;
  - a person stays at the E-stop;
  - record lateral error and ζ.
- **K7 (browser check):** upload the plant CAD PNG, scale from a known span,
  rotate, drag, lock, reload the page (it persists), and switch revision (none,
  or a copied layout).
- **§12.4 trigger row:** "tape junction/diverter behaviour" is deferred until
  junction logic is in scope.

---

## Order of work
1. Part 1.1 (sensor reading plus Monitor) with `test_mls` and
   `test_mls_track`, then the vehicle `candump` check.
2. Part 3, independent of the rest, testable on the laptop with the demo
   server.
3. Parts 1.2–1.3 (controller, authority) with their tests, then K6 on the
   vehicle.
4. Part 2 prep (audit, test migration branch), then deletion after U10 and
   K1's boot/ownership part.

## Files touched (representative)
- **Root:** `drivers/canbus/read_mls.py` (restored), `drivers/canbus/read_imu.py` (comment),
  `tests/test_mls.py`, `tests/run_all.py`, `README.md`.
- **amr_base:** `mls_track.py`, `line_follow.py`, `line_follow_node.py`,
  `drive_node.py`, `gating.py`, `cmd_mux_kinematics_node.py`, `setup.py`.
- **amr_interfaces:** `LineTrack.msg`, `LineFollowState.msg`, `CMakeLists.txt`.
- **amr_bringup:** `supervisor_node.py`, `mode_fsm.py`, `launch/base.launch.py`.
- **amr_navigation:** `layout.py`. **amr_web:** `server.py`, `editor.html`,
  `editor.js`, `mapview.js`, `amr.css`.
- **Docs:** `amr_ws/RUNBOOK.md` (engineering line-follow section; Routes →
  Layout), `unified_amr_service_plan.md` (U11 pointer, P9/K6/K7),
  `hardware-reconciliation.md`.

## Verification
- **Laptop:**
  - pure tests (`test_mls`, `test_line_follow`, `test_layout`, gating);
  - JS harnesses;
  - `test_server` layout endpoints;
  - the demo server (`python amr_ws/src/amr_web/tools/demo_server.py`) with a
    real CAD PNG, screenshot of the editor.
- **Vehicle:**
  - `colcon build`;
  - `env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test`;
  - `python3 tests/run_all.py`;
  - `candump can0,18A:7FF`;
  - Monitor `mls/track`;
  - K6 on tape.
