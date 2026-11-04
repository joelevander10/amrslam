# Line-following / tracked product removal plan (revised)

Branch `kyb-trackless` · repo `/home/gvipc-evo-01/agv_can` · read-only analysis, 2026-10-21

> **Status (2026-10-30):** P1-P6 are committed on `kyb-trackless` (`62959a6`, `ccd12fb`) and pushed. On gvipc-ce the overlay was clean-rebuilt and the unit reinstalled (P7 build/install); no tracked-era data was present to archive. **D24 is overridden by the owner: `kyb-trackless` (MCQ4-4) and `gvievo-main` are separate projects, so nothing here is merged into `gvievo-main` or `main`, and this vehicle deploys from `kyb-trackless`.** Still open: the D18 tags, the sim tests and the §6 on-vehicle checks, P3b (optional), and the D11 electrical-doc edit after the reader is physically removed.

> **Later (2026-10-30):** the Commission page, the commissioning node, `agv_core/blindrun.py` and the pp backend were removed, and the Status page folded into Home and Monitor (plan `noble-waddling-sutton`). The rows below that keep `commissioning`, `COMMISSIONING` (lease bit 4, mux 5; now retired, never reused), `blind_run` or `pp` describe the P1-P6 state, not today's.

> **Later (2026-11-02):** the blind run is back on `kyb-trackless` as its own page, `/blindrun`: `agv_core/blindrun.py`, the commissioning node, `CommissioningState`/`PlanCommissioning`, lease bit 4 and mux source 5 with their old meaning, and `blind_run.*` in the profile. PV only: the pp backend stays removed. An emergency aborts a running job, like a survey move (owner, 2026-11-02).

---

## 1. Summary

**What goes.** Everything below is the tracked AGV:

- **LINE layer:** the whole `amr_ws/src/amr_line` package (18 modules plus `__init__.py`, 40 tracked files), `line_layer.launch.py`, `p0_localization.launch.py` and `amr_bringup/tape_loc.py`.
- **LINE mode** in the supervisor and mode FSM, the `LEASE_LINE` lease bit, the mux `LINE` source and the line U-turn exemption. The exemption includes the `slow_exempt` parameter of `gating.warning_target` and `gating.field_limit`.
- **MLS tape-track and marker handling:** `amr_base/mls_track.py`, `amr_base/marker_events.py`, `agv_core/drivers/canbus/read_mls.py`, `guard.SENSOR_ALLOWED` and `canopen.write_sensor`.
- **The RFID stack:** `rfid_node`, `agv_core/drivers/rfid*.py`, the `rfid` owner lock, `tools/uhf-even/` (79 tracked files), and every RFID config key and alarm.
- **Tape missions:** `agv_core/mission.py` and `missions/*.json`.
- **Interfaces:** six of them (`LineTrack`, `LineState`, `MarkerEvent`, `StationDetection`, `SelectMission`, `LineTeach`), plus the LINE constants and fields in `ModeState`, `ControlLease`, `MuxState` and `MotionPermit`.
- **Web:** Run tracked, Teach, the Home tracked/trackless switch, the MLS/RFID strips and `/api/line/*`.
- **Profile and config:** the `tracked` flag and the `autopilot`, `rfid`, `mls`, `tape_loc` and `vehicle.sensor_lookahead_m` keys.
- **Tools:** the tape tools and the tape sim (`tape_synth_node`, `tape_plant`).
- **Docs:** the tracked docs and drawings.

**What stays.** The trackless stack in full:

- `amr_mission`, `amr_navigation`, `amr_localization`, `amr_maps`, `amr_description` and the trackless sim.
- `drive_node` as the only can0 owner. It still reads the **MLS gyro**: `mls_imu.py` and `read_imu.py` publish on `/imu/data_raw`, then `imu_bias_node` produces `/imu/data`, which feeds the EKF.
- The DIO island (`dio.py`, `panel_node`, `panel_io.PanelAdapter`), lease, arming and the pre-move horn.
- The mux sources for pendant, browser jog, teleop, commissioning, Nav2 FOLLOW and ROTATE.

**Key coupling risks (all checked in the code)**

| # | Coupling | What I found | How the plan handles it |
|---|---|---|---|
| R1 | **MLS gyro vs. tape track** | `mls_imu.py` imports only `read_imu`, and `mls_track.py` imports only `read_mls`. `read_imu` does not import `read_mls`. `MlsTrack._nmt_start` (`mls_track.py:305-315`, called at `:253` in `start()` and `:349` in `poll()`) is the only code in the running stack that sends NMT Start to node 10. It does so only when TPDO1 (1800h:01) is enabled and `mls_track_mode` is `tpdo`/`auto` (`drive_node.py:155` default `"auto"`). The MLS **boots Operational** (`test_mls_track.py:111`, `mls_track.py:306-310`), so TPDO1 0x18A streams after power-up until a Pre-operational reaches node 10. One example is the bench tool `drive_forward.py:274`, which broadcasts `0x80`. After P3 nothing restarts it. The gyro today is read by **SDO** (1806h disabled, `mls_imu.py:209-211`), and SDO works in Pre-operational, so plain removal does not regress the gyro. `MlsImu` switches to TPDO only if 1806h is enabled and maps 2034h:03 (`mls_imu.py:180-205`). Its `poll()` does nothing in TPDO mode (`:241`), so there is no stale fallback there. | P3 deletes `MlsTrack` with no gyro change, but ports the "reads only, NMT only to node 10, never 0/1/2" assertion into `test_mls_imu.py`. The NMT-start/SDO-fallback port into `MlsImu` (D6) is a **separate commit (P3b) or deferred** until 1806h is actually enabled. The P3 bench check and V2/V3 prove the SDO gyro with node 10 in Pre-operational. `SENSOR_NODE`, `IMU_ENABLED`, `imu_frame`, `gyro_sign`, the `imu/mls` diagnostic and `test_mls_imu.py` all stay. |
| R2 | **Mux line source** | `gating.select(..., line=None)` is the **last** positional parameter (`gating.py:270-274`, marked "APPEND ONLY"). Its LINE branch is at `:321-337`, `AUTO_SOURCES` at `:376`, and `line_uturn_exempt` at `:449-469`. `slow_exempt` is a parameter of both `warning_target` (`:436`, branch `:444-445`) and `field_limit` (`:531`, passed through at `:546`). In the mux node: `/amr/line_cmd` at `:377-391`, `exempt = line_uturn_exempt(...)` at `:429-431`, `field_limit(..., slow_exempt=exempt)` at `:432`, and the LINE `d_max`/yaw overrides at `:520-525`. | Removing the trailing `select()` parameter rebinds nothing. Remove `slow_exempt` from both gating functions and the `:432` call in the same commit, or the 50 Hz `_tick` raises NameError/TypeError and the whole base exits. FOLLOW and ROTATE keep the else-branch slew exactly as today. |
| R3 | **Mode FSM LINE mode** | `mode_fsm.py:18` unpacks the modes positionally with `range(8)`, and LINE=7 is last. | Change to `range(7)`. Values 0..6 stay the same on the wire. 7 is documented as retired. |
| R4 | **Profile `tracked` flag** | `server._product()` (`server.py:208-217`) swallows exceptions and falls back to `"both"`. If `config.TRACKED` is removed **before** the web edit, the tracked pages silently come back. The supervisor (`:140`) builds `fsm.Params(tracked)` from the flag. | Order: supervisor (P1), then web (P2), then config/profile (P5). |
| R5 | **RFID** | No trackless package reads `/amr/rfid` or `StationDetection`. `amr_localization/package.xml:6` only *advertises* a `station_id_node` that does not exist. `ownerlock.acquire()` raises `ValueError` on an unknown resource. | Delete the callers first, then the `rfid` resource. Drop the `station_id_node` mention. |
| R6 | **Shared interfaces** | Interface Python modules are imported eagerly. Seven modules import the tracked msgs/srvs: `cmd_mux`, `drive_node`, `panel_node`, `rfid_node`, `supervisor_node`, `tape_synth_node` and the web adapter. `ModeState.LINE` is dereferenced at import time in `supervisor_node.py:87` and `commissioning_node.py:59`. **Humble has no RIHS type hash** (that arrived in Iron), so a peer with a mismatched definition is not refused. `line_available`/`product` sit mid-message, before `fault_code`, `reason` and `last_survey_*`, so a stale peer decodes the tail as garbage without any error. Stale peers include an old web or foxglove_bridge process, a node from a partial deploy, or bag replay. | The interfaces come out only in P4, after every consumer is gone. P4 needs a clean rebuild **and a full `amr.service` restart** (supervisor, web and foxglove_bridge together). Old bags decode only against the `tracked-final` overlay. |
| R7 | **Strict profile loader** | Unknown and missing keys are both fatal. | `config._SCHEMA` and `profiles/agv-01.json` change in **one** commit, after every reader is gone. |
| R8 | **RUNBOOK §4 is generated** | `tests/test_alarms.py::test_runbook_section_4_matches_the_catalogue` checks byte equality. | Edit `agv_core/alarms.py`, then regenerate with `python3 -m agv_core.alarms --md`. |
| R9 | **LAN1 / DIO link (already moved on gvipc-ce, 2026-10-21)** | On this PC the live netplan is `/etc/netplan/60-lan1-dio-maint.yaml`, which gives **enp1s0 192.168.1.100/24** for the DIO island (192.168.1.40) and maintenance SSH. The old `60-rfid-lan1.yaml` is now `.off`, so netplan ignores it. The repo still only has the old copy, `manuals/obsolete/rfid-setup/60-rfid-lan1.yaml` (enp2s0, 192.168.1.10/24), and the other vehicles may still run that layout. | Commit the three live files (D19). Do not touch the active LAN1 file on this PC. A wrong LAN1 means `PANEL_STALE`: no MANUAL, no AUTO. |
| R10 | **Pinned test counts** | `tests/run_all.py` pins `MODULES` and `EXPECTED_CHECKS = 747`. | Lower the count deliberately, with a dated note, in each phase that removes checks. |
| R11 | **Stale overlay** | colcon never prunes. On this host, `install/amr_base/lib/amr_base/rfid_node` (and an older `web_panel_node`) and `install/amr_sim/lib/amr_sim/tape_synth_node` exist. So does the symlink `install/amr_bringup/share/amr_bringup/launch/line_layer.launch.py`, which points into `build/`. `git rm -r amr_ws/src/amr_line` leaves the ignored `__pycache__` dirs, so the directory survives and still matches `src/*/test`. | V-std does a full clean build (`rm -rf build install log`, as RUNBOOK §7:923 already does). P1 also runs `rm -rf amr_ws/src/amr_line`. Acceptance checks the executables. |
| R12 | **Vehicle branch** | Vehicles pull `gvievo-main` (`alpha-setup.md:62`). The removal lands on `kyb-trackless`. | Merge into `gvievo-main` (and `main`) before P7 (D24). |

---

## 2. Pre-work

### 2.1 Owner decisions (deduplicated, each with a recommended default)

| # | Question | Recommended default |
|---|---|---|
| D1 | Uncommitted working-tree edits: `agv_core/alarms.py` SCANNER_SILENT hint, `amr_ws/RUNBOOK.md` L747, `nanoscan3.yaml`, and `profiles/agv-01.json` `lidar.host_ip` 192.168.3.1 | Commit them as their own commit **before** the tag and the removal, so later diffs stay reviewable. |
| D2 | Boot mode after removal | **IDLE** for every vehicle. This matches `amr.env` and `amr-supervisor.sh`, which are stale today. |
| D3 | Retired wire values: `ModeState.LINE=7`, `MuxState.LINE=7`, `ControlLease.LINE=8`, `MotionPermit.LINE=4` | **Delete the constants** and leave one comment in each `.msg` and in each Python mirror (`# retired: 7 was LINE, never reuse`). Assert "not reused" in `test_mode_fsm` and `test_gating`. |
| D4 | `ModeState.line_available` / `ModeState.product` fields | **Delete both**, with a mandatory full service restart after deploy (R6). Old bags and Foxglove layouts then decode only against the `tracked-final` install. The zero-risk alternative is to keep both as deprecated, always-default fields so the wire layout does not change, and drop only the constants. |
| D5 | Profile `tracked` key | **Delete it.** The strict loader then refuses it as an unknown top-level key, and `test_config` gets a refusal check for it. Reword `gls6-system.md` L25/L41. |
| D6 | MLS NMT Start / gyro TPDO (R1) | **Not part of the removal.** P3 deletes `MlsTrack` without a gyro change, because the current SDO gyro works in Pre-operational. Before 1806h is ever enabled, land as its own commit (P3b) with its own bench test: a rate-limited (2 s) NMT Start addressed to node 10 only, sent in `mode == "tpdo"` when the stream is stale, with an SDO fallback after N stale periods. Moving the gyro to TPDO 0x28A (bench-checklists §3.3) is a later follow-up. |
| D7 | MLS TPDO1 0x18A (about 10 % of the bus, nobody listening after P3) | **Leave 1800h alone.** TPDO1 streams from MLS boot (Operational) until a Pre-operational reaches node 10, such as a bench `drive_forward` broadcast. After P3 nothing restarts it, so whether 0x18A is on the bus depends on the last NMT the sensor saw. Docs must not promise either state. Without a Router handler, the frames reach `sdo_read`'s drain loop and `pump()` (`canopen.py:259-261`). That is harmless. Disabling it for good (1800h:01 bit 31) is a later commissioning write. Because `SENSOR_ALLOWED` is deleted, that write uses a `read_imu`-style `--go` bench path. |
| D8 | MLS temperature (2070h, `MLS_TEMP`) | **Drop it with `mls_track`** and log a follow-up ("MLS/IMU device health"). `MLS_FIELD_WEAK` is tape-only either way. |
| D9 | `guard.check_sensor` mechanism | **Delete it.** If guarded IMU writes are wanted later, add an IMU allow-list (1806h:01, 2032h:01). |
| D10 | Code worth harvesting before deletion (`rfid._hardened()` TCP keepalive, `tape_locator.OdomBuffer`, `trip.py`) | **Delete.** The tag `tracked-final` keeps them, so the GLS driver can copy `_hardened()` from the tag. Do not port `line_at_home`. |
| D11 | RFID reader removed from the trackless product | **Yes**, including `rfid_node` from `base.launch.py`. The electrical docs change only after the reader is physically removed (D19). |
| D12 | `scanner_fields.yaml` `warning_2.stop_m` (0.40, sized for 0.85 m/s HIGH) | **Keep 0.40** (more margin at 0.60 m/s). Reword the comment only. Any value change needs Safety Designer parity. |
| D13 | Web naming | Relabel "Run trackless" to **"Run"**. Keep `/run-trackless` as an alias (kiosk bookmarks). Remove `_need` and all product gating. |
| D14 | `/demo` page | **Keep the page.** Strip the tape copy from `content.json`. |
| D15 | Home live row and freed Manual-page space | One wide **Speed** tile. The jog pad grows (`--lt-h` removed). Check on the 1280×720 panel at ui=l. |
| D16 | `tools/bag_report.py`, `tools/run_log.sh` | **Strip them to trackless.** Keep read_bag, wheels, imu, cmd_wheel_vel, mux, drives, panel, io and events. Add `/amr/run_state` and `/amr/localization_state`. |
| D17 | `manuals/obsolete/` | Delete **only tracked-only archive files** (Phase 6). Keep mixed files that trackless code cites. |
| D18 | Tags | Create `legacy-final` at `aa02ec7` (README and RUNBOOK claim it exists; it does not). Create `tracked-final` at the D1 commit. |
| D19 | Netplan files in the repo | Add `amr_ws/deploy/netplan/` with copies of the live `60-lan1-dio-maint.yaml` (enp1s0 .1.100), `70-lan2-nanoscan3.yaml` (enp2s0 .3.1) and `80-lan3-gls621.yaml` (enp3s0 .4.2). Keep `manuals/obsolete/rfid-setup/60-rfid-lan1.yaml` only as history. If you keep it, add it to the allow-list. On gvipc-ce nothing changes; the old `.off`/`.bak` files in `/etc/netplan` can optionally be deleted. Do any other vehicle still on the enp2s0 layout separately, at a console. |
| D20 | `site-layout-103.md` D1-D14 (home bay, physical Start to leave home) and the HOME-area projection geometry | Copy the product-neutral decisions (D1-D8, D10, D11, D13, D14) and the pillar geometry note into a short `manuals/site-requirements.md` before deleting. That note is also input for placing the GLS label. |
| D21 | Dead keys `imu.poll_period_s`, `imu.retry_period_s`, and `SafetyFieldStatus.msg` | **Leave them.** They are not tracked and are out of scope. Fix only the stale "DISPLAY ONLY" text in the `imu.*` tuning note. |
| D22 | On-vehicle data left by tracked code: `~/amr_maps/<id>/tapegraph/`, `~/.amr/teach/`, `$AMR_STATE_DIR/line_graph.json`, `~/.amr/reports/tape_health/`, `~/.amr/mls_cal/` (`read_mls.CAL_DIR`, L379), the tracked-era `~/.amr/runs/<stamp>/` (from `run_log.sh`), `~/.amr/logs/events.jsonl*` (holds LINE_*/TRACK_*/RFID_*/MLS_* codes, which render as UNKNOWN "Something stopped the vehicle" through `server.py:757` → `alarms.get()`), and `~/.amr/operations.jsonl` (kind `line`, `boot-line-*`) | **Archive it all to a tarball, then purge the tape data and rotate both journals**, during the deploy. Purging `tapegraph/` is safe: it sits beside `rev<N>` and `map_bundle.REV_RE` ignores it. The alternative to rotating is a small `RETIRED_CODES` set in `alarms.get()` returning an INFO row. Not recommended: it leaves a tracked trace in code. |
| D23 | GLS621 address | **Already resolved (2026-10-21).** The GLS is at 192.168.4.1 on enp3s0 (PC 192.168.4.2/24), so there is no clash with the nanoScan3 subnet (192.168.3.0/24). Update `gls6-system.md` and alpha-setup §2.2 to match, in Phase 6. |
| D24 | Which branch the vehicle deploys from | **Overridden 2026-10-30 (owner): `kyb-trackless` and `gvievo-main` are separate projects; no merge, MCQ4-4 deploys from `kyb-trackless`.** Original: Land `remove-tracked` → `kyb-trackless` → **`gvievo-main`** (the vehicle branch, `alpha-setup.md:62`), and into `main`, before P7. Expect conflicts on `gvievo-main` like those on `aop-demo-oct`. Update alpha-setup L62 if the vehicle branch changes. |
| D25 | `0910-fix-plan-001.md` (mixed: A1 PP_VETOED, A2/B3 bag_report, A3 `tools/*.sh` gitignore, B5/F7 sim `can_confirm`, all trackless-relevant) | **Move it to `manuals/obsolete/`** (allow-listed) instead of deleting it. Its citations then stay valid (`readiness.py:62`, `test_unified_sim.py:379`, `.gitignore` L15/L31) once each says "manuals/obsolete/0910-fix-plan-001.md". |

### 2.2 Safety net

```bash
cd /home/gvipc-evo-01/agv_can
git add agv_core/alarms.py amr_ws/RUNBOOK.md amr_ws/src/amr_bringup/config/nanoscan3.yaml profiles/agv-01.json
git commit -m "scanner host 192.168.3.1 (pre-removal WIP)"          # D1
git tag -a legacy-final aa02ec7 -m "last legacy standalone tape controller (slamm-line)"   # D18
git tag -a tracked-final HEAD  -m "last dual-product (tracked + trackless) build"
git push origin legacy-final tracked-final
git switch -c remove-tracked                                         # land phase by phase, merge into kyb-trackless, then gvievo-main (D24)
```

Record a baseline and keep the outputs, so later phases can be compared against it:

```bash
python3 tests/run_all.py            # expect 747
python3 -m pytest -q                # repo root, testpaths=tests (test_alarms etc.)
cd amr_ws && rm -rf build install log && colcon build --symlink-install && source install/setup.bash && source env/vehicle.sh
env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test
AMR_SIM_TESTS=1 ROS_DOMAIN_ID=89 python3 -m pytest -q src/amr_bringup/test/test_unified_sim.py
```

On the vehicle, save a copy of the installed overlay (`amr_ws/install`), `/etc/netplan/` and `~/.amr/` before deploying (RUNBOOK §7 rollback).

---

## 3. Phases

Each phase must pass the **standard verification** before the next one starts:

```bash
# V-std
cd /home/gvipc-evo-01/agv_can
python3 tests/run_all.py && python3 -m pytest -q
ruff check agv_core tests && (cd amr_ws && ruff check src)
cd amr_ws && rm -rf build install log && colcon build --symlink-install && source install/setup.bash && source env/vehicle.sh
env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test
AMR_SIM_TESTS=1 ROS_DOMAIN_ID=89 python3 -m pytest -q src/amr_bringup/test/test_unified_sim.py src/amr_bringup/test/test_fault_injection_sim.py
ls install/*/lib/*/ install/*/share/*/launch/ | grep -E 'rfid|tape|line_layer|p0_loc|web_panel'   # must be empty once the phase that deletes each one has landed
```

Do **not** deploy to the vehicle between P1 and P2. In that window the web still shows Run tracked, which would now get "unknown request". Deploy after P2, P3, P4 and P5 at the latest, from `gvievo-main` (D24).

Every grep below lists its **expected survivors** (items owned by a later phase). Any other hit is a real leftover.

### Phase 1: retire LINE mode and delete the LINE layer (bringup, amr_line, tape sim, tape tools)

**Delete files**

| Path | Why |
|---|---|
| `amr_ws/src/amr_line/` (entire package: 18 modules plus `__init__.py`, `test/`, `setup.py`, `setup.cfg`, `package.xml`, `resource/amr_line`; 40 tracked files) | The LINE layer. Its only importers are `line_layer.launch.py:43` and `tools/p0_report.py:35-36`. After `git rm -r`, also run `rm -rf amr_ws/src/amr_line` to remove the ignored `__pycache__` dirs (R11). Check with `git ls-files amr_ws/src/amr_line \| wc -l` == 0 and `test ! -e amr_ws/src/amr_line`, not with file counts. |
| `amr_ws/src/amr_bringup/launch/line_layer.launch.py` | Only `Supervisor._start_line` spawns it. |
| `amr_ws/src/amr_bringup/launch/p0_localization.launch.py` | Tracked plan-3 measurement tool. |
| `amr_ws/src/amr_bringup/amr_bringup/tape_loc.py` | Imported only by the two launch files above. |
| `amr_ws/src/amr_bringup/test/test_tracked_product.py` | Tests only the tracked flag, boot-to-LINE and `LEASE_LINE`. **Before deleting it**, port its trackless coverage (see Tests). |
| `amr_ws/src/amr_sim/amr_sim/tape_synth_node.py`, `amr_sim/amr_sim/tape_plant.py` | Tape sim. No launch file starts them. |
| `tools/p0_report.py`, `tools/test_p0_report.py`, `tools/tape_health_report.py`, `tools/test_tape_health_report.py` | They import `amr_line` or read its output. |

**Edit shared files**

| File | Symbol / location | Change | Trackless behaviour to keep |
|---|---|---|---|
| `amr_bringup/amr_bringup/mode_fsm.py` | L18 `... STOPPING, LINE = range(8)`; `MODE_NAMES[LINE]` L27; `LAYER_MODES` L31; `REQ_LINE` L36; `Params.tracked` L57-59; `Conditions.line_active` L79; `admit()` L126-138, L149-150; `Transaction.target` comment L180 | Change to `STARTING, IDLE, MAPPING, NAVIGATION, TRANSITIONING, FAULT, STOPPING = range(7)` with the comment "7 retired (LINE), never reuse". Set `LAYER_MODES = (MAPPING, NAVIGATION)`. Delete `REQ_LINE`, `Params.tracked` (and `Params` itself if it becomes empty), `Conditions.line_active` and the three LINE/tracked admit branches. The replacement set becomes `(REQ_IDLE, REQ_NAVIGATION, REQ_SURVEY_START)`, so `"line"` is now an "unknown request". | The BUSY/unstable/FAULT/recover rules, survey RETURNED/SAVE/ABORT, `wheels_still`, the `drives_ok` emergency rule, `commissioning_active`, the NAVIGATION job-held / RUN_FAULT rules, MAPPING unsaved/SAVING, STEPS/Transaction. Behaviour must equal today's `tracked=False` path. |
| `amr_bringup/amr_bringup/supervisor_node.py` | docstring L3-6; `from agv_core import config` L33; `LineState` import L51; `LEASE_LINE` L73; `_REQUEST_OF_TARGET[ModeState.LINE]` L87; `_MODE_OF_REQUEST[REQ_LINE]` L92; `self.tracked` / `admit_params` / `_auto_line_done` L135-142; `/amr/line_state` sub L191-196; `_allowed()` LINE branch L323-328; `_publish_mode` `m.line_available` L372 and `m.product` L375; `_srv_mode` text L398; `admit(..., self.admit_params)` L407, L453; `_auto_enter_line` L546-559; `_start_line` L787-790; `_ready_line` L792-797; `_LAYER_START`/`_LAYER_READY` LINE entries L876, L881; `_tick` calls L1095, L1112 | Delete all of these. Set `LEASE_MANUAL, LEASE_AUTONOMOUS, LEASE_COMMISSIONING = 1, 2, 4` with a "bit 8 retired" comment. Call `fsm.admit(...)` without params. The refusal text becomes "target must be IDLE or NAVIGATION (MAPPING is a survey START)". Stop setting `line_available` and `product`; the fields themselves go in P4. | Bits 1/2/4 and `_allowed()`: NAVIGATION gets MANUAL\|AUTONOMOUS, IDLE gets MANUAL, IDLE during commissioning gets COMMISSIONING, and the lease is inhibited during a transaction, STARTING, TRANSITIONING, FAULT, STOPPING or save. Also keep transactions, survey follow-ups, disk guard, boot report, web/foxglove respawn, recover/`_base_failure`, shutdown order, sdnotify and uptime. Boot ends in IDLE. |
| `amr_bringup/amr_bringup/readiness.py` | `line_gen`/`line_state` L100-101, `on_line` L158-161, reset L189, `line_state_for` L207-209, `_line_active_locked` L211-213, `conditions(line_active=...)` L228 | Delete. | Every other snapshot: drives/STANDBY, mux, wheels, panel, mapping/run/loc, map bundle, disk. |
| `amr_bringup/package.xml` | `<exec_depend>amr_line</exec_depend>` (L23) | Delete. | All other dependencies. |
| `amr_bringup/config/scanner_fields.yaml` | comment L18-19 (tracked-speed-plan-1 D8, 0.85 m/s) | Reword as self-contained: 0.40 m is kept as margin at the 0.60 m/s trackless ceiling. **Values unchanged.** | Every field index and value. |
| `amr_bringup/launch/base.launch.py` | comment L131-132 ("0.75 tracked cruise") | Reword for trackless 0.60 m/s. **The `rfid_node` block stays until P3.** | Everything. |
| `amr_sim/setup.py` | `tape_synth_node` entry L27 | Delete. | `fake_base`, `fake_imu`, `scan_synth`, `fake_panel`, `square_drive`. |

**Tests**

| Test | Action |
|---|---|
| **Add** `amr_bringup/test/test_supervisor_lease.py` (or extend `test_mode_fsm.py`), **before** deleting `test_tracked_product.py` | `test_lease_bits_per_mode`: build a `types.SimpleNamespace` (`txn`, `mode`, `inhibit_manual`, `snap.commissioning_active`, `_now`) and call `Supervisor._allowed`. Assert NAVIGATION → 1\|2; IDLE → 1; IDLE with commissioning active → 4; `txn` set, or mode in (STARTING, TRANSITIONING, FAULT, STOPPING), or `inhibit_manual` → 0; and bit 8 never set. Also port the ok assertions `fsm.admit(IDLE, REQ_NAVIGATION/REQ_SURVEY_START, cond())` (from `test_tracked_product.py:40-44`). |
| `amr_bringup/test/test_mode_fsm.py` | Rewrite `test_mode_names_cover_every_mode_and_layer_modes_are_named` (**L106-126**; it pins LINE==7 at L123 and LAYER_MODES at L124): pin `(TRANSITIONING, FAULT, STOPPING) == (4, 5, 6)`, assert `not hasattr(fsm, "LINE")`, `7 not in MODE_NAMES` and `LAYER_MODES == {MAPPING, NAVIGATION}`. Replace `test_line_mode_is_admitted_and_an_active_follower_blocks_leaving_it` (**L128-142**) with `test_unknown_request_kinds_are_refused`. It keeps `assert not fsm.admit(fsm.IDLE, "tape", cond()).ok` and adds `"line"` with an "unknown request" reason check. The stillness, drives_ok and commissioning rules are already covered for NAVIGATION/SURVEY_START at L46-65. |
| `amr_bringup/test/test_launch_layers.py` | Delete `LINE_EXES` (L40), `test_line_layer_is_only_the_follower` (L127-138) and `test_line_layer_assumes_the_field_clear_only_in_sim` (L141-149). Keep `rfid_node` in `HW_EXES` until P3. |
| `amr_bringup/test/test_unified_sim.py` | Drop `'LINE'` from `NAMES` (L49). Replace `wait_mode(7, 90), "LINE at boot"` and the following `request_mode(1)` (L295-299) with `wait_mode(1, 90), "IDLE at boot"`. Point the L379 citation at `manuals/obsolete/0910-fix-plan-001.md` B5 (D25). |
| `amr_bringup/test/test_r25_r27_supervisor_failures.py` | Delete L163 `sv._auto_enter_line = lambda: None`. The test must still reach IDLE. |
| `amr_bringup/test/test_disk_guard.py` | Delete L92 `s.admit_params = None` and the `REQ_LINE` half of L145. |
| `amr_localization/amr_localization/readiness.py:62` | Comment only: point the "fix plan 001 B5" citation at `manuals/obsolete/0910-fix-plan-001.md` (D25). |

**Verify:** V-std, then:

```bash
test ! -e amr_ws/src/amr_line
git grep -n -E "amr_line|line_layer|tape_loc|p0_localization|REQ_LINE|_auto_enter_line|_start_line|_ready_line|line_active|tape_synth|tape_plant" -- amr_ws/src tools
# expected survivors: gating.py / cmd_mux (P3); amr_web incl. test/test_teach_page.py (P2);
#   amr_interfaces LineTeach.srv / LineState.msg (P4); tools/run_log.sh:88 (P5)
```

In the unified sim, check that boot ends in IDLE and that `ModeState.mode` never takes the value 7.

### Phase 2: web UI (must land before any `config.TRACKED` / `MLS_MARKERS` / `TAPE_LOC_*` removal; see R4)

**Delete files:** `amr_web/amr_web/templates/run_tracked.html`, `templates/teach.html`, `templates/_line_track.html`, `static/linetrack.js` and `amr_web/test/test_teach_page.py`.

**Edit shared files**

| File | Symbol | Change | Keep |
|---|---|---|---|
| `amr_web/amr_web/server.py` | `MODE_LINE` L45; Protocol `line_clear`/`line_mission` L90-92; `create_app(product, markers_enabled)` L164-165; `_product` L208-217; `_markers_enabled` L219-228; `tracked_ok`/`trackless_ok` L229-230; context processor L232-238; `_need` L240-245 and its calls L309/314/319/325; `page_run_tracked` L328-331; `page_teach` L333-336; `/api/line/missions`, `/api/line/mission`, `/api/line/clear`, `/api/line/site`, `/api/line/teach`, `/api/line/at_home` L341-409; `page_home(markers_enabled=)` L253; `api_mode` L895-899 | Delete all tracked items. The context processor injects only `role`/`OPERATOR`/`ENGINEER`. `api_mode` accepts `{"idle", "navigation"}`, with the error "target must be idle or navigation". Keep `/run-trackless` as an alias (D13). | `/maps`, `/editor`, `/review` and `/run` return 200 unconditionally (`page='run'`, `sub=`). The `imu/mls` filter L768. `TRACK_M` L829-830 (wheel track). |
| `amr_web/amr_web/adapter.py` | imports `LineState`, `LineTrack`, `MarkerEvent`, `StationDetection` (L41, 42, 46, 51) and `LineTeach`, `SelectMission` (L57, 63); `MODE_NAMES[7]` L86; `LINE_NAMES` L89; `MUX_NAMES[7]` L98; `RFID_RECENT`/`RFID_QOS` L100-102; `_sensor_max_mm` L159-166; init fields L234-249 **except `_wheels`**; subs L261-267, L273-276; clients L358-361; callbacks L521-609; `manual_allowed` `& 8` branch L757-758; `state()` keys `line`, `line_track`, `rfid`, `markers` L785-788; wrappers L857-875; Spinner comment L1038 | Delete. | `visualization_msgs Marker/MarkerArray` (route preview), `_wheels` and `/wheel_states_ui`, every other state key, the MANUAL `&1` and COMMISSIONING `&4` bits, 1 s lease freshness, `SingleThreadedExecutor`. |
| `amr_web/amr_web/alarms.py` | `standing()` `line` block L79-81 | Delete. Optionally rename the `line` variable in `headline()` to `text`. | Every other standing source. |
| `templates/base.html` | L34 Run tracked, L43 Teach, L32-33 comments; L35 `{% if trackless_ok %}`; comment L69 | Delete the tracked links. Render the Run tab **unconditionally** (an undefined Jinja variable is falsy and would hide it). Label it "Run". | Engineer nav, PIN, subnav Run/Maps/Routes. |
| `templates/home.html` | nav switch L8-22; `navBusy`/`navSwitch` L82-101 and call L177; tiles `#live-rfid`, `#live-marker`, `#live-line` L36-46 and the `three` class; `liveTiles` RFID/marker/line part L119-145; `linetrack.js` L58; `job()` LINE branch L152-166; comments L23-24, L147-148 | Delete. | `#live-speed`, the trackless `job()` branch, alarms headline, ACTIONS, Pause/Abort. |
| `templates/manual.html` | include L10, script L19, `lineTrack/rfidTags` L34-35, comment L4-6 | Delete. | `jogpad.js`, jog-foot, manual-state tiles, arming note. |
| `static/amr.css` | L431-458 (`.line-track` ... `.rf-last`), L532-538 (`.home-nav`, `.seg`), `--lt-h` L423/L427-428, `.home-live.three`/`#live-line`/`.live-bar` L553-573, media query L620-622 | Delete. Drop `var(--lt-h)` from the pad `calc`. Make `.home-live` a single column. | `.live*` tile styles, pad sizing at s/m/l. |
| `static/demo/content.json` | "2 modes / tape guided", "Tape guided" and "Tape-free" cards | Rewrite to trackless only (D14). | The keys that `test_content_file_is_complete` needs. |
| `amr_web/tools/demo_server.py` | `line_clear`/`line_mission` stubs L814-818; strings L434-436 | Delete the stubs. Rename the cosmetic strings (`line_section`, "tape arrow A"). | The `imu/mls` fake diag L956-964. |
| `templates/maps.html:11` | placeholder `line_section` | Optional rename (e.g. `aisle_a`). If renamed, rename consistently in the same commit: `amr_ws/README.md:50`, `amr_interfaces/srv/StartSurvey.srv:3`, `amr_ws/RUNBOOK.md:201`, `amr_mission/test/test_map_bundle.py` (≈42 uses), `test_active_map_binding.py:6,11`, and `amr_bringup/test/test_uptime.py:26-32` (`line-a-to-b` → e.g. `route-a-to-b`). Cosmetic only. | — |

**Tests**

| Test | Action |
|---|---|
| `test_operator_ux.py` | **Delete:** `test_the_manual_page_shows_the_line_reading_above_the_pad` (L454), `test_rfid_tags_show_on_the_manual_and_run_tracked_pages` (L462), `test_the_adapter_keeps_the_rfid_link_and_the_last_tag_passes` (L518), the `_product_app` helper (L564), `test_a_product_the_profile_does_not_carry_is_404_not_hidden` (L585), `test_line_apis_reach_the_layer_and_never_move_anything` (L596), `test_run_tracked_has_no_arm_button_and_no_arm_api` (L606), `test_tracked_trackless_is_one_switch_on_home` (L613), and `_home` plus the three marker tests (L651-710). **Rewrite:** `test_home_shows_speed_rfid_and_line_tiles...` (L473) to check the Speed tile only and no `linetrack.js`. `test_home_shows_the_job_beside_the_status...` (L483): remove the docstring mention. `test_two_run_tabs...` (L573): one Run tab, no Teach/Run tracked links, Maps/Routes in the subnav. **Keep** `test_the_adapter_never_assigns_one_attribute_twice_in_init` (L711). |
| `test_server.py` | Remove `/run-tracked` from the page list (L432). |
| `test_demo.py` | Remove the `line=` parameter and `'line'` key (L24-31), the `line()` helper (L49-58) and the LINE fault case (L178). |
| **Add** (regression) | `/run-tracked`, `/teach` and `/api/line/*` return 404. `POST /api/mode {"target": "line"}` returns 400. `base.html` renders the Run tab with no product variable. `adapter.state()` has no `line`/`line_track`/`rfid`/`markers` keys. |

**Verify:** V-std, then run `git grep -n -E "tracked_ok|trackless_ok|_product\(|MODE_LINE|line_mission|line_teach|line_at_home|linetrack|_line_track|rfid|/api/line" -- amr_ws/src/amr_web`. It should come back empty except `/run-trackless`. Do a 1280×720 kiosk check of Manual and Home at ui=s/m/l: no scroll, square pad.

### Phase 3: base drivers, mux, panel, RFID and MLS-track libraries

**Delete files:** `amr_base/amr_base/mls_track.py`, `amr_base/amr_base/marker_events.py`, `amr_base/amr_base/rfid_node.py`, `amr_base/test/test_mls_track.py`, `amr_base/test/test_marker_events.py`, `agv_core/drivers/canbus/read_mls.py`, `agv_core/drivers/rfid.py`, `agv_core/drivers/rfid_survey.py`, `tests/test_mls.py`, `tests/test_rfid.py`.

**Edit shared files**

| File | Symbol | Change | Keep (trackless) |
|---|---|---|---|
| `amr_base/amr_base/drive_node.py` | docstring L6-8, L19-20; imports L68, L70, L75-77; `MARKER_QOS` L95-99; `marker_expectation` L103-114; `TRACK_UI_PERIOD_S`/`FIELD_EVENT_MIN_S` L127-128; params `mls_track_mode`/`_sdo_hz`/`_variant` L153-157; `_field_events` L210; `_track`/`_track_seq` L223-226; track/marker publishers and timer L237-246; `LineState` sub `_on_ask('line')` L249; `MlsTrack` build/start L389-406; `_track.poll` L545-546; `_publish_diagnostics` `mls/track` + `MLS_TEMP` + `marker_*` L951-971; `_on_mls_event` L973-982; `_publish_track` L984-1009; `_marker_*` L1011-1042 | Delete. `_publish_diagnostics` always publishes after the `imu/mls` status (drop the `if self._track is None` branch). | `MlsImu` build/start L375-388; `link.idle_hook` set to `imu.poll`; `_publish_imu` on `/imu/data_raw`; params `imu_enabled`, `imu_poll_hz`, `imu_stamp_every`, `gyro_sign`, `gyro_var`, `imu_frame`; the `imu/mls` diag; `RunState` and `_on_ask('run')`; `arm_policy`; `drive_gate`; lease; pp; monitor; `/wheel_states`; `/drives/*`. |
| `amr_base/amr_base/mls_imu.py` | — | **No change in P3** (R1, D6). The NMT-start / SDO-fallback port is P3b or deferred. | Everything in the file. |
| `amr_base/amr_base/cmd_mux_kinematics_node.py` | import `LineState` L48; `LINE_STATE_FRESH_S` L57-58; class defaults L83, L91-94; params `line_alpha_max`/`line_d_max`/`line_uturn_exempt_mps` L124-137, L174-177; `_line` L224, L364; sub L245-246; `/amr/line_cmd` `nav_topic` sub L377-384; `_on_line_state`, `_on_line` L386-391; `line=` in `select()` L426; `exempt = gating.line_uturn_exempt(...)` L429-431; LINE overrides L520-525 | Delete. Change **L432** to `sel, target = gating.field_limit(sel, self._view, self.gp, self.fp)`. Keep L443 `warning_target(self._view, self.fp)` as it is. | All sources and their precedence, the lease generation reset, survey spin cap, field_view/limit, SpeedRamp, `stop_d_max`, `/amr/mux_state`, `HAND_SOURCES`. FOLLOW/ROTATE slew stays exactly as today. |
| `amr_base/amr_base/gating.py` | `LINE=7` L33, `NAMES[LINE]` L42; `LEASE_LINE` L46; Params comment L66-67; `line` param L270-274; LINE branch L321-337; `AUTO_SOURCES` L376; `warning_target(..., slow_exempt)` L436 + docstring L439 + branch L444-445; `field_limit(..., slow_exempt=False)` L531 and pass-through L546; `UTURN_PHASES`/`line_uturn_exempt` L449-469 | Delete, including the `slow_exempt` parameter of **both** `warning_target` and `field_limit`. `AUTO_SOURCES = {FOLLOW, ROTATE}`. Leave the "7 / bit 8 retired" comment. Keep the APPEND-ONLY note on `select()`. | NONE..PENDANT 0..6, Lease/Panel/Permit/Manual/Drives, permit route caps, COMMISSIONING, pendant, `survey_spin_cap`, field_limit/SpeedRamp/stop_time, `drive_gate`, and `nav_topic` (used by `navigation_layer.launch.py:31` and `fake_base_node.py:22`). |
| `amr_base/amr_base/panel_io.py` | `LINE_ACTIVE` L211, `LINE_PREMOVE` L213, comment L209-210; `alarm_wanted(line_state, ...)` L216-230 | Remove the parameter and its clauses. Change every call site in the same commit (the calls are positional). | `RUN_ACTIVE=(2,4)`, `run_premove`, field logic, `motion_outputs`, `horn_wanted`, **all of `PanelAdapter`**. |
| `amr_base/amr_base/panel_node.py` | import L38, sub L85, `_line_state` L89, `_on_line` L130-131, argument L184 | Delete. | DioLink, ownerlock `dio`, `/amr/panel_state(_ui)`, `/amr/io`, coils, RunState/MuxState subs. |
| `amr_base/amr_base/arm_policy.py` | docstring L4-5, **L9-10** ("at Home", station parking), comment L38, L46 | Wording: the AUTO ask comes from `RunState.drive_power` only. Standby keeps "a parked vehicle or one pushed by hand" measured. | `want_power()` unchanged. |
| `amr_base/amr_base/canopen.py` | `write_sensor()` L498-508; comment L653-655 | Delete `write_sensor`. Re-justify the "never address node 10" rule by the gyro. | `read`, `idle`/`idle_hook`, `nmt`, per-drive NMT, `Router.add`. |
| `amr_base/amr_base/commissioning_node.py` | `ModeState.LINE: "LINE"` L59 | Delete (this must happen before P4). | The other mode names. |
| `amr_base/setup.py` | `rfid_node` entry L26 | Delete. | The other 5 entry points. |
| `amr_bringup/launch/base.launch.py` | `rfid_node` block L82-93, docstring L6 | Delete. | `drive_node`, `panel_node`, `fake_panel`, scanner, `cmd_mux`, `diff_drive_odom`, `imu_bias_node`, `scan_gate`, `commissioning`, `ekf_local`, sim fakes. |
| `agv_core/drivers/canbus/guard.py` | "THE LINE SENSOR" block L213-262 (`SENSOR_ALLOWED`, `check_sensor`, `is_sensor_allowed`) | Delete. | `ForbiddenWrite`, `ALLOWED`/`FORBIDDEN`, `PP_CONFIG_READ_ONLY`, `PDO_RANGES`, `check`/`is_allowed`. |
| `agv_core/drivers/canbus/read_imu.py` | docstring L16-21, `TPDO_SLOTS` comment L96-99 | Wording: the MLS is the gyro. TPDO1 0x18A is no longer used, and nothing starts it any more; it streams only while the MLS is Operational from boot (D7). Fix the stale path to `manuals/obsolete/slam-generalized-plan/hardware-reconciliation.md`. | The whole module. |
| `agv_core/drivers/canbus/lss.py:151`, `tests/test_lss.py:106` | comments ("carries the MLS stream" / "streams at 100 Hz") | Wording only: "TPDO1 0x18A may still stream (D7)". The filtering rationale stays true. | The code. |
| `tests/test_rpdo.py` L88-90 | comment | Wording: "the MLS may still stream TPDO1 on 0x18A". | The non-collision check L91. |
| `agv_core/drivers/canbus/__init__.py` | docstring list | Drop `read_mls`. | — |
| `agv_core/drivers/canbus/outputs.txt` | section "# read mls" L99-126 | Delete. | The node-10 lines in verify bus/drivers. |
| `agv_core/drivers/__init__.py` | Chafon docstring | Delete the sentence. | — |
| `agv_core/drivers/dio.py` | docstrings L12-13, L19-20, L41-56, comment **L69** ("the same reason RfidLink takes a codec") | Wording only: drop the RfidLink comparisons. Keep the TOPOLOGY note until the reader is physically removed (D11). **No code change.** | The whole `DioLink`. |
| `agv_core/ownerlock.py` | `RESOURCES "rfid"` L28, docstring L3-5 | Delete (the callers are gone in this phase). | `app`, `can`, `dio`, `scanner`. |
| `agv_core/panel.py` L13, L33-34; `tests/test_dio.py:173` | docstrings | Wording only. | — |

**Tests**

| Test | Action |
|---|---|
| `amr_base/test/test_gating.py` L86-115 | Bits list `[1, 2, 4]`, `PENDANT == 6`, `not hasattr(g, "LINE")`, `not hasattr(g, "LEASE_LINE")`. |
| `amr_base/test/test_field_limit.py` | Remove the `LINE`/`line_uturn_exempt` imports (L12, L27). L59: iterate `(FOLLOW, ROTATE)`. L66-69: use FOLLOW. Delete **L203-224**, which covers the W1/W2/PROT `slow_exempt=True` block and the `line_uturn_exempt` test. Port the non-exempt asserts to FOLLOW: W1 halves to 0.5, W2 gives 0.0, PROT gives `(NONE, 0.0)`. Add "warning 1 always applies". |
| `amr_base/test/test_mux_manual_profile.py` | Delete `test_line_steering_slews_at_line_alpha_max...` (L100) and `test_line_decelerates_at_line_d_max...` (L112-133). Add "FOLLOW uses `d_max`/`alpha_max`". |
| `amr_base/test/test_alarm_horn.py` | **Rewrite, do not drop.** Remove the LineState constants (L6) and the `line_state` argument. Map `(RUNNING, None)` → `(None, EXECUTING)` and `(HOLD, None)` → `(None, BLOCKED)`. Keep `test_no_field_no_alarm` (L16-17), `test_stale_field_data_does_not_sound_it` (L40-41), the `motion_outputs(True, alarm_wanted(..., EXECUTING ...))` check (L35-37), and the protective case `(None, EXECUTING, False)` in `test_a_field_switches_the_movement_horn_to_the_alarm_horn` (L47). Express the pre-move case with `run_premove=True` only. Fix the module docstring. |
| `amr_base/test/test_arm_policy.py` L20-41 | `Ask('line', ...)` becomes `Ask('run', ...)`. Keep the stale/latched/invalid semantics. |
| `amr_base/test/test_authority.py` L173 | `('line', 'none')` becomes `('rotate', 'none')`. |
| `amr_base/test/test_canopen.py` L221-223 | Comment only (gyro). The assertion stays. |
| `amr_base/test/test_pendant_gating.py`, `test_node_attributes.py` | Re-run only. |
| `amr_base/test/test_mls_imu.py` | Keep. **Add** the check ported from `test_mls_track.py::test_only_reads_and_nmt_start_to_the_mls_nothing_is_written`: across `start()` and many `poll()`s in SDO and TPDO mode, `link.writes == []` and `all(n == 10 for _, n in link.nmts)` (today: `link.nmts == []`). Never node 0, 1 or 2. |
| `amr_bringup/test/test_launch_layers.py` L37 | Remove `rfid_node` from `HW_EXES`. |
| `tests/test_canmon.py` L117-134, L142-144 | Delete the `SENSOR_ALLOWED`, `read_mls` and `write_sensor` checks. Keep the drive-guard and 0x1806-writable checks. |
| `tests/run_all.py` | Remove `test_rfid` and `test_mls` from `MODULES` (L31, L33). Set `EXPECTED_CHECKS` to 747 − 75 − 58 − (removed canmon checks, about 5), so about **609**. Use the measured number and a dated note: "2026-10-xx: tracked removal P3 (RFID driver, MLS track decoder, line-sensor guard)". |

**Verify:** V-std, then:

```bash
git grep -n -E "mls_track|marker_events|MarkerDetector|read_mls|SENSOR_ALLOWED|check_sensor|write_sensor|rfid|line_state|line_cmd|LEASE_LINE|gating\.LINE|uturn|slow_exempt" -- amr_ws/src agv_core tests
# expected survivors: amr_interfaces/*.msg (P4); agv_core/config.py, alarms.py, mission.py (P5);
#   tests/test_config.py rfid checks L391-395/L434/L463/L472 (P5); tests/run_all.py history comments (P5)
ros2 pkg executables amr_base | grep -c rfid_node     # 0
```

On the bench, on can0:

1. `drive_node` logs "MLS yaw rate by SDO polling" and `/imu/data_raw` runs at about 50 Hz.
2. Put node 10 in Pre-operational. Either power-cycle the MLS and then send NMT Pre-op addressed to node 10 only, or send the Pre-op straight away. Confirm with `candump can0` that 0x18A stops.
3. Confirm 2034h:03 still updates by SDO (0x58A replies) at about 50 Hz and `/imu/data_raw` keeps going with no `imu/mls` stale.

**Phase 3b (separate commit, optional or deferred, D6):** the `MlsImu` NMT Start to node 10 only (rate-limited to 2 s, in TPDO mode when stale) plus the SDO fallback after N stale periods, with tests. Bench test: enable 1806h on a bench sensor, send NMT Pre-op to node 10, and check that `MlsImu` restarts the stream or falls back to SDO.

### Phase 4: interfaces

**Delete files:** `amr_interfaces/msg/LineTrack.msg`, `msg/LineState.msg`, `msg/MarkerEvent.msg`, `msg/StationDetection.msg`, `srv/SelectMission.srv`, `srv/LineTeach.srv`.

**Edit shared files**

| File | Change | Keep |
|---|---|---|
| `amr_interfaces/CMakeLists.txt` | Remove L15 `StationDetection`, L31 `LineTrack`, L32 `LineState`, L33 `MarkerEvent`, L45 `SelectMission`, L46 `LineTeach`. | The other 18 msgs and 10 srvs, and the dependencies. |
| `msg/ModeState.msg` | Delete `uint8 LINE=7` (L11) and its comment, `bool line_available` (L28) and `string product` (L29), per D4. Add `# 7 retired (LINE), never reuse`. Drop the dual-product-plan citation. | STARTING..STOPPING 0..6, and the fields `instance`, `generation`, `mode`, `requested_mode`, `phase`, `operation_id`, `base_ready`, `layer_ready`, `active_map_*`, `manual_available`, `autonomous_available`, `fault_code`, `reason` and `last_survey_*`. |
| `msg/ControlLease.msg` | Delete `LINE=8` (L10) and add a "retired" comment. | `MANUAL=1`, `AUTONOMOUS=2`, `COMMISSIONING=4` and the lease semantics. |
| `msg/MuxState.msg` | Delete `LINE=7` (L11-12) and add a "retired" comment. | 0..6 and the field/scale fields. |
| `msg/MotionPermit.msg` | Delete L8-9 (`LINE=4`, reserved). | NONE/TELEOP/FOLLOW/ROTATE and `v_max`/`w_max`. |
| `amr_localization/package.xml:6` | Drop `, station_id_node` from the description (never existed; it was the planned `StationDetection` consumer). | The rest of the description. |
| `msg/RunState.msg`, `LocalizationState.msg`, `WheelStates.msg`, `PanelState.msg`, `DriveStatus.msg`, `Event.msg`, `IoImage.msg`, others | **No change.** | — |

**Verify:** V-std (it includes the full clean rebuild), then:

```bash
git grep -n -E "LineTrack|LineState|MarkerEvent|StationDetection|SelectMission|LineTeach|line_available|m\.product|ModeState.*product|station_id_node" -- amr_ws/src
# expected: empty  (demo.html {{ c.product }} is the demo content name and is not matched)
```

Humble does no type check (R6), so a mixed overlay silently corrupts the `ModeState` tail. Every node, the web and foxglove_bridge must run from the same overlay. The deploy is: on the vehicle, clean-build in place (P7), then do a witnessed `sudo systemctl restart amr.service` so every process restarts together. `deploy/install.sh` only installs the unit file and drop-ins. It deliberately does not restart anything and does not deploy an overlay.

### Phase 5: agv_core config, profile, missions, alarms, RUNBOOK §4, tools

Do this in **one commit** (R7).

**Delete files:** `agv_core/mission.py`, `missions/empty.json`, `missions/line-a.json`, `missions/mru-straight-markers.json`, `missions/site-103.json`, `missions/test-mru-straight.json` (the `missions/` directory disappears), and `tests/test_mission.py`.

**Edit shared files**

| File | Symbol | Change | Keep |
|---|---|---|---|
| `agv_core/config.py` | `_SCHEMA["autopilot"]` L430-489, its `_validate` block L1167-1185, the `AUTO_HIGH_RPM` derive L912-913 and `_DERIVED` row L1358; `vehicle.sensor_lookahead_m` L495 and `_validate` L968; `_SCHEMA["rfid"]` L502-519 and `_validate` L1165, L1231-1258; `_SCHEMA["mls"]` L602-609, `_read_marker_codes` L760-774, `_parse` mls L863-864, L896-897, `_validate` L976-979, `describe()` L1576-1580; `_SCHEMA["tape_loc"]` L610-627 and `_validate` L981-1003; `_TOP_LEVEL_SCALARS["tracked"]` L660, `_NULLABLE_TOP_LEVEL` L661, null handling L848-850 | Delete. Check that `_derive` still yields `MPS_PER_RPM`, `RAD_S_PER_RPM_DIFF` and `RPM_PER_MPS` without any autopilot input. | `vehicle.track_m` (`TRACK_M` is the **wheel track**), `wheel_dia_m`, `gear_ratio`, `motor_max_rpm`, `invert_*`; `can.sensor_node`; `imu.*`; `timing` (`AUTO_START_DELAY_S`, `PREMOVE_TIMEOUT_S`, `manual_watchdog_s`); `panel`, `pendant`, `horn`, `dio`, `drivers.ramp`, `blind_run`, `pp`, `monitor`, `lidar`. |
| `agv_core/config.py` docstrings | rule 5 L36-42, L31, L41, L92-95, L117-120, L122-150 (rfid notes), L292-300, L308-331 (mls), L333-361 (tape_loc), L363-380 (imu: fix "DISPLAY ONLY"), L406, describe L1516-1534 | Delete or reword. `tuning_notes()` parses this prose, so keep every remaining heading (at least 12 notes; about 19 are left). | — |
| `profiles/agv-01.json` | `"tracked"` L3, `"autopilot"` L4-43, `vehicle.sensor_lookahead_m` L49, `"rfid"` L61-78, `"mls"` L189-196, `"tape_loc"` L197-214 | Delete. | All other keys, including `lidar.host_ip` 192.168.3.1 and `imu`, `dio`, `can`. |
| `agv_core/alarms.py` | rows `TRACK_LOST`, `RFID_LINK_LOST`, `TRACK_RATE_LOW`, `AUTHORITY_LOST`, `MLS_FIELD_WEAK`, `MLS_TEMP` (D8), `MARKERS_LOST`, `AT_STATION`, `LINE_MISSION`, `LINE_MARKER`, `LINE_LOC`; `HOLD_CODES` `station`, `track`, `rate`, `authority`, `rfid`, `markers`, `drives` (L409-419; `drives` is emitted only by amr_line); wording L15, L60, **L87-88** (DRIVES_NOT_READY hint: "the hold cause 'drives'"), L125, L269-270, L346, L406-407 | Delete the rows and mappings and fix the wording. The L87-88 hint becomes "DriveStatus not operational while the mux had a command to apply. Since 2026-10-08 …". | `FIELD_BLOCKED`, `ESTOP`, `PATH_BLOCKED`, `WAITING_FOR_PREREQ`, `DRIVES_NOT_READY` (gating emits it), `CARD_CODES`, the `markdown()`/`card()` generator, `get()`/`UNKNOWN`, and the SCANNER_SILENT edit (committed in D1). |
| `amr_ws/RUNBOOK.md` §4 (L621-751) | generated block | Regenerate with `python3 -m agv_core.alarms --md` and paste between the BEGIN/END markers. **Do not hand-edit.** | — |
| `agv_core/kinematics.py` L4-5, `agv_core/events.py` L4/L12, `agv_core/canmon.py` L22 | docstrings | Wording only. For canmon L22: "the MLS gyro is SDO-polled at 50 Hz; TPDO1 0x18A may still stream (D7)". | The code. |
| `tools/bag_report.py` | `LINE_STATES`, `PASS_RE`, `MARGINAL_READS`, `LAT`, `MUX_SOURCES[7]`, `lateral()`, `osc_period()`, the `/amr/line_cmd` key rewrite L55, the tape/`line_track`/`line_cmd`/`line_state`/RFID/marker/passes/segments/U-turn sections, `config.RFID_TAG_CLEAR_S` L262, docstring L1-17 | Strip (D16). Add `/amr/run_state` and `/amr/localization_state`. | `read_bag`, `write_csv`, wheels, imu, cmd_wheel_vel, mux/drives/panel/io/events/rosout. |
| `tools/run_log.sh` | MISSION/DEST args L5/L11-12, L35, pre-flight L48-67, mission copy L77/L82-83, TOPICS L87-90 (incl. the L88 p0_localization/tape_loc comment) | Strip. The pre-flight checks `run_state` and `localization_state` instead. | Service/disk checks, snapshots, the trackless topic list. |
| `tools/uhf-even/` | entire directory (79 tracked files; 3 have non-ASCII names, which `git ls-files` prints quoted) | `git rm -r`. Verify with `git ls-files tools/uhf-even \| wc -l` == 0. | — |
| `.gitignore` | L7-10 (rfid-setup negation), L13-16 comment (L15 cites fix plan 001 A3), L30-35 (uhf-even, L31 cites A5) | Delete or reword. Any surviving fix-plan citation points at `manuals/obsolete/0910-fix-plan-001.md` (D25). | `*.sh` with the deploy/env/tools negations. |

**Tests**

| Test | Action |
|---|---|
| `tests/test_config.py` | Delete the mls block L195-216, the tape_loc block L217-231, partial-autopilot L84-86 and the autopilot leftovers L300-323. Re-point the rfid tuning-note checks L463 and L471-472 to `monitor.*`. Reword or drop the "rfid rules" comments and checks L391-395 and L434. **Add** refusals for leftover `autopilot`, `rfid`, `mls` and `tape_loc` sections, for `vehicle.sensor_lookahead_m`, and for the `tracked` key. |
| `tests/test_alarms.py` L67 | Docstring: the hold vocabulary is RunState only. No logic change. |
| `tests/run_all.py` | Remove `test_mission`. For `EXPECTED_CHECKS`, subtract 72 and the removed test_config checks (about 33), then add the new refusals: about **504 + refusals**. Use the measured number and a dated note. Reword the history comments L55, L60, L64, L67, L71-79. |
| `tests/helpers.py` L3-14, L59-61, L7 (K_RATIO) | Docstring history: reword or leave (allow-listed). |

**Verify:** V-std, plus:

```bash
python3 -c "import agv_core.config as c; print(c.PROFILE_NAME)"         # strict loader accepts the profile
python3 -m agv_core.alarms --md | diff - <(sed -n '/BEGIN GENERATED/,/END GENERATED/p' amr_ws/RUNBOOK.md | sed '1d;$d')
git grep -n -E "TRACKED|AUTO_HIGH_RPM|K_RATIO|SENSOR_LOOKAHEAD|SENSOR_MAX_MM|MLS_(MARKERS?|VARIANT|POLARITY_LOCK|TEACH_LOCK|EVENT_CLEAR|MARKER_CODES|FIELD_WEAK|TEMP)|RFID_|TAPE_LOC|agv_core\.mission|missions/" -- ':!manuals'
# expected survivors: refusal tests in tests/test_config.py; history notes in tests/run_all.py and tests/helpers.py; docs (P6)
```

### Phase 6: docs and drawings

**Delete files**

- Manuals: `manuals/mls-marker-plan.md`, `manuals/mls-diagnostics-plan.md`, `manuals/rfid-tag-table.md`, `manuals/site-layout-103.md` (after D20), `manuals/tape-localization-runbook.md`, `manuals/tracked-plan-3-tape-localization.md`, `manuals/tracked-speed-plan-1-no-slam.md`, `manuals/tracked-speed-plan-2-slam.md`.
- Obsolete archive:
  - `manuals/obsolete/slam-generalized-plan/dual-product-plan.md` and `.../line-follow-u11-layout-plan.md`
  - `manuals/obsolete/rfid-setup/rfid-setup.md`, and `.../rfid-link-notes.txt` after its §1 is inlined into alpha-setup
  - `manuals/obsolete/VEHICLE-TEST-line-layer.txt` and `manuals/obsolete/VEHICLE-SESSION-INDEX.txt`
  - `manuals/obsolete/agent-prompts/vehicle-dynamic-areas-and-lf-plan.md`
  - `manuals/obsolete/legacy-runs/0021…0025-auto_*/`. Either reword the `config.py` ~L118 citation, or keep 0023-0025 as loop-timing evidence.
- Drawings: `documentation/layout-reference/corner-tag-placement.{html,png}`, `mls-marker-codes-1-3.{html,png}`, `mls-marker-codes-standard.{html,png}`, `mls-zone-marker-placement.{html,png}`, `simple-reference.html`, and `homepos-area-projection/layout-projection.{svg,png}` (after D20).

**Move:**

- `0910-fix-plan-001.md` (repo root) to `manuals/obsolete/0910-fix-plan-001.md` (D25). Its citations in `readiness.py:62`, `test_unified_sim.py:379` and `.gitignore` already point there (P1/P5).
- Add `amr_ws/deploy/netplan/{60-lan1-dio-maint,70-lan2-nanoscan3,80-lan3-gls621}.yaml`, copied from the live `/etc/netplan` on gvipc-ce (D19). `manuals/obsolete/rfid-setup/60-rfid-lan1.yaml` stays in obsolete as history, or is deleted.
- Optional: the MLS vendor manual `manuals/obsolete/operating_instructions_mls_en_im0076568.pdf` to `manuals/mls/`. It is the live gyro reference.

**Edit shared files**

| File | Change |
|---|---|
| `README.md` | Lines to change: L4-6 and L10-20 (single product; `legacy-final` now exists). L31/L33 (Nodes: gyro only; Auto: trackless 0.60 m/s). L45-46. L89 (drop `read_mls stream`). L116/L122/L124-125/L131. **L134** (refresh "531 offline checks" to the measured count). L136. L161-162 (drop "the MLS track reading rides on TPDO1 0x18A at 100 Hz"; say the MLS gyro is SDO-polled at 50 Hz). L168 (lease without "line"). §"The MLS" L190-197 becomes "The MLS gyro". L211-215 (no `autopilot`, no `tracked`). L223/L236-237 (refresh the test count and fix the stale VEHICLE-TEST paths). L260-262 (drop the autopilot/branch invariant). L268-286 (drop the four tracked Known-gaps bullets). **Keep verbatim** the safety model, the guard table, one owner per device, the NMT-per-node invariant and `verify_bus` nodes 1, 2, 10. |
| `amr_ws/README.md` | L3-4 (stale plan paths), L206 (`RunState.drive_power` only), L281 (drop `LineState.code`), L327. Optional L50 `line_section` rename (P2). Keep the MLS IMU sections L94-107 and L175-198. |
| `amr_ws/RUNBOOK.md` | Intro L3-8 (boots to IDLE). §1: delete "Tape or trackless: the profile's `tracked` key" L92-133, including the LINE follower rules, branching and tracked speed. §2 L175. §3: delete "MLS marker codes" L579-612. §5 W9 L875 (boot reaches IDLE). §6 L891-905 and §7 L916 (history wording; `legacy-final` now exists). Add a §5 deploy note covering the full clean rebuild (`rm -rf build install log`), the witnessed `systemctl restart amr.service`, and archiving the tape data, purging it and rotating the journals (D22). **Keep the §1-§7 numbering** (cited by `guard.py:61`, `reports.py`, `server.py:1098`, `validate.sh:100`, `install.sh` and `alpha-setup`). Keep the path stable (`amr.service Documentation=`). |
| `alpha-setup.md` | L21-22, L113, L172, L188 (MLS row: "node ID 10, 125 kbps; gyro SDO-polled; TPDO1 0x18A no longer used"), L191, L771, L1240-1241, L1244, L1253, L1312-1313, L1325-1326: drop the RFID, mission, marker and track-variant rows. **L1236** (candump): expect `0x181` and `0x182`; `0x18A` only if the MLS is still Operational from boot (D7). **L97**: the example path becomes `/etc/netplan/60-lan1-dio-maint.yaml`. **L137**: refresh the "706 at the time of writing" count or drop it. §2.2 L291 and L347-370, and §10 L1275: rewrite for the three-port layout: enp1s0 DIO+maint .1.100, enp2s0 nanoScan3 .3.1, enp3s0 GLS .4.2, no gateway, `optional: true`, one /24 per port, avoiding 192.168.2.0/24 (Wi-Fi). Inline the renderer-scoping rationale from `rfid-link-notes.txt` §1. L408/L424-435: drop `rfid-test.sh`. The LAN2/LAN3 addressing is already settled (D23). L62: update if the vehicle branch changes (D24). **Keep** the can0 nodes 1, 2, 10, the `read_imu show` row L1242 and the I/O island ping L1243/L763. |
| `manuals/gls6-qr/gls6-system.md` | L25: "the SLAM/AMCL/Nav2 product" (no `tracked` key). L41: drop the tracked/LINE out-of-scope bullet. L267: change the GLS addressing to 192.168.4.1 (PC 192.168.4.2 on enp3s0) (D23). |
| `documentation/electrical/01-schematic/io-mapping.md` | L50: "MLSE (gyro/IMU)". L112 RFID OUT: change **only after the physical removal** (D11). |
| `manuals/obsolete/slam-generalized-plan/bench-checklists.md`, `hardware-reconciliation.md` | Keep. Optionally promote them out of `obsolete/` and fix the citing paths (`read_imu.py:16`, `lss.py:12`, `rpdo.py:11-13`, `config.py:174-175`). |

**Verify:** `python3 -m pytest -q tests/test_alarms.py`, then the final grep in §5.

### Phase 7: deploy and cleanup on the vehicle (see §6)

`git pull` `kyb-trackless` on the vehicle (no `gvievo-main` merge, D24 overridden). Then, with `amr.service` stopped and the session witnessed:

```bash
cd ~/agv_can/amr_ws && rm -rf build install log && colcon build --symlink-install
sudo deploy/install.sh && deploy/validate.sh          # paths relative to amr_ws; install.sh does not restart anything
ls install/*/lib/*/ install/*/share/*/launch/ | grep -E 'rfid|tape|line_layer|p0_loc|web_panel'   # must be empty
tar czf ~/tracked-data-$(date +%F).tgz ~/amr_maps/*/tapegraph ~/.amr/teach ~/.amr/line_graph.json \
    ~/.amr/reports/tape_health ~/.amr/mls_cal ~/.amr/runs ~/.amr/logs/events.jsonl* ~/.amr/operations.jsonl 2>/dev/null
rm -rf ~/amr_maps/*/tapegraph ~/.amr/teach ~/.amr/line_graph.json ~/.amr/reports/tape_health ~/.amr/mls_cal ~/.amr/runs/*   # tracked-era runs only
# rotate the journals (D22): move events.jsonl* and operations.jsonl aside after the tarball is verified
sudo systemctl restart amr.service                     # every process from the new overlay (R6)
```

Netplan (D19): **nothing to do on gvipc-ce**, which was migrated on 2026-10-21. Optionally run `sudo rm /etc/netplan/*.off /etc/netplan/*.bak-*`. For any other vehicle still on the enp2s0/.1.10 layout, do it console-attended: copy the files, run `sudo netplan generate && sudo netplan try`, then `ping -c3 192.168.1.40`.

---

## 4. Keep list (looks tape-related, stays)

| Item | Why trackless needs it |
|---|---|
| `amr_base/amr_base/mls_imu.py`, `agv_core/drivers/canbus/read_imu.py`, `tests/test_imu.py`, `amr_base/test/test_mls_imu.py` | The MLS gyro (2034h:03 by SDO, or 1806h by TPDO) feeds `/imu/data_raw`. |
| `drive_node` MLS IMU path, params `imu_enabled`/`gyro_sign`/`gyro_var`/`imu_frame`, diag `imu/mls` (read by `server.py:768` and `demo_server.py:956`) | It produces the EKF yaw rate and the IMU readiness. |
| `amr_localization/gyro_bias.py`, `imu_bias_node`, `config/ekf.yaml` imu0, `localization_monitor` IMU freshness, `mapping_session` IMU calibration check | The trackless IMU chain. |
| Profile `can.sensor_node=10` (`SENSOR_NODE`), `imu.enabled` (`IMU_ENABLED`), the distinct-node `_validate` L1009-1014 | Node 10 is the gyro. |
| `canopen.py` per-drive NMT ("never node 0 or 10"), `test_canopen.py:221-226` | Protects the gyro node from broadcast NMT. |
| `amr_description` `imu_frame`/`imu_joint` (0.092 m, "IMU inside the SICK MLS") | The EKF/IMU frame. |
| `tests/test_rpdo.py` L91 (0x18A non-collision) | TPDO1 may still stream (D7). Only the comment changes. |
| `agv_core/drivers/canbus/lss.py` 0x7E4 filtering, `tests/test_lss.py` | Still needed on a bus with drive traffic and a possible TPDO1. Only the comments change. |
| `agv_core/drivers/dio.py` (`DioLink`), `tests/test_dio.py`, `agv_core/panel.py`, `panel_io.PanelAdapter`, `panel_node` | Panel, lease, arming and horn. They contain no RFID code, only comments. |
| `/etc/netplan/60-lan1-dio-maint.yaml` (enp1s0 192.168.1.100/24) | The DIO island link and maintenance SSH (R9). |
| `config.TRACK_M` / `vehicle.track_m`, `track_width`, `jogpad.js TRACK_M`, `route.py cross_track_limit_m` | Wheel track and route deviation, not tape. |
| `timing.auto_start_delay_s` / `premove_timeout_s` | The `route_executor` pre-move horn. |
| `LocalizationState`, `RunState`, `WheelStates`, `PanelState`, `DriveStatus`, `Event`, `IoImage` msgs | Trackless interfaces. Only their amr_line consumer goes away. |
| `amr_mission/map_bundle.py`, `amr_localization/config/amcl.yaml`, `agv_core/kinematics.py` | Tape code imports them too, but trackless owns them. |
| `gating.nav_topic`, `drive_gate`, `gating.field_limit`/`warning_target` (without `slow_exempt`) | Used by `navigation_layer.launch.py`, `fake_base_node` and the AUTO field gating. |
| `scanner_fields.yaml` values (`warning_2.stop_m 0.40`) | The AUTO field gating; only the comment changes. |
| `agv_core/alarms.get()` / `UNKNOWN` | Generic safe fallback; old codes are handled by rotating the journals (D22). |
| "tape" meaning a measuring tape: `commissioning.html:61`, `commissioning_log.py`, `amr.urdf.xacro:29/32`, `amr_ws/README.md:166`, `test_blindrun_page.py` | Not magnetic tape. |
| `route_executor_node` "taper" | Speed taper. |
| `manuals/obsolete/legacy-runs/0026-0029-blind_*`, `blind_results.csv` | Odometry and commissioning evidence. |
| `manuals/obsolete/0910-fix-plan-001.md` (moved, D25) | Cited by trackless code (PP_VETOED, bag_report, `tools/*.sh` gitignore, sim `can_confirm`). |
| `manuals/obsolete/slam-generalized-plan/*` (except the 2 deleted), `codebase-improvement.md`, `2026-09-23-demo-page.md`, `vehicle-reports/2026-09-21-...md`, `can-monitoring-plan.txt`, etc. | Cited as rationale by trackless code. |
| `manuals/obsolete/operating_instructions_mls_en_im0076568.pdf` | The MLS vendor manual, including the gyro chapters. |
| `documentation/electrical/**` | As-built wiring. MLSE stays on CAN; RFID changes only on an electrical revision. |
| `manuals/bldc/**`, `manuals/certifications-compliance/**`, `manuals/gls6-qr/*.pdf`, `manuals/Neu-X102-N97_RS485_interface.md`, `manuals/artifact-slam-01/index.html` | Vendor or trackless documentation. |
| Branches `main`, `gvievo-main`, `trackless-kyb`, `aop-demo-oct`, `origin/*` (incl. `origin/gvievo-main`, `origin/gy-demo`) | History. `gvievo-main` receives the removal (D24). `aop-demo-oct` and `gvievo-main` will conflict with the web and doc removals. |

---

## 5. Residual-trace allow-list and final grep

**Allow-list (hits that may remain)**

1. Vendor and as-built material: `manuals/bldc/**`, `manuals/certifications-compliance/**`, `manuals/gls6-qr/*.pdf`, `manuals/obsolete/operating_instructions_mls_*.pdf` (or `manuals/mls/`), and `documentation/electrical/**` (RFID and "LINE SENSOR" until the physical removal).
2. Archived history the owner keeps (D17, D25): `manuals/obsolete/**` except the deleted files.
3. "Retired" comments: `# 7 retired (LINE)` and `# bit 8 retired (LEASE_LINE)` in `ModeState.msg`, `MuxState.msg`, `ControlLease.msg`, `mode_fsm.py`, `gating.py` and `supervisor_node.py`, and the matching "not reused" asserts in `test_mode_fsm.py`, `test_gating.py` and `test_supervisor_lease.py`.
4. Refusal tests that name forbidden keys or requests: in `tests/test_config.py`, the keys `autopilot`, `rfid`, `mls`, `tape_loc`, `tracked` and `sensor_lookahead_m`; in `test_mode_fsm.py`, the `"line"`/`"tape"` unknown-request asserts; and the web 404/400 regression tests.
5. History notes in `tests/run_all.py` (`EXPECTED_CHECKS` changelog), the `tests/helpers.py` docstring, RUNBOOK §6/§7 and README Status, if worded as history ("the tracked product was removed on …, tag `tracked-final`").
6. The `/run-trackless` alias and the word "trackless" in general.
7. Example ids `line_section` / `line-a-to-b`, if the optional rename (P2) is not done. They are cosmetic.
8. `.gitignore` is not allowed any hits.

**Final grep (should return nothing outside the allow-list)**

```bash
cd /home/gvipc-evo-01/agv_can
git grep -n -I -i -E \
 '\b(amr_line|line_?follow\w*|line_layer|LineState|LineTrack|LineTeach|MarkerEvent|StationDetection|SelectMission|station_id_node|line_track(_ui)?|line_marker|line_cmd|line_state|line_available|LEASE_LINE|REQ_LINE|MODE_LINE|tracked_ok|trackless_ok|run[-_]tracked|tape_(graph|loc|run|runtime|teach|health|locator|synth|plant)\w*|tapegraph|line_graph|mls_track|mls_cal|read_mls|marker_events|MarkerDetector|marker_codes|markers_enabled|SENSOR_ALLOWED|check_sensor|write_sensor|slow_exempt|rfid\w*|chafon|cf821|uhf-even|tag_table|speed_zone|uturn|u_turn\w*|autopilot|AUTO_HIGH_RPM|K_RATIO|SENSOR_LOOKAHEAD\w*|SENSOR_MAX_MM|MLS_(MARKERS?|VARIANT|POLARITY_LOCK|TEACH_LOCK|EVENT_CLEAR|MARKER_CODES|FIELD_WEAK|TEMP)|TRACK_LOST|TRACK_RATE_LOW|MARKERS_LOST|AT_STATION|LINE_(LOC|MISSION|MARKER)|AUTHORITY_LOST|p0_localization|p0_report|agv_core\.mission|config\.TRACKED|ModeState\.LINE|MuxState\.LINE|gating\.LINE)\b|"tracked"|missions/[a-z0-9-]+\.json|Run tracked|tape (AGV|follow\w*|product)|magnetic tape|0x18A @|track reading' \
 -- . ':!manuals/bldc' ':!manuals/certifications-compliance' ':!manuals/obsolete' \
      ':!documentation/electrical' ':!manuals/gls6-qr/*.pdf' ':!manuals/mls'
```

Then run a prose sweep and review every hit by hand. Expected survivors: measuring-tape uses, "taper", wheel `track_m`/`TRACK_M`/`track_width`, `cross_track`, the canvas `lineTo`/editor "line" tool, and `{{ c.product }}` in `demo.html`.

```bash
git grep -n -I -i -w -E 'tape|tracked|LINE|station|at Home' -- amr_ws/src agv_core tests tools profiles README.md alpha-setup.md amr_ws/README.md amr_ws/RUNBOOK.md \
  | grep -v -i -E 'measur|tape-measured|by tape|taper|tape read|tape A|# 7 retired|bit 8 retired|line_?width|lineTo|tool-line|btn-line|git-tracked|untracked'
```

Also check that no tracked artefact remains in the built overlay:

```bash
test ! -e amr_ws/src/amr_line
ls amr_ws/install | grep -i line                                            # must be empty
ls amr_ws/install/*/lib/*/ amr_ws/install/*/share/*/launch/ | grep -E 'rfid|tape|line_layer|p0_loc'   # must be empty
ros2 pkg executables amr_base | grep rfid_node                              # must be empty
ros2 interface list | grep -E 'Line|Marker|Station|SelectMission'           # must be empty
git ls-files tools/uhf-even amr_ws/src/amr_line | wc -l                     # 0
```

---

## 6. Risks and on-vehicle verification

**Residual risks**

| Risk | Mitigation |
|---|---|
| Gyro silently stops in TPDO mode (R1) | Today's configuration is SDO, which works in Pre-operational. The P3 bench check forces node 10 to Pre-operational and proves the SDO gyro. Do not enable 1806h before D6 (P3b) lands. |
| TPDO1 0x18A present or absent depending on the last NMT (D7) | Docs and acceptance rows accept either state. Nothing listens to it. |
| Tracked pages come back silently because of the `_product()` fallback (R4) | Land P2 before P5. The P2 regression tests check for 404. |
| Silent `ModeState` CDR mismatch from a partial deploy or stale process (Humble has no type hash, R6) | Full clean rebuild plus a witnessed `systemctl restart amr.service` in P4/P7. Old bags decode only against `tracked-final`. |
| Stale overlay keeps `rfid_node`, `tape_synth_node` and `line_layer.launch.py` discoverable (R11) | `rm -rf build install log` in V-std and P7. Executable checks in V-std, §5 and V1. |
| Mux `_tick` NameError/TypeError from a half-removed `slow_exempt` (R2) | Edit gating and the mux in one commit. ruff F821 and `test_mux_manual_profile`/`test_field_limit` in V-std. |
| Lost offline coverage of lease bits and alarm-horn rules | `test_supervisor_lease.py` (P1) and the rewritten `test_alarm_horn.py` (P3). |
| Profile no longer loads (strict loader) | P5 is one commit. Run `python3 -c "import agv_core.config"` before deploying. |
| Event history shows retired codes as UNKNOWN errors | Journals archived and rotated at deploy (D22). |
| LAN1 netplan break leads to `PANEL_STALE` (R9) | Not touched on gvipc-ce. On other vehicles: console-attended `netplan try`, ping 192.168.1.40, rollback copy of `/etc/netplan/`. |
| Vehicle deploys the old build because the removal never reached `gvievo-main` (R12) | D24 merge before P7. Check the `git log` on the vehicle. |
| Manual pad layout regresses | 1280×720 check at ui=s/m/l. |
| Lost diagnostics: MLS temperature and tape health | Accepted (D8); log a follow-up item. |
| A GLS branch, `aop-demo-oct` or `gvievo-main` conflicts on `_SCHEMA`, profile, ownerlock, `EXPECTED_CHECKS` or web | Land the removal first and rebase the GLS work onto it. |

**On-vehicle acceptance (after P7, witnessed; record in a vehicle report)**

| # | Check | Pass criterion |
|---|---|---|
| V1 | Boot | `amr.service` is active. The supervisor goes STARTING then **IDLE** with no LINE transaction (`journalctl -u amr`). `ModeState.mode == 1`. No `amr_line`, `rfid_node` or `line_follow_node` process (`ps`). `ros2 pkg executables amr_base` does not list `rfid_node`, and `install/` has no tape/rfid/line_layer entries. |
| V2 | CAN bus | `verify_bus` sees nodes 1, 2 and 10. `drive_node` logs "MLS yaw rate by SDO polling". There is no `mls/track` diagnostic, and the `imu/mls` diagnostic is OK. `candump can0` shows no NMT to node 0 or 10 from the drive link at arm, and no NMT Start to node 10 from anything. 0x18A may or may not be present (D7). |
| V3 | Gyro | `ros2 topic hz /imu/data_raw` is about 50 Hz, **also after node 10 is put in Pre-operational**: power-cycle the MLS, then send NMT Pre-op to node 10 only; SDO 0x58A replies keep coming. `/imu/data` is published once `imu_bias_node` has calibrated at standstill. Rotate 360° by jog: EKF yaw is within the existing tolerance (compare with the `tracked-final` baseline). `localization_monitor` does not report "stale: imu". |
| V4 | Panel and lease | DIO comms OK (no `PANEL_STALE`). Selector MANUAL/AUTO shows in `/amr/panel_state`. Start/Reset edges are seen. Lease: IDLE = MANUAL, NAVIGATION = MANUAL\|AUTONOMOUS, bit 8 never set. The `/amr/io` page lists DI/DO. |
| V5 | Arming and drive power | MANUAL: DI08 Manual Arm powers the drives, and releasing it disarms them. AUTO: power only while `RunState.drive_power` is set. The pre-move horn sounds during `RunState.premove`. The alarm horn sounds on EXECUTING/BLOCKED with the field rules, as before. A protective stop during EXECUTING switches DO00 off and DO01 on. |
| V6 | Manual jog | Pendant jog (held, not latched) and the browser jog pad (Manual page, no strip, square pad) both drive correctly. Releasing stops within `stop_d_max`. Mux source names show pendant/manual. |
| V7 | Scanner and fields | nanoScan3 at the new host IP. `/scan` is up. The protective field zeroes FOLLOW/ROTATE. Warning 1 slows (no exemption path is left). Warning 2 stops at 0.40 m. |
| V8 | Mapping | Survey START, then MAPPING, then a drive, then SAVE produces a map revision. The `mapping_session` IMU check passes. |
| V9 | Navigation | NAVIGATION on the saved map. Initial pose from the web. `LocalizationState` reaches ready. Run a route mission to the end, and try Pause/Abort. Route caps and the speed taper are respected at 0.60 m/s. |
| V10 | Web | The nav shows a single Run tab and no Teach link. `/run-tracked`, `/teach` and `/api/line/*` return 404. `/api/mode` with `line` returns 400. Home shows the Speed tile and the trackless job. The Alarms page and event history show no LINE/RFID/MLS_FIELD codes and no UNKNOWN rows from old journals. The Params page loads without errors. |
| V11 | Recovery and faults | Kill the nav layer: it recovers to IDLE or NAVIGATION as `test_unified_sim` expects. E-stop and the safety reset card work. A power-loss restart boots to IDLE. |
| V12 | Rollback drill (optional) | Check out `tracked-final` into a scratch overlay. It must build, which proves the tag can be recovered. |

---

## Critique log

_Main-session correction (2026-10-21): R9, D19, D23, the P6 netplan move, the P7 netplan steps and the matching keep/risk rows were rewritten to match the netplan migration done on gvipc-ce that day. The agents had worked from the repo docs, which still describe the old enp2s0/.1.10 layout._

1. **Partly accepted.** Issue: MLS NMT / TPDO1 stop / SDO-in-Pre-op proof. The MLS boots Operational (`test_mls_track.py:111`, `mls_track.py:306-310`), so "TPDO1 stops by itself after the next power cycle" is wrong. It stops only after a Pre-op such as `drive_forward.py:274`. The SDO gyro proof with node 10 in Pre-op was added to the P3 bench check and V2/V3. D7 and alpha-setup L1236 were reworded to "may or may not be present".
2. **Partly accepted.** Issue: "MLS stream @100 Hz" wording. README L161, alpha-setup L188 and L1240 and canmon L22 were already listed. Added alpha-setup L1236, plus comment-only edits for lss.py:151, test_lss.py:106 and test_rpdo.py. "TPDO1 is no longer started" replaces "TPDO1 has stopped".
3. **Accepted.** Issue: stale build/install artifacts and leftover `amr_line/__pycache__`. Confirmed on this host (`rfid_node`, `tape_synth_node`, `web_panel_node`, the `line_layer.launch.py` symlink, ignored `__pycache__`). V-std/P7 now use `rm -rf build install log`, and executable checks were added.
4. **Accepted.** Issue: P7 deploy paths and install.sh semantics. `amr_ws/amr_ws/deploy` does not exist, and install.sh says it does not start or restart anything. Fixed the paths and added the witnessed restart. The "type hash" wording was corrected.
5. **Accepted.** Issue: D22 missing `mls_cal`, `runs`, `events.jsonl`, `operations.jsonl`. `CAL_DIR` is at L379, `run_log.sh` writes to `~/.amr/runs`, `alarms.get()` returns UNKNOWN for unknown codes, and `operations.jsonl` has 6 line records here. Added archive, purge and journal rotation.
6. **Accepted.** Issue: `station_id_node` in `amr_localization/package.xml:6`. Confirmed; the node does not exist. Removed in P4.
7. **Accepted.** Issue: DRIVES_NOT_READY hint at L87. Confirmed it names the hold cause 'drives', which is being deleted. Reworded in P5.
8. **Accepted.** Issue: the mux calls `field_limit(..., slow_exempt=exempt)`, not `warning_target`. Confirmed at mux L432 and gating L531/L546. P3 rows were fixed.
9. **Accepted.** Issue: wrong test_mode_fsm line numbers. The file has 158 lines; the tests are at L106 and L128. Corrected.
10. **Accepted.** Issue: P3/P4 grep expectations. `test_config.py` rfid hits (L391-472) and `demo.html` `{{ c.product }}` were confirmed. Expected survivors were listed and the P4 pattern narrowed.
11. **Accepted.** Issue: alpha-setup L97/L137 and README L134. Confirmed (netplan example path, "706", "531"). Added to P6.
12. **Accepted.** Issue: arm_policy L9-10 "at Home"/station parking. Confirmed; reworded in P3.
13. **Accepted (optional).** Issue: `line_section`/`line-a-to-b` ids. Confirmed in about 50 places. Listed as an optional consistent rename and as allow-list item 7.
14. **Accepted.** Issue: `gvievo-main` is the vehicle branch. Confirmed by `alpha-setup.md:62` and the local and remote branches. Added D24/R12.
15. **Accepted.** Issue: file counts. `git ls-files` gives uhf-even 79 and amr_line 40 (18 modules plus `__init__`). Counts were corrected, and the checks now use `git ls-files | wc -l == 0`.
16. **Accepted (duplicate of 8).** Issue: a half-removed `slow_exempt` gives NameError/TypeError in `_tick`. Merged into R2/P3, with `test_field_limit` L203-224 handled.
17. **Accepted.** Issue: `test_tracked_product.py` holds the only `Supervisor._allowed` test. Confirmed: `git grep _allowed` in the tests finds only that file (sim excepted). `test_supervisor_lease.py` is ported in P1 before the deletion.
18. **Accepted.** Issue: test_alarm_horn trackless cases exist only in LineState form. Confirmed (L16-17, L35-37, L40-41, L47). Changed to rewrite, not drop.
19. **Accepted (duplicate of 9, plus).** Issue: keep the `'tape'` unknown-kind assert. Confirmed at L132; it is kept and `"line"` was added.
20. **Accepted (merged with 3).** Issue: stale `install/amr_line` through P1-P3. The clean rebuild is now in V-std for every phase.
21. **Accepted.** Issue: Humble has no type hashes, so a mismatch corrupts silently. R6, D4 and P4 were restated, with a mandatory full restart and the keep-the-fields alternative noted.
22. **Accepted.** Issue: the D6 NMT port is not needed for the current SDO gyro. Confirmed: `mls_track.py:251-253` sends NMT only with TPDO1 enabled, and `MlsImu.poll` returns unless in SDO mode. D6 moved to a separate P3b/deferred commit, and the NMT-safety assertion is ported to `test_mls_imu.py` in P3.
23. **Accepted (duplicate of 5).** Issue: `~/.amr/mls_cal`. Added. The tapegraph-sibling claim checks out (`REV_RE = ^rev(\d+)$`).
24. **Accepted.** Issue: incomplete per-phase expected hits. Confirmed `run_log.sh:88`, `test_teach_page.py`, the `LineTeach.srv` amr_line mention, the `run_all.py` rfid history, and `dio.py:69` (also added to the P3 wording edits). Expected survivors are now listed for each grep.
25. **Accepted.** Issue: `0910-fix-plan-001.md` is mixed. Its contents are A1 PP_VETOED, A2 bag_report, A3 `tools/*.sh` and B5 can_confirm, and `.gitignore` L15/L31 cite it. Moved to `manuals/obsolete/` (D25) instead of deleted.