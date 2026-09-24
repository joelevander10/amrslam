# amr_ws — ROS 2 Humble workspace for the SLAM AMR

Plans: `manuals/slam-generalized-plan/amr_implementation_spec.md` and
`manuals/slam-generalized-plan/unified_amr_service_plan.md`. The unified stack
boots hardware and web in IDLE, then changes mapping/navigation layers under one
supervisor. Software integration through commissioning parity is implemented;
the compact vehicle acceptance and witnessed service cutover remain. Operator
and rollback procedure: [RUNBOOK.md](RUNBOOK.md).

**Second vehicle: the AMR QR** (`AGV_PROFILE=amr-qr-01`, platform `qr_analog`: analog
drives, axle CANopen encoders, WitMotion IMU, one `qr_base_node`, ROS 2 Jazzy). Setup,
commissioning order and known limits: [AMR_QR.md](AMR_QR.md); install from zero:
[INSTALL_AMR_QR.md](INSTALL_AMR_QR.md); daily operation: [OPERATE_AMR_QR.md](OPERATE_AMR_QR.md).

## Build, test, run

```bash
cd ~/agv_can/amr_ws
colcon build --symlink-install
source install/setup.bash        # re-source after any build that ADDS a package
python3 -m pytest -q <affected tests>                 # normal edit: focused fast tests
AMR_SIM_TESTS=1 python3 -m pytest -q src/amr_bringup/test/test_unified_sim.py
                                                     # one milestone workflow, sequential
ruff check src/                  # ruff.toml here; spec §0.2
```

The verification policy is Pareto-based: use focused unit/contract tests for an
ordinary edit and the single unified simulation workflow once per integration
milestone. Older component launch suites remain opt-in for a failure in their
component; do not run all of them by default. Full-stack tests must be
sequential on the N97. Their pinned `ROS_DOMAIN_ID`s keep them isolated.

`~/.bashrc` sources `/opt/ros/humble` and this overlay, with
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. A shell started before a package was
added does not see it: `PackageNotFoundError` means re-source, not a bug.
Anything long-lived (`foxglove_bridge`, later systemd units) must be started
from the same overlay, or custom `amr_interfaces` messages will not decode.

| Launch | What |
|---|---|
| `python3 -m amr_bringup.supervisor_node --ros-args -p real:=true` | **production topology behind `amr.service`**: persistent base/web/optional Foxglove, IDLE at boot, at most one replaceable mapping/navigation layer, generation leases and bounded process-group cleanup |
| `amr_bringup base.launch.py real:=true\|false [...]` | **the persistent base layer** (unified plan U1): one URDF publisher, mux, odom, imu_bias, EKF; `real:=true` adds drive_node (can0), panel_node (DIO) and the nanoScan3 (`scanner.launch.py`); `real:=false` adds fake base/IMU/panel and optional `scan_synth:=true`. No web, no Foxglove, no SLAM/AMCL. Required nodes end the launch when they exit |
| `amr_bringup mapping_layer.launch.py [maps_dir] [generation:=N] [internal:=true]` | **mode layer**: slam_toolbox + mapping_session only. One survey = one fresh instance |
| `amr_bringup navigation_layer.launch.py map_id:=… revision:=N [generation:=N] [autostart:=false]` | **mode layer**: verified bundle → map_server, AMCL, localization_monitor, controller, behaviors, lifecycle managers, route_executor. One map = one instance |
| `amr_bringup drivers.launch.py [lidar] [pc_loss_ms] [feedback_hz] [gyro_sign] [panel:=real\|fake]` | bench wrapper = `base real:=true` (+ Foxglove). `sudo systemctl stop agv_controller` first; drive_node/panel_node refuse otherwise |
| `amr_bringup sim.launch.py [slip_noise_std] [scan_synth] [foxglove]` | wrapper = `base real:=false` (+ Foxglove); touches no hardware |
| `amr_bringup mapping.launch.py [sim:=true] [maps_dir] [clutter_count] [foxglove]` | **diagnostics / sim tests only**: base + web + mapping_layer without the supervisor. The web pages need the supervisor's lease and mode (jog, survey start, initial pose and mission controls are refused here); drive with the ROS services and teleop shown below |
| `amr_bringup nav.launch.py [sim:=true] [map_id] [revision] [maps_dir] [clutter_count] [web] [foxglove]` | **diagnostics / sim tests only**: base + web + navigation_layer without the supervisor, same limits. Mutually exclusive with mapping. Interactive operation is `amr.service` (the supervisor) only |
| `amr_bringup lidar.launch.py` | scanner commissioning: `scanner.launch.py` + URDF (+ Foxglove) |
| `amr_description description.launch.py [laser_x:=…]` | robot_state_publisher only |

Survey in sim (spec §6.1), from three terminals:
```bash
ros2 launch amr_bringup mapping.launch.py foxglove:=true
ros2 service call /amr/survey/start amr_interfaces/srv/StartSurvey "{map_id: line_section, description: 'floor mark A'}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop   # drive a loop, come back
ros2 service call /amr/survey/returned std_srvs/srv/Trigger      # closure evidence in the reply and /amr/mapping_state
ros2 service call /amr/survey/save amr_interfaces/srv/SaveMap "{note: 'seams ok'}"
```
Foxglove: 3D panel, frame `map`, show `/map`, `/scan`, `/odometry/filtered`. The saved
bundle lands in `<maps_dir>/<map_id>/rev<N>/` (`map.pgm`, `map.yaml`, `posegraph.*`,
`manifest.yaml`). A failed save leaves `.draft-*`, never a `rev*`.

Localise on a saved map (spec §4.2, §4.3):
```bash
ros2 run amr_mission world_bundle ~/amr_maps            # once: the sim world as sim_factory/rev1 (surfaces only)
ros2 launch amr_bringup nav.launch.py clutter_count:=12 foxglove:=true
# Foxglove: 3D panel, frame map; "Publish pose estimate" on /initialpose at the start mark, then drive a little:
ros2 topic echo /amr/localization_state                   # CHECKING -> can_confirm once converged + settled + scan consistent
ros2 service call /amr/localization/confirm std_srvs/srv/Trigger   # operator: scans align -> READY
ros2 service call /sim/set_pose amr_interfaces/srv/SetPose2D "{x_m: 2.0, y_m: 0.0, yaw_rad: 3.14}"  # kidnap test
```

Draw and run a route (spec §5–§7), web app on **http://<robot>:5001**:
1. `/editor`: pick the map, set the start pose (click, drag for heading), add straights (click ahead) and
   turns (CW/CCW 45/90/180/270), **Validate**, **Save revision**, **Create mission**.
2. `/run`: set the initial pose on the map, drive a little, **Confirm** when the scan aligns → READY.
3. Load the mission (READY), selector **AUTO**, physical **Start** → EXECUTING. Pause / Abort /
   Prepare-resume on the page; Start confirms a prepared resume. In sim the panel is
   `ros2 service call /sim/panel/set_mode std_srvs/srv/SetBool "{data: true}"` and
   `ros2 service call /sim/panel/press_start std_srvs/srv/Trigger`; an obstacle is
   `ros2 service call /sim/add_obstacle amr_interfaces/srv/AddObstacle "{x_m: 3, y_m: 0, size_m: 0.6}"`.

Teleop into the sim: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop`
(only under panel MANUAL: the mux takes authority from the panel, never from the command stream).

## Decisions recorded here

**TF tree.** `map → odom → base_footprint → base_link → {laser_frame, imu_frame,
wheel_left, wheel_right}`. The URDF fixes `base_footprint → base_link`, so the
*moving* transform is `odom → base_footprint` (diff_drive_odom now, the EKF from
T3 with `base_link_frame: base_footprint`). Spec §2.3 says "EKF publishes
odom→base_link" and "base_footprint is the parent of base_link" — both cannot
hold, a frame has one parent. `amr_bringup/test/test_sim_tf.py` launches the
composed sim and fails if any frame ever has two.

**Geometry.** Measured 2026-09-15: track 0.487 m (also in `profiles/agv-01.json`),
wheel radius 0.09 m, nanoScan3 0.964 m ahead of the axle with the scan plane
0.110 m above the floor, MLS IMU 0.092 m ahead of the axle. Laser yaw and
lateral offset verified 2026-09-16 (see "Lidar commissioning"). Still `MEASURE`:
chassis box. Still `VERIFY`: IMU lateral/vertical position inside the MLS.

**Repo modules.** ROS nodes reuse `config`, `kinematics` etc. from the repo
root via `amr_base.agv_repo` (bare imports, layer dirs on `sys.path`, same as
`app/server.py`). `AGV_CAN_ROOT` overrides the search. Profile selection is the
controller's `AGV_PROFILE`.

**IMU chain.** `/imu/data_raw` (raw, biased; the MLS node in T10, `fake_imu`
in sim) → `imu_bias_node` → `/imu/data` → EKF, yaw rate only. The bias node
averages stationary samples (stillness from `/wheel_states`, never from the
gyro itself), publishes nothing until the first 2.3 s window completes, and
re-estimates on every later stop; `/imu/recalibrate` discards the estimate.
This replaces `imu_filter_madgwick` (D-3). Measured in sim with the D-3 noise
figures and 2 % wheel slip over an 18 m square: raw odometry 0.5 % / 1.2°,
fused 0.07 % / 0.2°.

**Survey session (T4).** `mapping_session_node` owns IDLE → MAPPING →
RETURN_REVIEW → SAVING → SAVED. Start needs a fresh scan, a calibrated IMU,
still wheels and `map→base_footprint`; it records the start reference from TF.
"Returned" computes closure evidence = current SLAM pose − reference; nothing
forces the pose onto the mark (the sim test injects a real return error and
checks the evidence reports it). Save pauses `slam_toolbox` (its pause service
is a toggle with no state in the reply, so the node tracks it), serialises the
pose graph, writes the occupancy from the last `/map`, hashes everything into
`manifest.yaml`, verifies, then renames staging → `rev<N>` atomically.
Revisions are immutable; a new survey after SAVED/abort needs a relaunch,
because `slam_toolbox` has no reset service in Humble. PGM `free_thresh` is
0.196 so unknown (205) survives a round trip under both loader conventions.

**Localisation readiness (T5).** `localization_monitor_node` runs
`amr_localization/readiness.py`: UNLOCALIZED → CHECKING (on `/initialpose`) →
READY (operator `confirm`, allowed only when converged, settled 2 s, sensors
fresh and the scan consistent) → LOST. Three loss triggers, all sustained 1 s:
a stale stream (§2.4 ages), covariance growth (σ_xy 0.22 m / σ_yaw 10°), and
**scan consistency**: beams *longer* than the saved map allows from the
estimated pose (they pass through mapped walls). Short beams — unmapped clutter
in front of a wall — are ambiguous and only lower the informational
`scan_match`; with 12 clutter boxes it falls to 35 % in the open area while
AMCL stays within 6 cm. The "map→odom jump" trigger (0.15 m / 5°) is measured
**at the robot** (same current odom pose under old and new transform), because a
0.5° yaw correction 20 m from the odom origin moves the frame 17 cm without the
robot moving. AMCL: `likelihood_field_prob` + beam skip, `sigma_hit 0.1`, no
recovery injection — a lost robot stops and is relocalised by the operator.
Measured in sim (12 clutter boxes, 2 % slip, 0.6 m/s): p95 0.06 m / 0.2°;
kidnap detected in < 5 s of motion; recovery to 2 cm after a new initial pose.
The world-as-bundle fixture (`amr_mission.fixtures`) writes **surfaces only**:
a solid map biases the likelihood field toward the obstacle ahead by ~5 cm
along a corridor, which a lidar-built map never has.

**Lidar window (T6+).** The nanoScan3 is a 275° scanner but the mount has a wall
behind it: about **±95°** is usable (operator, 2026-09-16). `scan_synth` defaults
to 190°/381 beams and `nanoscan3.yaml` masks the driver to ±1.658 rad
(verified on the unit 2026-09-16, nothing returns inside 2.25 m). Consequence for
routes: the swept area behind the vehicle is never observed live, so turn
clearance comes from the *saved map* plus the scan's forward window only.

**Lidar commissioning (T10, lidar half, 2026-09-16).** Measured on the vehicle
with `lidar.launch.py foxglove:=true`, `agv_controller` running (it no longer
touches the scanner):

| Check | Result |
|---|---|
| `/scan` rate | 34.05 Hz, period 26–33 ms (`skip: 0`) |
| stamp → receive delay | ~1 ms; the 0.15 s scan age limit has >100× margin |
| window | −96.2° … +95.7°, 1152 beams at 0.167°, 976 valid; nearest return 2.25 m |
| orientation | box 2.00 m ahead → +x, box 1.20 m left → +y; no yaw/mirror error |
| range | box face at 2.00 m by tape read 2.014 m mean over 100 scans, std 1.2 mm |
| scan plane | 0.110 m above the floor (accepted from the mount measurement) |
| fields | `/output_paths`: 3 valid paths, path 0 is the safety output, monitoring case 1 |

Open: the scanner reported a *contamination warning* (window needs cleaning) and
the `/output_paths` status polarity was not confirmed (echo it with the field clear
and with a box inside). The physical stop is the OSSD pair into the FX3 and is
not checked through ROS.

**Drive node and MLS gyro (T9 + T10 IMU half, 2026-09-16).** `amr_base/drive_node`
is the one owner of `can0`: both BLV-R drives and the MLS IMU in one bus thread,
`amr_base/canopen.py` (PDO layout, scaling, CiA-402 arm/disarm, PC-loss guard) and
`amr_base/mls_imu.py` (gyro by TPDO if the sensor has one enabled, else SDO polling)
underneath, both unit-tested against a scripted bus. Every write goes through
`drivers/canbus/guard.py`; `1016h` (consumer heartbeat) was added to its allow-list.

| Wire | Frame | Rate |
|---|---|---|
| RPDO1 `0x200+n` | `6040h` controlword u16 + `60FFh` target r/min i32 | 50 Hz, both drives, every tick (no deadband) |
| TPDO1 `0x180+n` | `6041h` statusword + `606Ch` velocity r/min | event timer = inhibit = 20 ms (`feedback_hz`) |
| TPDO2 `0x280+n` | `6064h` position counts + `1001h` error register | same |
| PC heartbeat `0x700+100` | NMT operational | 100 ms; drives consume it with `1016h` = `pc_loss_ms` |
| MLS `2034h:3` (+`2035h` every 10th) | SDO poll | 50 Hz (`1806h` yaw-rate TPDO is disabled on the unit) |

Measured with the vehicle parked, drives armed at zero (`ros2 run amr_base drive_node`):

| Check | Result |
|---|---|
| preflight / arm | both drives `0x1737` Operation enabled in ~0.7 s; encoder scale **1 080 000 counts per wheel turn** (`608Fh` 36 000 × gear 30, `6091h` 1:1) |
| `/wheel_states` | 47 Hz (one message per complete TPDO1+TPDO2 pair from both drives), positions valid |
| `/imu/data_raw` | 49.8 Hz by SDO polling; z reads the known ~0.06 °/s bias at rest |
| TPDO spacing | 20.00 ms on both COB-IDs (`candump -t d`) |
| bus load | ~440 f/s total ≈ 45 % of 125 kbps. **Inhibit time must equal the event timer**: with inhibit 0 the servo-locked position counter dithering one count fired TPDO2 on every drive cycle, ~1200 f/s, and the MLS SDO replies missed their 50 ms window |
| exit (SIGINT) | zero setpoint → speed-zero wait → Shutdown/Disable voltage → NMT Pre-op; drives read `0x1A50` Switch on disabled afterwards |

Policy (`canopen.decide`, tabled in `test_canopen.py`): armed automatically with zero
targets (`auto_arm`), retry every 2 s if the enable fails (usually ETO); a drive that
leaves Operation enabled while armed → disarm and re-arm with zero (the safety chain
took it, exactly canworker's level-held MANUAL); a drive that goes **silent** (0.6 s
without frames, *or* without TPDO1 or TPDO2 on its own even while heartbeats/SDO replies
still arrive) or raises an **alarm** → FAULT, setpoint zero at once (sent to each drive
independently, a failed send retried 5 ticks), latched until
`/drives/ack_fault`. If that zero cannot be delivered, or a drive with fresh status is
still not at speed zero 3 s later, `drive_node` stops producing the PC heartbeat so the
drives' own `1016h` reaction takes over (8130h; power cycle). A nonfinite
`/cmd_wheel_vel` clears the command (zero), it does not keep the last one.
`/wheel_states` positions are continuous (wrap-safe `6064h` count deltas, rebaselined
without moving on invalid feedback, re-arm or scale change); a wheel is valid only with
a new, fresh TPDO1 *and* TPDO2, and once started the topic keeps publishing (marked
invalid) when a pair is missing. `DriveStatus.operational` also needs fresh TPDOs. Motion needs a fresh `/cmd_wheel_vel` (0.2 s watchdog inside the
node, `canopen.target_rpm`) which the mux only produces under panel authority, so
arming alone moves nothing. Services: `/drives/arm`, `/drives/disarm`, `/drives/ack_fault`.

**PC-loss response, observed 2026-09-16.** `disarm()` did not retire `1016h`, so a
*clean* exit of `drive_node` left both drives expecting a heartbeat that stopped:
0.5 s later both were in FAULT (error register `0x11`, statusword `0x1A98`) and
`agv_controller` refused to arm — which is the drive-side reaction working as
designed, triggered by the wrong event. `disarm()` now writes `1016h = 0` on
both drives **first**, before the speed-zero wait, so only a crash or a `kill -9`
trips the guard. Clearing `8130h` needs a drive power cycle (40C0h is deny-listed
on purpose); the drives' `1016h` is volatile, so the power cycle also clears it.
Since R05, a zero RPDO precedes the `1016h` clear (non-blocking), the PC heartbeat is
also produced inside the blocking arm/disarm SDO sequences, arm rolls back per node
from its first write, and a disarm that could not finish stays owed and is retried.

**Ground test, 2026-09-16** (`drivers.launch.py panel:=fake lidar:=false`, moves
commanded on `/cmd_vel_teleop` at 0.10 m/s / 0.30 rad/s, closed on `/odom_raw`,
gyro integrated alongside; the vehicle was first checked on blocks: +x turns both
wheels forward, +yaw turns left-back/right-forward):

| Move | Odometry | Gyro ∫ | Note |
|---|---|---|---|
| straight 1.0 m ×2 | 1.047 m each | −0.5° (rest bias × 12 s) | **tape 1.048 m** → 0.1 % |
| reverse 1.0 m | 1.033 m | — | straight |
| CCW 90° | +95.5° | **+96.8°** | `gyro_sign = +1` confirmed |
| CW 90° | −97.9° | −97.8° | |
| CCW 180° | +182.5° (read −177.5° wrapped) | +188.5° | script only sampled 11 Hz of the 50 Hz IMU — see next row |
| CW 180° | −183.9° (read +176.1° wrapped) | **−184.2°** | full 50 Hz sampling: 0.3° |

Odometry scale is right on the ground; gyro and wheels agree on rotation to
< 1 % both ways and across the ±π wrap, so the 0.487 m track is good to a few
mm. Overshoots (4–5 cm, 5–8°) are the test script's reaction time plus the
mux ramp, not the vehicle. After the clean SIGINT both drives read `0x1A50`
Switch on disabled, error register 0, `1016h = 0`: the guard fix holds.

Still to do on the bench (needs a person at the vehicle): **(1)** deliberate
`kill -9` of the node with `pc_loss_ms:=500` — the accidental version above is
the only evidence so far. **(2)** 100 Hz feedback (`feedback_hz:=100`), watching
the bus load and the IMU rate. Blocks/ground moves and `gyro_sign` are done (table above).

**Routes (T6).** `amr_navigation`: spec §6.4 schema (`route.py`), compiler
(`compiler.py`: straight must lie forward on the current heading within 1 mm,
turns keep their full signed magnitude — CW 270 stays −3π/2), validation
(`validate.py`: map id/revision/**sha256** must match the loaded bundle,
allowed angles, footprint sweeps along every line and the full disc of the
footprint's reach at every pivot, unknown cells blocked, optional
`keepout.yaml` next to the map, `repeat_count` closure), stores
(`<map>/routes/<id>/rev<N>.yaml`, `missions/<id>.yaml` with both hashes).
Footprint: **one file**, `amr_description/config/footprint.yaml` (MEASURE),
read by the validator and injected into the Nav2 local costmap at launch;
margin 0.20 m = cross-track limit 0.10 + localisation allowance 0.10.
`amr_web` (Flask + rclpy adapter, port 5001) serves the maps/survey page, the
canvas editor (map metres persisted, pixel↔world identical to `amr_maps.grid`)
and the run page. No endpoint publishes a velocity.

**Execution (T7).** `route_executor_node` runs one Nav2 action at a time:
`FollowPath` (RPP, `desired_linear_vel 0.30`, no rotate-to-heading, no
reversing, goal checker **0.025 m** — at 0.05 the checker fires the instant it
is inside and every straight ends ~4.5 cm short, which becomes cross-track at
the next corner) and `Spin` (behavior_server, `cmd_vel` remapped to
`/cmd_vel_rotate`, 0.30 rad/s). No planner, no BT: nothing undrawn can move
the vehicle. Checks against the estimated pose: cross-track (limit 0.10 m,
twice that during the first metre while RPP converges), endpoint within
0.08 m / 5° **and** wheels still 0.3 s before advancing, turn travel from the
executor's own unwrapped odometry yaw within 2°, centre drift on odometry
≤ 0.05 m, final map heading within 5° (hardware placeholders: sim AMCL yaw
wanders 1–3° during a spin). A turn's commanded angle folds in the entry heading
error (bounded ±10°) so consecutive turns land on the drawn heading. Measured in sim: straights end within
2 cm, turns within 3°, peak ground-truth cross-track 0.064 m.

**Permission (T7/T8).** The mux (`amr_base.gating`) takes authority from the
**panel** (MANUAL = teleop, AUTO = executor) and the executor's `MotionPermit`
lease (FOLLOW or ROTATE, 0.3 s expiry); a command must also be fresh (0.2 s);
loss of authority zeroes output at once. Executor states IDLE → READY (mission
loaded, localisation READY, start gate 0.10 m / 5°) → EXECUTING on a physical
Start edge under AUTO → PAUSED / BLOCKED (progress kept; `prepare-resume`
re-checks pose, corridor, clearance stable 1 s, sensors, AUTO; Start
continues the *remaining* straight or signed angle) / FAULT (stale wheels,
panel, localisation LOST, `/initialpose` mid-step; `ack` only returns to
IDLE) / DONE. Selector to MANUAL mid-run aborts. Obstruction = scan points in
free map space inside the active step's swept footprint (1.5 m horizon on a
line, the full disc on a turn). Sim panel: `fake_panel_node`; the real adapter
(T12) publishes the same `PanelState`.

**Panel and horn (T12, 2026-09-16).** `amr_base/panel_node` wraps the repo's
`drivers/dio.py` scan thread and `core/panel.py` debounce unchanged and publishes
`/amr/panel_state` at 50 Hz: `valid` only while DIO comms are good and a
debounced baseline exists (anti-tie-down: a button held at start produces no
edge; a comms gap re-baselines instead of inventing a press). The horn coil
follows canworker's rule at the ROS boundary — drives armed **and** a fresh
non-zero `/cmd_wheel_vel` — as a claim renewed every tick with `HORN_HOLD_S`, so
a dead node drops it on the next DIO scan. No services, no parameters that could
fake an edge. `drivers.launch.py panel:=real` is the default; `fake` is the
simulated panel for the bench. Both `drive_node` and `panel_node` refuse to
start while the `agv_controller` unit is active (`amr_base.legacy_guard`): socketcan
and Modbus TCP both accept a second client, so nothing else would stop two owners.

Verified at the panel 2026-09-16, and this found a wiring error in the profile:
the real mapping is **DI1 Start, DI2 Reset, DI3 AUTO/MANUAL**, DI0 empty
(`profiles/agv-01.json` shipped 0/1/2, so the legacy app had been reading a
Reset press as a flick of the selector and never saw the real selector at all).
With the corrected profile: one Start press → one edge, one Reset press → one
edge, the selector level tracks, Start held 3 s → one edge, 50.0 Hz.

Live under the real panel the same day (`drivers.launch.py lidar:=false`, moves
on `/cmd_vel_teleop` at 0.10 m/s): selector **AUTO** → 21 s of forward commands,
0.000 m; **MANUAL** → 0.320 m, horn and lights on while moving, off on stop;
selector flipped to AUTO **mid-move** → stopped at 0.329 m and stayed stopped
while commands continued; reverse 0.667 m under MANUAL, horn on. The DIO
cable-pull case (comms loss → `valid=false` → no authority) is covered by
`test_panel_io.py`, not yet pulled on the vehicle.

**Sim world.** `amr_maps/worlds/sim_factory`: 30 × 20 m, four 16 m rack rows
at y = ±2.4 / ±7.2, start mark (0, 0) at the west end of the B/C aisle.
`scan_synth_node` raycasts it from ground truth with the URDF laser offset,
275°, 551 beams @ 10 Hz by default (configurable up to the wire's 1652),
σ 0.01 m, optional clutter boxes. Survey acceptance: occupied cells agree with
the world > 0.9 both ways at 2-cell tolerance (measured 1.000).

**Acceleration limits.** `cmd_mux` defaults `a_max`/`alpha_max` to
`min(spec, hardware)` where hardware is the profile's 6083h ramp through
`kinematics.max_yaw_accel()` (reconciliation D-1). Parameters can lower them,
never exceed hardware.

**Lidar ownership.** `sick_safetyscanners2` and the controller's
`drivers/lidar.py` both needed `192.168.3.2:6060`; only one could hold it.
Decision 2026-09-15: the ROS driver owns it. The legacy listener, the `/lidar`
page and the zone rail were removed on 2026-09-16; the controller no longer
touches the scanner. The ROS driver writes the scanner's
channel-0 data-output settings over CoLa2; that is not the verified safety
configuration (fields, monitoring cases), which stays humans-only.

**Python environment.** User-site `numpy 2.2.6` + `opencv-python-headless 5.0`
are a consistent pair and are what `amr_tools` (mask painting, T4) will use.
They break `cv_bridge` (system, built on numpy 1.21). Nothing in this project
uses `cv_bridge` — there is no camera (D-4) — so this is a known, accepted
conflict. Do not import `cv_bridge`; if a camera ever appears, revisit.

**ROS domain / DDS scope (2026-09-16).** Vehicle = `ROS_DOMAIN_ID=10`,
simulation = `20`, launch tests pin 61–67 (`amr_bringup/domains.py`). The shell on
the vehicle sources `env/vehicle.sh` from `~/.bashrc`; for sim work in a shell,
`source ~/agv_can/amr_ws/env/sim.sh`. This is enforced, not advisory:
`drivers.launch.py` (and `mapping`/`nav` with `sim:=false`) refuse any domain but
10; `sim.launch.py` (and `mapping`/`nav` with `sim:=true`) refuse 10 and refuse an
*unset* domain, because default 0 is what every unconfigured ROS host on the LAN
uses. DDS is loopback-only (`config/cyclonedds-local.xml`, `CYCLONEDDS_URI`):
nothing off this host can join either graph, nothing is bound on the Wi-Fi
address, and a sim on this host cannot leak either. Foxglove is unaffected — it
uses `foxglove_bridge` on :8765, not DDS. `lo` has no multicast, so discovery is
unicast over `MaxAutoParticipantIndex` = 120 slots; raise it if a launch ever has
more participants. To let a laptop join deliberately: copy the XML, bind `wlp1s0`,
point `CYCLONEDDS_URI` at the copy for that session only.

Also sourced by anything long-lived: `deploy/amr-supervisor.sh` is the unified
systemd entry point; `deploy/amr-launch.sh` remains the standalone legacy-unit
wrapper during the rollback window.

**Unified deployment (U9).** `deploy/amr.service` conflicts with all three old
application units, binds to can0 and starts the supervisor through the coherent
overlay/domain wrapper. It has a bounded restart rate and preserves SIGINT for
ordered lease revocation and drive/panel shutdown. `amr.env` contains paths and
feature switches only; boot never selects a map or resumes work.
`deploy/validate.sh` is the no-side-effect P8 check. `sudo deploy/install.sh`
backs up and installs unit files but never changes enabled/running state.
Vehicle acceptance, witnessed cutover and rollback: [RUNBOOK.md](RUNBOOK.md).
