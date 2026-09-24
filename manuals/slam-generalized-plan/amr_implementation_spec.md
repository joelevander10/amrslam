# AMR Implementation Specification — Manual Mapping and Drawn Routes

**Revision:** 2026-09-15, operator workflow revision.

**Progress:** T1–T3 are complete; preserve and extend their implementation.

**Workflow:** manually drive and map the operating area, return to the starting position and orientation, save the map, draw a route, then run it using encoder/IMU/lidar localization.

**Authority:** This revision supersedes conflicting workflow, hardware, interface and task instructions in [amr_slam_architecture.md](amr_slam_architecture.md), the previous implementation spec and [hardware-reconciliation.md](hardware-reconciliation.md). The reconciliation remains useful background. Current profile/driver code and verified measurements take precedence over older generic hardware assumptions. Future hardware changes must update this baseline explicitly.

**Terminology:** SLAM builds the map during the manual survey. At runtime, wheel encoders and the IMU provide local odometry, and lidar localizes the robot against the saved map using AMCL. Runtime does not continually rewrite the approved map. This supplies the requested map-based navigation while keeping the reference for drawn routes stable.

## 0. Baseline and implementation rules

### 0.1 Hardware checked against the current codebase

This records the hardware targeted by active code, not the original generic BOM. Code inspection does not independently verify wiring, firmware or mechanical calibration. Host OS/CPU/CAN service were also inspected read-only on 2026-09-15.

| Item | Current baseline | Evidence / qualification |
|---|---|---|
| Drives | Oriental Motor BLV-R / BLVD-KRD, nodes 1 left and 2 right | [README](../../README.md), [controller](../../canworker.py), [profile](../../profiles/agv-01.json). No Kinco DEC conversion. |
| Encoders | Motor-integrated: signed `6064h` position, `606Ch` velocity | Controller reads position and checks scaling with `608Fh`/`6091h`. No separate CiA-406 encoders. Motor feedback does not directly measure ground slip or gearbox backlash. |
| Geometry | Wheel radius 0.09 m, track 0.487 m, gearbox 30:1, motor maximum 4,000 rpm | Active profile and [URDF](../../amr_ws/src/amr_description/urdf/amr.urdf.xacro). |
| IMU | Inside SICK MLS, CAN node 10 | [read_imu.py](../../drivers/canbus/read_imu.py): gyro `2034h`, timestamp `2035h`. No Yahboom parser or Madgwick stage. |
| Lidar | SICK nanoScan3; sensor `192.168.3.10`, host `192.168.3.2`, UDP 6060 | [ROS config](../../amr_ws/src/amr_bringup/config/nanoscan3.yaml). Approximately 34 Hz previously observed; use incoming scan geometry rather than fixed beam counts. |
| Sensor mounts | Lidar x = 0.964 m ahead of axle, scan height = 0.110 m above floor; IMU x = 0.092 m ahead | Measurements recorded in [workspace README](../../amr_ws/README.md)/URDF. Laser lateral offset/yaw and IMU lateral/vertical offsets still have verification markers. |
| Collision footprint | Not yet a verified physical outline | URDF chassis dimensions are placeholders. Measure body, protrusions and relevant load before physical route validation. Sensor position alone does not define the footprint. |
| RFID | Chafon CF821, TCP `192.168.1.200:2022` | [rfid.py](../../drivers/rfid.py); current parser extracts a two-byte ID. Optional identity/coarse seed, not a precision pose source. |
| I/O and panel | Modbus TCP `192.168.1.30:502`, 16 DI/16 DO | [dio.py](../../drivers/dio.py): Reset DI0, Start DI1, AUTO/MANUAL DI2, horn DO0 in profile. Preserve physical-panel motion authorization. |
| CAN interface | CANable 2.0; SocketCAN `can0` over SLCAN, 125 kbit/s; `use_rpdo: false` | Host `canable@.service` runs `slcand -s4`. Native `gs_usb` and 1 Mbit/s are not the present baseline. |
| Host | N97, four cores, approximately 8 GB RAM; Ubuntu 22.04.5, Python 3.10, ROS 2 Humble | Host inspection and workspace. No OS upgrade required for the next tasks. |

The code does not establish complete physical stop-chain wiring. Preserve independently wired stop functions; this phase does not implement or claim safety-rated software. `lidar.zones_validated` is false in the active profile: unvalidated field bits must not be presented as verified clearance.

### 0.2 Completed work

| Task | Implementation to retain |
|---|---|
| T1 — DONE | Interfaces, description and scaffold. Current messages include `StationDetection`, not `QrDetection`. |
| T2 — DONE | Fake base/wheel model, command mux, raw odometry, kinematics and square-drive tests. |
| T3 — DONE | Fake IMU, stationary gyro-bias correction, EKF, fused-odometry test and composed TF test. |

Completion is confirmed by the operator and workspace README; source and existing passing test artifacts were inspected. This documentation edit does not rerun those tests. Extensions specified below are new work, not a request to redo T1–T3.

### 0.3 Coding rules

- Python 3.10-compatible authored nodes; upstream packages supply existing C++ algorithms. Public functions have type hints; use workspace ruff rules.
- Reuse `config`, `core/kinematics`, CAN guard and framework-independent drivers through `amr_base.agv_repo`. Do not import `app.server` into ROS nodes: it constructs the legacy controller.
- The strict vehicle profile owns hardware configuration. ROS YAML owns algorithm/launch settings; avoid independently editable copies of geometry and drive limits.
- SI internally. Route files/UI may use degrees and `cw`/`ccw`; convert explicitly at ROS boundaries.
- No blocking CAN/HTTP work or sleeps in control callbacks. Serialize CAN ownership/state transitions even with worker threads. INFO logs state changes; DEBUG carries periodic detail.
- Each task has relevant simulation/unit verification. Hardware work includes a bench checklist; this document does not authorize unattended actuation during coding.

## 1. Operator workflow and workspace

### 1.1 Workflow

| Stage | Operator action | System behavior |
|---|---|---|
| Map | Select New map, mark physical start position/heading, manually drive the area | Live SLAM with encoders, IMU and lidar; show growing map and survey trace. |
| Close survey | Return manually to the same physical position/orientation; select Returned to start | Review closure evidence. The operator returns the vehicle; scan matching and graph optimization estimate the closure. |
| Save | Review and save a named map revision | Save occupancy map, complete serialized SLAM session, start reference and review metadata together. |
| Draw | Open saved map; draw straight moves and insert directed turns | Validate continuity, allowed angles, footprint clearance and map revision; save route. |
| Run | Load route, establish localization at its start, select AUTO and press physical Start | Execute the ordered route; stop before each explicit rotation. |
| Obstruction/intervention | Clear the cause and explicitly prepare resume | Remain stopped until prerequisites hold and physical Start confirms resumption; do not choose another aisle. |

Manual movement means operator-controlled powered teleop using the existing operating procedure. It does not assume disabled drives can be pushed or authorize releasing a motor brake.

### 1.2 Route editor: web app

**Use a small web app for operator mapping/route editing; retain Foxglove for engineering inspection.** The page supplies constrained line/turn tools, undo, validation, save/load and run-state display. The laptop browser renders; the robot executes control.

Foxglove supports map/path visualization and click-to-publish poses/points through a capable connection ([3D panel documentation](https://docs.foxglove.dev/docs/visualization/panels/3d)). Ordered-route storage, angle constraints and revision validation still need application logic. A dedicated web page fits the existing Flask experience and expresses these rules directly; no custom Foxglove extension is required initially.

Reuse suitable styles/pure UI helpers, but put the ROS-facing backend in `amr_web`. It must not instantiate the legacy controller. Selecting or saving anything in the editor does not command motion.

### 1.3 Responsibilities

| Package | Responsibility |
|---|---|
| `amr_interfaces`, `amr_description`, `amr_base`, `amr_sim` | Preserve T1–T3 foundations; extend interfaces, permission gating and simulation. |
| `amr_localization` | Existing bias/EKF plus AMCL and localization readiness. |
| `amr_navigation` | RPP/controller, costmap and Spin configuration; route geometry/clearance validation. |
| `amr_mission` | Route compiler/executor, action sequencing, permission/stop state, optional station events. |
| `amr_web` | Mapping/session controls, route editor, live state and ROS API adapter. |
| `amr_bringup` | Exclusive mapping/runtime/simulation profiles and lifecycle/ownership coordination. |
| `amr_maps`, `amr_tools` | Versioned artifacts, survey review and optional offline diagnosis. |

## 2. Interface contracts

### 2.1 Topics

Retain existing `amr_interfaces` definitions. New status messages are additions; no camera messages are required.

| Topic | Type / producer | Contract |
|---|---|---|
| `/wheel_states` | Existing `WheelStates`; drive owner or fake base | Wheel SI position/velocity, validity/acquisition timestamp. Target 100 Hz after transport validation. Odom and bias node consume it. |
| `/odom_raw` | `nav_msgs/Odometry`; existing odometry node | Position increments generate odometry; EKF presently fuses twist only. No TF in normal operation. |
| `/imu/data_raw` | `sensor_msgs/Imu`; MLS adapter or fake IMU | Raw biased measurement for existing bias node; target 100 Hz subject to TPDO validation. |
| `/imu/data` | `sensor_msgs/Imu`; bias node | Corrected yaw rate for EKF; available after calibration. |
| `/odometry/filtered` | `nav_msgs/Odometry`; EKF at 50 Hz | Local motion estimate; SLAM/AMCL access odometry through TF. |
| `/scan` | `sensor_msgs/LaserScan`; SICK driver or synthesizer | SLAM during survey; AMCL/costmap at runtime. Preserve timestamps/geometry. |
| `/map` | `nav_msgs/OccupancyGrid`; SLAM or map server | One owner per mode; viewer and static costmap consume it. |
| `/cmd_vel_teleop` | `geometry_msgs/Twist`; manual adapter | Mux accepts only under MANUAL/teleop authority. |
| `/cmd_vel` | `geometry_msgs/Twist`; Nav2 controller server | Mux accepts only during an authorized straight move; no other runtime producer here. |
| `/cmd_vel_rotate` | `geometry_msgs/Twist`; remapped behavior server | Mux accepts only during an authorized explicit turn. |
| `/cmd_wheel_vel` | Existing `WheelVelocities`; mux at 50 Hz | Sole drive owner/fake base; independent command timeout. |
| `/drives/status` | Existing `DriveStatus`; drive owner | Prerequisites/diagnostics. Zero command alone does not prove stopped wheels. |
| `/scan/field_status` | Existing `SafetyFieldStatus`; explicit SICK-message adapter | Informational; track freshness/validity separately. Native driver messages do not automatically have this custom type. |
| `/station/detections` | Existing `StationDetection`; optional RFID adapter | Identity only. |
| `/amr/route_preview` | `nav_msgs/Path`; route compiler | Web/Foxglove preview; separate `MarkerArray` topic shows numbered turns with direction/magnitude. Path alone cannot represent directed rotations. |
| `/amr/motion_permit` | New `MotionPermit`; coordinator at 10 Hz | Header, `run_id`, allowed source (`none`, `teleop`, `follow`, `rotate`), enabled flag, reason. Mux expires on local receipt age >0.3 s. |
| `/amr/run_state` | New `RunState`; coordinator | State, run ID, map/route revision, step index, remaining turn, readiness/block reason. |

Sensor streams use best-effort/volatile QoS compatible with existing nodes. Commands are reliable/volatile depth 1; map reliable/transient-local; TF standard ROS TF QoS. Match actual upstream publishers/subscribers, including EKF odometry, rather than declaring every stream SensorData. The backend explicitly converts ROS state to its web API.

### 2.2 Services/actions

| Interface | Contract |
|---|---|
| `/amr/run_mission` | Retain `RunMission`: `mission_id` selects a manifest containing one validated route revision. Acceptance loads READY; physical Start authorizes execution. |
| `/amr/pause`, `/amr/abort` | `std_srvs/Trigger`; inhibit output and cancel the active action. Abort discards resumable progress. |
| `/amr/resume` | `std_srvs/Trigger`; validate and prepare continuation; physical Start confirms it. |
| `/initialpose` | Operator estimate or explicitly accepted optional RFID seed, while stopped and relocalizing. |
| `/follow_path` | Humble `nav2_msgs/action/FollowPath`, one straight segment, explicit RPP/goal-checker IDs. |
| `/spin` | Humble `nav2_msgs/action/Spin`, signed relative angle and bounded time allowance. |
| `/imu/recalibrate` | Existing service; AUTO remains inhibited until corrected IMU/readiness returns. |

Web API covers map/route list/load, start/finish survey, return confirmation, map review/save, route validation/save, mission load, pause/abort/prepare-resume and streamed state. These call a robot-side coordinator. The browser never schedules route wheel commands. Preserve panel-owned enable/acknowledgement; do not add unrestricted drive-reset endpoints.

### 2.3 Frames — retain the completed T3 tree

| Transform | Owner |
|---|---|
| `map → odom` | SLAM Toolbox during mapping; AMCL at runtime; mutually exclusive. |
| `odom → base_footprint` | EKF, `base_link_frame: base_footprint`. |
| `base_footprint → base_link` | URDF fixed axle-height offset. |
| `base_link → laser_frame`, `imu_frame`, wheels | URDF with verified sensor extrinsics. |

Route poses locate `base_footprint` in `map`. Configure all consumers consistently. Do not reintroduce `odom → base_link`; the composed TF test already covers the resolved tree.

### 2.4 Time/validity

- Stamp at acquisition and preserve the stamp through processing. Track left/right ages and reject stale or excessively skewed pairs.
- Unwrap signed encoder counters and MLS's 16-bit millisecond clock. Distinguish wraps from restart; discard the first delta after reset/invalid gaps.
- Use monotonic local time for live watchdogs/leases; ROS time for sensor/TF alignment and simulated replay. Clock adjustments must not extend permission.
- Reject invalid/out-of-order data; republishing must not make old measurements look fresh.
- Initial age limits: scan 0.15 s, wheels 0.10 s, corrected IMU 0.20 s, odom/required TF 0.20 s. Validate against measured latency. A stale required stream inhibits AUTO and survey capture; low-speed manual recovery is a separate operator mode subject to drive/panel health.

## 3. Base and hardware integration

### 3.1 One CAN owner

Implement `drive_node` for both BLV-R drives; the MLS adapter shares one serialized CAN dispatcher. Reuse verified helpers/guard rules. Do not run independent SDO clients that drain each other's replies or run the legacy controller alongside ROS actuator ownership.

Port arm/disarm/fault behavior as a state machine with partial-transition cleanup and bounded I/O. Arm with zero targets. Disarm, abort, failed enable and restart invalidate autonomous permission/progress. Fault acknowledgement does not resume a route. Preserve prohibited writes and recursively guarded PDO mappings.

### 3.2 Wheel units and feedback

`60FFh` uses signed motor rpm; ROS commands use wheel rad/s. Include mechanical ratio and configured signs. Read/validate actual position scaling and relevant drive settings; do not apply gearbox/feed factors twice. Motor-integrated encoders still require signed counter-wrap handling.

Target control 50 Hz and feedback 100 Hz via PDOs once validated. Statusword (2 bytes), velocity (4) and position (4) do not fit in one classic 8-byte PDO. Define the multi-PDO layout and pair timing before calculating traffic. Keep occasional diagnostics out of the control path.

### 3.3 CAN readiness

125 kbit/s SLCAN is the baseline, not a guarantee of target rates. Bench-verify PDO support, aggregate traffic, latency, drop/reconnect behavior and command loss. A coordinated 1 Mbit/s migration is an option if needed; it does not block simulation/UI work. Host setup must configure the actual transport, not merely change a profile value.

Specify and verify an independent drive response to PC/CAN-owner loss. Drive heartbeat production lets the PC monitor the drive; it does not configure the drive to detect PC death. Do not infer holding torque or stopping behavior from a generic CiA-402 state name.

### 3.4 MLS, lidar and I/O

- MLS: reuse object dictionary/scaling and respect the four-active-TPDO/COB-ID constraints. Verify delivered rate and timestamps; internal 200 Hz sampling does not imply a 200 Hz ROS stream.
- Lidar: ROS owns UDP 6060 during ROS operation. CoLa2 data-output setup is an explicit commissioning operation. Do not run the legacy listener on that endpoint or equate field messages with verified physical stop-chain wiring.
- DIO: wrap the existing threaded driver. Panel AUTO/MANUAL and Start/Reset remain authoritative; stale panel data inhibits permission. Never allow competing DO writers.
- RFID: add only when station features require it. Map/draw/run works without tags.

### 3.5 Mux extensions to T2

Keep kinematics and curvature-preserving speed limits; add permission/source gating and independent command freshness before physical route operation. A 0.5 s teleop ownership window must not retain a nonzero command past the 0.2 s motion timeout. Teleop expiry does not automatically resume AUTO.

Use the mux as the software acceleration limiter; omit an additional Nav2 velocity smoother initially. Respect drive accel/decel and combined per-wheel slew during mixed translation/rotation. Fault/timeout inhibits bypass ordinary smoothing. Measure actual stopping separately from zero-setpoint issuance.

## 4. Estimation and runtime localization

### 4.1 Existing EKF/IMU chain

Retain [ekf.yaml](../../amr_ws/src/amr_localization/config/ekf.yaml): planar, 50 Hz, wheel forward velocity and yaw rate plus corrected IMU yaw rate. No absolute wheel pose or IMU yaw fusion. This implemented T3 baseline supersedes the older statement that gyro exclusively owns heading. Tune wheel-yaw covariance for backlash/slip before changing policy.

Retain stationary bias estimation: fresh valid wheel stillness, 0.3 s settle plus 2.0 s samples, no corrected publication before initial calibration. Map/run readiness includes calibration. Hardware validation checks bias stability and minimum usable sample count; stationary simulation is not sensor qualification.

### 4.2 AMCL

Use saved occupancy, `/scan` and EKF odometry TF; explicit frame IDs. Initial configuration: `laser_model_type: likelihood_field_prob`, `do_beamskip: true`, 500–2,000 particles, update thresholds 0.05 m / 0.03 rad. Tune against factory data. Humble's default `likelihood_field` does not gain beam skipping merely from `do_beamskip` ([source](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_amcl/src/amcl_node.cpp)).

At startup the operator selects approximate map position/heading or confirms placement at the saved start mark. AMCL refines it. Do not silently treat a route start or saved shutdown pose as current physical position.

### 4.3 Readiness

States: UNLOCALIZED, CHECKING, READY, LOST. Require fresh sensors/TF, stable localization and configured covariance/consistency limits. Initial acceptance includes operator confirmation that scans align with fixed structure. Low covariance alone cannot establish the correct aisle.

An unexpected map correction or stale localization stops execution and requires revalidation. Initial correction trigger: >0.15 m or 5° change in `map → odom` between accepted updates; tune from logs. No `/initialpose` injection during a segment.

Optional RFID seeding is stopped/explicit only, with a calibrated broad read-zone prior and uncertain heading. Repeated reads must not reset converged AMCL. IDs must be unique in the deployed registry; the two-byte extraction is not a universal EPC identity guarantee.

## 5. Fixed-route execution with Nav2 components

### 5.1 Execution choice

Use controller server/RPP for supplied straight paths and behavior server/Spin for explicit turns. The executor calls these actions directly. This profile does not use `NavigateToPose`, Smac replanning or a stock recovery BT, which could insert undrawn paths/movements.

Run map server, AMCL, local costmap, controller and Spin under lifecycle management. RPP follows a path with collision checks; it is not a detour planner ([Humble documentation](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/README.md)). Obstacle policy: stop, wait, explicitly resume the same route.

### 5.2 Straight moves

- Compile each line into a sampled `nav_msgs/Path` in `map`, initial spacing 0.05 m, with the segment's forward heading.
- One `FollowPath` goal per line; explicit RPP/goal-checker IDs. Disable reversing and implicit rotate-to-heading behavior; no smoothing across corners. Small steering corrections to track the line are expected.
- Route speed cap 0.50 m/s (initially 0.30; 0.40 on 2026-09-17, 0.50 on 2026-09-18), reducible per route. Step types: `straight` (to a point on the heading line), `rotate` (in place, 45/90/180/270), `reverse` (≤ 2 m back along the heading at half the cap, rear unscanned), `arc` (forward along a circle, R ≥ 1 m, 45–180°, speed ≤ 0.9·w_cap·R). A straight longer than `long_min_length_m` (default 4 m) may run at `long_linear_mps` (up to the vehicle ceiling 0.70 m/s; null = off). The executor sends the step's speed to the controller on `/speed_limit` and as the permit `v_max`.
- Initial endpoint tolerance 0.05 m / 2°; maximum cross-track error 0.10 m plus route-specific clearance checks. Passing the endpoint outside tolerance faults; it does not authorize reversing/circling back.
- Verify endpoint and stopped feedback for 0.3 s before advancing; an action success alone is insufficient.

### 5.3 Directed rotations

- Allowed magnitudes: **45°, 90°, 180°, 270°**, with independent **CW/CCW** choice, viewed from above. ROS positive yaw is CCW.
- Store direction and full magnitude. **CW 270° remains CW 270°**, not CCW 90° merely because the final orientation matches.
- Send signed relative Spin angle and a bounded allowance derived from angle/speed/ramp. Cap 0.34 rad/s (initially 0.30; 0.24 on 2026-09-17; +40 % to 0.34 on 2026-09-18 with `rotational_acc_lim` 0.17); set minimum spin speed low enough for the stopping tolerance.
- Track unwrapped accumulated yaw in continuous `odom`; independently verify direction, total travel and final map heading. Do not use shortest-angle-to-goal as progress. Humble exposes relative Spin goals and traveled-angle feedback ([action](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_msgs/action/Spin.action), [implementation](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_behaviors/plugins/spin.cpp)).
- Translation stops before rotation. Initial bounds: travel/final heading within 2°, centre drift within 0.05 m, stopped for 0.3 s. Out-of-tolerance results require review, not an unrequested corrective turn.
- Pausing retains completed angular travel. Resume only the remaining signed angle after validating continuity; never repeat a whole 270° turn after partial execution.

### 5.4 Clearance/obstructions

Use a measured polygon footprint and margin. Validate the translated footprint along every line and the complete swept area of each rotation, not just centreline/endpoints. Unknown map cells are blocked for approval.

Initial local costmap: rolling 5 × 5 m, 0.05 m resolution, static map plus live obstacle layer/inflation and configured footprint. Any keepout mask applies to both offline route validation and runtime checking. Enlarge the window if footprint/stopping horizon requires it.

Retain RPP/Spin collision checks. Additionally gate execution on current obstacle data and the active step's swept footprint/stopping envelope; stock behavior alone is not acceptance evidence for every turn. With one front-biased 275° scanner, unobserved space is not assumed clear. Initially reject turns whose required sweep cannot be established clear under the commissioning coverage policy; allowed turn locations may be restricted until coverage is verified.

An obstruction enters BLOCKED with zero permission. After clearance is stable for 1 s, the operator may prepare resume; physical Start confirms it. No automatic detour, backup, recovery spin, or costmap clearing to remove a real obstruction. Device/localization faults enter FAULT rather than indefinite obstacle waiting.

## 6. Manual mapping, map save and editor

### 6.1 Live survey and mandatory return

1. Select New map while stopped in MANUAL. Verify encoders, calibrated IMU, lidar and TF. Mark axle-centre position and heading physically; save a descriptive reference.
2. Start SLAM Toolbox **online asynchronous mapping**, loop closing enabled. Record the start reference. AMCL/route control are inactive; local odometry and manual control remain active.
3. Manually drive through the operating area at ≤0.30 m/s, slowly around corners, revisiting junctions and stable structures from multiple directions. Show live map, pose, trace and health. Survey trace is evidence, not the eventual route.
4. **Return physically to the same start mark and heading.** This is mandatory for an approvable survey. Initial placement target: within 0.10 m and 5°. This is a commissioning target, not a claim that the UI independently measures placement error.
5. Stop, allow matching/optimization to settle, then select Returned to start. Overlapping observations let SLAM estimate closure. Never force final pose to equal initial pose or insert a zero-error constraint solely from the button press.
6. Review start-area alignment, seams and wall consistency. If poor, drive another overlapping loop and return again. Operator return confirmation alone does not prove a graph loop constraint was accepted.

This replaces the previous offline-only default: mapping runs and is visible while driving. SLAM Toolbox supports online asynchronous mapping and serialization ([Humble documentation](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/humble/README.md)). Use a minimal mapping launch; throttle UI/map refresh and, if necessary, processing based on measured latency. Do not silently substitute recording without a live map. Stop and diagnose overload; optional bags permit later investigation/reprocessing.

Here, “manual loop closure” means the operator physically closes the survey trajectory and reviews the result. A pose-graph node-dragging UI or forced-constraint editor is not required initially.

### 6.2 Review and coherent save

States: IDLE, MAPPING, RETURN_REVIEW, SAVING, SAVED; failures retain a draft. Approval requires return confirmation, usable sensor data and closure review. Expose accepted loop-constraint evidence where available; otherwise label the result operator-reviewed rather than claiming automatic verification.

Initial review targets: start-area seams/double walls no larger than two 0.05 m cells; known physical spans within 2% of measurement. These are proposed commissioning checks, not guaranteed accuracy. Ambiguous/poor structure requires resurvey. Keep movable clutter out of the reference where practical; never erase unobserved space into trusted free space.

After review, stop admitting new mapping data using the supported backend operation, retain the final solution, and save a coherent bundle:

- Occupancy image/YAML with resolution, origin, thresholds and interpretation.
- Complete SLAM serialization, including companion data files, for continuation/inspection.
- Manifest: map ID/revision, hashes, frame/geometry, profile/sensor revision, software/config versions, physical start reference, return/review results and timestamps.
- Optional MCAP and metadata linked to the session.

Stage writes, verify all files, then publish the revision atomically. Saved means the entire bundle succeeded. Approved revisions are immutable. Draw routes only after final map optimization/save; map changes require new revisions and route revalidation.

Optional bags include `/scan`, `/wheel_states`, `/imu/data_raw`, `/imu/data`, `/odom_raw`, `/odometry/filtered`, `/tf`, `/tf_static` and configuration/session metadata. Replay isolated from hardware with `/clock` and `use_sim_time`. Use recorded odometry TF or recomputed EKF TF, never both; filter recorded mapping `map → odom` when rerunning SLAM.

### 6.3 Editor behavior

- Top-down saved map, footprint, editable start pose, numbered lines/turns, CW/CCW arrows and angles, zoom/pan and undo/redo.
- Straight tool adds forward segments. Turn tool changes outgoing heading by an allowed directed angle. Preview geometry; reject or explicitly offer snapping for invalid bends rather than silently rounding them.
- Arbitrary initial heading is allowed. Subsequent changes use explicit 45/90/180/270 turns. Collinear moves need no turn; no reverse/arc/spline primitives initially.
- Save/load route revisions; validate map identity, length, angle, continuity, footprint and tolerances on the backend. Editing/selection never moves the robot.
- Respect map-origin translation/yaw, resolution and image-row direction. Persist map metres, not screen pixels. Test conversion under zoom/pan and rotated origins.

### 6.4 Canonical route schema

Editor and executor use the same validated artifact. Human-readable degrees convert to SI during compilation.

```yaml
schema_version: 1
route_id: route_a
revision: 1
map: {id: line_section, revision: 1, sha256: "<saved-map-bundle-hash>"}
frame_id: map
start: {x_m: 2.0, y_m: 2.0, yaw_deg: 0.0}
limits:
  linear_mps: 0.50
  long_linear_mps: 0.70   # null = no long-straight boost
  long_min_length_m: 4.0
  angular_rad_s: 0.34
  position_tolerance_m: 0.05
  heading_tolerance_deg: 2.0
  cross_track_limit_m: 0.10
steps:
  - {id: s1, type: straight, to: {x_m: 5.0, y_m: 2.0}}
  - {id: s2, type: rotate, direction: ccw, angle_deg: 90}
  - {id: s3, type: straight, to: {x_m: 5.0, y_m: 4.0}}
  - {id: s4, type: rotate, direction: cw, angle_deg: 270}
  - {id: s5, type: straight, to: {x_m: 3.0, y_m: 4.0}}
repeat_count: 1
```

A straight starts at the preceding endpoint and lies forward along current heading. A turn retains position and advances heading by its signed full angle. In this example CW 270° changes north-facing to west-facing by the long clockwise sweep. Store compiled expected poses/validation alongside the authored primitives; never discard turn magnitude.

`repeat_count` is a positive finite integer. Repetition >1 requires final position/heading to match the start within validation tolerance; otherwise require an explicit return sequence. Never auto-connect endpoints. A minimal mission manifest selects one route revision; station dwell/handshake features can be added later.

## 7. Executor and motion ownership

### 7.1 State machine

| State/event | Behavior |
|---|---|
| IDLE → READY | Load immutable validated map/route; establish localization, calibration and hardware health. No motion. |
| READY + AUTO + physical Start edge | Confirm start alignment, assign run ID, EXECUTING. Initial start gate 0.10 m / 5°; otherwise manually reposition. No automatic trip to route start. |
| EXECUTING | One action at a time; permit only its source. Check completion/stopped feedback before advancing. |
| Pause / obstacle | Inhibit output immediately, cancel action, enter PAUSED/BLOCKED and retain progress. |
| Prepare resume + physical Start | Recheck pose, clearance, sensors, mode and cancellation; continue remaining step. Teleop expiry alone never resumes. |
| Abort / MANUAL takeover | Inhibit/cancel. Abort discards run; manual takeover invalidates continuation until explicitly revalidated. |
| Sensor/drive/panel fault or localization LOST | Inhibit/cancel, FAULT with reason. Acknowledgement alone does not authorize motion. |
| Final step | Confirm stopped, clear permission, DONE. Another run requires another Start edge. |
| Process/host restart | IDLE/UNLOCALIZED. Stored progress is diagnostic until revalidated; no automatic replay. |

### 7.2 Sequencing

Executor owns run/step IDs, action handles and the permit lease. Mux combines permission with fresh commands, physical mode/drive state and required health. Drive owner independently times out wheel commands. A zero message is not a latched inhibit.

Ignore callbacks from old runs. During cancellation keep output inhibited until acknowledgement or server failure; clear cached source commands before a new action. Never start a second action while the old one may still execute. Browser loss stops held manual input; an automatic route is robot-owned and does not depend on browser refreshes. Coordinator loss expires permission.

A paused straight resumes from current along-segment position only within its approved corridor. A paused turn resumes remaining signed travel only with continuous valid odometry. Manual repositioning, a localization reset or lost odometry continuity invalidates progress-based resume. Do not replay completed station side effects when later adding handshakes.

## 8. Simulation and acceptance

Preserve T1–T3 tests. Extend scan simulation with measured mounting offset, 275° field of view and configurable scan geometry/rate. Ground truth and encoder reports must be distinct so slip does not make odometry magically correct.

| Test | Acceptance |
|---|---|
| Foundation | Existing kinematics, wheel/bias models, square/fused odometry and single-parent TF tests remain passing. |
| Survey/return | Manual-style tour with modest return error yields reviewable closure. Confirmation neither forces pose equality nor approves a bad map. |
| Save/load | Occupancy/serialization/manifest restore consistent geometry; partial saves never appear approved. |
| Coordinates | Click/render round trip with nonzero origin/yaw, row inversion, zoom/pan and nondefault resolution. |
| Compiler | All eight angle/direction combinations; reject invalid bends/angles, map hashes, zero lengths, colliding sweeps and invalid repeats. |
| Execution | Twenty repeated route runs satisfy every segment's tolerances and peak cross-track bound; no corner cutting/detour. |
| Rotations | CW/CCW 45/90/180/270 across yaw wrap; signed travel, drift, endpoint, stopping and paused-turn remainder verified. |
| Obstacles | Line/turn obstructions stop motion; clearance alone does not resume; no backup/replan. |
| Failures | Pause/abort/takeover, command/permit loss, stale sensors, localization jump, delayed results, restart and revision mismatch. |
| Watchdogs | Zero setpoint by 250 ms after last accepted command; permit loss within 300 ms plus one control tick. Physical stopping is measured separately with deceleration. |
| Factory [HW] | Measured geometry, survey closure, stopped relocalization, tracking, swept-area coverage and stop/restart behavior. Simulation does not establish factory accuracy. |

Use reproducible seeds and independent ground truth. Captured factory bags and physical references establish real performance. No camera/QR tests are required.

## 9. Launch, deployment and ownership

| Profile | Components |
|---|---|
| `sim.launch.py` | Existing T3 fake base/IMU, real mux/odom/bias/EKF/URDF; optional scan world extension. |
| `drivers.launch.py` | Single CAN owner/MLS, DIO, SICK, bias/EKF/URDF; optional RFID; no legacy owner. |
| `mapping.launch.py` | Real/fake estimation, live asynchronous SLAM, session coordinator/web UI; optional bag/viewer; no AMCL/route actions. |
| `nav.launch.py` | Real/fake estimation, immutable map, AMCL, local costmap, controller/Spin, executor/web UI; no map updates/global planner. |
| `replay_mapping.launch.py` | Optional isolated recorded sensors/time and one TF authority; no hardware output. |

Use `sim:=true/false`, map/route revision and optional viewer arguments. Validate mutually exclusive profiles; transitions happen while stopped. Services source Humble and the overlay explicitly. Establish hardware health/calibration, then activate estimation/localization/controller dependencies before READY. Launch ordering alone is not readiness.

Separate simulation/hardware ROS domains and choose DDS interface scope **before the first physical ROS drive launch**, not after T9. Respect the actual CANable service; refuse competing legacy ownership. Service restart never starts a route. One correctly sourced bridge is sufficient when Foxglove is requested.

The user-site NumPy/OpenCV versus `cv_bridge` conflict is accepted for this camera-free scope, as recorded in the workspace README. Drawing an occupancy map in a browser does not require adding a camera/image bridge dependency.

## 10. Revised task plan

T1–T3 keep their meaning and DONE status. Later IDs replace the former QR/free-navigation definitions. Hardware work can proceed alongside simulation/editor work but must finish before a powered live survey.

| Task | Status / deliverable | Acceptance / dependencies |
|---|---|---|
| T1 | **DONE** — interfaces, description, scaffold | Preserve definitions/resolved TF. |
| T2 | **DONE** — fake base, mux, raw odometry | Preserve tests; permission extension in T8/T9. |
| T3 | **DONE** — fake IMU, bias, EKF | Preserve estimator/config/composed tests. |
| T4 | Live mapping, scan simulation, session/return review, coherent save/load | Realistic simulated return error; failed saves cannot approve. |
| T5 | Saved-map AMCL and readiness | Reload, initialize, validate, detect loss and recover stopped. |
| T6 | Web editor, route schema/compiler and geometric validation | Draw/save/load lines and directed turns; coordinates, sweeps, revisions. |
| T7 | Straight FollowPath and directed Spin execution | Eight turns, straight tracking; no shortcuts or detours. T5/T6 first. |
| T8 | Panel/mission permission, pause/abort/resume, obstruction/staleness | Failure tests, stale callbacks rejected, physical Start retained. |
| T9 [HW] | BLV-R owner, wheel PDO/scaling and independent loss response | Reuse libraries/guard; prove ownership, throughput and faults on bench. No separate encoder node. |
| T10 [HW] | MLS acquisition and SICK commissioning | Actual timestamps/rates, calibration, frames and lidar ownership. |
| T11 | Deployment, environments/domains, operator runbook and full acceptance | Live map → return/review → save → draw → run; restart stopped; sustained latency/resource check. |
| T12 [HW] | DIO/panel adapter; optional RFID | Panel required before powered operation; RFID does not block core workflow. |

**Practical order:** T4–T8 in simulation; T9/T10/T12 on bench; integrated operator workflow in T11. CAN/PDO readiness and domain isolation precede T9 powered tests. A real manual survey requires the real sensor/control chain, not just simulated mapping completion.

## 11. Deferred scope and limits

- Camera/QR, precision docking and millimetre station repeatability are not required or supplied by present identity-only RFID.
- Free-space planning, automatic detours, curved/reverse routes, fleet integration and complex station handshakes are later features.
- Drawn lines/angles are nominal geometry; closed-loop tracking makes small corrections. Map resolution and hardware validation set achievable tolerances.
- Live mapping on N97 is the requested default. Measure with the minimal stack; core count alone establishes neither overload nor guaranteed performance.
- Independently wired stop functions stay separate; this specification does not extend deferred certification scope.

## 12. Revision summary

Preserves completed T1–T3; replaces generic Kinco/external-encoder/serial-IMU/QR assumptions with the active code baseline; requires live manual survey and return-to-start review; selects a web editor; and replaces destination planning with straight moves and directed rotations. Runtime localizes against the saved map. Map changes require a reviewed revision and route revalidation.
