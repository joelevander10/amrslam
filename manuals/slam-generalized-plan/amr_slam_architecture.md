# SLAM-Based AMR — Software Architecture (Prototype → Production-Grade)

**Platform:** Differential-drive AMR, Kinco CANopen servo drives, SICK NanoScan3, absolute wheel encoders (CAN), Yahboom IMU, floor-facing QR camera, Intel i3-N300 mini PC, Ubuntu + ROS 2.
**Application:** Factory floor, single production line section (500–3000 m²), fixed building structure with movable trolleys/pallets.
**Scope:** SLAM-related software + electronics architecture. Power electronics excluded.

---

## 1. Key Design Decisions (Locked)

| # | Decision | Choice | Rationale |
|---|----------|--------|-----------|
| D1 | SLAM operating mode | **Map-once (survey run) → pure localization at runtime, with supervised re-mapping when layout changes** | This is the standard industrial pattern. KUKA Navigation Solution, Hikrobot RCS-2000 AMRs, MiR, and SICK LiDAR-LOC all commission with a SLAM survey run, then operate in localization-only mode against a frozen reference map. Continuous SLAM in production is rare in industry because map drift from moving pallets corrupts the reference. Layout changes trigger a supervised re-map + map approval step. |
| D2 | Runtime localizer | **AMCL (nav2_amcl) on static map**, with slam_toolbox localization-mode as fallback option | AMCL is the most battle-tested 2D localizer, tolerant of partial map mismatch (trolleys/pallets) via its beam mixture model. slam_toolbox localization mode kept as a config-switchable alternative if AMCL struggles with clutter ratio. |
| D3 | Map creation | **slam_toolbox (sync mode) during survey runs** | De-facto ROS 2 standard, loop closure, serialized pose-graph allows later map extension/continuation — mirrors Hikrobot "map stitching" workflow. |
| D4 | Nav LiDAR source | **Single NanoScan3: safety fields (hardwired OSSD) + measurement data over Ethernet** | NanoScan3 streams full scan data (UDP, ~30 ms) alongside safety function. One sensor, two consumers. Safety path stays hardwired and software-independent. |
| D5 | Motor control | **Kinco drives in CiA-402 Profile Velocity mode; PC sends wheel velocity setpoints over CANopen** | Drive closes torque + velocity loops at kHz rate; PC only does kinematics + path tracking at 20–50 Hz. Matches your existing Kinco FD EDS/CANopen work. |
| D6 | CAN topology | **One shared 1 Mbps CAN bus: 2× drives + 2× absolute encoders** | Bus load analysis at 100 Hz PDO traffic stays well under 40% at 1 Mbps (consistent with your earlier CAN load analysis). |
| D7 | Floor camera | **QR/DataMatrix code reading at stations — absolute position fix + sequence/station verification** | Not continuous visual odometry. QR codes act as ground-truth anchors at pick/drop/charge stations, correcting accumulated localization error exactly where precision matters. Same pattern as Hikrobot/Geek+ hybrid "laser SLAM + QR" navigation. |
| D8 | Fleet interface | **Standalone; reserve a thin MQTT topic namespace for later** | No dispatch integration now; mission layer designed so a VDA5050/MQTT adapter can be bolted on without touching nav internals. |

---

## 2. Hardware Additions (Required / Recommended)

Your listed hardware is sufficient for SLAM, but the following are needed to make the system work as a whole:

| Item | Need | Recommendation | Notes |
|------|------|----------------|-------|
| CAN interface for mini PC | **Required** | PEAK PCAN-USB (or PCAN-USB FD), or CANable 2.0 (candleLight fw) | i3-N300 mini PCs have no native CAN. Both options expose Linux SocketCAN natively (`can0`). PEAK = industrial reliability; CANable = cheap prototype spare. Avoid serial-protocol USB-CAN adapters (USR-CANET class) — you already rejected these. |
| Floor camera | **Required (spec)** | Global-shutter mono USB3/MIPI camera, 1.2–2 MP (e.g., Daheng MER2, Arducam global shutter), wide-angle lens, **ring/diffuse LED illumination**, mounted 100–200 mm above floor in shrouded housing | Rolling shutter + motion = unreadable codes. Controlled illumination is non-negotiable on factory floors (oil sheen, shadows). |
| Floor codes | Required (consumable) | Laminated DataMatrix or QR labels, ~100×100 mm, at stations + optionally along corridors every 5–10 m | DataMatrix tolerates partial damage better than QR; both supported by the same pipeline. |
| Wi-Fi | Required (dev) | If the mini PC ships with Intel AX211 (CNVio2), verify chipset compatibility — swap to AX210NGW if needed | Same issue you hit on the previous IPC. Needed for SSH/Foxglove/teleop during commissioning. |
| E-stop chain | Required (interface only) | NanoScan3 OSSD pair → Kinco drive STO inputs, hardwired; software only *reads* status via NanoScan3 Ethernet telegram | Keeps safety independent of the SW stack — prerequisite for later ISO 3691-4 / PL d work you're already doing on the tow tractor. |
| RTC / time sync | Recommended | chrony, PC as local clock master | Sensor fusion needs monotonic, consistent timestamps; no NTP guaranteed on the floor. |
| Rear coverage | Deferred | NanoScan3 covers 275°. Rear blind spot acceptable for forward-dominant prototype; flag for production (2nd scanner or ultrasonics) | Affects safety case later, not SLAM. |
| Wheel encoder mounting | Already planned | Absolute encoders external to gearbox — confirm CANopen encoder profile CiA 406 support | Your encoder coupling design work applies directly. |

---

## 3. Platform Baseline

| Layer | Choice | Notes |
|-------|--------|-------|
| OS | **Ubuntu 24.04 LTS** | Supported to 2029; pairs with ROS 2 Jazzy. |
| ROS 2 distro | **Jazzy Jalisco (LTS, EOL 2029)** | Mature by now; all packages below released for Jazzy. Fallback: Humble on 22.04 if a vendor driver lags. |
| RMW / DDS | **CycloneDDS** (`rmw_cyclonedds_cpp`) | Lower CPU + better behavior on lossy links than Fast-DDS defaults; relevant on an i3-N300. |
| CAN | Linux **SocketCAN** | `can0` @ 1 Mbps, kernel-level, no vendor daemon. |
| Language split | **Python (rclpy)** for mission/QR/glue; **C++ (existing packages)** for nav2, AMCL, slam_toolbox, EKF | You don't write C++ — the C++ is all upstream packages you configure, not author. Your authored code stays Python. |
| Process supervision | systemd units + ROS 2 launch; lifecycle nodes where supported | Auto-restart, ordered bringup. |
| Visualization / tooling | **Foxglove Studio** (remote, via foxglove_bridge) + RViz2 on dev laptop | Foxglove over Wi-Fi avoids running RViz on the i3-N300. |

---

## 4. ROS 2 Node Architecture

Five layers. Arrows are topic/service flows (described in tables — no diagram per your preference).

### 4.1 Layer 1 — Hardware Drivers / HAL

| Node | Package | Language | Function | Key topics out | Key topics in |
|------|---------|----------|----------|----------------|---------------|
| `kinco_drive_node` | **ros2_canopen** (`canopen_402_driver`) *or* custom rclpy node wrapping `python-canopen` | C++ (upstream) / Python (custom) | CiA-402 state machine (NMT, op-enable), Profile Velocity setpoints via RPDO, actual velocity/status via TPDO, fault readout | `/drives/status`, joint states | `/cmd_wheel_vel` (rad/s L,R) |
| `wheel_encoder_node` | custom rclpy + `python-canopen` (CiA 406) | Python | Reads absolute encoder TPDOs @ 100 Hz, computes Δposition with wrap handling, publishes per-wheel position/velocity | `/wheel_states` | — |
| `diff_drive_odom_node` | custom rclpy (or `diff_drive_controller` if going ros2_control route) | Python | Forward kinematics from **external encoders** (not motor encoders — eliminates gearbox backlash/slip ambiguity), publishes odometry + TF `odom→base_link` (TF optionally delegated to EKF) | `/odom_raw` | `/wheel_states` |
| `cmd_vel_mux_kinematics` | custom rclpy | Python | Inverse kinematics `cmd_vel → wheel velocities`, velocity/accel limiting, command timeout watchdog (stop on 200 ms silence) | `/cmd_wheel_vel` | `/cmd_vel` (from nav2), `/cmd_vel_teleop` |
| `sick_nanoscan_node` | **sick_safetyscanners2** | C++ (upstream) | NanoScan3 Ethernet (COLA2/UDP): publishes `LaserScan`, plus safety field status, contamination warnings, OSSD state as diagnostics | `/scan`, `/scan/field_status` | — |
| `imu_node` | Yahboom vendor serial driver (rclpy) + **imu_filter_madgwick** | Python + C++ | Raw 9-axis @ ≥100 Hz → Madgwick orientation filter → `sensor_msgs/Imu` with covariance | `/imu/data` | — |
| `floor_camera_node` | `v4l2_camera` or vendor SDK wrapper | C++/Python | Raw image stream, hardware-triggered or free-running 15–30 fps, ROI cropped | `/floor_cam/image_raw`, `/floor_cam/camera_info` | — |

**ros2_canopen vs custom Python CANopen — recommendation:** start with **custom rclpy node using `python-canopen`** (you already have working Kinco FD EDS parsing and asyncio CANopen code — reuse it), structured behind a clean `/cmd_wheel_vel` interface. Migrate to `ros2_canopen` + `ros2_control` only if you later want controller-chain features (it's a heavier integration with real configuration cost). The interface boundary makes the swap invisible to everything above.

### 4.2 Layer 2 — State Estimation

| Node | Package | Function |
|------|---------|----------|
| `ekf_local` | **robot_localization** (EKF) | Fuses `/odom_raw` (x,y,yaw velocities) + `/imu/data` (yaw rate, optionally yaw) → smooth `odom→base_link` TF + `/odometry/filtered`. IMU corrects encoder yaw error from wheel slip / unequal wheel wear. |
| `amcl` | **nav2_amcl** | Particle-filter localization on static map → `map→odom` TF. Tuned for cluttered industrial space (see §6.3). |
| `qr_pose_node` | custom rclpy + OpenCV/`zxing-cpp` | Detects floor code, solvePnP on code corners + camera intrinsics → robot pose in map frame (codes have surveyed map coordinates) → publishes `PoseWithCovarianceStamped`. Used to (a) re-initialize/correct AMCL at stations via `/initialpose` with tight covariance, (b) verify station identity for sequencing, (c) drive final precision docking moves. |

### 4.3 Layer 3 — Mapping (Commissioning Mode Only)

| Node | Package | Function |
|------|---------|----------|
| `slam_toolbox` | **slam_toolbox** (sync) | Survey-run SLAM. Output: (a) `.pgm/.yaml` occupancy map for AMCL/nav2, (b) serialized pose-graph (`.posegraph`) enabling later map continuation/extension without full re-survey. |
| `map_saver` | nav2_map_server | Snapshot occupancy grid. |

Mapping and localization are **mutually exclusive launch profiles** (`mode:=mapping` vs `mode:=localization`) — same driver/estimation layers underneath.

### 4.4 Layer 4 — Navigation (nav2)

| Component | Plugin choice | Rationale |
|-----------|---------------|-----------|
| Global planner | **Smac Planner 2D** (or NavFn) | Grid A*; diff-drive can rotate in place, no need for Hybrid-A* kinematic constraints. |
| Controller | **Regulated Pure Pursuit (RPP)** | Predictable, smooth, low CPU — the right choice for an i3-N300 and for factory aisles. MPPI as later upgrade if tight maneuvering demands it. |
| Behavior tree | nav2_bt_navigator, default `navigate_to_pose` BT + recovery branches (clear costmap → spin → backup → fail) | Standard. |
| Global costmap | static layer + obstacle layer + inflation layer + **keepout filter** + **speed filter** | See §7. |
| Local costmap | obstacle layer (NanoScan3) + inflation, rolling 5×5 m | Dynamic obstacle avoidance (trolleys, people). |
| Smoother / behaviors | nav2 default | — |

### 4.5 Layer 5 — Mission / Application

| Node | Implementation | Function |
|------|----------------|----------|
| `mission_executor` | custom rclpy, explicit FSM (or py_trees if missions get complex) | Executes mission = ordered list of waypoints/stations from YAML/JSON; calls nav2 `NavigateToPose` action; at stations: trigger QR verification → precision align → I/O handshake placeholder. |
| `station_registry` | YAML config | Station ID → map pose → QR code ID → approach pose → tolerances. |
| `teleop_node` | teleop_twist_keyboard / joy | Commissioning + survey driving. |
| `diagnostics_aggregator` | diagnostic_aggregator | Single health view: CAN bus state, drive faults, scanner contamination, localization covariance, battery (your scope). |
| `fleet_adapter` (stub, later) | rclpy + paho-mqtt | Reserved: maps mission_executor API ↔ MQTT/VDA5050. Not built now; interface contract documented. |

---

## 5. Coordinate Frames (TF Tree)

```
map → odom → base_link → laser_frame
                        → imu_frame
                        → floor_cam_frame
                        → wheel_left / wheel_right (visualization only)
```

| Transform | Published by | Rate |
|-----------|--------------|------|
| `map → odom` | AMCL (runtime) / slam_toolbox (survey) | ~scan rate |
| `odom → base_link` | robot_localization EKF | 50 Hz |
| `base_link → sensors` | static_transform_publisher from URDF (measured mechanically, laser extrinsics calibrated by scan alignment) | static |

URDF: minimal xacro — chassis box, wheel geometry, sensor mounts. Needed for TF + footprint, not simulation fidelity.

---

## 6. SLAM & Localization Design Detail

### 6.1 Commissioning workflow (the "industrial pattern")

1. **Survey run** — teleop the robot through the area at ≤0.3 m/s, slam_toolbox sync mode, deliberate loop closures (revisit start, drive both aisle directions).
2. **Map QA** — inspect in Foxglove/RViz: wall straightness, loop-closure seams, ghosting. Re-run if needed.
3. **Map editing ("setting software", §7)** — clean stray pixels, mark keepout/speed zones, define stations.
4. **QR survey** — drive to each floor code, record its precise map pose into `station_registry.yaml` (semi-automated: QR node observes code while AMCL is well-converged, operator confirms).
5. **Freeze + version** — map, zone masks, and station registry are versioned artifacts (git). Runtime loads a tagged release.
6. **Re-map trigger** — when permanent layout changes (new machine, wall), repeat from 1, optionally continuing from the saved pose-graph to extend rather than redo.

### 6.2 Static vs dynamic landmark handling

| Object class | Examples | Handled by | Mechanism |
|--------------|----------|-----------|-----------|
| Permanent structure | Walls, columns, fixed machines, racking | **Static map** | Frozen occupancy grid from survey. The localization reference. |
| Semi-permanent clutter | Pallets parked for days, trolleys staged in lanes | **Excluded from static map; tolerated by AMCL; blocked by costmap obstacle layer** | During survey, either physically clear them or erase them in map editing. AMCL's `z_rand`/`z_hit` mixture absorbs the mismatch at runtime. |
| Dynamic obstacles | Moving trolleys, forklifts, people | **Local/global costmap obstacle layer** | Live NanoScan3 marking + raytrace clearing; never touches the localization map. |
| Process zones | Pallet staging lanes, trolley parking | **Keepout / speed filter masks** | Painted in map editor; planner never routes through staging lanes even when momentarily empty — this is how you stop "the map said it was free yesterday" failures. |
| Precision anchors | Stations, charge dock | **Floor QR codes** | Absolute pose fix independent of LiDAR clutter. |

**Known structural risk — flag early:** the NanoScan3 is mounted low (safety scan plane, ~150–200 mm). At that height it sees *exactly* the movable clutter (pallet bases, trolley wheels) and relatively little permanent structure. This is the classic single-low-scanner AMR weakness. Mitigations in order of cost:
- Survey with the floor in "typical clutter" state, not empty — AMCL then matches the average scene.
- Keep QR anchor density high enough that error never accumulates past station tolerance.
- Tune AMCL for high outlier ratio (§6.3).
- If localization stability is still poor: add a small elevated nav-only 2D LiDAR (e.g., 360° unit at 1.8–2 m mast or above-load height) seeing walls/columns only — common Hikrobot/KUKA production configuration. Architecture already supports a second `/scan_nav` source with zero structural change (AMCL input remapped).

### 6.3 AMCL tuning baseline for cluttered floors

| Parameter | Direction | Why |
|-----------|-----------|-----|
| `z_hit` ↓ (~0.7), `z_rand` ↑ (~0.25) | Tolerate beams hitting unmapped objects | Pallets/trolleys |
| `laser_likelihood_max_dist` ~2.0 | Softer likelihood field | Mismatch tolerance |
| `update_min_d/a` small (0.05 m / 0.03 rad) | Frequent updates | Smooth correction |
| particles 500–2000 adaptive | KLD sampling | i3-N300 friendly |
| `do_beamskip: true` | Skip persistently-mismatched beams | Direct clutter countermeasure |

Acceptance metric: localization covariance bounded + QR-measured error at stations < ±30 mm / ±1° before relying on QR-free corridor travel.

### 6.4 QR pipeline detail

- Detection: `zxing-cpp` (fast DataMatrix/QR) or OpenCV `QRCodeDetector`; ROI-cropped, ~10–15 fps is ample.
- Pose: camera intrinsics from `camera_calibration` checkerboard; corner solvePnP → camera-to-code pose → TF chain → robot pose in map.
- Usage policy: at stations only (prototype). Corridor codes optional later for long-aisle drift correction.
- Precision docking: final 0.5 m approach servoed directly on QR pose error (simple P-controller on lateral/heading error), bypassing nav2 — this is how QR-hybrid AMRs achieve ±5–10 mm at stations while SLAM alone gives ±20–50 mm.

---

## 7. Zone & Map "Setting Software"

Prototype = configuration-file driven, production = thin web UI. The data model is the deliverable; the UI is cosmetic.

| Artifact | Format | Edited with | Consumed by |
|----------|--------|-------------|-------------|
| Occupancy map | `.pgm` + `.yaml` | GIMP (cleanup) | AMCL, global costmap static layer |
| Keepout mask | `.pgm` + `.yaml` (same geometry) | GIMP layer / simple OpenCV paint script | nav2 `KeepoutFilter` |
| Speed mask | `.pgm` + `.yaml` | same | nav2 `SpeedFilter` (slow zones near workstations) |
| Station registry | YAML | text editor / small Flask page later | mission_executor, qr_pose_node |
| Mission definitions | YAML/JSON | text editor → later Flask UI | mission_executor |

All under git; robot pulls a tagged config release. This versioning discipline is also what your CE technical-file process will eventually want (configuration management evidence).

---

## 8. Motion Control Chain (end-to-end)

1. nav2 RPP controller → `/cmd_vel` (20 Hz)
2. `cmd_vel_mux_kinematics`: priority mux (teleop > nav), accel/jerk limits, inverse kinematics → `/cmd_wheel_vel`
3. `kinco_drive_node`: rad/s → drive units, RPDO Target Velocity @ 50 Hz, SYNC-driven TPDO feedback @ 100 Hz
4. Drive: internal velocity + current loops (kHz)
5. Watchdogs: (a) node-level 200 ms cmd_vel timeout → zero velocity, (b) drive-side CANopen heartbeat consumer → drive faults to safe stop on PC death, (c) NanoScan3 OSSD → STO, fully hardwired, software not in loop.

Sample PDO mapping sketch (only code in this document, per your note):

```python
# kinco_drive_node core loop (python-canopen)
node.rpdo[1]['Target velocity'].raw = int(wheel_rad_s * RADS_TO_COUNTS)
node.rpdo[1].transmit()
# feedback via tpdo callback -> publish joint state
```

---

## 9. CAN Bus & Timing Budget

| Traffic | Rate | Frames/s (approx) |
|---------|------|-------------------|
| 2× drive RPDO (velocity cmd) | 50 Hz | 100 |
| 2× drive TPDO (vel/status) | 100 Hz | 200 |
| 2× encoder TPDO | 100 Hz | 200 |
| SYNC + heartbeats + EMCY headroom | — | ~120 |
| **Total** | | **~620 f/s ≈ 8–9 % @ 1 Mbps** |

Comfortable margin; consistent with your earlier 1 Mbps CANopen analysis. Keep encoders and drives on distinct COB-ID ranges; no bus segmentation needed.

---

## 10. Compute Budget (i3-N300, 8 E-cores, 16 GB)

| Workload | Est. load |
|----------|-----------|
| nav2 stack (RPP + costmaps + BT) | 1–1.5 cores |
| AMCL | 0.3–0.5 core |
| sick driver + EKF + drivers | 0.5 core |
| QR detection (ROI, 15 fps) | 0.3–0.5 core |
| slam_toolbox (survey only) | 1–2 cores (offline-ish, fine) |
| Headroom | ≥40 % |

Rules: no RViz on robot (Foxglove bridge instead), CycloneDDS, image transport kept local (no raw image over Wi-Fi), `performance` governor during operation.

---

## 11. Repository & Configuration Layout

```
amr_ws/src/
  amr_bringup/        # launch profiles: mapping.launch.py, nav.launch.py, drivers.launch.py
  amr_description/    # URDF/xacro, static TFs
  amr_base/           # kinco_drive_node, encoder_node, odom, kinematics/mux  (Python)
  amr_localization/   # EKF + AMCL configs, qr_pose_node
  amr_navigation/     # nav2 params, costmap filters, BT XML
  amr_mission/        # mission_executor, station_registry, mission YAMLs
  amr_maps/           # versioned maps, masks, station registries (git-tagged releases)
  amr_tools/          # calibration scripts, map paint tool, QR survey helper
```

Config philosophy: every tunable in YAML, zero magic numbers in nodes — also makes the stack consumable by your LLM-agent documentation workflow.

---

## 12. Bringup Plan (Phased Milestones)

| Phase | Goal | Exit criteria |
|-------|------|---------------|
| P0 | Bench: CAN up, drives spin in 402 velocity mode, encoders + IMU streaming | Wheel velocity tracks setpoint; encoder counts sane |
| P1 | Teleop drive + raw odometry | Straight-line 10 m: odom error < 2 %; rotation: yaw error < 5°/360° before fusion |
| P2 | EKF fusion + NanoScan3 `/scan` live | Smooth fused odom; scan visualized in Foxglove |
| P3 | Survey run + map QA | Clean map of target section, loop closures verified |
| P4 | AMCL localization + manual goals via Foxglove | Robot holds localization through full section incl. typical clutter |
| P5 | nav2 autonomous point-to-point + keepout/speed zones | Repeated A↔B runs, zero interventions, dynamic obstacle stop/replan |
| P6 | QR station fix + precision docking | Station repeatability ±10 mm measured |
| P7 | Mission executor multi-station sequences | Full shift-length soak test, diagnostics clean |

Each phase isolates one failure domain — do not skip P1/P2 odometry validation; bad odometry is the root cause of 80 % of "SLAM doesn't work" outcomes.

---

## 13. Risks & Mitigations Summary

| Risk | Severity | Mitigation |
|------|----------|------------|
| Low scan plane sees mostly movable clutter | High | §6.2 strategy; budget for elevated nav LiDAR in production BOM |
| Yahboom IMU quality (hobby-grade, drift, poor temp stability) | Medium | Use yaw-rate only in EKF (not absolute yaw); calibrate gyro bias at startup while stationary; production swap: industrial-grade IMU (e.g., WitMotion HWT905 class or better) — driver-level change only |
| Wheel slip on dusty/oily floor corrupts odom | Medium | External encoders already help; EKF gyro fusion; AMCL absorbs residual |
| NanoScan3 single source for safety + nav | Medium | Acceptable per SICK design (data output is parallel, non-interfering); document for safety case; nav degrades gracefully if data stream drops (stop via cmd timeout) |
| i3-N300 saturation during survey + record | Low | Survey with rosbag recording of raw sensors, offline map refinement possible |
| QR codes contaminated/worn | Low | DataMatrix ECC, laminated, station logic falls back to LiDAR-pose with wider tolerance + operator alert |

---

## 14. Complete Package / Library Bill of Materials

| Package | Source | Role |
|---------|--------|------|
| ros-jazzy-navigation2, nav2-bringup | apt | Planning, control, costmaps, BT |
| ros-jazzy-slam-toolbox | apt | Survey mapping |
| ros-jazzy-robot-localization | apt | EKF fusion |
| sick_safetyscanners2 | apt / github (SICKAG) | NanoScan3 driver |
| ros-jazzy-imu-filter-madgwick, imu_tools | apt | IMU orientation |
| python-canopen, python-can | pip | Drives + encoders (reuse your existing code) |
| zxing-cpp (python bindings) or OpenCV | pip/apt | QR/DataMatrix |
| v4l2_camera or vendor SDK | apt/vendor | Floor camera |
| camera_calibration (image_pipeline) | apt | Intrinsics |
| foxglove_bridge | apt | Remote visualization |
| diagnostic_aggregator | apt | Health monitoring |
| teleop_twist_keyboard / joy | apt | Commissioning |
| chrony | apt | Time discipline |

Everything above is open-source, no licensing cost, and standard enough that hiring/contracting against this stack is easy — relevant for productization.
