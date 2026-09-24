# Install list — SLAM AMR on gvievo-01

**Written:** 2026-09-15 · branch `slam-roadmap`
**Built from:**
- `amr_slam_architecture.md` §3 and §14 (the generic bill of materials)
- `amr_implementation_spec.md` (tooling and test harness)
- `hardware-reconciliation.md` (what our hardware adds or removes)
- the existing code in this repository

**Where they disagree, `hardware-reconciliation.md` wins.** So this list is
Ubuntu 22.04 + ROS 2 Humble, not 24.04 + Jazzy, and it has no camera, QR or
serial-IMU packages.

**Nothing here has been installed on the vehicle yet.** Package names are the
Humble apt names. Confirm each resolves with `apt-cache policy <name>` before
relying on it.

---

## 0. Decisions that set this list

| Item | Choice | Source |
|---|---|---|
| OS on the robot | **Ubuntu 22.04.5** (already installed, kernel 5.15) | reconciliation D-7 |
| ROS 2 distro | **Humble** (EOL May 2027) | reconciliation D-7 |
| Python | **3.10** (system Python on 22.04). No `match`, no PEP 695 syntax | reconciliation D-7 |
| DDS middleware | **CycloneDDS** | architecture §3 |
| Robot session | **Headless**: no desktop, no RViz on the robot | reconciliation D-6 |
| Visualisation | **Foxglove on the Windows laptop** through `foxglove_bridge`. No RViz2 anywhere | architecture §3 |
| Survey mapping | **Record a rosbag on the robot, then build the map on the robot with Nav2 and the controller stopped** | reconciliation D-6 |
| Build and sim tests | **On the robot**, only while the vehicle is not operating | this document §1 |
| CAN | SocketCAN `can0`, CANable 2.0 (already working) | reconciliation §1 |

**If the decision moves to 24.04 + Jazzy:**
- Replace `humble` with `jazzy` in every package name below.
- Python becomes 3.12. pip then refuses system-wide installs (PEP 668), so use a venv or apt.
- Re-validate every hardware item in §2.5 before trusting the stack.

---

## 1. Machines

Only two machines exist. There is **no Ubuntu dev laptop**, and the plan does
not depend on one.

| Machine | OS | Role |
|---|---|---|
| **Robot PC** (Intel N97, 4 cores, 8 GB) | Ubuntu 22.04 | Everything ROS: drivers, EKF, AMCL, Nav2, mission. Also `colcon build`, the `sim:=true` tests, survey bag recording and the map build |
| **Windows laptop** | Windows 10/11 | No ROS. Foxglove, SSH, map and mask editing, MEXE02, SICK tools, the existing offline suite |

**What the Ubuntu laptop would have done, and where it goes instead:**

| Job | Now done on | Consequence |
|---|---|---|
| `colcon build`, unit and `launch_testing` sim tests | Robot PC, over SSH | Stop `agv_controller` and any ROS launch first. Builds and sim tests compete with the runtime for 4 cores |
| Offline map build from the survey bag | Robot PC, Nav2 and controller stopped | This is the fallback the reconciliation already names (D-6). Slower, but nothing is running that can drop scans |
| RViz2 (goals, initial pose, costmaps, TF) | Foxglove on Windows | Foxglove shows maps, scans, TF and costmaps, and can publish `/initialpose` and goal poses. No DDS over Wi-Fi is needed: the bridge is a single WebSocket (port 8765) |
| Map cleanup, keepout and speed masks | Windows (GIMP or Paint.NET) | Copy `.pgm`/`.yaml` both ways with `scp` |
| Opening survey bags | Foxglove on Windows | Record in MCAP so no conversion is needed |

**Do not install ROS 2 on Windows or in WSL2 as a substitute.** It adds a second
ROS install to keep in step, and Foxglove already covers what is needed.

---

## 2. Robot PC

### 2.1 Before installing anything

The robot's current Python set is a known-good baseline for CAN, Modbus and RFID.
Record it first so it can be restored:

```bash
python3 --version
pip3 freeze > ~/pre-ros-pip-freeze.txt
dpkg -l > ~/pre-ros-dpkg.txt
sudo systemctl stop agv_controller     # it owns can0
```

### 2.2 OS packages (apt)

| Package | Why | Where it is used |
|---|---|---|
| `can-utils` | `candump`, `cansend`, `slcand` | bench checklists §3.3, §4.3 |
| `iproute2` | `ip link set can0 type can bitrate …` (normally already installed) | bench checklists §2 |
| `ethtool` | NIC link and error counters on `enp2s0` | rfid-setup.md |
| `tcpdump` | nanoScan3 UDP capture on port 6060 | lidar_brief.md |
| `chrony` | Consistent clock for sensor fusion; the robot PC acts as the local time master | architecture §2 |
| `linux-tools-common`, `linux-tools-$(uname -r)` | `turbostat` for N97 thermals; `cpupower` for the `performance` governor | reconciliation §6, architecture §10 |
| `lm-sensors` | Temperature readout during the Nav2 soak | reconciliation §6 |
| `git` | Map, mask and station releases are git-tagged | architecture §7 |
| `openssh-server` | Headless access | — |
| `curl`, `software-properties-common`, `locales` | Prerequisites for adding the ROS 2 apt source | ROS 2 install docs |
| `python3-pip` | For the libraries in §2.4 | — |

`netplan`/`systemd-networkd` are already in use for the RFID link. Nothing to
install; see `manuals/rfid-setup/`.

### 2.3 ROS 2 Humble packages (apt)

**Base and tooling**

| Package | Why |
|---|---|
| `ros-humble-ros-base` | Headless core, including `robot_state_publisher` and rosbag2. **Not** `desktop` on the robot |
| `ros-humble-rmw-cyclonedds-cpp` | DDS layer; set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| `python3-colcon-common-extensions` | `colcon build` / `colcon test` |
| `python3-rosdep` | Resolves package dependencies from `package.xml` |
| `python3-vcstool` | Pulls source packages if a binary lags |
| `ros-humble-xacro` | `amr_description/urdf/amr.urdf.xacro` |
| `ros-humble-tf2-tools` | `view_frames`, for the T1 check "TF tree renders" |
| `ros-humble-rosbag2-storage-mcap` | Survey bags in MCAP, which Foxglove opens directly |

**Navigation and localisation**

| Package | Why | Architecture ref |
|---|---|---|
| `ros-humble-navigation2` | Planner (SmacPlanner2D), controller (Regulated Pure Pursuit), costmaps, keepout and speed filters, behaviour tree, `map_server`, AMCL | §4.2, §4.4 |
| `ros-humble-nav2-bringup` | Reference launch files and parameters to start from | §14 |
| `ros-humble-slam-toolbox` | Survey mapping: live during the survey if the CPU allows, otherwise replayed from the bag with Nav2 stopped | §4.3 |
| `ros-humble-robot-localization` | `ekf_local`: wheel odometry plus gyro yaw rate | §4.2 |

**Drivers, diagnostics, operation**

| Package | Why | Note |
|---|---|---|
| `ros-humble-sick-safetyscanners2` | nanoScan3 `/scan` and field status | **Verify nanoScan3 measurement-data support on the unit's firmware** (reconciliation §6) |
| `ros-humble-diagnostic-updater` | Per-node health publishers; replaces the table in `core/health.py` | reconciliation D-8 |
| `ros-humble-diagnostic-aggregator` | One health view | architecture §4.5 |
| `ros-humble-foxglove-bridge` | The only visualisation path: Foxglove on the Windows laptop connects to it. Replaces RViz2 | architecture §3 |
| `ros-humble-teleop-twist-keyboard` | Survey-run driving | architecture §4.5 |
| `ros-humble-joy`, `ros-humble-teleop-twist-joy` | Only if a gamepad is used for the survey | optional |

**Test harness (spec §0.2, §8)**

| Package | Why |
|---|---|
| `ros-humble-launch-testing`, `ros-humble-launch-testing-ros` | The six `launch_testing` integration cases in `sim:=true` |
| `ros-humble-ament-copyright`, `ros-humble-ament-flake8`, `ros-humble-ament-pep257` | Default lint tests generated with every `ament_python` package |

### 2.4 Python libraries

The existing modules the ROS nodes will import need these (reconciliation D-8:
`config`, `guard`, `kinematics`, `dio`, `rfid`).

| Library | Why | Where |
|---|---|---|
| `python-can` | Bus access for the drive node and every `drivers/canbus/` tool | `canworker.py`, `rpdo.py`, `lss.py`, `read_imu.py` |
| `pymodbus` **3.x** | Modbus TCP I/O island. The code imports `pymodbus.client.ModbusTcpClient`, which needs 3.x. The 22.04 apt package is 2.x, so use pip | `drivers/dio.py`, `drivers/modbus_io.py` |
| `pyserial` | slcan port discovery fallback | `verify_drivers.py` |
| `flask` | Current web UI, until it is replaced or wrapped | `app/server.py` |
| `canopen` (python-canopen) | Named by the spec for the drive node and its mocked-bus unit tests (T9) | spec §3.1, §10 |
| `pytest` | Package unit tests | spec §0.2 |
| `opencv-python-headless` | `amr_tools/mask_paint.py` and map-generation tools (spec §1, T4), if written. Headless build, since the robot has no display | spec §1 |
| `ruff` | Lint rule from spec §0.2 | spec §0.2 |

**Do not upgrade** `python-can`, `pymodbus` or `flask` on the robot if the
versions in `~/pre-ros-pip-freeze.txt` already work. Install only the missing
ones.

### 2.5 Hardware support: nothing to install, verify only

| Item | Check |
|---|---|
| CANable 2.0 (candleLight firmware) | `gs_usb` kernel module is loaded, `ip link show can0` exists |
| CAN bitrate | Still 125 kbps until task T0. Do **not** change the `can0` bitrate as part of installing |
| nanoScan3 on Ethernet | UDP on 6060 is visible with `tcpdump`. **Do not change the scanner's configuration** (lidar_brief.md) |
| RFID + DIO on `enp2s0` | `ping 192.168.1.30`; the netplan file is unchanged |
| Wi-Fi | If the card is an AX211 (CNVio2), check the driver. Swap to AX210 if needed (architecture §2) |

### 2.6 Commands

```bash
# --- OS packages ---
sudo apt update
sudo apt install -y can-utils iproute2 ethtool tcpdump chrony lm-sensors git \
  openssh-server curl software-properties-common locales python3-pip \
  linux-tools-common linux-tools-$(uname -r)

# --- ROS 2 apt source (check docs.ros.org/en/humble/Installation if this has changed) ---
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

# --- ROS 2 Humble, headless ---
sudo apt install -y \
  ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-xacro ros-humble-tf2-tools ros-humble-rosbag2-storage-mcap \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-sick-safetyscanners2 \
  ros-humble-diagnostic-updater ros-humble-diagnostic-aggregator \
  ros-humble-foxglove-bridge ros-humble-teleop-twist-keyboard \
  ros-humble-launch-testing ros-humble-launch-testing-ros \
  ros-humble-ament-copyright ros-humble-ament-flake8 ros-humble-ament-pep257

sudo rosdep init        # once per machine; skip if it says already initialised
rosdep update

# --- environment (append once) ---
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# --- Python: install only what the freeze shows is missing ---
pip3 install --user "pymodbus>=3" python-can pyserial flask canopen pytest ruff \
  opencv-python-headless
```

---

## 3. Map build on the robot (replaces the laptop step)

No extra packages beyond §2.3. The order matters because of the 4-core budget
(reconciliation D-6):

1. Survey drive: record raw sensors only (`/scan`, `/odom_raw`, `/imu/data`, `/tf`, `/tf_static`) to MCAP. `slam_toolbox` and Foxglove are not running.
2. Stop `agv_controller` and every ROS launch.
3. Replay the bag into `slam_toolbox` (sync mode) on the robot.
4. Save with `ros2 run nav2_map_server map_saver_cli`, and serialise the pose graph.
5. `scp` the `.pgm`/`.yaml` to Windows for cleanup and mask painting, then copy them back and git-tag the release.

Map QA (architecture §6.1 step 2) is done in Foxglove on Windows, from the saved
map or with `foxglove_bridge` running while nothing else is.

---

## 4. Windows laptop

No ROS on this machine.

| Tool | Why | Rule |
|---|---|---|
| **Foxglove app** (foxglove.dev) | Live view through `foxglove_bridge` (`ws://<robot-ip>:8765`), publishing `/initialpose` and goals, opening MCAP bags. Replaces RViz2 | Not during a survey recording (reconciliation D-6) |
| **OpenSSH client** (built into Windows) + `scp` | Builds, launches, logs and file transfer on the robot | — |
| **GIMP** (or Paint.NET) | Occupancy-map cleanup, keepout and speed mask painting (architecture §7) | Keep the image size and resolution unchanged; masks must match the map geometry |
| **Git** for Windows | Map, mask and station releases, and this repository | — |
| **Oriental Motor MEXE02** | Only way to change the BLV-R CAN bitrate (task T0) and load inertia | Drives go **last** in the bitrate migration (bench checklists §2.3) |
| **SICK Safety Designer** | nanoScan3 configuration | **Humans only, read-only by default.** A change invalidates the verified safety configuration |
| **SICK configuration software for the MLS** | Fallback if the MLS does not answer LSS | bench checklists §2.1 |
| Python 3.11 + `flask`, `python-can`, `pymodbus>=3`, `pyserial` | Existing offline suite: `python tests/run_all.py`, pinned at 696 checks | Set `PYTHONUTF8=1`, or one lidar test fails on cp1252 |
| A second CAN adapter | Not software, but required on the bench for T0 recovery | bench checklists §0 |

---

## 5. Not installing: dropped by the reconciliation

| Generic plan item | Why it is gone |
|---|---|
| `imu_filter_madgwick` / `imu_tools` | The MLS already outputs a fused quaternion; the EKF uses gyro yaw rate only (D-3). Add it back only if a raw-IMU source replaces the MLS |
| Yahboom / WitMotion serial IMU driver | The IMU is CANopen node 10 (D-3) |
| `zxing-cpp`, OpenCV QR pipeline | No floor camera; RFID gives identity, not pose (D-4) |
| `v4l2_camera`, `camera_calibration` (image_pipeline) | No camera (D-4) |
| Kinco EDS / `KincoScaler` | BLV-R takes r/min directly in `60FFh` (D-1) |
| CiA 406 encoder driver | Encoders are inside the drives; read through `6064h`/`606Ch` (D-2) |
| `ros2_canopen` / `canopen_402_driver` | Custom node ported from `canworker.py`; `ros2_canopen` has no equivalent of the write deny-list (architecture §4.1, D-8, D-9) |
| `paho-mqtt` | Fleet adapter deferred (architecture D8) |
| Cartographer | `slam_toolbox` was chosen (architecture D3) |
| `ros-humble-desktop`, RViz2, rqt, `nav2-rviz-plugins` | No Ubuntu laptop, and the robot is headless. Foxglove covers it |
| ROS 2 on Windows / WSL2 | Not needed: Foxglove talks to the bridge directly |

---

## 6. After installing: checks

On the robot, in order:

- [ ] `pip3 freeze` diffed against `~/pre-ros-pip-freeze.txt`. Only additions, no version changes to `python-can`, `pymodbus`, `flask`
- [ ] `python3 tests/run_all.py` still prints 696 checks, all passed
- [ ] `sudo systemctl start agv_controller`; `/monitor` shows both drives and the MLS
- [ ] `ros2 doctor --report` shows `RMW_IMPLEMENTATION = rmw_cyclonedds_cpp`
- [ ] `ros2 pkg list` contains `nav2_bt_navigator`, `slam_toolbox`, `robot_localization`, `sick_safetyscanners2`, `foxglove_bridge`
- [ ] On the robot: `ros2 topic pub /chatter std_msgs/msg/String "data: hi"` in one SSH session is seen by `ros2 topic echo /chatter` in another
- [ ] `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` on the robot; Foxglove on Windows connects to `ws://<robot-ip>:8765` and lists `/chatter`
- [ ] `chronyc tracking` reports a synchronised or local-master state
- [ ] `sudo turbostat --quiet --interval 5` runs; record idle package temperature as the baseline
- [ ] **Before running two stacks together:** the ROS drive node and `agv_controller` must never both open `can0` (README "one thread owns the bus")

---

## 7. Open before this list is final

- **`sick_safetyscanners2` against the nanoScan3 firmware in the unit.** Confirm before building on it (reconciliation §6).
- **Humble vs Jazzy.** Reconciliation D-7 recommends Humble. Humble reaches end of life in May 2027, so revisit before productisation.
- **`python-canopen` or plain `python-can` for the drive node.** The port from `canworker.py` needs only `python-can`. `canopen` is listed because the spec's T9 mocked-bus tests use it; drop it if those tests use `python-can`'s virtual bus instead.
- **Build and map time on the N97.** `colcon build` of the workspace and a bag replay into `slam_toolbox` have not been timed on the robot. Measure both once; if the map build is too slow, an Ubuntu machine comes back into scope.
- **CAN bitrate unit.** Architecture §9 has `amr-can.service` set `can0` at bring-up. The unit must carry 125 kbps until T0 is done on the bench, not 1 Mbps.
