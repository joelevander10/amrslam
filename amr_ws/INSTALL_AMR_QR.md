# Installing the SLAM stack on the AMR QR, from zero

Target: the AMR QR's PC, **Ubuntu 24.04 + ROS 2 Jazzy**, profile `amr-qr-01`, user
`amrslam`. The code is the gvievo-01 stack (gvievo-01-gvievo-main, 2026-10-23) with the
AMR QR platform on top. Background and design: [AMR_QR.md](AMR_QR.md). Day-to-day
operation after install: [OPERATE_AMR_QR.md](OPERATE_AMR_QR.md).

**Updating a PC that already runs the previous AMR SLAM code?** Go to
[Part U](#part-u--updating-from-the-previous-amr-slam-code) at the end instead.

Parts A–D move nothing. Part E moves the wheels: **vehicle on blocks, drive wheels off
the ground**. Part F is on the floor.

**Before you start**
- The PC already runs Ubuntu 24.04, has internet, and you have a user with sudo.
- It is cabled exactly as the old QR controller used it: the I/O modules on 192.168.1.x,
  the lidar port, the USB CAN adapter and the WitMotion IMU.
- The code lives in **`~/agv_can`**. Several scripts expect `~/agv_can/amr_ws`, so do not
  rename it.

---

## Part A — Base system

**1. Update and basic tools**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl git unzip can-utils net-tools
sudo usermod -aG dialout $USER      # serial access: IMU and CAN adapter
```
Log out and back in (or reboot) so the `dialout` group takes effect.

**2. Locale** (ROS needs UTF-8)
```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

## Part B — ROS 2 Jazzy

**3. Add the ROS 2 apt source**
```bash
sudo add-apt-repository universe -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

**4. Install ROS and the packages the stack uses**
```bash
sudo apt install -y ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-navigation2 ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-sick-safetyscanners2 ros-jazzy-foxglove-bridge \
  ros-jazzy-diagnostic-updater ros-jazzy-diagnostic-aggregator \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-rosbag2-storage-mcap
```
Use `ros-base`, not `desktop`. Visualisation is Foxglove on a laptop.

**5. Python libraries**
```bash
sudo apt install -y python3-can python3-serial python3-flask python3-yaml python3-numpy python3-pytest
pip install --break-system-packages "pymodbus>=3.10"
python3 -c "import pymodbus; print(pymodbus.__version__)"    # must be >= 3.10
```
The code calls pymodbus with `device_id=`, which needs 3.10 or newer. Ubuntu's apt
version is older, so pymodbus comes from pip.

## Part C — The code

**6. Clone into `~/agv_can`**
```bash
cd ~
git clone https://github.com/joelevander10/amrslam.git agv_can
ls ~/agv_can        # agv_core/, profiles/, amr_ws/, tests/, ...
```
`agv_core/` is the vehicle library (profile loader, drivers). It is not a ROS package:
`env/vehicle.sh` (step 7) and the service put `~/agv_can` on `PYTHONPATH`.

**7. Shell environment**: append to `~/.bashrc`
```bash
source /opt/ros/jazzy/setup.bash
export AGV_PROFILE=amr-qr-01
[ -f ~/agv_can/amr_ws/install/setup.bash ] && source ~/agv_can/amr_ws/install/setup.bash
source ~/agv_can/amr_ws/env/vehicle.sh      # ROS_DOMAIN_ID=10, CycloneDDS, PYTHONPATH for agv_core
```
Then:
```bash
source ~/.bashrc
echo $ROS_DISTRO $AGV_PROFILE $ROS_DOMAIN_ID     # jazzy amr-qr-01 10
```

**8. Check that the profile loads**
```bash
cd ~/agv_can
python3 -c "from agv_core import config; print(config.PROFILE_NAME, config.PLATFORM, config.TRACKED, round(config.MAX_SPEED_MPS,3))"
# amr-qr-01 qr_analog False 0.283
```

**9. Build**
```bash
cd ~/agv_can/amr_ws
colcon build --symlink-install
source install/setup.bash
```
Always build with `--symlink-install`, so a `git pull` of Python or launch changes needs
only a restart. Rebuild after a pull that adds a package, a node or a message.

**10. Offline tests** (no hardware)
```bash
cd ~/agv_can && python3 tests/run_all.py | tail -2
# 762 checks, "all checks passed". The suite pins AGV_PROFILE=agv-01 itself, so the
# amr-qr-01 export from step 7 does not affect it.
cd amr_ws/src && AGV_PROFILE=agv-01 python3 -m pytest -q amr_base/test/test_qr_*.py \
  amr_base/test/test_wasd_teleop.py amr_localization/test amr_description/test
```

## Part D — Hardware links (parked, nothing moves)

**11. Stop the old QR controller.** Stop `app.py` (Flask, port 5050) and disable whatever
starts it at boot (systemd unit, cron job, autostart). It owns the same I/O modules, CAN
bus and IMU port. Nothing checks for it any more, so make sure it is really gone:
`ss -ltn | grep 5050` must print nothing.

**12. Network**
```bash
ping -c2 192.168.1.40      # CK5162E (digital I/O)
ping -c2 192.168.1.30      # CKDA08ETH (analog out)
```

**13. Stable USB names and the CAN interface**

a. Read the vendor ID, product ID and serial of each USB device:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep -m3 -E 'idVendor|idProduct|serial'   # IMU
udevadm info -a -n /dev/ttyACM0 | grep -m3 -E 'idVendor|idProduct|serial'   # CAN adapter
```

b. Edit `~/agv_can/amr_ws/deploy/qr/99-amr-qr.rules` so the IDs match (add
`ATTRS{serial}=="..."` if two devices share a vendor/product), then install it:
```bash
sudo cp ~/agv_can/amr_ws/deploy/qr/99-amr-qr.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
ls -l /dev/amr_imu /dev/amr_can
```

c. Bring the CAN adapter up as `can0` at 125 kbit/s:
```bash
sudo cp ~/agv_can/amr_ws/deploy/qr/slcan-can0.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now slcan-can0
ip -details link show can0          # UP, bitrate 125000
```
If `can0` already exists without slcand, the adapter runs candleLight firmware. Skip the
service and use `sudo ip link set can0 up type can bitrate 125000` instead.

d. `profiles/amr-qr-01.json` already uses `"imu_port": "/dev/amr_imu"`.

**14. Sensor checks** (read-only tools)

a. Panel: press E-stop, START and STOP one at a time. DI0 / DI1 / DI2 must follow.
```bash
ros2 run amr_base qr_calibrate io
```

b. Encoders: push the vehicle FORWARD by hand. Both speeds must read **positive**. If one
reads negative, set `enc_invert_<side>: true` in the profile. The first lines show each
encoder's 6001h / 6002h and the wrap range used; with `enc_range_counts: 0` it is read
from the encoder itself. A `REJECTED` count on a line means readings are being thrown
away (a range that does not match the unit) - report it.
```bash
ros2 run amr_base qr_calibrate enc
```

c. IMU: spin the vehicle LEFT (counter-clockwise). The output must show frame `0x52`, and
`wz` must be **positive**; if it is negative, set `"imu_gyro_sign": -1.0`. The absolute
`yaw` value does not matter (it is wherever the unit powered up; only the yaw RATE is
used).
```bash
ros2 run amr_base qr_calibrate imu
```
If the rate is under ~40 Hz (the AMR QR's unit shipped at 9600 baud / 10 Hz), reconfigure
it, then set `"imu_baud": 115200` in the profile, power-cycle the IMU and run
`qr_calibrate imu` again:
```bash
ros2 run amr_base qr_calibrate imu-setup --rate 50 --baud 115200 --go
```
If the unit ignores the commands, the tool says so: use the WitMotion PC software instead.

## Part E — First motion (ON BLOCKS)

**15. Safety first**
- If the CK5162E / CKDA08ETH support a "communication lost → outputs off" setting,
  turn it on.
- Confirm the E-stop cuts the motor drivers in hardware, not only through DI0.
- Test: pull the I/O module's LAN cable. The motors must stop.

**16. Measure the feedforward**
```bash
ros2 run amr_base qr_calibrate breakaway --wheel both --go
ros2 run amr_base qr_calibrate ff --wheel both --go          # type YES when asked
```
When a run ends (finished, Ctrl-C or an error) the tool brakes until the wheels stop,
then releases the brakes so the vehicle can be pushed. If anything ever leaves a brake or
a direction output on (a crash, `kill -9`), free everything with:
```bash
ros2 run amr_base qr_calibrate brake-off
```
Copy the printed `ff_motor_rpm_per_volt` and `ff_offset_v` into the profile. The tool
also prints the largest `vehicle.motor_max_rpm` the loader accepts; raise
`motor_max_rpm` (and `v_max_v` if needed) up to it.

**17. First ROS bring-up**

Terminal 1 — the base, no lidar yet:
```bash
ros2 launch amr_bringup drivers.launch.py lidar:=false
```

Terminal 2 — status:
```bash
ros2 topic echo /drives/status --once       # operational: true, link_state: armed (MANUAL)
ros2 topic echo /amr/panel_state --once     # valid: true, mode_auto: false
```
Drive power follows gvievo's power model: in MANUAL the drives are powered (the QR panel
has no Manual Arm input); in AUTO they sit in **standby** (0 V, brakes held,
`link_state: standby`) until a run asks for power after Start. See OPERATE §0.

Terminal 3 — keyboard drive, slow speeds. The panel must be MANUAL, which it is at boot.
```bash
ros2 run amr_base wasd_teleop
```
**Hold** W / S (forward / back), A / D (turn left / right), or the arrow keys; releasing
the key stops. Q / E and Z / C drive arcs, space stops, `+` / `-` and `]` / `[` change the
linear and turn speed (start 0.10 m/s, 0.30 rad/s). `wasd_teleop` is a new entry point:
after the `git pull` that adds it, rebuild once with
`colcon build --symlink-install --packages-select amr_base` and source again.
(`teleop_twist_keyboard` also works: `i` forward, `,` back, `j` / `l` turn, `k` stop.)

What to check:
- Forward turns both wheels forward. Left turns the left wheel back and the right
  wheel forward.
- A wrong sign latches a FAULT ("turning AGAINST the command") within a second. Fix
  the profile, restart, then clear the fault:
  ```bash
  ros2 service call /drives/ack_fault std_srvs/srv/Trigger
  ```

## Part F — Lidar and the floor

**18. Lidar**
- The scanner is at 192.168.1.2 (`nanoscan3.amr-qr-01.yaml`); `host_ip: "auto"` picks
  this PC's address on the route to it. If no route exists at launch, `lidar.host_ip` in
  the profile is used instead (set it to this PC's 192.168.1.x address).
- Start it:
  ```bash
  ros2 launch amr_bringup lidar.launch.py foxglove:=true
  ```
- In Foxglove (`ws://<robot-ip>:8765`), a box 2 m ahead must appear at +x and a box to
  the left at +y. If not, correct `laser_yaw` / `laser_x` in
  `amr_description/config/vehicle.amr-qr-01.yaml`.
- Measure the real front and rear overhang. Update `footprint.amr-qr-01.yaml` and
  `chassis_x` together, then rerun `pytest src/amr_description/test/test_vehicle.py`.

**19. Ground test** (on the floor, `drivers.launch.py` + teleop at 0.1 m/s)
- Straight 1 m twice, reverse 1 m: compare `/odom_raw` with a tape measure.
- Turn 90° and 180° both ways: compare odometry with the gyro.
- Target: within 1 %. If not, correct `vehicle.track_m` or `qr_base.enc_counts_per_rev`.

## Part G — Run it as a service

**20. Install the systemd service**

The deploy files are already written for user `amrslam` and profile `amr-qr-01`
(`amr.service`, `amr.env`, `amr-web.sudoers`); do not edit them on the PC, or the next
`git pull` stops on them. A different user name: change it in the repo and push.
```bash
cd ~/agv_can/amr_ws/deploy
sudo ./install.sh                    # validates + installs amr.service, sysctl + journald drop-ins
sudo install -o root -g root -m 0440 amr-web.sudoers /etc/sudoers.d/amr-web && sudo visudo -c
sudo systemctl restart systemd-journald
sudo systemctl enable --now amr.service
journalctl -u amr.service -f
```
The sudoers line lets the web page's **Restart** button restart `amr.service`.
Open `http://<robot-ip>:5001/`. The Status page should show Mode `IDLE`, Selector
`MANUAL` and Drives `ARMED`. Next: [OPERATE_AMR_QR.md](OPERATE_AMR_QR.md).

---

## If something goes wrong

| Symptom | Likely cause |
|---|---|
| `qr_base_node` exits with "owned by ..." | Another process holds the dio/can/imu lock (a `qr_calibrate` still running, or a second launch) |
| `ModuleNotFoundError: agv_core` | `~/agv_can` not on `PYTHONPATH`: source `env/vehicle.sh` (the service does it itself) |
| `CAN unavailable` | `can0` down (13c) or the adapter was replugged |
| Drives never `operational` | DIO or analog module unreachable, encoders silent (`candump can0`), E-stop pressed |
| `hardware launch refuses domain` | `ROS_DOMAIN_ID` isn't 10; `env/vehicle.sh` not sourced |
| `TypeError ... device_id` | pymodbus older than 3.10 (step 5) |
| Robot slow | Expected until step 16 raises `motor_max_rpm` |

---

## Part U — Updating from the previous AMR SLAM code

The repo moved to the latest gvievo-01 code: first gvievo-main (2026-11-02: `config.py`,
`core/` and `drivers/` became the `agv_core/` library, the legacy controller is gone),
then **KYB trackless** (2026-11-04: wait steps, Go home, S-curve, Measure, the new page
layout; the tape/RFID package `amr_line` removed). Messages changed both times. A plain
`git pull` plus a **full** rebuild does it - `rm -rf build install log` also clears the
removed `amr_line` package out of the install space. Nothing on the robot outside
`~/agv_can` changes, and `~/amr_maps` and `~/.amr` are kept.

```bash
sudo systemctl stop amr.service 2>/dev/null; pkill -f "ros2 launch" ; true
cd ~/agv_can
git status --short                 # local edits block the pull - see below
git checkout -- .                  # drop them (the repo now has the robot's values)
git pull
git log --oneline -1               # the "rebase on gvievo-01-gvievo-main" commit or newer
cd amr_ws && rm -rf build install log
colcon build --symlink-install && source install/setup.bash
source env/vehicle.sh
python3 -c "from agv_core import config; print(config.PROFILE_NAME, config.PLATFORM)"   # amr-qr-01 qr_analog
cd ~/agv_can && python3 tests/run_all.py | tail -2                                     # 762 checks
```
`git status` listing `profiles/amr-qr-01.json` or `nanoscan3.amr-qr-01.yaml`: the repo
already carries the robot's values (IMU 115200 baud, right encoder inverted, measured
feedforward, scanner 192.168.1.2). Note anything else you changed on the robot before
the checkout and put it back in the repo.

Then re-run Part G step 20 (the unit files changed: watchdog, sudoers, journald), and
repeat the quick checks: step 14 (`qr_calibrate io / enc / imu`), step 17 on blocks
(`drivers.launch.py lidar:=false` + `wasd_teleop`), step 18 (lidar). The scan filters
are off since the lidar was replaced, so `~/.amr/robot_fixed.yaml` and
`~/.amr/scan_mask.yaml` are no longer read and can be deleted.

After the restart, press **↻** at the top right of the web page once so the browser
loads the new pages. Routes and missions saved before the update keep working: a
mission is compiled when it is loaded, so it runs at the new 0.27 m/s with the S-curve
without being saved again.

`deploy/netplan/` is KYB's three-port network layout (DIO, nanoScan3 and GLS camera each on
its own port). **Do not copy it** to the AMR QR: everything here stays on 192.168.1.x.

What behaves differently after the update: [OPERATE_AMR_QR.md](OPERATE_AMR_QR.md) §0 and §4–§5.
