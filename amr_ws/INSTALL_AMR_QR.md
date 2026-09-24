# Installing the SLAM stack on the AMR QR, from zero

Target: the AMR QR's PC, **Ubuntu 24.04 + ROS 2 Jazzy**, profile `amr-qr-01`.
Background and design: [AMR_QR.md](AMR_QR.md). Day-to-day operation after install:
[OPERATE_AMR_QR.md](OPERATE_AMR_QR.md).

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

**6. Unpack into `~/agv_can`**
```bash
cd ~
unzip ~/Downloads/gvievo-01-slam-roadmap-amr-qr.zip
mv gvievo-01-slam-roadmap agv_can
ls ~/agv_can        # config.py, profiles/, amr_ws/, ...
```

**7. Shell environment**: append to `~/.bashrc`
```bash
source /opt/ros/jazzy/setup.bash
export AGV_PROFILE=amr-qr-01
[ -f ~/agv_can/amr_ws/install/setup.bash ] && source ~/agv_can/amr_ws/install/setup.bash
source ~/agv_can/amr_ws/env/vehicle.sh      # ROS_DOMAIN_ID=10, CycloneDDS on loopback
```
Then:
```bash
source ~/.bashrc
echo $ROS_DISTRO $AGV_PROFILE $ROS_DOMAIN_ID     # jazzy amr-qr-01 10
```

**8. Check that the profile loads**
```bash
cd ~/agv_can
python3 -c "import config; print(config.PROFILE_NAME, config.PLATFORM, round(config.MAX_SPEED_MPS,3))"
# amr-qr-01 qr_analog 0.283
python3 main.py      # must REFUSE ("use the ROS stack") - correct on this vehicle
```

**9. Build**
```bash
cd ~/agv_can/amr_ws
colcon build --symlink-install
source install/setup.bash
```
Always build with `--symlink-install`. The nodes find `config.py` and `profiles/`
through the symlinks, so after a profile edit you only restart the nodes; no rebuild.

**10. Offline tests** (no hardware)
```bash
cd ~/agv_can && python3 tests/run_all.py | tail -2
# 923 checks. The one allowed FAIL, "pp enabled with unset drive values is refused",
# was already failing before the AMR QR work.
cd amr_ws && python3 -m pytest -q src/amr_base/test/test_qr_*.py src/amr_description/test/
```

## Part D — Hardware links (parked, nothing moves)

**11. Stop the old QR controller.** Stop `app.py` (Flask, port 5050) and disable whatever
starts it at boot (systemd unit, cron job, autostart). It owns the same I/O modules, CAN
bus and IMU port, and `qr_base_node` refuses to start while it runs.

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

d. In `~/agv_can/profiles/amr-qr-01.json` set `"imu_port": "/dev/amr_imu"`.

**14. Sensor checks** (read-only tools)

a. Panel: press E-stop, START and STOP one at a time. DI0 / DI1 / DI2 must follow.
```bash
ros2 run amr_base qr_calibrate io
```

b. Encoders: push the vehicle FORWARD by hand. Both speeds must read **positive**. If one
reads negative, set `enc_invert_<side>: true` in the profile.
```bash
ros2 run amr_base qr_calibrate enc
```

c. IMU: spin the vehicle LEFT (counter-clockwise). The output must show frame `0x52`, and
`wz` must be **positive**; if it is negative, set `"imu_gyro_sign": -1.0`. If only `0x53`
appears, or the rate is under ~40 Hz, set the unit to 115200 baud and 50-100 Hz with gyro
output using the WitMotion PC tool, then update `imu_baud`.
```bash
ros2 run amr_base qr_calibrate imu
```

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
ros2 topic echo /drives/status --once       # operational: true
ros2 topic echo /amr/panel_state --once     # valid: true, mode_auto: false
```

Terminal 3 — keyboard drive, slow speeds. The panel must be MANUAL, which it is at boot.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
```

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
- Put the scanner's IP and this PC's address into
  `amr_ws/src/amr_bringup/config/nanoscan3.amr-qr-01.yaml`. Keep clear of 192.168.3.1,
  which is the GLS621.
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
```bash
cd ~/agv_can/amr_ws/deploy
sed -i "s/gvipc-evo-01/$USER/g" amr.service amr_nav.service amr_mapping.service amr.env amr_legacy.env
sed -i 's/^AGV_PROFILE=agv-01/AGV_PROFILE=amr-qr-01/' amr.env
sudo ./install.sh                    # validates + installs unit files, starts nothing
sudo systemctl enable --now amr.service
journalctl -u amr.service -f
```
Open `http://<robot-ip>:5001/`. The Status page should show Mode `IDLE`, Selector
`MANUAL` and Drives `ARMED`. Next: [OPERATE_AMR_QR.md](OPERATE_AMR_QR.md).

---

## If something goes wrong

| Symptom | Likely cause |
|---|---|
| `qr_base_node refuses to start` | `app.py` still running, or something else listens on port 5050 |
| `CAN unavailable` | `can0` down (13c) or the adapter was replugged |
| Drives never `operational` | DIO or analog module unreachable, encoders silent (`candump can0`), E-stop pressed |
| `hardware launch refuses domain` | `ROS_DOMAIN_ID` isn't 10; `env/vehicle.sh` not sourced |
| `TypeError ... device_id` | pymodbus older than 3.10 (step 5) |
| Robot slow | Expected until step 16 raises `motor_max_rpm` |
