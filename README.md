# gvievo-01 — AGV / AMR platform

> ### This repo: the AMR QR (`amr-qr-01`)
> The gvievo-01 stack below, with the **AMR QR** platform added: analog BLDC drivers
> (CKDA08ETH + CK5162E), CANopen wheel encoders, a WitMotion IMU, a START/STOP panel and
> scan filters for its nanoScan3. Ubuntu 24.04 + ROS 2 Jazzy, SLAM only.
> Start here: [AMR_QR.md](amr_ws/AMR_QR.md) (design) ·
> [INSTALL_AMR_QR.md](amr_ws/INSTALL_AMR_QR.md) (install / update) ·
> [OPERATE_AMR_QR.md](amr_ws/OPERATE_AMR_QR.md) (daily operation).
> Everything below describes gvievo-01 (`agv-01`), which still loads from this repo.

Control software for a 150 kg differential-drive vehicle. Two Oriental Motor
BLV-R drivers in CiA 402 Profile Velocity mode share one `can0` with a SICK MLS
whose built-in gyro carries the heading; a SICK nanoScan3 safety scanner and a
Modbus I/O island sit on Ethernet. The vehicle runs
as one systemd unit, `amr.service`: a supervisor, the hardware nodes, a web UI
on port 5001, and one *mode layer* at a time.

> ### Status (2026-10-27)
> **One product: the trackless SLAM AMR** (`MAPPING`/`NAVIGATION`:
> slam_toolbox, AMCL, Nav2). The tracked product (the `LINE` mode, `amr_line`,
> the MLS tape reading, the RFID reader and the tape missions) was removed on
> 2026-10-27 (`manuals/trackless-only-removal-plan.md`); its last build is the
> tag `tracked-final` (removal plan D18).
>
> **The legacy standalone controller is gone (U11).** `main.py`, `app/` and
> `canworker.py` were deleted on 2026-09-21; the last deployable revision is
> `aa02ec7` (tag `legacy-final`, removal plan D18). Its offline run logs are
> archived under `manuals/obsolete/legacy-runs/`. Nothing else on the vehicle
> owns `can0`.
>
> **The nanoScan3 is owned by the ROS stack** (`sick_safetyscanners2`); its
> liveness is read from `/amr/localization_state`. The protective-field
> outputs reach the drives through the FX3 in hardware, and the software only
> *reads* them (`/output_paths`) to explain a stop.

| | |
|---|---|
| Runtime | `amr.service` → `deploy/amr-supervisor.sh` → ROS 2 Humble, `amr_ws/` |
| Bus | CANopen 125 kbps, 50 Hz feedback by TPDO, setpoint by RPDO |
| Nodes | 1 left driver, 2 right driver, 10 SICK MLS (gyro only) |
| Manual | pendant 0.50 m/s, web jog 0.40 m/s, only under selector MANUAL |
| Auto | 0.60 m/s on a route, only under selector AUTO after a physical Start |
| Tests | `tests/run_all.py` (pinned check count) + `pytest` per ROS package, no hardware |

---

## ⚠️ Safety model

**This software moves a 150 kg vehicle and has no authentication.** Anyone who
can reach port 5001 can drive it. Keep it on a trusted network.

**The panel owns every entry into motion.** Manual jog (browser or pendant)
works only while the selector rests in MANUAL and the supervisor has granted
the manual lease; every autonomous start — a route, a survey move — needs the selector in AUTO *and* a fresh physical Start press. No
web page can start the vehicle.

**A manual direction is HELD, not latched.** The jog pad re-POSTs while a
button is down; the command mux drops a stream whose stamp is older than its
`valid_for_s` (0.2 s). A closed tab, a dropped Wi-Fi link and a released
button are indistinguishable to the vehicle, which is the point.

**Arming excites the motor; it does not free it.** CiA 402 "Operation enabled"
is a servo lock, not a free shaft. Only the drive's `FREE` input releases the
brake, and `FREE` is on the CAN write deny-list. A disarmed vehicle cannot be
pushed either.

**Nothing in this repo is in the safety path.** The chain is lidar/encoders →
FX3 → HWTO1/HWTO2 → STO, wired in hardware with EDM returned to the FX3. Nothing
read over CAN or Ethernet is rated, redundant or certified, and nothing here may
decide whether it is safe to move. What the software adds is *recovery*: after
a lidar stop it resumes by itself once the field is clear; after the E-stop
button it waits for Start.

---

## Running

```bash
sudo systemctl start amr.service          # the only AGV boot service
systemctl status amr.service --no-pager
journalctl -u amr.service -f
```

Everything operational — boot checks, mode swaps, surveys, routes, holds,
rollback — is in [amr_ws/RUNBOOK.md](amr_ws/RUNBOOK.md). Deployment files
(`amr.service`, `amr.env`, the launch wrappers, `install.sh`, `validate.sh`)
are under [amr_ws/deploy/](amr_ws/deploy/).

**Which vehicle** comes from `AGV_PROFILE`, naming a file in `profiles/`.
Unset means `agv-01`. A name with no matching file is fatal at boot.

Bench work with the service stopped:

```bash
python3 -m agv_core.drivers.canbus.verify_bus       # nodes 1, 2, 10 answer
python3 -m agv_core.drivers.canbus.verify_drivers   # BLV-R identity and state
python3 -m agv_core.drivers.canbus.read_imu show
```

Every bench tool takes the CAN owner lock first and refuses, naming the owner,
while `amr.service` (or another tool) holds it.

### Dependencies

ROS 2 Humble with Nav2, slam_toolbox, robot_localization,
`sick_safetyscanners2`; `python-can`, `pymodbus`, `flask`. The vehicle image
has them; nothing is pinned in `pyproject.toml` so the offline suite runs on a
dev box without them.

---

## Layout

**`agv_core/` is the library, `amr_ws/` is the runtime.** The runtime imports
the library; the library imports neither ROS nor Flask.

```
agv_core/          The vehicle library. No ROS, no Flask, no web tier.
  config.py          Loads and validates the profile. Everything imports this.
  kinematics.py      Wheel <-> body conversions from the profile geometry.
  ownerlock.py       The can/dio owner locks (fcntl); one owner per device.
  panel.py, canmon.py, events.py, motion.py, lidarframe.py
  drivers/           dio.py (Modbus I/O island)
  drivers/canbus/    CANopen decoders, the write deny-list (guard.py), RPDO
                     setup, and the bench tools listed above.

amr_ws/            The ROS 2 workspace (colcon, symlink install).
  src/amr_base         drive_node (the can0 owner), panel_node, cmd_mux,
                       odometry, the MLS gyro reader
  src/amr_bringup      the supervisor, launch files, deploy/ and env/
  src/amr_mission      routes, the run FSM, survey moves, the executor
  src/amr_navigation   route compiler, validation, Nav2 parameters
  src/amr_localization SLAM/AMCL glue, readiness, the localisation monitor
  src/amr_maps         map bundles and revisions
  src/amr_web          the Flask UI on 5001 and its ROS adapter
  src/amr_sim          fake base/panel/IMU, synthetic scan
  src/amr_interfaces   messages and services

tests/             Offline checks for agv_core. run_all.py runs them.
profiles/          One JSON per vehicle. Every tunable parameter lives here.
manuals/           Device documentation, the SLAM plan, vehicle reports.
pyproject.toml     Packages agv_core; also the shared ruff and pytest config.
```

**`agv_core` is a real package, imported by package path.** `from agv_core
import config`, `from agv_core.drivers.canbus import guard`. ROS nodes run out
of the colcon install space, which is outside the repo, so
`amr_ws/deploy/amr-launch.sh`, `env/vehicle.sh` and `env/sim.sh` put the repo
root on `PYTHONPATH`; `amr_ws/deploy/validate.sh` refuses to install if
`agv_core/` is not where the launch wrapper expects it.

**Dependency graph:** `agv_core.config` imports only the standard library;
everything else imports it. No cycles. Nothing in `agv_core/drivers/canbus/`
imports `config` — those modules stay runnable on a bench. `tests/test_layout.py`
pins the package boundary.

---

## How it works

### One owner per device

`amr_base.drive_node` is the only process that opens `can0`, `panel_node` the
only one that writes the I/O island; each takes its owner lock at start. Inside
`drive_node`, one bus thread does every CAN transaction: feedback arrives by
TPDO at 50 Hz, the setpoint leaves by RPDO, and the MLS gyro is SDO-polled at
50 Hz. Everything above it — mux, executor, survey moves — sends
stamped commands on topics and holds a *permit*; nothing else touches the bus.

### Authority

The supervisor grants one **lease** at a time (manual or autonomous),
the command mux applies only the source that lease allows, and the
physical panel gates every transition: MANUAL for jog, AUTO plus a Start edge
for anything autonomous, Reset to end a run. A lease is bound to a
*generation* that rotates on every mode swap, so a replaced layer's commands
can never look fresh to the new mux.

### What the vehicle may write over CAN

[agv_core/drivers/canbus/guard.py](agv_core/drivers/canbus/guard.py) enforces a
deny-list on every write path of the bus owner (`amr_base.canopen`):

| | |
|---|---|
| **permitted** | `6040h` controlword, `6060h` modes, `6083h`/`6084h` ramps, `60FFh` target velocity, `1017h` heartbeat, PDO communication parameters |
| **refused** | `403Eh` (bit 6 is **FREE** — releases the holding brake on *both* drive wheels), `40D0h` clear ETO, `40C0h` alarm reset, `6040h` bit 7 fault reset, `1010h`/`1011h` store/restore, all `4xxxh` parameters, `607Ah`/`6081h` and the position-mode safety objects (profile position was removed 2026-10-30) |

A denied write raises rather than being silently dropped. RPDO mappings are
admitted only if the mapped object is itself writable — a mapping is a write
path by another name. Clearing ETO or resetting an alarm automatically would
be an automatic restart, which ISO 3691-4 prohibits: a latched drive alarm is
cleared by a power cycle, not by software.

### The MLS gyro

The SICK MLS on node 10 is used only for its **gyro**, which carries the
heading for SLAM (the EKF weights it far above the wheels). `drive_node` reads
the yaw rate by SDO (2034h:03), which works whether node 10 is Operational or
Pre-operational, and publishes it on `/imu/data_raw`; `imu_bias_node` turns it
into `/imu/data`. The sensor's TPDO1 (`0x18A`) is no longer used: it may still
be on the bus if the MLS has stayed Operational since power-up, and nothing
listens to it.

---

## Configuration

Everything tunable lives in `profiles/<name>.json`, selected by `AGV_PROFILE`.
**`agv_core/config.py`'s docstring is the manual.**

1. **Unknown and missing keys are both fatal.**
2. **Only primitives are stored.** Anything derivable is derived.
3. **Validation runs before anything is published.**
4. **`profile_name` must match the filename.**

Speeds, ramps and the jog and pendant limits are all there. The
vehicle boots into IDLE. The web UI's Params page shows every
value the running nodes actually use beside the profile's.

---

## Testing

```bash
python3 tests/run_all.py                                   # pinned check count, no hardware
cd amr_ws && source install/setup.bash && source env/vehicle.sh
env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test   # per package
ruff check agv_core tests && (cd amr_ws && ruff check src)
```

`run_all.py` **pins both the module list and the total check count**; a module
that stops being imported fails the run instead of silently costing coverage.
Raise `EXPECTED_CHECKS` deliberately, with a note, when adding checks; never
lower it to get a green run.

Simulation launch tests are opt-in (`AMR_SIM_TESTS=1`) and check mechanisms,
not tolerances. Vehicle acceptance is a witnessed session (for the removal,
the V1-V12 table in `manuals/trackless-only-removal-plan.md` §6); earlier
reports are archived under `manuals/obsolete/vehicle-reports/`.

---

## Invariants

**`agv_core` may not import ROS or Flask.** The bench tools and the nodes both
import it. `tests/test_layout.py` scans for it, lazy imports included, and
also refuses a bare sibling import inside the package.

**`agv_core/drivers/canbus/` is a runtime dependency, not a scratch directory.**
`amr_base.canopen` imports from it. Do **not** make those modules import
`config`; where they need a vehicle-specific value, take it as a parameter
defaulting to the module constant.

**NMT is per node.** The drive link puts *the drives* in Pre-operational for
PDO setup and starts *the drives*; it never broadcasts. A broadcast silenced
the MLS on the vehicle (2026-09-21).

**Wheel sign convention:** both drivers take a **positive** `60FFh` to travel
forward. If you swap a motor, re-flash a driver or remount a wheel, re-verify
on blocks and set `vehicle.invert_left` / `invert_right`.

---

## Known gaps

- **`6064h` scaling is unverified** beyond the (retired) blind-run commissioning runs.
- **Backlash is unmeasured.** The encoders are motor-side of a 30:1 gearhead.
- MEXE02 load inertia is still `0: Small (2×)`; should be `1: Medium (7.5×)`.
