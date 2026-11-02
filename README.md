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
magnetic line sensor (track reading and IMU); a SICK nanoScan3 safety scanner,
a Chafon RFID reader and a Modbus I/O island sit on Ethernet. The vehicle runs
as one systemd unit, `amr.service`: a supervisor, the hardware nodes, a web UI
on port 5001, and one *mode layer* at a time.

> ### Status (2026-09-21)
> **Two products on one stack.** The vehicle is a magnetic-tape AGV (the
> `LINE` mode: `amr_line`, ported from the tape controller that drove it
> before) with the SLAM AMR (`MAPPING`/`NAVIGATION`: slam_toolbox, AMCL, Nav2)
> as the upgrade. Which one a vehicle is comes from its profile.
>
> **The legacy standalone controller is gone (U11).** `main.py`, `app/` and
> `canworker.py` were deleted on 2026-09-21 after the ported LINE mode was
> accepted on the vehicle; the last deployable revision is tagged
> `legacy-final`. Its offline run logs are archived under
> `manuals/obsolete/legacy-runs/`. Nothing else on the vehicle owns `can0`.
>
> **The nanoScan3 is owned by the ROS stack** (`sick_safetyscanners2`); its
> liveness is read from `/amr/localization_state`. The protective-field
> outputs reach the drives through the FX3 in hardware, and the software only
> *reads* them (`/output_paths`) to explain a stop.

| | |
|---|---|
| Runtime | `amr.service` → `deploy/amr-supervisor.sh` → ROS 2 Humble, `amr_ws/` |
| Bus | CANopen 125 kbps, 50 Hz feedback by TPDO, setpoint by RPDO |
| Nodes | 1 left driver, 2 right driver, 10 SICK MLS (track at 100 Hz + gyro) |
| Manual | pendant 0.50 m/s, web jog 0.40 m/s, only under selector MANUAL |
| Auto | tracked NORMAL 0.50 m/s, HIGH 0.85 m/s (`auto_high_ratio` 1.7) only inside an RFID high zone; trackless 0.60 m/s |
| Tests | `tests/run_all.py` 531 checks + `pytest` per ROS package, no hardware |

---

## ⚠️ Safety model

**This software moves a 150 kg vehicle and has no authentication.** Anyone who
can reach port 5001 can drive it. Keep it on a trusted network.

**The panel owns every entry into motion.** Manual jog (browser or pendant)
works only while the selector rests in MANUAL and the supervisor has granted
the manual lease; every autonomous start — a route, a survey move, the tape
follower — needs the selector in AUTO *and* a fresh physical Start press. No
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
python3 -m agv_core.drivers.canbus.read_mls stream --nmt   # the track reading
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
  panel.py, canmon.py, events.py, motion.py, blindrun.py, lidarframe.py
  drivers/           dio.py (Modbus I/O island), rfid.py (Chafon reader)
  drivers/canbus/    CANopen decoders, the write deny-list (guard.py), RPDO
                     setup, and the bench tools listed above.

amr_ws/            The ROS 2 workspace (colcon, symlink install).
  src/amr_base         drive_node (the can0 owner), panel_node, cmd_mux,
                       odometry, commissioning, the MLS track reader
  src/amr_bringup      the supervisor, launch files, deploy/ and env/
  src/amr_line         LINE mode: the tape engine (autopilot.py, branch.py are
                       the verbatim port), the follower node, its authority
  src/amr_mission      routes, the run FSM, survey moves, the executor
  src/amr_navigation   route compiler, validation, Nav2 parameters
  src/amr_localization SLAM/AMCL glue, readiness, the localisation monitor
  src/amr_maps         map bundles and revisions
  src/amr_web          the Flask UI on 5001 and its ROS adapter
  src/amr_sim          fake base/panel/IMU, synthetic scan and tape
  src/amr_interfaces   messages and services

tests/             531 offline checks for agv_core. run_all.py runs them.
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
TPDO at 50 Hz, the setpoint leaves by RPDO, and the MLS track reading rides on
TPDO1 `0x18A` at 100 Hz. Everything above it — mux, executor, follower — sends
stamped commands on topics and holds a *permit*; nothing else touches the bus.

### Authority

The supervisor grants one **lease** at a time (manual, autonomous, commissioning
or line), the command mux applies only the source that lease allows, and the
physical panel gates every transition: MANUAL for jog, AUTO plus a Start edge
for anything autonomous, Reset to end a run. A lease is bound to a
*generation* that rotates on every mode swap, so a replaced layer's commands
can never look fresh to the new mux.

### What the vehicle may write over CAN

[agv_core/drivers/canbus/guard.py](agv_core/drivers/canbus/guard.py) enforces a
deny-list on every write path of the bus owner (`amr_base.canopen`):

| | |
|---|---|
| **permitted** | `6040h` controlword, `6060h` modes, `6083h`/`6084h` ramps, `60FFh` target velocity, `607Ah`/`6081h` for PP, `1017h` heartbeat, PDO communication parameters |
| **refused** | `403Eh` (bit 6 is **FREE** — releases the holding brake on *both* drive wheels), `40D0h` clear ETO, `40C0h` alarm reset, `6040h` bit 7 fault reset, `1010h`/`1011h` store/restore, all `4xxxh` parameters, the PP safety objects |

A denied write raises rather than being silently dropped. RPDO mappings are
admitted only if the mapped object is itself writable — a mapping is a write
path by another name. Clearing ETO or resetting an alarm automatically would
be an automatic restart, which ISO 3691-4 prohibits: a latched drive alarm is
cleared by a power cycle, not by software.

### The MLS

The line sensor serves both products: its **track reading** (LCP positions,
`#LCP`, marker) feeds the tape follower, and its **gyro** carries the heading
for SLAM (the EKF weights it far above the wheels). Tape to the vehicle's left
reads *positive* millimetres. The reader starts the node by NMT so TPDO1 flows
and falls back to SDO polling (monitor only, ~10 Hz) if it does not; the
follower refuses to run on SDO samples or below `line_min_track_hz`.

---

## Configuration

Everything tunable lives in `profiles/<name>.json`, selected by `AGV_PROFILE`.
**`agv_core/config.py`'s docstring is the manual.**

1. **Unknown and missing keys are both fatal.**
2. **Only primitives are stored.** Anything derivable is derived.
3. **Validation runs before anything is published.**
4. **`profile_name` must match the filename.**

Speeds, ramps, the jog and pendant limits, the PP lock and the tape engine's
35 gains (`autopilot`) are all there — and the product: top-level
`"tracked": false` is the SLAM AMR, `true` the tape AGV that boots into LINE
and refuses maps and routes, `null` both: boots into LINE, trackless by the Home
switch (RUNBOOK §1). The web UI's Params page shows every
value the running nodes actually use beside the profile's.

---

## Testing

```bash
python3 tests/run_all.py                                   # 531 checks, no hardware
cd amr_ws && source install/setup.bash && source env/vehicle.sh
env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test   # per package
ruff check agv_core tests && (cd amr_ws && ruff check src)
```

`run_all.py` **pins both the module list and the total check count**; a module
that stops being imported fails the run instead of silently costing coverage.
Raise `EXPECTED_CHECKS` deliberately, with a note, when adding checks; never
lower it to get a green run.

Simulation launch tests are opt-in (`AMR_SIM_TESTS=1`) and check mechanisms,
not tolerances. Vehicle acceptance is a witnessed session from a
`VEHICLE-TEST-*.txt` file at the repo root; reports go to
`manuals/vehicle-reports/`.

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

**`autopilot.py` and `branch.py` are the port.** They are the engine that drove
this vehicle on tape and are not edited; if the gains look wrong, the gains
move in the profile.

---

## Known gaps

- **Tracked jobs are sim-tested only** (2026-10-08): the mission is the site's RFID
  tag table (`missions/line-a.json`, schema in `agv_core/mission.py`) - Home, an
  always-stop, destination stops (MRU1-4), a track-end U-turn and a speed toggle.
  The job (mission + destination) is set on Run tracked; the vehicle may start anywhere
  on the line, then the physical Start runs it. Tag ids `0010`... are assumed to be what the RFID
  panel shows; stop positions, the 2 s speed change, the tape-end overshoot and the
  pivot reacquisition angle are settled on the floor.
- **No floor run of the tape follower yet** (LINE acceptance section 4).
- **0.85 m/s (tracked HIGH) runs with no software gate, unvalidated** (operator
  decision 2026-10-08, manuals/tracked-speed-plan-1-no-slam.md D2): the nanoScan3
  field set (`lidar.zones_validated` false) and the STO stopping distance from 0.85 m/s
  are a floor check before the first HIGH run. There is no FX3 encoder speed monitoring
  yet. HIGH needs a mission `high_zone`; `line-a` has none until the shortest straight
  is surveyed, so it runs at NORMAL.
- **`6064h` scaling is unverified** beyond the commissioning runs.
- **Backlash is unmeasured.** The encoders are motor-side of a 30:1 gearhead.
- **RFID reader is still on the US region (902.75–927.25 MHz)** and every read so far reports channel 0;
  Indonesia permits 920–923 MHz. Protocol is captured and CRC-verified (2026-10-08). The 0x0050
  heartbeat ships off (`rfid.heartbeat_s` 0) until a bench run shows it does not disturb streaming.
- MEXE02 load inertia is still `0: Small (2×)`; should be `1: Medium (7.5×)`.
