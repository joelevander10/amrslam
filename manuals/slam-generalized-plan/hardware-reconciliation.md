# Hardware Reconciliation — deltas from the generic SLAM plan

**Written:** 2026-09-03 · branch `refactor02` · against `amr_slam_architecture.md` v1
and `amr_implementation_spec.md`
**Purpose:** those two documents were generated against a generic AMR. This one
records where **gvievo-01's actual hardware and existing codebase** diverge, and
what each divergence costs or saves.
**Scope confirmed with the operator:** open-source SLAM; virtual path over a map;
no certification scope; no fleet integration; MLS *hardware stays installed* —
only the magnetic-tape-following feature is retired.

Read the architecture doc first. Where this document contradicts it, the hardware
wins. Hardware steps that follow from it are written up in
[`bench-checklists.md`](bench-checklists.md).

---

## 1. Platform at a glance

| | Generic plan assumed | We actually have | Delta |
|---|---|---|---|
| Drives | Kinco FD, CANopen | **Oriental Motor BLV-R (BLVD-KRD)**, nodes 1 & 2 | **D-1** |
| Wheel encoders | Separate CiA-406 absolute encoders, nodes 3 & 4 | **Motor-integrated**, read from the drive nodes | **D-2** |
| IMU | Yahboom 9-axis on serial | **Inside the SICK MLS**, CANopen node 10 | **D-3** |
| Station anchors | Floor QR/DataMatrix camera | **Chafon CF821 UHF RFID**, TCP | **D-4** |
| Discrete I/O | not mentioned | **Modbus TCP 16-DI/16-DO** @ 192.168.1.30 | **D-5** |
| Compute | i3-N300, 8 cores, 16 GB | **Intel N97, 4 cores, 8 GB** | **D-6** |
| OS / distro | Ubuntu 24.04 + ROS 2 Jazzy | **Ubuntu 22.04.5**, kernel 5.15, Python 3.10 | **D-7** |
| Existing software | greenfield | **~7 k lines, 341 offline checks, working vehicle** | **D-8** |
| Lidar | SICK nanoScan3, OSSD + Ethernet | same | none ✓ |
| CAN adapter | "PEAK or CANable required" | **CANable 2.0 already working** (`can0`) | already done ✓ |
| CAN bitrate | 1 Mbps | **125 kbps today** — agreed to raise | §5 |

Geometry already measured and in `profiles/agv-01.json`, so the spec's
`# MEASURE` placeholders are resolved: wheel radius **0.09 m** (not 0.085),
track **0.486 m** (not 0.450), gear ratio **30:1**, motor max **4000 r/min**
→ **1.257 m/s** top speed.

---

## 2. Deltas that change the design

### D-1 — Drives are Oriental Motor, not Kinco *(net simplification)*

Spec §3.4 hands the agent a Kinco-specific scaling formula
(`DEC = rpm × 512 × enc_res / 1875`) and a `KincoScaler` class to unit-test. **All
of that is deleted.** `60FFh` on the BLV-R takes **signed r/min directly** — there
is no DEC unit, no encoder-resolution term, and no conversion class. Kill the
`vel_dec_per_rad_s` parameter; the only conversion left is rad/s → r/min, which
is arithmetic.

The CiA-402 sequence in spec §3.1–3.2 is already implemented and tested in
[`canworker.py`](../../canworker.py) against these exact drives, including the
statusword masks, the 3-attempt fault policy, and a never-auto-reset rule.

**One hardware fact the generic plan does not contain, and it constrains §3.7.**
`6083h` on the BLV-R limits *both* the forward ramp *and* the rate at which the
wheel **difference** can slew. So yaw acceleration is capped in hardware:

```
alpha_max = 2 × 6083h × RAD_S_PER_RPM_DIFF        (core/kinematics.py:max_yaw_accel)
          = 2 × 2000 × 6.46e-4  =  2.586 rad/s²   at the current ramp setting
```

The spec's default `alpha_max: 1.0 rad/s²` happens to sit below that, so it is
safe — but it must be **derived from `6083h`, not hardcoded**, or a later ramp
change silently makes the mux command a yaw rate the drives cannot produce.

**Also verified:** `6064h` Position actual value is INT32, read-only, and
PDO-mappable — so wheel position for odometry is available without SDO polling.

### D-2 — No external encoders *(saves a node, costs observability)*

Spec §3.5 (`wheel_encoder_node.py`, CiA-406, object `0x6004`, nodes 3 & 4) has no
hardware to talk to. Wheel feedback comes from the **drive** nodes via `6064h`
/ `606Ch`. That deletes a node, deletes the wrap-safe multiturn arithmetic, and
removes **200 frames/s** from the bus budget.

**Be honest about what is lost.** The plan chose external encoders specifically to
*"eliminate gearbox backlash/slip ambiguity"*. Our encoders are on the **motor
side of a 30:1 gearbox**, so backlash and any compliance between motor and wheel
are structurally unobservable. Consequences:

- Odometry sees commanded shaft motion, not achieved wheel motion.
- Backlash appears as heading error on every direction reversal — worst exactly
  where a differential drive spins in place.
- Mitigation is D-3: fuse **gyro yaw rate** and let it own heading, using wheel
  odometry for translation only. This is the standard gyro-odometry arrangement
  and it is why D-3 matters more than it looks.

Set `enc_to_wheel_ratio` to the 30:1 gear ratio (already `gear_ratio` in the
profile) rather than the spec's default of 1.0.

### D-3 — The IMU is inside the MLS, on CAN, and its quality is measured

Spec §3.1's `imu_node` (Yahboom serial + `imu_filter_madgwick`) and §0.1's
"if Yahboom frame format unknown, implement against WitMotion-style 0x55 header
and flag VERIFY" both disappear. **No serial parser, no unknown frame format.**
The IMU is CANopen objects on node 10.

The architecture doc's risk table lists *"Yahboom IMU quality (hobby-grade,
drift, poor temp stability) — Medium"*. Ours is better, and it is **measured, not
assumed** (661 samples, vehicle stationary, 2026-09-03):

| | measured | note |
|---|---|---|
| gyro z bias | **+0.0574 °/s** | → 3.4 °/min heading drift |
| gyro z noise | **σ 0.0291 °/s** | *below one LSB* (LSB = 0.061 °/s) |
| gravity vector | **0.9894 g** | 1.1 % scale error, uncalibrated — normal |
| internal sampling | 200 Hz | timestamp object `2035h`, 1 ms, wraps at 65.536 s |

Objects: Euler `2030h:1-3` (1/10000 rad), quaternion `2031h:1-4`, raw accel
`2033h:1-3` (2⁻¹¹ g/LSB), **raw yaw rate `2034h:1-3`** (125·2⁻¹¹ °/s/LSB),
yaw reset `2032h:1`, orientation LPF `202Dh:4`. `2006h:2 = 1` confirmed — the IMU
and the temperature sensor share that enable bit, and temperature reads, so it
is on.

**Four consequences for the design:**

1. **Yaw-rate-only into the EKF**, exactly as spec §4.3 already recommends. The
   3.4 °/min drift makes absolute yaw useless; over a 100 ms scan interval it is
   nothing. The manual is explicit that yaw is for relative measurement only.
2. **`imu_filter_madgwick` is probably unnecessary.** The MLS already publishes a
   fused, gravity-referenced quaternion. Roll and pitch do not drift. Adding a
   second fusion stage on top of the sensor's own is duplicated work.
3. **Frames.** The IMU sits at (89.1, 2.5, −9.4) mm from the MLS LED edge, which
   is at the *front lookahead* position — not `base_link`. Yaw rate needs no
   correction (rigid body), but **any accelerometer use needs lever-arm
   compensation**, and `imu_frame` in the TF tree must be the real mounting
   point, not a copy of `base_link`.
4. **It is on the CAN bus, and the MLS has a hard TPDO limit** — see §5.

**Standing risk:** the IMU lives in the sensor whose primary feature is being
retired. Any future decision to remove the MLS hardware silently removes the
gyro that D-2's mitigation depends on. Record it in the BOM as a *navigation*
component, not a line-following one.

### D-4 — RFID is not a QR camera, and this is a real capability loss

The largest architectural delta. Spec §6 and task **T7** build an entire QR
subsystem: global-shutter camera, ring illumination, intrinsics calibration,
`zxing-cpp` detection, `solvePnP`, `/initialpose` injection with tight
covariance, and a precision-docking servo. We have a **UHF EPC Gen2 reader on
TCP** instead.

**The difference is categorical, not incremental:**

| | Floor QR + solvePnP | UHF RFID |
|---|---|---|
| Output | full relative **pose** (x, y, θ) | **identity only** |
| Accuracy | ±5–10 mm at stations | read *zone*, order of metres |
| Enables docking | yes | **no** |

So:

- **`precision_align_node.py` (spec §6.4, §3.x) cannot be built with this
  hardware.** Docking accuracy is whatever SLAM alone gives — the plan's own
  figure is ±20–50 mm. If ±10 mm is ever required at a station, a camera has to
  come back into the BOM. That is a *product* decision to take now, not a
  surprise at T7.
- **Invert the AMCL injection policy.** Spec §6.3 injects `/initialpose` with
  *tight* covariance because a QR pose is precise. An RFID read means "somewhere
  in the antenna's read zone", so it must be injected with **wide** covariance —
  a seed for the particle filter, not a correction. Injecting it tight would
  actively corrupt a well-converged AMCL.
- **Its correct job is the kidnapped-robot problem**, which is exactly what the
  operator asked it for: at power-on, the vehicle does not know where it is;
  driving over a known tag collapses the hypothesis space to one region and lets
  AMCL converge. That is genuinely valuable and it is the right tool for it.
- Corridor drift correction (spec §6.4 "optional later") is **not** available
  from RFID. Drift is bounded by lidar alone.

Rename the subsystem `station_id_node` rather than `qr_pose_node` so the
interface does not promise a pose it cannot deliver. `stations.yaml` keeps its
shape; the `qr_code_id` field becomes `rfid_tag` (4 hex chars, matching the
existing `branch_latch` convention in the profile).

### D-5 — There is a Modbus I/O island the plan does not know about

16 DI / 16 DO at **192.168.1.30:502**, already implemented in
[`drivers/dio.py`](../../drivers/dio.py): own thread, 20 Hz scan, 2.0 ms per read,
health-monitored, with a `/io` lamp page. It belongs in **Layer 1** as a node
publishing DI state, and it is where the safety-PLC handshake and E-stop status
will land when that arrives.

Network note worth carrying into the ROS design: the **RFID reader is
daisy-chained through the DIO module's second port**, so both are behind one
cable on `enp2s0`. Losing the DIO module takes the reader with it, and a NIC
carrier check cannot distinguish the two. Both need separate diagnostics
publishers so the shared cause is visible.

### D-6 — Compute is half what the plan budgeted

Plan §10 budgets against an **i3-N300 (8 E-cores, 16 GB)** and claims ≥40 %
headroom. We have an **Intel N97: 4 E-cores, 8 GB**. Same Alder Lake-N family, so
per-core is comparable and the delta is roughly **half the throughput**.

Re-running their budget on 4 cores:

| Workload | Plan estimate | On N97 |
|---|---|---|
| nav2 (RPP + costmaps + BT) | 1–1.5 | 25–38 % |
| AMCL | 0.3–0.5 | 8–13 % |
| drivers + EKF | 0.5 | 13 % |
| QR detection | 0.3–0.5 | **removed (D-4)** |
| **Runtime subtotal** | | **≈ 45–65 % of 4 cores** |
| slam_toolbox (survey) | 1–2 | **25–50 % on top** |

Runtime is workable. **Survey mapping is the problem** — slam_toolbox on top of
the runtime stack would saturate the machine during the one activity where
dropped scans directly corrupt the deliverable.

**Mitigation, and it is the plan's own:** record raw sensors to a rosbag during
the survey drive and **build the map offline** (on a laptop, or on the robot
afterwards with nav2 stopped). Make this the *default* survey workflow in §6.1
rather than a fallback. Also drop the plan's assumption that Foxglove runs during
survey; record first, look later.

8 GB RAM is adequate — a 3000 m² map at 5 cm is ~1.2 M cells, trivial — but leaves
no room for a graphical session on the robot. Headless only.

### D-7 — Humble, not Jazzy

Ubuntu **22.04.5**, kernel 5.15, Python **3.10**. Jazzy requires 24.04. Two
options:

- **ROS 2 Humble on 22.04** (EOL May 2027). Everything in the plan's BOM has
  Humble releases. Zero OS risk, and the CAN/lidar/Modbus hardware is already
  known-good on this exact install.
- **Upgrade to 24.04 + Jazzy** (EOL 2029). Longer runway, but re-validates the
  whole hardware stack — SocketCAN adapter, pymodbus, the working service.

**Recommendation: Humble now.** The objective is a working SLAM robot; a distro
upgrade buys years of support on a prototype that may not survive to need them,
and spends the one thing in short supply — a known-good hardware baseline. Revisit
before productisation.

Note this contradicts spec §0.2's "Python ≥3.12" — we are on 3.10. No `match`
statements, no PEP 695 type syntax.

### D-8 — There is a working vehicle, and it changes the migration shape

The plan is greenfield. We have ~7 k lines and **341 offline checks that need no
bus, no hardware and no daemon**. [`potential-ros-migration.md`](../potential-ros-migration.md)
§7 recommended a *bridge*: keep the existing process as bus owner and put ROS
above it, talking (v, ω) across a socket.

**That recommendation should be revised, and this document is the reason.** §7
was written on the assumption that tape following stays the primary mode, so the
boundary existed to protect a control loop that was still in charge. With
tape-following retired, `autopilot.py` is what gets deleted — and what remains
below the old seam is not a *framework*, it is a set of **plain Python libraries**:

| Module | Framework-agnostic? | Disposition |
|---|---|---|
| [`config.py`](../../config.py) | yes — stdlib only | **Import it from the ROS node.** ROS parameters silently ignore undeclared YAML keys; this loader makes unknown *and* missing keys fatal. Keep it and let ROS params carry only what launch needs. |
| [`core/kinematics.py`](../../core/kinematics.py) | yes — its docstring already names this exact seam | Becomes the mux node's inverse kinematics. |
| [`drivers/canbus/guard.py`](../../drivers/canbus/guard.py) | yes | Keep the write deny-list. Nothing in ROS supplies it. |
| [`core/health.py`](../../core/health.py) | yes | Maps onto `diagnostic_updater`; port the two-tier policy, not the code. |
| [`drivers/dio.py`](../../drivers/dio.py), [`drivers/rfid.py`](../../drivers/rfid.py) | yes | Wrap each in a thin node; the thread + `snapshot()` shape survives. |
| `canworker.py` CiA-402 sequence | partly | Extract the arm/disarm/fault state machine as a library; discard the Flask-facing queue. |
| `core/autopilot.py`, `core/branch.py` | n/a | **Retired with tape following.** |

So the recommendation becomes **library reuse, not a bridge**: one ROS workspace,
Layer 1 nodes that `import config, guard, kinematics`. That gets the plan's
`sim:=true` harness (§8, which is the best thing in either document) without a
socket in the control path, two config systems, or two watchdog models.

The 341 checks mostly survive, because they test *modules* rather than the
daemon — `test_config`, `test_control` (including the plant simulation), and
`test_canworker`'s guard scans all still apply. `test_web` and the branch tests
go with the features they cover.

**One timing fact to carry over:** the existing 50 Hz loop runs ~20.4 ms average
against a 20 ms budget, peaking near 31 ms, largely because `60FFh` is a blocking
SDO write per tick. Spec §3.3 already specifies RPDO1 for the controlword +
target velocity, which fixes this. It is a prerequisite, not an optimisation.

### D-9 — the write deny-list blocked PDO configuration, and fixing it found a hole

*Added 2026-09-06, from reading the code against this plan.*

Spec §3.3 (RPDO1 for controlword + `60FFh`) and §5's MLS IMU TPDO enable both
need SDO writes to the CiA 301 PDO configuration ranges — `1400h`/`1600h` for
an RPDO, `1800h`/`1A00h` for a TPDO. Every one of them was refused by
[`guard.py`](../../drivers/canbus/guard.py)'s "not on the permitted-write list"
branch. Two of the plan's tasks were blocked on a file neither document
mentions.

The naive fix — add the ranges to `ALLOWED` — **would have opened the exact hole
the deny-list exists to close.** An RPDO mapping is a write path by another
name: map `403Eh` into an RPDO and a two-byte CAN frame releases the holding
brake on both drive wheels, with no SDO write anywhere and every existing check
passed. Bit 6 of `403Eh` is on the forbidden list precisely because a single
stray frame can do this; mapping it into a PDO reaches it by a route the list
did not cover.

So the ranges are admitted per-range, on their own terms:

| range | rule | why |
|---|---|---|
| `1600h`–`17FFh` RPDO mapping | permitted **only if the mapped object is itself writable** — the deny-list is applied recursively to `value >> 16` | it is a write path; see above |
| `1A00h`–`1BFFh` TPDO mapping | permitted for any object | it is a **read** path — the device transmits. The posture is read-mostly, not read-nothing; refusing these would forbid reading a temperature by PDO while permitting it by SDO |
| `1400h`/`1800h` comm params | permitted | COB-ID and transmission type decide *where* and *when* a PDO goes, never *what* it carries |

Sub 0 of a mapping object is the entry count (0–8), not an object reference, so
`check()` now takes the subindex — a mapping entry cannot be judged without it,
and guessing would refuse the `count = 0` write that begins every remap.

**Carry this into the ROS drive node.** The recursive-mapping rule is not a
quirk of the current process; it is a property of CANopen, and `ros2_canopen`
supplies no equivalent. It is one more entry on D-8's list of things that
survive the migration as a library.

---

## 3. What survives unchanged

Worth stating so none of it gets rewritten:

- **D1–D4, D8** (map-once → localise, AMCL, slam_toolbox, single nanoScan3, MQTT
  deferred) — all correct for us. The operator confirms Hikrobot ships nanoScan3
  in exactly this dual role at scale.
- **§5 TF tree, §6.1 commissioning workflow, §6.2 static/dynamic landmark
  strategy, §6.3 AMCL clutter tuning, §7 zone/mask data model.**
- **§8 the sim layer.** `fake_base_node` / `scan_synth_node` / `fake_imu_node`
  and the six `launch_testing` cases are the strongest part of the spec and map
  onto our hardware with only `fake_qr_node` → `fake_rfid_node` changed. This is
  also the natural successor to the existing hardware-free suite.
- **§12 phased bringup.** P1's "do not skip odometry validation" is *more*
  important for us, not less, given D-2's motor-side encoders.
- **CANable 2.0** — the plan lists a CAN interface as a required purchase; it is
  installed and working.

---

## 4. Revised task plan

Deltas against spec §10 only:

| Task | Change |
|---|---|
| T2 | Reuse `core/kinematics.py`; derive `alpha_max` from `6083h` per D-1. |
| T3 | `fake_imu_node` noise model uses the **measured** figures in D-3, not generic values. |
| T7 | **Rewrite.** QR/solvePnP/precision-docking → `station_id_node` + wide-covariance `/initialpose` seeding. Docking servo is out of scope until a camera exists. |
| T9 | **Halves.** No `wheel_encoder_node`; one drive node reading `6064h`/`606Ch`. Port from `canworker.py` rather than writing fresh. Delete `KincoScaler`. |
| T10 | **Halves.** No IMU serial driver (D-3), no camera calibration (D-4). Add: MLS IMU TPDO configuration. |
| — | **New T12:** Modbus DIO node from `drivers/dio.py` (D-5). |
| — | **New T0:** CAN bitrate migration to 1 Mbps (§5). Blocks T9. |
| — | **New T0b:** PDO configuration through the write deny-list (D-9). **Done 2026-09-06** — `guard.py` admits the CiA 301 PDO ranges per-range, with RPDO mapping entries validated recursively. Blocks T9 and the MLS IMU TPDO work in T10. |

---

## 5. CAN bus: revised budget, and a migration hazard

**Both devices support 1 Mbps** — confirmed in the manuals. Revised budget with
our actual device set:

| Traffic | Rate | Frames/s |
|---|---|---|
| 2× drive RPDO (ctrlword + `60FFh`) | 50 Hz | 100 |
| 2× drive TPDO (`6041h` + `606Ch`/`6064h`) | 100 Hz | 200 |
| MLS IMU TPDO (yaw rate) | 100 Hz | 100 |
| SYNC / heartbeat / EMCY headroom | — | ~120 |
| **Total** | | **~520 f/s ≈ 7 % @ 1 Mbps** |

Lower than the plan's 620 f/s, because D-2 removes the two encoder nodes. Bus
length at 1 Mbps is capped at **25 m** — irrelevant on this vehicle.

**MLS TPDO constraint the plan does not know:** the sensor allows **at most 4
TPDOs active simultaneously**, and valid COB-IDs are only `0x180 / 0x280 / 0x380
/ 0x480` + node ID. Currently TPDO1 (track data) holds `0x18A`. Retiring
tape-following frees that slot. Probed state:

```
1803h TPDO4 Euler        COB-ID 0x48A DISABLED   (raw 0x8000048A — just the MSB)
1804h TPDO5 quaternion   COB-ID 0x000 DISABLED
1805h TPDO6 acceleration COB-ID 0x000 DISABLED
1806h TPDO7 yaw rate     COB-ID 0x000 DISABLED
```

Enabling Euler is **clearing one bit**. Yaw rate needs a COB-ID assigned first.

### The migration hazard

The three devices change bitrate by **three different mechanisms**, and a
half-completed migration leaves a bus where nothing talks to anything:

| Device | Current | Mechanism |
|---|---|---|
| BLV-R × 2 | 125 kbps (default is 500) | **MEXE02 support software only** — no DIP switch |
| SICK MLS | 125 kbps (its default) | **LSS (CiA 305)** or SICK's configurator; requires restart |
| PC (`can0`) | 125 kbps | `ip link` / profile |

Practical order: change the **drives last**, because MEXE02 talks over the same
bus. Do the MLS by LSS first (it can be scripted against CiA 305 from our own
code, which is worth having as a repeatable tool), verify it alone at 1 Mbps with
the drives powered down, then move the drives, then the PC.

Budget a bench session with the vehicle jacked up. This is `T0` and it blocks
everything on CAN.

---

## 6. Open items to verify before relying on this

- **`6064h` scaling.** The manual says "user-defined position units (step)". The
  steps-per-revolution and any active feed constant must be read from the drive,
  not assumed, before odometry is trusted.
- **`sick_safetyscanners2` on Humble against nanoScan3 specifically** — the driver
  covers the safetyscanners family, but confirm nanoScan3 measurement-data output
  is supported on the firmware in the unit, and that OSSD/field status come
  through as the plan's §4.1 assumes.
- **Backlash magnitude.** Measure it (command a reversal, compare commanded shaft
  motion to measured yaw from the IMU). It sizes the D-2 risk and decides whether
  external encoders come back into the BOM.
- **MLS behaviour with tape-following disabled.** The IMU objects are independent
  of the track objects, but confirm that a sensor seeing no tape does not
  suppress TPDO transmission or set a persistent event flag that masks a real one.
- **N97 thermals under sustained nav2 load.** A 4-core N-series in a fanless
  enclosure will throttle before it saturates; measure with `turbostat` during a
  soak, not by core count alone.

---

## 7. Summary

The generic plan is sound and most of it stands. The reconciliation is:

**Cheaper than planned:** no Kinco scaling layer, no external encoder node, no
IMU serial driver, no camera or illumination BOM, no camera calibration
procedure, ~200 f/s less bus traffic, and a working CAN adapter plus measured
vehicle geometry already in hand.

**More expensive than planned:** motor-side encoders lose gearbox observability
(D-2), compute is halved so survey mapping must move offline (D-6), and the
bitrate migration is a coordinated three-device operation with a real
bricking-shaped failure mode (§5).

**The one capability that is simply gone:** precision docking. RFID returns
identity, not pose, so ±5–10 mm at stations is not reachable without adding a
camera. Decide whether that matters before T7, not during it.

**The one recommendation this document reverses:** `potential-ros-migration.md`
§7's bridge. With tape following retired there is nothing left below the seam but
libraries, so the right move is a single ROS workspace that imports them —
keeping the strict profile loader and the bus write deny-list, which ROS supplies
no equivalent for.
