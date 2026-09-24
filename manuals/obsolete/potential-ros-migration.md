# Potential ROS 2 migration — assessment

**Assessed:** 2026-09-03 · branch `refactor02` @ `62f9f50`
**Question:** should gvievo-01 move onto ROS 2?
**Answer:** **No — not at the present scope.** Adopt at a boundary if and when
free navigation arrives. Nothing else on the current roadmap justifies it.

---

## 1. Summary

ROS 2's value is **ecosystem reuse** — Nav2, tf2, ros2_control, rosbag2, rviz,
and third-party sensor drivers. This vehicle consumes almost none of that
surface, because its path is a magnetic tape and its position is an RFID tag
identity. What a port would cost is not code volume; it is the four properties
this codebase spent its design effort on — a validating profile loader, a single
owner of the bus, a hardware-free test suite, and bench scripts that run
standalone. ROS 2 hosts none of the four and actively erodes two.

The decision is **not close today** and **not permanent**. Section 6 states the
trigger that flips it and section 8 lists the cheap moves that keep the option
open in the meantime.

---

## 2. What this vehicle actually is

| | |
|---|---|
| Guidance | SICK MLS magnetic tape, lateral error in mm, TPDO1 `0x18A` at ~50 Hz |
| Localisation | none — position is an RFID station tag identity |
| Path | the tape. Branch choice by PLC-style seal-in latch, [`core/branch.py`](../core/branch.py) |
| Actuation | two Oriental Motor BLV-R, CiA 402 Profile Velocity, nodes 1 and 2 |
| Bus | one `can0` at **125 kbps**, shared with the sensor |
| Control | 50 Hz tick, **20 ms budget**, measured 20.4 ms avg / 29–31 ms max |
| I/O | Modbus TCP 16-DI/16-DO and a Chafon CF821 reader, each on its own thread |
| UI | Flask, two operator pages plus `/monitor` and `/io` |
| Size | ~7 k lines Python + JS, one process, one operator, one vehicle profile |
| Tests | 341 checks, **no hardware, no daemon** |
| Safety | lidar/encoders → FX3 → HWTO1/HWTO2 → STO, in hardware, EDM returned |

The last line is the one that governs everything else: **no software in this
repo is in the safety path**, and the CAN monitoring layer is documented as
diagnostic only.

---

## 3. What ROS 2 supplies, against what is used

| ROS 2 offers | Used here? |
|---|---|
| Nav2 — planners, controllers, costmaps, recoveries | **No.** There is no map and no planner. The path is physical. |
| tf2 transform tree | **No.** One sensor, one fixed offset `Ls`, a scalar in [`core/autopilot.py`](../core/autopilot.py). |
| SLAM / AMCL | **No.** No odometry-based localisation anywhere. |
| ros2_control + hardware interfaces | **Marginal.** [`canworker.py`](../canworker.py) already speaks CiA 402 with per-mode ramp writes. |
| `ros2_canopen` / `cia402_driver` | **Marginal, and the weakest fit.** It supplies less than what exists and none of the deny-list in [`drivers/canbus/guard.py`](../drivers/canbus/guard.py). |
| Driver ecosystem | **No.** SICK MLS, BLV-R, Chafon CF821 and the Modbus island all had to be written here regardless. |
| Node isolation / multi-process supervision | **No.** One process, one operator. Isolation would buy fault containment nobody has asked for. |
| rosbag2 + rviz + PlotJuggler | **Partially replaced.** Per-run CSV+PNG in [`core/runlog.py`](../core/runlog.py) and the `/monitor` page cover the questions actually asked. |
| Fleet / multi-robot | **Not by ROS.** Fleet managers speak VDA5050 over MQTT — see §6. |

Roughly: of the reasons teams adopt ROS 2, exactly one — Nav2 — would ever
apply to this vehicle, and only under a change of guidance principle.

---

## 4. What a migration would cost

### 4.1 The strict profile loader has no ROS equivalent

[`config.py`](../config.py) makes **unknown and missing keys both fatal**, stores
only primitives, validates into a fresh namespace and publishes only on success,
and enforces cross-field constraints no single module could check —
`autopilot.ramp_accel_rpm_s` must stay below `drivers.ramp.auto.accel`, and
`timing.driver_timeout_s` must exceed `telemetry_period_s`.

ROS 2 parameters give none of that. Parameters present in a YAML file that a
node never declares are **silently ignored**, which is precisely the failure the
loader exists to prevent: a typo'd gain leaves the vehicle running on a number
nobody chose. `on_set_parameters_callback` can validate a set, but there is no
"reject the whole profile atomically" semantic and no unknown-key rejection.

The realistic outcome is that `config.py` survives the migration verbatim,
wrapped in a node — at which point ROS has added a parameter system that is not
being used.

### 4.2 "One thread owns the bus" gets harder, not easier

The core invariant — an SDO transfer is a send/recv **pair** that must not
interleave, and `sdo_read()` drains RX before transmitting — is currently
guaranteed structurally: one thread, a queue, and `Future`s for slow actions.

ROS 2 does not hand this over; it makes it easier to lose. Callbacks dispatch
concurrently under the default executor, so restoring the guarantee means a
single-threaded executor plus mutually-exclusive callback groups — the same
property, now enforced by convention that a later contributor can break silently.
The source scan in `tests/test_layout.py` that fails the build on any `bus.` call
inside a `with self._lock:` block no longer maps onto the resulting shape.

Note also the reason that scan exists: this bug **shipped once** and presented as
409s on arm *and* disarm with `/api/state` hanging. It is not hypothetical.

### 4.3 The 341 offline checks become integration tests

This is the single largest regression in the trade, and the one most likely to be
underestimated.

Today the suite runs with no bus, no hardware, no daemon, and includes a plant
simulation integrating `e_dot = v*theta + Ls*omega` against the real
`LineFollower` — with a negative control asserting the rig can actually see
instability. The ROS-native equivalent is `launch_test`: processes started per
case, DDS discovery in the loop, wall-clock waits, and a flake rate that grows
with the node count. Fast, deterministic, hardware-free control tests are worth
more to this project than anything ROS would supply in exchange.

### 4.4 The bench scripts stop being standalone

[`drivers/canbus/`](../drivers/canbus/) is deliberately *not* a package: its modules
import each other by bare name so they run on a laptop against a live bus during
commissioning. Inside a ROS package that becomes "source the workspace, then run
a node" — and the commissioning workflow that verified the wheel sign convention
and the `invert_error` finding on 2026-08-31 is exactly the workflow that gets
more expensive.

### 4.5 The branch ladder loses its reviewability

[`core/branch.py`](../core/branch.py) is transcribed rung for rung from a drawn
ladder, deliberately, so the code can be **diffed against the drawing** rather
than merely computing the same answer — including the scan-order semantics that
decide a simultaneous set-left/set-right. Re-expressing that as a lifecycle node
with topics keeps the behaviour and discards the property that made it
reviewable by the person who drew it.

### 4.6 Timing: DDS does not buy determinism

The tick already runs at 20.4 ms average against a 20 ms budget with peaks near
31 ms, and `60FFh` is still a **blocking SDO write per tick**. ROS 2 on a stock
kernel adds discovery traffic, serialisation, and an executor between the timer
and the work; it does not add real-time guarantees. The named RPDO1 migration
(−1.8 ms × 2 nodes) is a larger and cheaper win than any framework change, and
it is available today.

---

## 5. Safety and compliance: neutral, with one caveat

**Neutral**, because the safety chain is hardware — lidar/encoders → FX3 → STO
with EDM returned — and nothing in this repo is rated, redundant or certified.
ROS 2 is not safety-certified either, and that is irrelevant when the software is
outside the safety boundary in both cases.

**The caveat:** the ISO 3691-4 / EN 1175 work in
[`manuals/motor-drive-compliance-assessment.md`](motor-drive-compliance-assessment.md)
notes that under Track B the vehicle boundary widens. Migrating pulls several
hundred thousand lines of third-party middleware inside the vehicle, all of it
subject to whatever software-lifecycle argument the technical file eventually
needs. That is not a blocker; it is a cost to book against the migration rather
than discover during an assessment.

---

## 6. What would flip the decision

**Primary trigger — free navigation.** If the tape comes off and guidance becomes
lidar SLAM plus a planner, Nav2 is worth more than anything that would be written
here, and the guidance layer is a rewrite in any framework. This is the one case
where the answer becomes an unambiguous yes.

**Secondary triggers, weaker:**

- **Hiring.** ROS is a shared vocabulary; a bespoke stack is onboarding cost. Real,
  but does not outweigh §4 for a one- or two-engineer team.
- **A payload sensor that ships a ROS driver and nothing else.** Weigh the driver
  against a bridge process; usually the bridge wins.

**Not a trigger — fleet coordination.** Fleet managers speak **VDA5050 over
MQTT**, a JSON message spec. Implementing it against the existing state model is
a few hundred lines and needs no ROS. Adopting ROS *for* fleet integration would
be solving the problem in the wrong layer.

**Not a trigger — a second vehicle.** That is already a new JSON file in
`profiles/`, by design.

---

## 7. If it flips: the migration shape

Do **not** port the vehicle. Partition it, keeping the deterministic layer as it
is and putting ROS above the boundary:

```
  ROS 2 domain                        existing process
  ─────────────────────────────       ─────────────────────────────
  Nav2 / SLAM / costmaps
  pose controller  ──(v, ω)──►   HTTP/JSON or a thin socket
                                       │
                                       ├─ canworker.py    bus owner, CiA 402
                                       ├─ guard.py        write deny-list
                                       ├─ health.py       two-tier watchdogs
                                       ├─ config.py       strict profile
                                       └─ drivers/        MLS, RFID, DIO
```

Two facts make this the natural seam rather than a compromise:

1. **[`core/kinematics.py`](../core/kinematics.py) is already written for it.** Its
   docstring states the separation explicitly — a SLAM pose controller producing
   (v, ω) converts through the same `body_to_wheels()` the line follower uses. The
   control law is what is line-specific; the kinematics are not.
2. **You would want this partition even in a fully ROS system.** CiA 402 arm/disarm
   sequencing, the write deny-list, and the hardware watchdog table do not belong
   inside a DDS callback under any architecture.

Under that shape the migration is additive: `core/autopilot.py` is bypassed in
nav mode, everything below the seam is untouched, and the 341 checks keep
running.

**Rough effort.** The boundary approach is weeks. A full port is a rewrite of
everything except `core/` and the manuals — months of one engineer, with an
extended period during which neither codebase is the trustworthy one. That
interval, not the total, is the real risk on a vehicle that currently works.

---

## 8. Moves now that keep the option open

None of these are ROS work; all are worth doing on their own merits.

- **Keep [`core/kinematics.py`](../core/kinematics.py) free of control logic and
  sensor knowledge.** It is the documented seam. A shortcut through it is the one
  change that would make a later ROS boundary expensive.
- **Keep the `/api/*` surface in [`app/server.py`](../app/server.py) stable and
  complete.** It is the bridge interface, whether the client is a browser, a
  VDA5050 adapter or a ROS node.
- **Keep `drivers/canbus/` free of `config` imports.** Already an invariant;
  it is also what lets the bus layer be reused unchanged under any supervisor.
- **When the station/route layer is built, keep the route model as data**
  (profile rows, as `branch_latch` already is), not as code. A data route model
  ports; a coded one does not.

---

## 9. What is worth more than a framework change

Ranked, from the existing Known Gaps:

1. **RPDO1 migration for `60FFh`** — removes ~1.8 ms × 2 from a 20 ms budget that
   is currently peaking at 31 ms. The only change here that improves determinism.
2. **Drive a real curve.** The ~0.8 m tightest-radius figure is simulation. The
   binding constraint is the entry transient against the ±100 mm window, which is
   exactly the kind of number that moves on real hardware.
3. **Curvature feedforward**, if tighter radii are required — structurally correct,
   drives `e_ss` to zero at any gain, and is a code change rather than a profile edit.
4. **Fix the `rfid.*` docstring block in `config.py`.** A strict loader whose own
   documentation is the loose part is a trap for the next person.

Each of these makes the vehicle measurably better. A migration, at today's scope,
makes it different.

---

## 10. Verify before relying

- **`ros2_canopen` maturity** — the assessment above treats `cia402_driver` as
  weaker than the hand-rolled path. Confirm against the current release before
  quoting it in a decision; this moves.
- **ROS 2 parameter semantics** — the "undeclared parameters in YAML are silently
  ignored" behaviour is the default and is the crux of §4.1. Re-check on the
  distribution actually chosen.
- **DDS transport choice** — if a boundary bridge is ever built, measure the
  added jitter on the real PC rather than assuming it is negligible.
- The `can-monitoring-plan.txt` open item about SDO abort `0800 0024h` from
  `1003h` sub 1 applies to *any* stack, ROS included, and is still untested.
