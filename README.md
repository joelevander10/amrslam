# gvievo-01 — AMR base platform

CANopen control software for a 150 kg differential-drive vehicle. Two Oriental
Motor BLV-R drivers in CiA 402 Profile Velocity mode share one `can0` with a
SICK MLS sensor; a SICK nanoScan3 safety scanner, a Chafon RFID reader and a
Modbus I/O island sit on Ethernet. A Flask web UI provides a manual jog pad and
diagnostic pages.

> ### Status: this is the base platform, not a navigating vehicle.
> **Magnetic-tape line following has been retired.** `core/autopilot.py` (the
> PID), `core/branch.py` (the junction ladder), the per-run logging, the `/auto`
> page and every tape parameter are deleted. **The vehicle cannot drive itself
> today** — it jogs from the manual pad, and that is all.
>
> What remains is everything a SLAM stack needs underneath it: a strict profile
> loader, single-owner bus access with a CiA 402 arm sequence, a CAN write
> deny-list, differential-drive kinematics, a two-tier hardware-health table,
> and drivers for every device on the vehicle. See
> [manuals/slam-generalized-plan/](manuals/slam-generalized-plan/) — read
> `hardware-reconciliation.md` first; it wins over the other two wherever they
> disagree.
>
> **The nanoScan3 is owned by the ROS stack.** The legacy UDP listener
> (`drivers/lidar.py`), the `/lidar` page and the zone rail were removed on
> 2026-09-16 so `sick_safetyscanners2` can hold `192.168.3.2:6060`. The
> scanner's liveness is now read from `/amr/localization_state` (`scan_age_s`)
> on the ROS side; this app shows nothing about it.

| | |
|---|---|
| Jog speed | 1200 r/min = **0.377 m/s** (ceiling 4000 r/min = 1.257 m/s) |
| Bus loop | 50 Hz, telemetry 5 Hz, drive monitoring 10 Hz |
| Nodes | 1 left driver, 2 right driver, 10 SICK MLS (IMU only) |
| CAN | 125 kbps — **migration to 1 Mbps is task T0** |
| `use_rpdo` | **false** — the setpoint is still a blocking SDO write |
| Tests | 784 offline checks, all passing |

---

## ⚠️ Safety model

**This software moves a 150 kg vehicle and has no authentication.** Anyone who
can reach port 5000 can drive it. Keep it on a trusted network.

**The web app cannot start the vehicle.** There is no `/api/arm`. The only route
here that produces motion is `/api/drive`, and only while the vehicle is already
armed in MANUAL — which only the physical panel can bring about. Everything else
only ever stops.

**MANUAL is an armed state.** With `panel.manual_auto_arm`, the selector resting
in MANUAL *is* the arm command: the drives energise with nobody pressing
anything, and re-energise on their own once the safety chain is restored.
Arming is not motion — an armed vehicle is excited and holding zero.

**A manual direction is HELD, not latched.** The browser re-POSTs `/api/drive`
every ~100 ms while a button or key is down. Miss enough in a row
(`manual_watchdog_s` = 0.6 s) and the bus thread zeros the setpoint. A closed
tab, a dropped Wi-Fi link and a released button are indistinguishable to the
vehicle, which is the point.

**Arming excites the motor; it does not free it.** CiA 402 "Operation enabled"
excites the motor and the velocity loop then holds zero — a servo lock, not a
free shaft; a non-excited brake motor has the brake clamped instead. Only the
`FREE` input releases it, and `FREE` is on the write deny-list. A disarmed
vehicle cannot be pushed either.

**The only output this software energises is the horn-and-lights coil**
(`horn.do_channel`, DO00). It follows the *commanded* setpoint — high whenever
the vehicle is armed and asked to move, manual or auto alike — and it is a
claim the bus tick must renew every 20 ms: stop renewing and the DIO scan drops
it within `HORN_HOLD_S`. Nothing gates on it; a horn that failed cannot prevent
a jog.

**Nothing in this repo is in the safety path.** The chain is lidar/encoders →
FX3 → HWTO1/HWTO2 → STO, wired in hardware with EDM returned to the FX3. Nothing
read over CAN or Ethernet is rated, redundant or certified, and nothing here may
decide whether it is safe to move.

---

## Running

```bash
python3 main.py                         # 0.0.0.0:5000
python3 main.py --port 5001
AGV_PROFILE=agv-02 python3 main.py      # a different vehicle
```

**Which vehicle** comes from the `AGV_PROFILE` environment variable, naming a
file in `profiles/`. Unset means `agv-01`. There is no CLI flag and no fallback:
a name with no matching file is fatal at boot and the error lists the profiles
that do exist.

In production this runs as the `agv_controller` systemd unit, which sends
SIGINT rather than SIGTERM so the `KeyboardInterrupt` path de-energises the
motors on the way out.

```ini
ExecStart=/usr/bin/python3 /home/gvipc-evo-01/agv_can/main.py
Environment=AGV_PROFILE=agv-01
```

`main.py` is thin and sits at the repo root precisely so `ExecStart` names a path
that will not move again when the tree below it is reorganised.

`--debug` enables Flask autoreload and is **off by default**: a reload would
open `can0` twice and orphan an armed driver.

### Dependencies

Standard library, plus `flask` and `python-can`.

---

## Layout

```
main.py            Entry point. Thin: sets up sys.path, calls the server.
config.py          Loads and validates the profile. Everything imports this.
canworker.py       Bus thread: NMT, SDO, 50 Hz loop, arm/disarm, telemetry.

core/              No hardware, no Flask. Computation and record-keeping.
  kinematics.py      body <-> wheels. No control logic, no sensor knowledge.
  motion.py          Manual jog pad table, labels, key bindings.
  health.py          Hardware liveness: the two-tier watchdog table.
  panel.py           Operator panel: debounced edges over the DI image.
  lidarframe.py      nanoScan3 telegram decode, for the bench tool only.
  canmon.py          Drive monitoring: the round-robin SDO object table.
  events.py          Operator event ring buffer (200). Survives a reload.

drivers/           Everything that talks to a device.
  rfid.py            Chafon CF821 station-tag reader. Own thread and socket.
  dio.py             16-in/16-out digital I/O over Modbus TCP. Own thread.
  lidar_scan.py      nanoScan3 bench tool (scan/watch/raw/capture). Hand-run
                     only; needs the ROS driver stopped to bind the port.
  canbus/            CAN layer, and standalone bench tools:
    guard.py           the write deny-list, incl. PDO mapping validation
    rpdo.py            RPDO1 setpoint frames (behind can.use_rpdo)
    read_imu.py        the IMU inside the MLS — read, bias, TPDO enable
    lss.py             CiA 305 bitrate migration
    verify_bus.py      bus discovery and collision checks
    verify_drivers.py  SDO helpers, adapter discovery
    drive_forward.py   CiA 402 words and SDO download
    bus_health.py      statusword -> state name
    alarms.py          EMCY / NMT / statusword-flag tables

app/               The web tier.
  server.py          Flask routes.
  templates/         base.html is the shared shell.
  static/            app.css and the per-page scripts.

tests/             784 offline checks. run_all.py runs them. No hardware.
profiles/          One JSON per vehicle. Every tunable parameter lives here.
manuals/           Driver, sensor and RFID documentation, plus the SLAM plan.
```

**Layers say what a module may touch.** `core/` reaches neither the bus nor the
browser, `drivers/` owns every device conversation, `app/` only serves.

**The layer directories go on `sys.path`; they are not packages.** Every module
imports its neighbours by bare name, which is what lets `drivers/canbus/` still
run standalone on a bench. The cost is that module *basenames* are one flat
namespace — two modules may never share a name, and none may shadow a stdlib
module. `tests/test_layout.py` enforces both.

**Dependency graph:** `config` imports only the standard library; everything
else imports `config`. No cycles. Nothing in `drivers/canbus/` imports `config`.

---

## How it works

### One thread owns the bus

Every CAN frame goes through `canworker.py`. Flask request handlers never touch
the bus — they either mutate a setpoint under a lock (cheap, non-blocking) or
push a slow action onto a queue and wait on a `Future`.

This is not stylistic. An SDO transfer is a send/recv **pair** that must not
interleave with another, and `sdo_read()` drains the RX queue before
transmitting — two threads doing SDO at once read each other's replies.

`TpdoTap` wraps the bus and routes every **unsolicited** frame — EMCY alarms and
drive heartbeats — before the SDO helpers can discard them. Both helpers open by
draining the RX queue and then keep only frames matching `0x580+node`, so any
frame class not routed by the tap is silently lost for as long as a transfer is
in flight, which is most of the time.

### The control tick, and where its time goes

Measured over the last tape-following runs (`logs/0023`–`0025`, 6010 ticks):

| percentile | achieved period |
|---|---|
| p50 | 20.1 ms |
| p95 | **28.1 ms** |
| p99 | **32.9 ms** |
| max | **37.9 ms** |

The median is fine — `_pump()` fills the remainder of the period, so a tick whose
work fits paces to 20 ms. **The problem is the tail**, and it has one cause:
blocking SDO polls arrive in *bursts*, not as steady load.

| what | rate | round trips | cost when it lands |
|---|---|---|---|
| setpoint `60FFh` | most ticks | 2 | ~3.6 ms, nearly every tick |
| `_poll_telemetry` | 5 Hz | 6 | **~10.8 ms, one tick in ten** |
| `_poll_monitor` | 10 Hz | 2 | ~3.6 ms, one tick in five |
| `_poll_imu` | 25 Hz | 1 | ~4 ms, one tick in two — display only, one object per poll |

A round trip is ~1.8 ms at 125 kbps. Two fixes are staged and both are unblocked
by the deny-list change below:

- **RPDO1 for the setpoint** — built, behind `can.use_rpdo`, shipping `false`.
  Removes the ~3.6 ms baseline.
- **TPDO for telemetry** — *not built*. Removes the ~10.8 ms spike, which is
  where the pain actually is. `TpdoTap` is already shaped for it.

Both, plus the 1 Mbps migration, are written up in
[manuals/codebase-improvement.md](manuals/codebase-improvement.md).

### What the vehicle may write over CAN

CANopen is bidirectional, and several objects can defeat safety behaviour from a
single stray frame. [drivers/canbus/guard.py](drivers/canbus/guard.py) enforces a
deny-list on every write path, and `/monitor` publishes it:

| | |
|---|---|
| **permitted** | `6040h` controlword, `6060h` modes, `6083h`/`6084h` ramps, `60FFh` target velocity, `1017h` heartbeat |
| **refused** | `403Eh` (bit 6 is **FREE** — releases the holding brake on *both* drive wheels), `40D0h` clear ETO, `40C0h` alarm reset, `6040h` bit 7 fault reset, `1010h`/`1011h` store/restore, `40C6h` and all `4xxxh` parameters |

A denied write raises rather than being silently dropped: a command a caller
believed had landed is its own hazard. Clearing ETO or resetting an alarm
automatically would be an automatic restart, which ISO 3691-4 prohibits.

**PDO configuration is admitted per-range, not wholesale.** An RPDO mapping is a
write path by another name — map `403Eh` into one and a two-byte CAN frame
releases both wheel brakes with every check above passed. So:

| range | rule |
|---|---|
| `1600h`–`17FFh` RPDO mapping | permitted **only if the mapped object is itself writable** — the deny-list is applied recursively |
| `1A00h`–`1BFFh` TPDO mapping | permitted for any object: a TPDO is a **read** path |
| `1400h`/`1800h` comm params | permitted: they decide where and when, never what |

### Hardware health

[core/health.py](core/health.py) holds one table, one row per supervised device,
and each row declares its own tier:

| device | fed by | timeout | losing it stops |
|---|---|---|---|
| `driver:1`, `driver:2` | heartbeat, or a telemetry read that answered | `driver_timeout_s` 0.6 s | **everything** |
| `rfid`, `dio` | each link's own `snapshot()` | per-device | nothing today |

Losing a driver means the wheels can be neither commanded nor observed, so every
mode stops and an arm is refused. A source that has **never** answered is not a
fault — only a device that answered once and then stopped counts as lost.

This is deliberately **not** the browser watchdog: that covers an absent
*operator*, this covers an absent *device*.

### Drive monitoring

**Diagnostic only — not a safety path.** `/monitor` shows both drives side by
side, so a left/right divergence is visible at a glance. Three channels feed it:
EMCY pushed on `0x080+n`, heartbeat pushed on `0x700+n`, and analogue values
polled by SDO one object per node per tick, round-robin.

Data types come from the manual, not the monitoring plan: the plan guesses
`40A4h` and `40A3h` as INT32 when both are **INT16**, and a 16-bit signed value
read as 32-bit is plausible-looking garbage rather than an error.

---

## Configuration

Everything tunable lives in `profiles/<name>.json`, selected by `AGV_PROFILE`.
**`config.py`'s docstring is the manual** — it carries a TUNING NOTES section
explaining every non-obvious setting, because JSON cannot hold comments.

The loader is deliberately strict, and this is the main thing the JSON buys over
Python constants:

1. **Unknown and missing keys are both fatal.** A typo'd key would otherwise
   leave a value at whatever the code last defaulted to.
2. **Only primitives are stored.** Anything derivable is derived, so the file
   cannot hold a geometry that contradicts itself.
3. **Validation runs before anything is published.** `load()` parses and
   validates into a fresh namespace and publishes only on success.
4. **`profile_name` must match the filename.**

Derived from the profile, never stored:

```
MPS_PER_RPM    3.1416e-4     RAD_S_PER_RPM_DIFF   6.4642e-4
RPM_PER_MPS    3183.1        MAX_SPEED_MPS        1.2566
MANUAL_HALF_RPM  720         ACCEL/DECEL_RPM_S    from drivers.ramp.auto
HORN_HOLD_S      0.1         max(5·loop_period_s, 2·dio.scan_period_s)
```

**`6083h` is the ceiling on yaw acceleration, not just on forward ramp.** It
caps how fast the wheel *difference* can slew;
`kinematics.max_yaw_accel()` turns it into rad/s² and gives 2.59 at the present
2400. Anything producing (v, ω) must derive its limit from there rather than
carry a hardcoded one, or a later ramp change silently commands a yaw rate the
drives cannot produce.

---

## Testing

```bash
python3 tests/run_all.py      # 784 checks, no hardware
python3 -c "import main"      # exits 1 with a named check on a bad profile
```

`run_all.py` **pins both the module list and the total check count**. A module
that stops being imported costs coverage while the run still ends in "all checks
passed", so a dropped module fails the run instead. Raise `EXPECTED_CHECKS`
deliberately when adding checks; never lower it to get a green run.

It also contains **source scans** that fail the build on structural regressions:
any `self._read` / `_write` / `_nmt` / `bus.` call inside a `with self._lock:`
block, a module whose filesystem anchor followed it into a new directory, and a
basename collision on the flat `sys.path`. Keep them.

> **The plant simulation is gone.** It integrated `e_dot = v*theta + Ls*omega`
> against the real `LineFollower`, with a negative control asserting the rig
> could actually *see* instability. That last property is the one worth
> rebuilding when a navigation controller arrives: a simulation that cannot fail
> is not evidence, and the `6083h` slew limit it modelled is a property of these
> drives, not of the line follower.

After a hardware change: `/manual` → the selector in MANUAL arms → jog each
direction on blocks and confirm the wheels turn the correct way.

---

## Invariants

**Never do bus I/O while holding `Controller._lock`.** `sdo_read()` drains RX
through `TpdoTap`, which calls a handler, which takes the same lock. This
shipped once and presented as 409s on arm *and* disarm with `/api/state`
hanging. The lock is an `RLock` as a backstop; keeping bus I/O out of locked
sections is the actual fix, and the source scan enforces it.

**`drivers/canbus/` is a runtime dependency, not a scratch directory.**
`canworker` imports from it. Do **not** make those modules import `config` —
they must stay runnable standalone on a bench; where they need a vehicle-specific
value, take it as a parameter defaulting to the module constant.

It is also deliberately **not** a package. Its modules import each other by bare
name so they run standalone, and adding `__init__.py` would resolve them twice.

**Wheel sign convention:** both drivers take a **positive** `60FFh` to travel
forward. If you swap a motor, re-flash a driver or remount a wheel, re-verify on
blocks and set `vehicle.invert_left` / `invert_right` rather than editing
`motion.py`'s table — the table stays in vehicle terms.

**Only operator-relevant transitions may emit events.** At 50 Hz a single chatty
call site flushes the entire 200-entry ring within four seconds. Nothing on a
per-tick path emits, and anything edge-triggered fires once per **edge**.

**The panel owns every entry into motion.** Keep it that way when navigation
lands: the temptation will be a "go" button on a page, and the panel owning this
is what makes a browser incapable of starting the vehicle.

---

## Known gaps

- **No autonomous mode exists.** The panel's Start button is a stub that says
  so. `AUTO` on the selector is a disarmed resting state.
- **`can.use_rpdo` has never run on hardware.** Neither has the IMU TPDO enable
  or the LSS bitrate change. All three have bench checklists in
  [manuals/slam-generalized-plan/bench-checklists.md](manuals/slam-generalized-plan/bench-checklists.md).
- **`6064h` scaling is unverified.** The manual says "user-defined position
  units (step)"; steps-per-rev must be read from the drive before odometry is
  trusted.
- **Backlash is unmeasured.** The encoders are motor-side of a 30:1 gearhead, so
  backlash is structurally unobservable from them. The gyro makes it measurable.
- **RFID wire protocol is inferred, not captured.** The framing is lifted from
  the same reader hardware on the KIM2A vehicle.
- **`config.py`'s `rfid.*` docstring is out of date.** It documents keys that are
  not in `_SCHEMA` (`inventory_cmd`, `epc_offset`, `handshake_hex`,
  `poll_period_s`, `comms_timeout_s`) and says `enabled` ships false while the
  profile has it true.
- MEXE02 load inertia is still `0: Small (2×)`; should be `1: Medium (7.5×)`.
