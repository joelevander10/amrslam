# Improvements worth backporting to the line-following AGV

**Written:** 2026-09-06 · branch `refactor02`
**For:** the coding agent working on the **line-following** vehicle, with real
hardware.
**From:** work done while preparing the SLAM migration
([`manuals/slam-generalized-plan/`](slam-generalized-plan/)). That work
retires `core/autopilot.py` and `core/branch.py`. **This document is only the
part that does not** — the things that make the tape-following vehicle better
and have nothing to do with SLAM.

> ### Status of everything below
> **Written and covered by the offline suite. None of it has touched hardware.**
> The suite went 866 → 1007 checks, all passing. Every item is marked with what
> is *tested*, what is *asserted from a manual*, and what is *unverified*.
> [`manuals/slam-generalized-plan/bench-checklists.md`](slam-generalized-plan/bench-checklists.md)
> has the per-item bench procedure. Treat the distinction as load-bearing — this
> moves a 150 kg vehicle.

---

## 0. Take these, in this order

| | item | built? | what it buys | risk |
|---|---|---|---|---|
| **A** | [`guard.py`](../drivers/canbus/guard.py) admits PDO configuration, safely | **yes** | unblocks B and C; closes a hole | **none** — behaviour identical until you use PDOs |
| **B** | **TPDO for telemetry** | **no — recommended** | kills the ~11 ms spike on 7 % of ticks | medium |
| **C** | RPDO1 for `60FFh` | **yes**, behind a flag | ~3.6 ms off *every* tick's work | medium, flag-gated |
| **D** | The MLS IMU — you already own a gyro | **yes** (reader) | turns your simulated curve limits into measurements | low, read-only |
| **E** | CAN 125 k → 1 Mbps + an LSS tool | **yes** (tool) | ~8× cheaper SDO; makes B and D comfortable | **high** — see §6 |

**A is a prerequisite for B and C and costs nothing. Take it regardless.**

---

## 1. The measured problem, from your own logs

Your README says the loop is "healthy" at 20.4 ms average / 29–31 ms max. **That
is no longer true.** From the three most recent runs in `logs/` (6010 ticks,
`0023`/`0024`/`0025`, 2026-09-04):

| percentile | `loop_ms` |
|---|---|
| p50 | 20.1 ms |
| p75 | 20.2 ms |
| p90 | 21.8 ms |
| p95 | **28.1 ms** |
| p99 | **32.9 ms** |
| max | **37.9 ms** |

```
ticks over the 20 ms budget   5421 / 6010  =  90.2 %
ticks over 25 ms                418        =   7.0 %   (~every 14 ticks)
ticks over 30 ms                254        =   4.2 %   (~every 24 ticks)
```

### Read this distribution carefully, because it says something non-obvious

The median is 20.1 ms. **The typical tick is fine** — `_pump()` fills the
remainder of the period, so a tick whose work fits simply paces to 20 ms. The
median being 20.1 rather than 25 means most ticks have headroom.

**The problem is entirely in the tail.** 7 % of ticks overrun by 5–18 ms, and
that tail has a specific, identifiable cause: the blocking SDO polls, which are
*bursts*, not a steady load.

| what | rate | SDO round trips | cost when it lands |
|---|---|---|---|
| setpoint `60FFh` | most ticks | 2 | **~3.6 ms, on nearly every tick** |
| `_poll_telemetry` (`6041h`, `606Ch`, `1001h` × 2 nodes) | 5 Hz | 6 | **~10.8 ms, one tick in ten** |
| `_poll_monitor` (round-robin × 2 nodes) | 10 Hz | 2 | ~3.6 ms, one tick in five |
| `_poll_field` (`2024h`) | 2 Hz | 1 | ~1.8 ms, one tick in 25 |

A round trip is ~1.8 ms at 125 kbps — two ~111-bit frames on a 125 kbit wire.
The 5 Hz telemetry burst lands on 10 % of ticks and costs ~10.8 ms; the observed
"7 % of ticks over 25 ms" is that burst.

**So the ranking is:**

- **B (TPDO telemetry) attacks the tail** — the 10.8 ms burst, which is where
  the pain actually is.
- **C (RPDO1 setpoint) attacks the baseline** — 3.6 ms off every tick, which
  also drags the whole tail down by 3.6 ms and restores headroom.
- Together they take almost all blocking SDO out of the control tick.

### What the overrun actually costs you

Be precise about this, because it is **not** instability. `autopilot.update()`
clamps measured `dt` to `[0.004, 0.1]` s, so a 38 ms tick arrives as
`dt = 0.038` and the PID integrates it correctly. The control law adapts.

What you lose is **update latency**: at `auto_rpm` 1200 (0.377 m/s), a 33 ms
tick instead of 20 ms is 13 ms of travel with no new steering correction — about
**5 mm**. Bounded, and survivable, which is why the vehicle tracks at 1.5–2 mm
RMS anyway. But it is jitter you do not need, it is worst exactly when telemetry
and monitor coincide, and it caps how far you can raise `k_ratio` — because the
yaw-rate slew argument in `autopilot.py` assumes the tick actually arrives.

---

## 2. Improvement A — the write deny-list now admits PDO configuration

**Built and tested. Behaviour is identical to today unless you write a PDO
object.** Take it first; B and C are blocked without it.

### The problem

`guard.py` refused **every** CiA 301 PDO configuration object — `1400h`/`1600h`
(RPDO), `1800h`/`1A00h` (TPDO) — through its "not on the permitted-write list"
branch. Correct by default, but it means neither B nor C can be attempted at
all.

### Why the obvious fix is dangerous

Adding those ranges to `ALLOWED` **opens the exact hole the deny-list exists to
close.**

> **An RPDO mapping is a write path by another name.** Map `403Eh` into an RPDO
> and a two-byte CAN frame releases the holding brake on **both drive wheels**
> — with no SDO write anywhere, and every existing deny-list check passed.
> `403Eh` is on the forbidden list precisely because one stray frame can do
> this. A PDO mapping reaches it by a route the list did not cover.

### What was done instead

Each range is admitted on its own terms:

| range | rule | why |
|---|---|---|
| `1600h`–`17FFh` RPDO mapping | permitted **only if the mapped object is itself writable** — the deny-list is applied recursively to `value >> 16` | it is a write path |
| `1A00h`–`1BFFh` TPDO mapping | permitted for any object | it is a **read** path — the device transmits. The posture is read-*mostly*, not read-nothing; refusing these would forbid reading a temperature by PDO while permitting it by SDO |
| `1400h`/`1800h` comm params | permitted | COB-ID and transmission type decide *where* and *when* a PDO goes, never *what* it carries |

`check()` gained a `sub` argument, because sub 0 of a mapping object is the
entry **count** (0–8), not an object reference — and guessing would refuse the
`count = 0` write that begins every remap.

**Files:** [`drivers/canbus/guard.py`](../drivers/canbus/guard.py),
[`canworker.py`](../canworker.py) (`_write` passes `sub`),
[`tests/test_canmon.py`](../tests/test_canmon.py) (+11 checks),
[`tests/helpers.py`](../tests/helpers.py) (`_why` signature).

**The check that matters:** `*** an RPDO may NOT map 403Eh behind the deny-list ***`

---

## 3. Improvement B — TPDO for telemetry *(recommended, not built)*

**This is the biggest single win available to the line-following vehicle, and I
did not build it.** Flagging it because the measurement in §1 says it is worth
more than the thing I *did* build.

### The case

`_poll_telemetry()` fires six blocking SDO reads back to back at 5 Hz —
statusword, actual velocity and error register for each of two drives. That is
~10.8 ms landing inside a single 20 ms tick, one tick in ten. It is the tail in
§1.

The drives can **push** all of it instead. Spec §3.3 of the SLAM plan already
specifies the mapping, and it applies unchanged to the line-following vehicle:

| PDO | content | bytes | transmission |
|---|---|---|---|
| TPDO1 | `6041h` statusword (u16) + `606Ch` actual velocity (i32) | 6 | event-timer 10 ms |
| TPDO2 | `603Fh` error code (u16) + `6061h` mode display (i8) | 3 | on change |

### Why this fits your architecture unusually well

**`TpdoTap` already does the hard part.** It routes unsolicited frames by
arbitration id before the SDO helpers can bin them, and it already handles
`0x080+n` (EMCY) and `0x700+n` (heartbeat). Drive TPDOs on `0x180+n` / `0x280+n`
are *the same pattern, one more dict entry* —
[`canworker.py`](../canworker.py)'s `TpdoTap._route` is built exactly for this.

**No COB-ID collision.** Drive TPDO1 would be `0x181`/`0x182`. Your MLS streams
on `0x18A`. Check it anyway, but the arithmetic is clean.

**It closes a real hole the README already documents.** `_poll_telemetry` keeps
the last statusword when an SDO read returns `None`. `health.py` covers that
today by timeout — but pushed telemetry means the statusword is *fresh or
absent*, never silently stale.

### How to build it

`configuration_steps()` in [`drivers/canbus/rpdo.py`](../drivers/canbus/rpdo.py) is
the pattern: return the writes as **data**, assert the ordering in a test, then
execute. Disable → remap → re-enable, with the entry count zeroed before the
entries are touched. A TPDO version is the same shape against `1800h`/`1A00h`.

Improvement A already permits it — TPDO mappings pass for any object, because a
TPDO is a read path.

**Risk:** medium. You are changing how the vehicle learns a drive has faulted.
Verify `health.py` still trips on a pulled drive **before** trusting it — that
check is in the bench checklist for exactly this reason.

---

## 4. Improvement C — RPDO1 for `60FFh`

**Built, offline-tested, wired in behind `can.use_rpdo`, shipping `false`.**

Your README's Known Gaps already names this:

> `60FFh` is a blocking SDO write per tick. RPDO1 migration would remove
> ~1.8 ms × 2 nodes from the 20 ms budget.

and [`manuals/potential-ros-migration.md`](potential-ros-migration.md)
§9 ranks it first of everything worth doing, above any framework change.

One 6-byte frame per node per tick — controlword `6040h` (u16) + target velocity
`60FFh` (i32) — replaces two blocking round trips.

### The safety argument, which is not the obvious one

An RPDO is unacknowledged, so the naive reading is that this trades safety for
speed. It does not:

- **Nothing was checking the acknowledgement anyway** on the per-tick path. A
  failed setpoint write is corrected by the next tick 20 ms later, and the
  vehicle has never depended on that ack.
- What actually stops the vehicle when a drive goes quiet is
  [`core/health.py`](../core/health.py), fed by the `1017h` producer heartbeat and
  by telemetry replies. **Untouched by this.**
- The drive's own heartbeat *consumer* is what stops it if the PC dies. A
  drive-side setting, unaffected either way.
- A late tick, by contrast, **is** a steering update the vehicle does not get.

The one property genuinely lost is "this specific frame arrived". It was never
used.

### A hazard I hit while building it — worth knowing even if you skip C

Moving the controlword onto a PDO gives **bit 7 — Fault reset, which is `40C0h`
by another name — a second route to the drive that the SDO guard never sees.**
And that route fires 50 times a second. A controlword assembled with that bit
set would auto-reset an alarm continuously: **the automatic restart ISO 3691-4
prohibits, arrived at by accident.**

`rpdo.pack()` therefore runs every controlword through `guard.check()` on every
frame. `guard` already owns that rule, so it is reused rather than restated.

### Shape

- [`drivers/canbus/rpdo.py`](../drivers/canbus/rpdo.py) — packing, the 7-step setup
  sequence returned as data, and `configure()` with the SDO writer **injected**
  (so a bench script, `canworker`, or anything else supplies its own).
- [`canworker.py`](../canworker.py) — `_write_target()` branches on
  `config.CAN_USE_RPDO`; setup runs at arm time **before the NMT start**,
  because PDO mapping belongs in Pre-operational.
- [`tests/test_rpdo.py`](../tests/test_rpdo.py) — 47 checks. The three silent
  failure modes are each covered: swapped payload halves, remapping a live PDO,
  and a forbidden object reached through the mapping.

**Flag-gated deliberately.** It changes the motion path; backing out is a
profile edit, not a revert. Both paths stay tested.

**Acceptance criterion:** `/monitor`'s loop `work_avg_ms` drops by ~3.6 ms. If
it does not, the drive is not acting on the RPDO and something is wrong — do not
proceed to the floor.

---

## 5. Improvement D — you already own a gyro, and it is on the wrong side of a retirement

**Built: a read-only reader + bias tool. This is the item most specific to *your*
open questions.**

### The fact

There is a **6-axis IMU inside the SICK MLS**, exposed as CANopen objects on node
10. You are already talking to that node 100 times a second. Measured on this
unit, 661 samples stationary, 2026-09-03:

| | measured | |
|---|---|---|
| gyro z bias | **+0.0574 °/s** | 0.94 LSB → 3.44 °/min drift |
| gyro z noise | **σ 0.0291 °/s** | 0.48 LSB — **below one LSB** |
| gravity | 0.9894 g | 1.1 % scale error, uncalibrated |

I cross-checked the scaling before trusting it: the stated LSBs imply
**±2000 °/s**, **±16 g**, and **±π rad exactly filling an int16**, with a
65.536 s timestamp wrap. Three standard full scales and an Euler range that
exactly fills its word is not what a mis-read table looks like. σ below one LSB
means the gyro is **quantisation-limited, not noise-limited**.

### Why this matters to *line following* specifically

Your README's entire curve-capability section is **simulation**:

> **No curve has been driven yet.** The numbers below are from a
> constant-curvature simulation.

and the two things it says are load-bearing are both *calculated*, not measured:

1. **"The binding limit on `K_RATIO` is `6083h`, not stability of the ideal
   plant"** — yaw acceleration capped at `2 × 6083h × RAD_S_PER_RPM_DIFF` =
   2.59 rad/s². Right now that is arithmetic in
   [`core/kinematics.py`](../core/kinematics.py). **The gyro measures whether it is
   true.**
2. **"The binding constraint is the entry transient against the ±100 mm sensor
   window"** — which decides your tightest usable radius (~0.8 m simulated) and
   whether `k_ratio 11.3 → 18` actually reaches 0.5 m.

**So the highest-value use is the least clever one: log measured yaw rate in
`run.csv` next to `omega_cmd`.** Every curve-tuning decision you make currently
compares a commanded yaw against a simulation. One extra column compares it
against reality, for free, on hardware you already own.

Second use, also concrete: **commanded ω vs measured ω is a slip detector.** A
150 kg vehicle, and "traction breakaway measurement" is already on your deferred
list. This measures it.

### On curvature feedforward — promising, but read this first

Your README calls curvature feedforward "the structurally correct fix" that
"needs a curvature estimate". The gyro gives you one: `κ̂ = ω_measured / v`.

**But be careful.** Feeding measured ω forward makes `ω_cmd ≈ ω_meas` in steady
state, which is a unity positive-feedback loop — neutrally stable, i.e.
**functionally an integrator**. It nulls `e_ss` for the same reason `Ki` would,
and it inherits the same windup concerns. It is *not* a free lunch, and it is
not the same thing as feeding forward a *known* path curvature. Prototype it
against `tests/helpers.py`'s plant simulation before it goes near the vehicle.

### The catch, and it is a real one

The MLS allows **at most 4 active TPDOs**, and valid COB-ID bases are only
`0x180 / 0x280 / 0x380 / 0x480` + node. Your track data holds `0x18A` (TPDO1).
Three slots are free — the yaw-rate slot (`1806h`) is one of them.

**But at 125 kbps, bus load is the constraint.** Track data at 100 Hz is already
~9 % of the wire. A gyro TPDO at 100 Hz adds another ~9 %, on top of your SDO
traffic. Either slow the gyro's event timer, or do §6 first.

**Open question you must answer at the bench:** does the MLS keep transmitting
with tape-following disabled or no tape present? The IMU objects are documented
as independent of the track objects, but that is **not verified**, and it
matters to you more than it does to the SLAM variant.

**Files:** [`drivers/canbus/read_imu.py`](../drivers/canbus/read_imu.py) (`show`,
`bias`, `tpdo` — reads safe, writes need `--go`),
[`tests/test_imu.py`](../tests/test_imu.py) (46 checks).

---

## 6. Improvement E — CAN 125 kbps → 1 Mbps

**Tool built and tested offline. The protocol is CiA 305 as specified and
`NOT ONE BYTE OF IT HAS BEEN EXERCISED AGAINST YOUR SENSOR.`**

Everything in §1 is priced at ~1.8 ms per SDO round trip, which is a direct
consequence of 125 kbps. At 1 Mbps a round trip drops roughly 8×, and the entire
tail in §1 mostly evaporates without touching a line of control code. It also
makes §5's gyro TPDO comfortable rather than marginal.

**Both your devices support it** — confirmed in the manuals.

> ### This is the one item that can cost you a day
> The three devices change bitrate by **three different mechanisms**, and a
> half-completed migration is a bus where nothing talks to anything:
>
> | device | mechanism |
> |---|---|
> | BLV-R × 2 | **MEXE02 only — no DIP switch**, and it talks over the same bus |
> | SICK MLS | LSS (CiA 305), or SICK's configurator |
> | PC | `ip link` / the profile |
>
> **Drives last.** Have a second adapter before you start.

[`drivers/canbus/lss.py`](../drivers/canbus/lss.py) does the MLS half:
`scan` and `verify` are read-only, `set` needs `--go`. It defaults to
**selective** switching by identity rather than the global form (which addresses
every LSS node at once), stages/stores/activates as three separate steps so
there is a window to abort, and puts the rate on the wire as a **table index** —
writing `1000000` where an index belongs would silently select something else.

**Run `lss.py scan` first.** If the sensor does not answer, it does not speak
LSS — use SICK's configurator and do not start guessing at command specifiers
with your only line sensor.

---

## 7. What NOT to take

These are SLAM-specific and would be dead weight or worse on your branch:

- **The README retirement banner** I added — it announces that line-following is
  being retired. On your branch it is false.
- **The `hardware-reconciliation.md` D-9 section** — same content as §2 here,
  but framed around the ROS migration.
- **`manuals/slam-generalized-plan/` generally.** §5 of the reconciliation doc
  (bitrate) and D-3 (the IMU) are the only parts that concern you, and both are
  summarised above.
- **Anything implying `core/autopilot.py` or `core/branch.py` should go.** On
  your branch they are the product.

---

## 8. Corrections to your docs, found while reading

Independent of everything above. Each is a fact that is currently wrong in a
document a future reader will trust:

| where | says | actually |
|---|---|---|
| `README.md` summary table | Cruise **1600 r/min = 0.503 m/s** | `profiles/agv-01.json` has `auto_rpm: 1200` → 0.377 m/s |
| `README.md` ×3 | **689** offline checks | `run_all.py` pinned **866** before my changes (1007 after) |
| `README.md` Known Gaps | "`__pycache__` is tracked in git — run `git rm -r --cached`" | **already fixed** in commit `7074ed2`. `git ls-files` returns 0. Delete the gap. |
| `README.md` Layout | — | omits `drivers/lidar_scan.py` and `drivers/modbus_io.py` |
| `README.md` generally | — | no mention of slow zones, station stops (`stop_until_start_button`), `panel.manual_auto_arm`, or `auto_resume_hold_s` — all shipped and all in the profile |
| `manuals/potential-ros-migration.md` | 12 links like `](../core/branch.py)` | the file is in `manuals/`, so they resolve to `manuals/core/branch.py`. **All dead.** Fixed on my branch |

**Still genuinely open** (your Known Gaps is right about this one): the
`config.py` `rfid.*` docstring documents `inventory_cmd`, `epc_offset`,
`handshake_hex`, `poll_period_s` and `comms_timeout_s` — **none of which are in
`_SCHEMA`** — and says `enabled` ships `false` while the profile has it `true`.
The duplication is gone, the rest is not. A strict loader whose own
documentation is the loose part is a trap.

---

## 9. How to take these

Cherry-pickable in dependency order. **A first, always.**

```
A   drivers/canbus/guard.py, canworker.py (_write passes sub),
    tests/test_canmon.py, tests/helpers.py            +11 checks
C   drivers/canbus/rpdo.py, tests/test_rpdo.py,
    canworker.py (_write_target, arm), config.py,
    profiles/agv-01.json                              +47 checks
D   drivers/canbus/read_imu.py, tests/test_imu.py     +46 checks
E   drivers/canbus/lss.py, tests/test_lss.py          +37 checks
```

Each new module is standalone in `drivers/canbus/` — **no `config` import**, per
your existing invariant, so all four still run on a bench laptop. None collides
on the flat `sys.path` namespace and none shadows a stdlib module (checked by
hand; `test_layout` would catch it).

Raise `EXPECTED_CHECKS` and add the module to `MODULES` in
[`tests/run_all.py`](../tests/run_all.py) deliberately, as your own docstring
insists.

### Verification, and its honest limits

```bash
python3 tests/run_all.py        # must print "all checks passed"
```

**I could not run the full suite in one process.** This machine has no
`python-can`, no `matplotlib` and no `flask`, so `test_logging`, `test_web` and
`test_layout` never executed. I verified 737 checks across the other 13 modules
with zero failures, and cross-checked the arithmetic: the three skipped modules
account for 270 checks both before and after (866 − 596 and 1007 − 737), so
nothing I changed is hiding in them. **Run the whole suite on the vehicle before
you trust any of it.**

Then work the bench checklist. **Wheels off the floor for §4.3 and §4.4** — the
watchdog test in §4.4 is the one that decides whether C is safe to take to the
floor, and it is not optional.
