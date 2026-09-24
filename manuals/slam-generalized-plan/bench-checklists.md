# Bench checklists — T0, T0b, IMU TPDO, RPDO1

**Written:** 2026-09-06 · branch `refactor02`
**Why this exists:** the code for all four items below is written and covered by
the offline suite, but none of it has touched hardware. Spec §0.3 ends an `[HW]`
task at "code complete + sim-verified + bench checklist written for human" —
this is that checklist.

**Read [`hardware-reconciliation.md`](hardware-reconciliation.md) first.** The
ordering below is its §5, and the ordering is the whole safety argument.

---

## 0. Before any of it

| | |
|---|---|
| Vehicle | **jacked up, wheels clear of the floor** |
| Service | `sudo systemctl stop agv_controller` — it owns `can0` |
| Selector | MANUAL, and understand that `panel.manual_auto_arm` means MANUAL is an **armed** state |
| Offline suite | `python3 tests/run_all.py` → 1007 checks, all passing |
| Second adapter | have one. A half-migrated bus is recovered with a second adapter, not with hope |

```bash
sudo systemctl stop agv_controller
python3 tests/run_all.py            # must print "all checks passed"
```

> **The failure mode that costs a day** is a bus whose nodes disagree about
> bitrate. The BLV-R drives have **no DIP switch** — they are reconfigured only
> through MEXE02, over the same bus. Get them to the wrong rate with nothing
> else on that rate and the recovery is a bench with a second adapter and a
> known-good rate to meet them on. Do §2 in the stated order.

---

## 1. T0b — PDO configuration through the write deny-list

**Already done in software; verify only.** `guard.py` now admits the CiA 301 PDO
ranges per-range, with RPDO mapping entries validated recursively against the
deny-list. Nothing to do at the vehicle — this is here so the ordering is
visible: it blocks both §3 and §4.

```bash
python3 tests/run_all.py            # test_canmon covers the deny-list changes
```

Expected: `an RPDO may NOT map 403Eh behind the deny-list` passes.
That check is the one that matters — it is the hole opened by the naive version
of this change.

---

## 2. T0 — CAN 125 kbps → 1 Mbps

Three devices, three mechanisms. **Drives last**, because MEXE02 talks over the
same bus.

### 2.1 Does the MLS speak LSS at all? (read-only, nothing changes)

```bash
python3 drivers/canbus/lss.py scan
```

- [ ] All four identity fields answer (vendor, product, revision, serial)
- [ ] **Record the four values here:** `________________________________`

> **If nothing answers, STOP.** The sensor's LSS support is *reported* in the
> reconciliation doc, not verified. A silent node means either it does not speak
> LSS or the bus is not at 125 kbps — try `--bitrate 500000` before concluding.
> If it genuinely does not answer, use SICK's own configurator and skip to §2.3.
> Do **not** start guessing at command specifiers.

### 2.2 Move the MLS

- [ ] **Power the BLV-R drives down.** They do not speak LSS, so a global switch
      cannot reach them — but verifying the sensor alone at the new rate is only
      possible with them off the bus.

```bash
python3 drivers/canbus/lss.py set --to 1000000 --go
```

The tool stages, stores, then activates, in that order, and prints each frame.
A refusal at any step leaves the bus at the old rate.

- [ ] Staging accepted (`the node refused the rate` did **not** appear)
- [ ] Store accepted — **without this it reverts on the next power cycle**
- [ ] Activate sent

Then bring the PC up to meet it:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
python3 drivers/canbus/lss.py verify --bitrate 1000000
```

- [ ] The node answers at 1 Mbps
- [ ] **Power-cycle the sensor and re-run `verify`** — this is what proves the
      store actually took

### 2.3 Move the drives (MEXE02, Windows)

- [ ] Power the drives back up **with the PC's `can0` back at 125 kbps**, or
      MEXE02 will not see them
- [ ] Set both drives to 1 Mbps in MEXE02, store, power-cycle
- [ ] Set `can0` to 1 Mbps

### 2.4 Move the profile

- [ ] `profiles/agv-01.json` → `"can": { "bitrate": 1000000 }`
- [ ] `python3 -c "import main"` exits 0
- [ ] `python3 drivers/canbus/verify_bus.py` — both drives and the sensor answer
- [ ] `python3 drivers/canbus/verify_drivers.py` — no collisions

### 2.5 Restore service

- [ ] `sudo systemctl start agv_controller`
- [ ] `/monitor` shows both drives, heartbeats live, no EMCY
- [ ] Jog each direction on blocks. **Wheel sign convention re-verified** —
      a bitrate change cannot flip it, but this is the cheapest possible check
      and the consequence of being wrong is the vehicle accelerating off a line

---

## 3. IMU on node 10

Depends on §1. Independent of §2 — do it at either bitrate.

### 3.1 Is the IMU even on? (read-only)

```bash
python3 drivers/canbus/read_imu.py show
```

- [ ] `enabled (2006h:02)` reads **1**. If it reads 0 the IMU *and* the
      thermometer are off and every reading below is meaningless
- [ ] Gyro reads near zero on all three axes with the vehicle still
- [ ] `|accel|` is within a few percent of 1 g — a free scale check that needs
      no reference equipment. The recorded value is **0.9894 g**
- [ ] The four TPDO slots print, and `1803h Euler` shows `COB-ID 0x48A disabled`

> If the TPDO table does not match the probe recorded in D-3
> (`1803h` addressed-but-disabled, `1804h`/`1805h`/`1806h` at COB-ID 0), the
> sensor has been reconfigured since 2026-09-03. Record what it actually shows
> before changing anything.

### 3.2 Re-measure the bias

```bash
python3 drivers/canbus/read_imu.py bias --seconds 60
```

**Vehicle stationary and undisturbed.** Nothing in software can tell a genuine
bias from somebody leaning on the chassis.

- [ ] **Bias:** `________ °/s` (recorded 2026-09-03: **+0.0574**)
- [ ] **Sigma:** `________ °/s` (recorded: **0.0291**)
- [ ] Reported as *quantisation-limited* (σ below one LSB = 0.0610 °/s)

> σ **above** one LSB means either the vehicle moved during the run or this unit
> is noisier than the one measured. Re-run before believing it.

### 3.3 Put yaw rate on a TPDO

The sensor allows **at most four active TPDOs**, and valid COB-ID bases are only
`0x180 / 0x280 / 0x380 / 0x480` + node. TPDO1 (`0x18A`) currently carries track
data; it is freed when tape-following retires.

```bash
python3 drivers/canbus/read_imu.py tpdo gyro --cob-base 0x280        # dry run
python3 drivers/canbus/read_imu.py tpdo gyro --cob-base 0x280 --go
```

- [ ] The dry run prints the intended COB-ID and refuses to write
- [ ] The write is accepted
- [ ] **Restart the sensor** — the write does not take effect until it does
- [ ] Frames appear on `0x28A`: `candump can0,28A:7FF`
- [ ] `read_imu.py show` reports the gyro slot **ENABLED**
- [ ] **Power-cycle and re-check.** If it reverts, the configuration was not
      stored and needs SICK's configurator

### 3.4 The open question D-3 flags

- [ ] **With tape-following disabled, confirm the sensor still transmits.** The
      IMU objects are independent of the track objects, but confirm that a
      sensor seeing no tape does not suppress TPDO transmission or set a
      persistent event flag that masks a real one:
      `python3 drivers/canbus/tune_mls.py events`

---

## 4. RPDO1 for `60FFh`

Depends on §1. **This changes the live motion path.** It ships behind
`can.use_rpdo`, default `false`, so backing out is a profile edit.

### 4.1 What it does

One 6-byte frame per node per tick (controlword `6040h` u16 + target velocity
`60FFh` i32) replaces two blocking SDO writes at ~1.8 ms each. The loop
currently runs 20.4 ms average against a 20 ms budget, peaking near 31 ms.

**Expected result: ~3.6 ms off the average tick.** That number is the acceptance
criterion — if the tick does not get faster, the RPDO is not being acted on and
the drive is probably still taking the old SDO path.

### 4.2 Turn it on

- [ ] `profiles/agv-01.json` → `"can": { "use_rpdo": true }`
- [ ] `python3 -c "import main"` exits 0
- [ ] `sudo systemctl restart agv_controller`
- [ ] **Wheels still clear of the floor**

### 4.3 Verify on blocks

- [ ] Arm in MANUAL. `journalctl -u agv_controller -n 30` shows no RPDO setup
      failure — the sequence aborts before re-enabling the PDO if any step is
      refused, so a failure here leaves the PDO disabled rather than half-mapped
- [ ] `candump can0,201:7FF,202:7FF` shows 6-byte frames at ~50 Hz while jogging
- [ ] Decode one by hand: bytes 0–1 are `0F 00` (Operation enabled), bytes 2–5
      are the signed r/min little-endian
- [ ] **Jog each direction.** Wheels turn the correct way — a swapped payload
      would read the low half of the velocity as a controlword, which is a
      plausible value rather than an error
- [ ] Release the button: the setpoint zeroes within `manual_watchdog_s`
- [ ] `/monitor` → **loop `work_avg_ms` has dropped by roughly 3.6 ms**
      `________ ms` (was ~20.4)

### 4.4 Verify the watchdogs still bite

An RPDO is unacknowledged, so this is the part worth being careful about. What
detects a silent drive is `health.py` fed by the `1017h` heartbeat, **not** the
setpoint write — confirm that is still true:

- [ ] Pull power to one drive while armed in MANUAL. Within
      `driver_timeout_s` (0.6 s) the event log reports it and the vehicle stops
- [ ] Restore power. The log reports it answering again
- [ ] Arm is refused while a drive is silent

### 4.5 Only then, on the floor

- [ ] Wheels down, an auto run on tape, compare `logs/NNNN-*/run.csv`
      `loop_ms` against a run from before the change
- [ ] Tracking RMS unchanged — this should affect timing, not control

**Back out:** set `"use_rpdo": false` and restart. The SDO path is unchanged and
still tested.

---

## 5. What is still not verified after all of this

Carried forward from `hardware-reconciliation.md` §6, none of it addressed here:

- **`6064h` scaling** — "user-defined position units (step)". Steps-per-rev and
  any feed constant must be **read from the drive**, not assumed, before
  odometry is trusted.
- **Backlash magnitude** — command a reversal, compare commanded shaft motion to
  measured yaw from the now-working gyro. This sizes the D-2 risk and decides
  whether external encoders come back into the BOM. §3 is what makes this
  measurable.
- **`sick_safetyscanners2` on Humble against nanoScan3** specifically.
- **N97 thermals** under sustained nav2 load — measure with `turbostat`, not by
  core count.
