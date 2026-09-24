# EVOTY tow-tractor AGV — Safety system brief

**Vehicle:** EVOTY differential-drive tow tractor, 600 kg towed, 1.0 m/s design speed, flat floor
**Purpose of this document:** a single-page picture of the safety chain as it stands — what is wired, what is configured, what it achieves, and what it does not yet achieve.
**Status:** Category 0 stop chain built and commissioned. **Not the shippable ISO 3691-4 configuration.**
**Date:** 2026-09-04

---

## 1. Architecture at a glance

```
  DEMAND SOURCES                 EVALUATION                 POWER REMOVAL
  --------------                 ----------                 -------------

  E-stop, 2 x NC  --X1->I1--+
                  --X2->I2--+
                            +--> AND --+
  nanoScan3 OSSD  ----->I3--+          |
  (protective fld)----->I4--+          +--> EDM block --> Q1 --> HWTO1+ (pin 11)  both drives
                                       |    (Cat. 3)      Q2 --> HWTO2+ (pin 26)  both drives
  AUTO/MANUAL     ----->I8-------------+                   |
  selector             (gates lidar only)                  v
                                                      torque removed
  Restart ack     ----->I7--> Restart block --+       brake applies <=35 ms
                                                           |
                             I5 <-- EDM- (pin 14) drive L -+
                             I6 <-- EDM- (pin 14) drive R -+
                                  both HIGH = torque confirmed removed

  ===================== separate, non-safety =====================

  nanoScan3 --Ethernet--> Linux controller PC   3 zone flags + point cloud
  192.168.3.10            192.168.3.2:6060 UDP  -> target speed, navigation, HMI
                                                 NEVER stops the vehicle
```

Two independent chains. The **safety** chain is entirely hardwired: scanner OSSD → FX3 → drive STO inputs. The **data** chain is Ethernet only and is advisory. They share a sensor and nothing else.

---

## 2. Device inventory

| Role | Part | Config tool | Notes |
|---|---|---|---|
| Safety laser scanner | SICK **NANS3-AAAZ30AN1** (nanoScan3 Core I/O) | **Safety Designer** | Ethernet system plug `NANSX-AAABAEZZ1` — without it there is no data output at all |
| Safety controller CPU | SICK **FX3-CPU00000**, FW V4.05.0 | **Flexi Soft Designer** ≥ V1.7.1 | Config lives in the **system plug**, not the CPU |
| Safety I/O module | SICK **FX3-XTIO84002**, FW V3.10.0 | Flexi Soft Designer | 8 safe inputs, 4 safe outputs, 2 test-pulse generators |
| Motion module (future) | SICK **FX3-MOC0**, FW V1.10.0 | Flexi Soft Designer **only** | Listed `N. a.` in Safety Designer — this is why the FX3 station is not in Safety Designer |
| Safety encoders (future) | 2 × SICK **DFS60S-BAOL01024** Sin/Cos | via FX3-MOC0 | Speed monitoring V1 |
| Drive amplifiers | 2 × Oriental Motor **BLVD-KRD** | RS-485 / CANopen | STO certified PL d / Cat 3, PFH_D 2.90×10⁻⁷ /h |
| Vehicle controller | Linux PC | — | Non-safety. Reads scanner over Ethernet |

**Two tools, two projects, two reports.** The scanner is configured and verified in Safety Designer; the FX3 station is configured and verified in Flexi Soft Designer. Projects do not migrate between the tools. They connect over hardwired OSSDs, not a configuration link, so two device reports in the technical file is normal and correct.

---

## 3. Scanner configuration — nanoScan3

**Monitoring plane:** 1 (the device has exactly one; the "1" is an index, not a limit you chose).
**Monitoring cases:** 1 active. Field set 1.

| Cut-off path | Field | Type | Range | Consumed by |
|---|---|---|---|---|
| 1 | inner | **protective** (safe) | **2.15 m** max | OSSD pair 1.A / 1.B → FX3 I3/I4 **and** universal I/O |
| 2 | middle | warning | ≤ 10 m | universal I/O → PC (via data telegram) |
| 3 | outer | warning | ≤ 10 m | universal I/O → PC (via data telegram) |

Key figures:

| Parameter | Value |
|---|---|
| Object resolution | 50 mm |
| **Protective field range at 50 mm resolution** | **2.15 m** — *not* the datasheet's 3 m |
| Multiple sampling | 4 |
| **Safety response time** | **130 ms** (4 × 30 ms + 10 ms) |
| Scan cycle / rate | 30 ms / 33 Hz |
| Angular range / resolution | 275° (−47.5°…+227.5°) / 0.17° → 1651 points |
| Scan plane height | ≤ 200 mm above floor |

The 2.15 m cap is load-bearing: **do not design planner or field geometry around a 3 m safety stop.** The vehicle's own braking distance stacks on top of the 130 ms.

---

## 4. FX3 I/O map

| Terminal | Element | Test pulse | Function |
|---|---|---|---|
| **I1 / I2** | E-Stop ES21, dual-channel equivalent — `GENERAL-ESTOP` | X1 / X2 | 2 × NC emergency stop. Odd/even rule gives cross-circuit detection |
| **I3 / I4** | nanoScan3, dual-channel equivalent — `LIDAR-FRONT` | **none** | Scanner OSSD 1.A / 1.B. Scanner self-tests at ~300 µs; XTIO filter ignores ≤ 0.9 ms |
| **I5** | Single-channel — `EDM-L` | none | EDM− (pin 14) drive L |
| **I6** | Single-channel — `EDM-R` | none | EDM− (pin 14) drive R |
| **I7** | Single-channel NO | **none** | Restart acknowledge pushbutton. Must **not** reference a test output |
| **I8** | Single-channel | none | AUTO / MANUAL selector. See §5.3 |
| **Q1** | Output element — `STO-CH1` | **ON** | → HWTO1+ (pin 11), **both** drives in parallel |
| **Q2** | Output element — `STO-CH2` | **ON** | → HWTO2+ (pin 26), **both** drives in parallel |
| **Q3 / Q4** | *unallocated* | — | Reserved for the safety-switched brake circuit (F-13) |
| **X1 / X2** | Test pulse generators | — | 120 mA each, loop resistance < 100 Ω |
| **A1 / A2** | +24 V / 0 V | — | Separate feed per module. Scanner 0 V bonded to the same star point |

**Dual-channel evaluation happens in the XTIO module**, not the logic — equivalence, sequence and discrepancy timing are done in hardware before the signal reaches the logic editor. That is why no Emergency stop or Light curtain function block appears in the program, and why the logic stays short.

**Paralleling by channel** (Q1 → both drives' HWTO1, Q2 → both drives' HWTO2) rather than one output per drive: both drives get bit-identical commands with identical timing, which is the symmetry requirement that stops a differential tugger yawing under braking. It also leaves Q3/Q4 free.

Test pulses stay **ON** for Q1/Q2. Gap < 650 µs at ≥ 200 ms interval; the BLVD-KRD ignores HWTO dropouts under 1 ms by design. No spurious trips, full short-to-24 V detection retained. Disabling them degrades **all four** outputs on the module.

---

## 5. FX3 logic

### 5.1 As built

```
GENERAL-ESTOP --+
                +- AND 1 --> Restart block --> EDM block --+--> Q1  STO-CH1
LIDAR gate  ----+           (Release 1)      (Control)     +--> Q2  STO-CH2
                                                  ^
EDM-L --+                                         |
        +- AND 2 ---------------------------------+
EDM-R --+                                    (EDM feedback signal)
```

- **EDM block** — Max. feedback delay **100 ms**, `Use Fault present` enabled. Polarity needs no inversion: the BLVD-KRD asserts EDM only when both HWTO are off, so Control = 0 → feedback = 1, exactly the inverse the block expects.
- **Restart block** — acknowledge on I7, minimum pulse 100 ms. `Restart required` output pulses at 1 Hz for an indicator lamp. This satisfies the no-automatic-restart requirement (F-17).
- Every configured time must exceed the **logic execution time** (a multiple of 4 ms; read it from the report), and each carries ±10 ms plus one execution time of accuracy.

### 5.2 A trap that was hit and fixed

Wiring the EDM AND directly to an output produces a relaxation oscillator on a safety output — outputs drop, EDM asserts, outputs re-enable, EDM clears, repeat. The feedback **must** go into the EDM block's feedback terminal, never straight to Q.

### 5.3 AUTO / MANUAL gating — designed, implementation not confirmed

The lidar stop is to apply only in automatic mode, selected by a switch on I8. The correct wiring is to take the **MANUAL** contact, not the AUTO contact, and OR it with the lidar signal:

```
LIDAR-FRONT --+
              +- OR --> into AND 1
MANUAL (I8) --+
```

With the MANUAL contact, a broken wire or lost signal falls to "not manual" → the lidar stop stays active → **fail-safe**. Taken from the AUTO contact instead, a wire break silently bypasses the scanner. The E-stop is never gated and remains active in both modes.

> **This creates an obligation.** ISO 3691-4 does not permit simply disabling the protective device in manual mode. Compensating measures are required — reduced speed, hold-to-run control, or a restricted operating zone with the operator in view. **None of these exist yet.** Until they do, manual mode is a bench and maintenance facility only, not an operating mode.

---

## 6. Drive-side power removal

Per drive, at CN4:

| XTIO | Drive pin | Signal |
|---|---|---|
| Q1 | 11 | HWTO1+ |
| A2 (0 V) | 12 | HWTO1− |
| Q2 | 26 | HWTO2+ |
| A2 (0 V) | 27 | HWTO2− |
| +24 V (A1) | 28 | EDM+ |
| → I5 / I6 | 14 | EDM− |
| — | 25 | +V *not connected* |
| — | 13 | 0V internal *not connected* |

**All three factory jumpers (25–11, 12–26, 13–27) must be removed on both drivers.** Out of the box they put both HWTO photocouplers in series across the drive's own rail — STO is defeated and the drive behaves perfectly, which is exactly why this gets missed. Witness and record it against the driver serial numbers (F-09).

The previous bench arrangement — one NC emergency-stop contact interrupting the 13–27 link — is a **single point of failure** and caps the architecture at Category 1. It stopped the motors correctly, which is why it was misleading.

**Without EDM monitored, the TÜV certificate does not apply.** The PL d / SIL 2 claim carries the condition *"It is necessary to monitor the EDM output using an external device."* This is the single largest integrity improvement over the old wiring.

---

## 7. Non-safety data path

Detailed in [lidar-data-brief.md](lidar-data-brief.md). Summary:

| Item | Value |
|---|---|
| Scanner / PC | `192.168.3.10` / `192.168.3.2`, mask `255.255.255.0` |
| Stream | UDP to `192.168.3.2:6060`, continuous, every measurement |
| Config channel | TCP CoLa 2, port 2122 |
| Decoder | `sick_safetyscanners2` (ROS 2) or `sick_safetyscanners_base` — do not hand-roll |
| Wire format reference | SICK doc **8022706** |
| Blocks enabled | Device Status, Configuration of Data Output, Measurement Data, Local I/Os |

The PC reads the three zone flags to set target speed, plus device status, contamination, active monitoring case, scan timestamp and the configuration checksum. The point cloud drives localisation.

**Three rules:** this data never stops the vehicle; the scanner configuration is read-only to the PC; and **absence of data is never "clear"** — a stale telegram must fall to innermost-occupied, target speed 0, within ~150 ms.

---

## 8. Response-time budget

| Contributor | Value |
|---|---|
| nanoScan3 detection → OSSD off | **130 ms** |
| FX3 input filter + logic + output | read from the Flexi Soft report — small, but *measure it* |
| Optional: XTIO fast shut-off | bypasses logic/bus, but **adds 8 ms** to input and output device response times |
| BLVD-KRD HWTO → torque removed | ≤ 15 ms |
| Brake engagement | ≤ 35 ms |
| Vehicle braking distance at 1.0 m/s | **not yet measured** — see F-14 |

The last row is the gap. ISO 13855 field sizing cannot be finalised without a witnessed braking type test at minimum bus voltage, and the spread — not the mean — sets the protective field.

---

## 9. Safety performance

| Item | Status |
|---|---|
| Target | **PL d / Cat 3** |
| Architecture | HFT = 1, dual-channel throughout, EDM cross-monitoring — **Cat 3 genuinely met for STO** |
| 2 × BLVD-KRD STO | 5.80×10⁻⁷ /h — **58 % of the entire PL d budget** |
| Remaining for lidar + FX3 + encoders + wiring | 4.20×10⁻⁷ /h |
| Indicative total | ≈ 7.8×10⁻⁷ /h — under the 1×10⁻⁶ ceiling with ~22 % headroom |

Consequences:

- **No room for a third series subsystem** in the stop chain. A safety contactor, a second relay stage or a gateway in series may push it over. Model before adding.
- **PL e is unreachable with these drives.** Two drives alone are 5.8× over the 10⁻⁷ ceiling.
- The SICK PFH_D figures above are indicative placeholders. Substitute the real datasheet values before the SISTEMA run.

---

## 10. What is not built yet

This chain is **Category 0 power removal**: a demand removes torque immediately from both drives. The following are required for ISO 3691-4 and do not exist:

| Missing | Depends on |
|---|---|
| **SS1** — delayed-off timer driving QSTOP before STO | FX3 timer; blocked by F-01 |
| **Safe speed monitoring** | FX3-MOC0 + 2 × DFS60S |
| **Speed-dependent protective field switching** | safe speed → monitoring case selection → scanner static inputs A1/A2 |
| **Safety-switched brake circuit** | Q3/Q4 + a second XTIO; blocked by F-13 |
| **Manual-mode compensating measures** | see §5.3 |
| **Warning means** (audible/visual) | vehicle level |
| **Rear scanner** | if the tugger is bidirectional — the tag `LIDAR-FRONT` anticipates this |
| **Measured braking performance** | F-14 type test protocol |

A second XTIO will be needed: scanner static control inputs A1/A2 for field switching, plus the brake circuit, exceeds what remains on the current module.

---

## 11. Open findings — the ones that gate this chain

Full register in [bldc-compliance.md](bldc-compliance.md).

| # | Finding | Severity (ISO 3691-4 track) | Why it gates the safety chain |
|---|---|---|---|
| **F-01** | Brake performs dynamic stopping, which the manufacturer prohibits | 🔴 **Blocking** | The stopping distance for ISO 13855 must be the one that applies when the SS1 ramp fails. Needs a written position from Oriental Motor. **Longest lead time — start it first.** |
| **F-13** | One brake serves emergency *and* parking duty, upstream of the 1:30 gearhead, no independent secondary | 🔴 **Blocking** | A single mechanical failure removes both duties on one wheel → yaw with 600 kg behind the hitch. An independent wheel brake closes F-13, F-11 and F-02 together |
| **F-02** | No SBC; brake explicitly outside the safety chain | 🔴 Major → approaching blocking | Mitigated by 0° gradient under Track A; that mitigation does not survive F-12 |
| **F-12** | No declared maximum design gradient or route survey | 🔴 Major | "Flat floor" is an operational assumption, not a design declaration |
| **F-14** | No braking performance type test protocol | 🟠 Major | Until measured, the protective field geometry is unfounded |
| **F-15** | No Safety Requirement Specification, no ISO 13849-2 validation | 🟠 Major | Part 2 validation is mandatory, not optional |
| **F-09** | STO wiring integration checks | 🟠 Major under Track B | **Addressed by this build** — jumpers removed, EDM monitored, test-pulse compatibility confirmed. Needs witnessing and recording |
| **F-06** | No protection against left/right configuration divergence | 🟠 Major under Track B | Partly addressed by the PC's startup checksum compare on the scanner; the *drives* still need golden parameter files under version control |
| **F-17** | Drive can be configured to auto-restart | — | **Addressed** — Restart block on I7. Also set ETO reset action to `1 — ON edge`, not level mode |
| **F-03** | `FREE` input releases both brakes from a non-safety input | 🟠 Major | Leave `FREE` unassigned and block it in the fieldbus command map |

**F-01 and F-13 together determine whether the current single-brake architecture survives at all.** Nothing in this build closes either.

---

## 12. Configuration and document control

| Artefact | Where it lives | Notes |
|---|---|---|
| Scanner configuration | nanoScan3 **system plug** | Verified. Read-only to the PC. Changes require Safety Designer on Windows and **re-verification** |
| FX3 configuration | FX3 **system plug** | Survives a CPU swap |
| Authorized-client password | System plug | Forced on first login on a virgin plug. **No recovery — record it in the project file** |
| Scanner verification report | PDF, technical file | Contains the cut-off path indices used by the PC decoder |
| FX3 report | PDF, technical file | Must contain test-pulse parameters, logic execution time, I/O matrix, both checksums |
| Drive parameters | Ordinary non-volatile memory, **no safety-rated lock** | F-06 — golden files under version control, startup read-back and compare |

Log out to **Machine operator** before leaving the laptop, so an unattended session cannot transfer a configuration.

---

## 13. Recurring obligations

- **STO verification test every 3 months**, results recorded — or the PL d claim lapses.
- **Brake capability test** at the same interval, using the wheel safety encoders as the measurement channel. Trend the stopping distance (F-07).
- **Contamination warning** from the scanner → schedule a cover clean between shifts rather than waiting for the error state.
- **Re-verify** after any configuration change, on either device.
- **Change control:** any gradient introduced to the route reopens F-02 and F-12.

---

## 14. Do not

- Deactivate the Q1–Q4 test pulses — unnecessary here, and it degrades all four outputs on the module.
- Reference the OSSD or EDM inputs to a test output.
- Reference the restart-acknowledge input to a test output — short-circuit detection can synthesise a restart pulse.
- Assign `FREE` to any direct input on the drivers; block it in the fieldbus command map (F-03).
- Leave ETO reset action at level mode (F-17).
- Let the PC's zone data influence anything that must be trusted to stop the vehicle.
- Treat this chain as complete.

---

*Sources: Flexi Soft Hardware OI 8012478; Flexi Soft in Flexi Soft Designer OI 8012480; nanoScan3 I/O OI IM0087137; BLV Series R Function Edition HP-5142-6; BLVD installation manual; SICK data-output TI 8022706. Companion documents in this repository: [bldc-compliance.md](bldc-compliance.md), [agv-motor-stop-requirements.md](agv-motor-stop-requirements.md), [lidar-data-brief.md](lidar-data-brief.md), and the Safety Chain Bring-Up artifact.*
