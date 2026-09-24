# Motor & Drive Compliance Assessment — BLV-R Series

**Assessed against:** [agv-motor-drive-stopping-requirements.md](agv-motor-drive-stopping-requirements.md)
**Date:** 2026-09-01
**Type:** Gap analysis on hardware already procured
**Regulatory basis — two tracks:**

- **Track A (current):** Indonesia domestic — no mandatory CE/UKCA. Standards are applied **voluntarily** as internal engineering practice. Where a clause is cited, it is a self-imposed target, not a legal obligation. §1–§9 are written to this track.
- **Track B:** a real **ISO 3691-4 certification** of the vehicle. **§10 sets out everything that must be added.** Several severity calls in §6 change under Track B — the reassessment is in §10.4, and the affected findings carry a pointer.

---

## 0. Configuration under assessment

| Item | Model | Evidence |
|---|---|---|
| Motor ×2 | BLMR6400SKM-GFV-□ — 400 W, pinion shaft, **with electromagnetic brake** | opman p.9 (model code), p.11 |
| Gearhead ×2 | GFS6G30FR — hollow shaft flat, **1:30** | opman p.11 (400 W + GFS6G□FR) |
| Driver ×2 | BLVD-KRD — DC input, 48 V class | opman p.11; opman_fun p.510–511 |
| Drive layout | Differential, 2 independently driven + braked wheels | Project |
| Safety controller | SICK Flexi Soft **FX3** | Project — *out of scope of this document* |
| Sensing | SIL 3 safety lidar + SIL 3 safety encoder **per wheel** | Project — *out of scope* |
| Brake wiring | **Driver-controlled (default)** — coil on the motor encoder/brake connector, switched by driver logic | opman p.13; opman_fun p.44 |
| SS1 realisation | **FX3 delayed-off STO** — FX3 commands deceleration, waits, then drops HWTO | Project decision |
| Gradient | **0° — flat floor only** | Project decision |

**Combination validity: CONFIRMED.** BLMR6400SKM-GFV-□ + GFS6G□FR is a listed pairing for driver BLVD-KRD (opman p.11), and the pair appears in the DoC-9205 Appendix under BLVD-KRD. No unapproved combination.

---

## 1. Headline verdict

| Required function | Provided by drive? | Verdict |
|---|---|---|
| **STO** | ✅ Certified — TÜV SÜD **Z10 106467 0002 Rev. 00**, PL d / Cat 3, SIL 2 | **PASS** |
| **SS1** | ❌ Not a drive function | **CONDITIONAL** — buildable externally, but see F-01 |
| **SBC** | ❌ Explicitly declared *not* safety-related | **FAIL as wired** — see F-02 |
| **SLS** | ❌ Not a drive function | **PASS by other means** — FX3 + SIL 3 encoders |
| Fail-safe brake | ✅ Spring-applied, engages on power loss | **PASS** (as a *holding* brake only) |
| Brake for dynamic stopping | ⚠️ **Prohibited by the manufacturer** | **BLOCKING** — see F-01 |
| PL d / Cat 3 | ✅ For STO only, PFH<sub>D</sub> = 2.90×10⁻⁷ /h | **PASS** with budget caveat |
| Safety manual | ✅ opman_fun Part 4 "Power removal function", p.209–220 | **PASS** |

**Bottom line:** the hardware is **fit to keep**. The STO is genuinely certified to exactly the integrity you need, and the two-drive PFH<sub>D</sub> still fits inside a PL d budget. Two gaps are load-bearing and must be closed before the protective field geometry can be fixed: the **brake is doing dynamic-stopping work the manufacturer forbids (F-01)**, and the **brake is not in the safety chain (F-02)**. Neither requires new motors or drivers; both require a decision and paperwork.

The **0° gradient answer materially de-risks F-02** — with no static hold-on-gradient duty, the anti-rollaway safety function collapses to holding against push and impact, which is a much lower bar than the requirements document assumed.

> **Under Track B this verdict tightens.** ISO 3691-4 certifies the *truck*, not the drive, and it treats the brake as a genuine emergency brake — which makes F-01 harder to argue, not easier. It also expects a **declared maximum design gradient with a parking-brake test at it**, which erodes the flat-floor mitigation above. Seven further gaps (F-12…F-18) open that Track A does not raise at all. See **§10**.

---

## 2. Stop categories — IEC 60204-1

| Stop category | Required | Drive capability | Verdict |
|---|---|---|---|
| **Category 0** | ✅ | STO / power removal function. Declared **"Stop category 0 (IEC 60204-1)"** in the safety parameters table | **PASS** — opman_fun p.210 |
| **Category 1** | ✅ | **Not provided by the drive.** No certified SS1. Must be assembled from: FX3 → deceleration command → FX3 safety timer → HWTO OFF | **CONDITIONAL** |
| **Category 2** | ⚠️ optional | Not used | N/A |

### Category 1 command path — what is actually available

The drive offers a hard-wired **QSTOP input** — the correct functional deceleration trigger for the FX3 to drive, in preference to a fieldbus message:

- `QSTOP input action` = **2**: "Deceleration rate stop (according to the Quick stop rate parameter) (current is cut off after stopping)" (opman_fun p.36)
- `Quick stop rate` parameter sets the deceleration, default 1,000 (r/min)/s (opman_fun p.37)
- Stop-action priority level **2** — above STOP input (level 4) and above normal profile stops (level 5) (opman_fun p.39)

**Use QSTOP on a direct input, not a CANopen/Modbus message.** A hard-wired input removes the communication stack from the deceleration path and leaves the drive free to fault without stranding the stop command. Note CN4 provides only **4 direct inputs (IN0–IN3)** — budget one for QSTOP on each driver.

> ⚠️ **The deceleration ramp is not a safety function.** QSTOP, the Quick stop rate parameter, and the drive's execution of them are all standard (non-safety-rated) functions. The integrity of your SS1 lives entirely in the **FX3 safety timer + STO**. This is a legitimate SS1-t construction, but it has one unavoidable consequence, stated in F-01 below: **the stopping distance used for the ISO 13855 protective field must be the one that applies when the ramp fails**, not the one you measure when it works.

---

## 3. Drive safety functions — IEC 61800-5-2

| Function | Required | Present in BLVD-KRD | Verdict / route |
|---|---|---|---|
| **STO** | ✅ Mandatory | ✅ **Yes, certified.** Two-channel HWTO1/HWTO2 shutting off upper and lower inverter arm drive signals independently | **PASS** |
| **SS1** | ✅ Mandatory | ❌ No | External: FX3 timer + QSTOP + STO. See F-01 |
| **SS2** | ⚠️ Optional | ❌ No | Not used |
| **SOS** | ⚠️ Optional | ❌ No | Not used |
| **SLS** | ✅ Strongly rec. | ❌ No | **PASS by other means** — FX3 + SIL 3 wheel encoders. Satisfies §7 "acceptable only if achieved by other safe-speed means". Encoders are on the **wheels**, downstream of the gearbox — this is the correct location and covers gearbox ratio faults for speed sensing |
| **SDI** | ⚠️ Optional | ❌ No | Available from FX3 if needed |
| **SBC** | ✅ Required (fail-safe brake in use) | ❌ **No — and explicitly disclaimed** | **FAIL as wired.** See F-02 |
| **SBT** | ⚠️ Recommended | ❌ No | Manual test procedure required. See F-07 |
| **SSM** | ⚠️ Optional | ❌ No | Available from FX3 |
| **SLA / SAR** | ⚠️ Optional | ❌ No | Ramp bounded functionally by Quick stop rate only |

**Required minimum set was STO + SS1 + SBC. Delivered by the drive: STO only.**

### 3.1 STO — verified detail

From opman_fun **p.210, "1-2 Safety parameters"**:

| Parameter | Value |
|---|---|
| Safety integrity level | **SIL 2** * |
| PFH<sub>D</sub> | **2.90×10⁻⁷ [1/h]** |
| Hardware fault tolerance | **HFT = 1** |
| Subsystem type | Type A |
| Mission time | **20 years** |
| **Response time** | **15 ms or less** |
| Performance level | **PL d (Category 3)** * |
| MTTF<sub>D</sub> | High |
| DC<sub>avg</sub> | High |
| Stop category | **0 (IEC 60204-1)** |

\* **Conditional on EDM monitoring.** The asterisk in the manual reads: *"It is necessary to monitor the EDM output using an external device."* Without EDM fed back to the FX3, the SIL 2 / PL d claim does not stand. This is not optional.

Certificate: **TÜV SÜD Z10 106467 0002 Rev. 00**, test report OT102914T, issued 2024-08-14, **valid until 2029-08-12**. Tested to IEC/EN 61800-5-2:2016/2017, IEC/EN 61508-1/-2 (SIL 2), EN ISO 13849-1:2023 (Cat. 3, PL d), IEC/EN 61000-6-7. Safety function listed: **STO (EN/IEC 61800-5-2) — and nothing else.**

Nomenclature on the certificate (`BLVD – K a RD – [A]`) covers BLVD-KRD. DoC-9205 lists BLMR6400SKM-GFV-F/-B against BLVD-KRD. **Your exact part numbers are inside both documents.**

---

## 4. Integrity — ISO 13849-1

### 4.1 PFH<sub>D</sub> budget

PL d requires **1×10⁻⁷ ≤ PFH<sub>D</sub> < 1×10⁻⁶ /h** for the complete safety function, and a differential-drive stop needs **both** drives to remove torque — so both STO subsystems are in series and their PFH<sub>D</sub> values **add**:

```
2 drives × 2.90e-7  =  5.80e-7 /h
PL d ceiling         =  1.00e-6 /h
Remaining budget     =  4.20e-7 /h   for lidar + FX3 + encoders + wiring
```

**The two drives alone consume 58 % of the entire PL d budget.** That is the single most important number in this assessment.

Indicative check with typical SICK figures (**replace with the values from your actual datasheets before the SISTEMA run**):

| Subsystem | Indicative PFH<sub>D</sub> |
|---|---|
| 2 × BLVD-KRD STO | 5.80×10⁻⁷ |
| Safety lidar | ~8×10⁻⁸ |
| FX3 CPU + I/O module | ~2×10⁻⁸ |
| 2 × safety encoder | ~1×10⁻⁷ |
| **Total** | **≈ 7.8×10⁻⁷** |

Under 1×10⁻⁶ — **PL d holds** — but with roughly 22 % headroom. Consequences:

- You have **no room for a third series subsystem** in the stop chain. Adding a safety contactor, a second relay stage, or a gateway in series may push you over. Model it before you add it.
- If the risk assessment ever raises the target to **PL e**, this hardware cannot reach it. PL e needs PFH<sub>D</sub> < 10⁻⁷ /h; two drives alone are 5.8× over that ceiling. **This is the hard limit of the chosen drives.**

### 4.2 Category 3 — architecture

✅ HFT = 1, DC<sub>avg</sub> High, MTTF<sub>D</sub> High, dual-channel HWTO with EDM cross-monitoring. Category 3 is genuinely met **for STO**, provided EDM is monitored.

Fault detection behaviour (opman_fun p.216):

| HWTO1 | HWTO2 | EDM | Motor |
|---|---|---|---|
| ON | ON | OFF | Excitation |
| OFF | OFF | **ON** | Non-excitation |
| ON | OFF | OFF | Non-excitation |
| OFF | ON | OFF | Non-excitation |

Discrepancy between channels ⇒ motor de-energised and EDM stays OFF ⇒ FX3 must latch a fault. The manual is candid: *"Not all dangerous failures can be detected with the EDM output."*

### 4.3 Symmetry between the two drives

The requirements document is right that asymmetry is a safety requirement, not a convenience. Both drives are the same model with the same PFH<sub>D</sub>, so the **hardware** is symmetric.

**However — there is no safety-rated configuration lock.** No parameter write-protection, no safety-configuration CRC, and no mechanism that prevents the two drivers from carrying different `Quick stop rate`, `QSTOP input action`, or `Control resolution` values. Parameters live in ordinary non-volatile memory and are writable over RS-485/CANopen at any time. A silent parameter divergence between left and right produces exactly the uncommanded-yaw / jackknife case the requirements document warns about. See F-06.

### 4.4 Proof-test interval — **mandatory, recurring**

opman_fun **p.218**:

> *"According to use conditions of the safety related parts of a control system, perform a verification test of the power removal function **at least once three months**. Keep the verification result on record."*

**The PL d claim is not valid without this.** It is a condition of use, not advice. Procedure (p.218): power up with both HWTO ON → confirm motor excites and EDM is OFF → drop both HWTO → confirm non-excitation and EDM ON. Also required at commissioning, after maintenance, and **after any driver replacement**.

---

## 5. Braking and holding

| Requirement | Finding | Verdict |
|---|---|---|
| **Service brake** | Regenerative/electrical deceleration via QSTOP ramp. Functional only, not safety-rated | ⚠️ Adequate as the *functional* service brake |
| **Fail-safe holding brake** | ✅ Spring-applied, electrically released. Brake **holds** in Motor non-excitation, ETO, Power-removal-with-ETO, and Alarm(non-excitation) states; **releases** only in Motor excitation, FREE, and Alarm(excitation) (opman_fun p.44). Holds on loss of main power | **PASS** |
| **Anti-rollaway / hold on gradient** | Gradient is 0°. No static gradient-hold duty. Residual duty = hold against manual push and against impact | **PASS** — greatly relaxed by the flat-floor answer |
| **Brake sizing basis** | You hold the brake holding-torque figure. Sizing check at 0° reduces to a push/impact case, not a gravity case | ⚠️ Confirm against RA load cases |
| **Automatic brake application** | Applies on E-stop (via HWTO), power loss, and non-excitation alarms. **Does not apply** in Alarm(excitation) or FREE | ⚠️ See F-03 |
| **Stopping distance repeatability** | ❌ **Not repeatable** — varies with battery voltage. See F-04 | **FAIL** |
| **Brake wear detection** | ❌ None. No SBT, no brake-current monitor, no wear feedback | ⚠️ See F-07 |
| **Manual brake release** | ⚠️ The `FREE` input releases the brake — but it is an **ordinary, non-safety-rated input** (opman_fun p.45). It is not a deliberate mechanical action and it *is* defeatable in normal operation | **See F-03** |

### 5.1 Brake timing (for the ISO 13855 calculation)

From the power-removal timing chart, opman_fun **p.214**, measured from HWTO1/HWTO2 going OFF:

| Event | Time |
|---|---|
| Driver enters power removal status | ≤ 15 ms |
| EDM output ON | ≤ 15 ms |
| Motor non-excitation | ≤ 15 ms |
| **Electromagnetic brake → Hold** | **≤ 35 ms** |

Release side (opman_fun p.215): brake releases ≤ 50 ms after ETO-CLR.

Your total stopping distance for the protective field:

```
s_total = v · (t_lidar + t_FX3 + t_delay + 0.035)  +  v² / (2 · a_achieved)
```

where `t_delay` is the FX3 safety timer and `a_achieved` is whichever of the two cases below applies. At v = 1.0 m/s the ≤35 ms brake latency alone contributes **35 mm** — small, and not the problem. The problem is `a_achieved`.

---

## 6. Findings register

### F-01 — 🔴 BLOCKING — The brake is doing dynamic-stopping work the manufacturer prohibits

**Evidence.** Three separate statements, all unambiguous:

- opman **p.4** (WARNING): *"Do not use the brake mechanism of the electromagnetic brake motor **as a safety brake**. It is intended to hold the moving part and motor positions. Using it as a safety brake may result in injury or damage to equipment."*
- opman **p.7**: *"The electromagnetic brake of the motor is used for holding the motor shaft. **Actuating the electromagnetic brake to hold the motor shaft while the motor is rotating may result in damage** to equipment."*
- opman_fun **p.211**: *"**Do not use the brake mechanism of the electromagnetic brake motor for braking the motor rotation.** This may result in injury or damage to equipment."*

And, critically, opman_fun **p.210**:

> *"**Be sure to check the motor is in a standstill state before executing the power removal function.** Executing the power removal function while the motor is operated may cause damage to the motor, the driver, or equipment."*

**Why this bites.** Your SS1 is `FX3 delayed-off STO`. The safety integrity is in the timer + STO. So the safety case must survive the deceleration ramp failing to happen at all — that is the entire reason the timer exists. When the ramp fails, the timer expires, HWTO drops **while the vehicle is moving**, and the only retarding force left is the electromagnetic brake, applied dynamically at speed. That is precisely what all four quotations forbid.

**Quantifying the alternative.** If you may *not* rely on the brake, the fallback deceleration is rolling and drivetrain resistance only. From 1.0 m/s:

| Rolling resistance μ | Deceleration | Coast distance |
|---|---|---|
| 0.010 (PU on smooth concrete) | 0.098 m/s² | **5.1 m** |
| 0.015 | 0.147 m/s² | **3.4 m** |
| 0.020 | 0.196 m/s² | **2.6 m** |

Drivetrain drag through the 1:30 gearhead will reduce these, but the order of magnitude is **metres**. A protective field of that depth is not workable in an unfenced pedestrian aisle. **The brake is load-bearing for your stopping distance whether the manual likes it or not.**

**Fair reading.** These warnings target *routine* dynamic braking — the wear and thermal case of stopping a rotating load on every cycle. Emergency-only dynamic application of a spring-applied brake is normal practice on AGVs and is what EN 1175 / ISO 3691-4 anticipate for an emergency brake. Vendors routinely permit a bounded number of dynamic stops. So this is very likely a **"get it in writing"** problem rather than a fatal one — but you cannot close the safety file on your own reading of a warning that says the opposite.

**Actions.**
1. **Obtain a written statement from Oriental Motor** covering emergency dynamic braking of the BLMR6400SKM brake: permitted energy per stop (J), permitted number of dynamic stops over life, resulting brake service life, and any speed ceiling. This is the single highest-priority open item in the file.
2. Until that statement exists, **size the ISO 13855 protective field on the coast case** (no brake contribution). Expect it to be impractical — which is the point: it makes the dependency visible.
3. Tune the FX3 `t_delay` so the ramp normally completes and the brake engages at or near zero speed. This makes dynamic application the rare exception rather than the design case, and is what you will want to be able to tell Oriental Motor.
4. Consider fitting a **separate service/emergency brake** on the drive wheels if the vendor statement comes back restrictive. This is the only answer that fully removes the dependency.

---

### F-02 — 🔴 MAJOR — No SBC; the brake is explicitly outside the safety chain

**Evidence.** With the default wiring you have chosen, the brake coil sits on the motor's encoder/brake connector and is switched by driver logic. The manual disclaims it twice, in the safety chapter itself:

- opman_fun **p.214**: *"The ETO-MON output, the READY output, the PWR/SYS LED, and **the electromagnetic brake are not safety-related parts of a control system**."*
- opman_fun **p.215**: same sentence repeated for the release direction.

There is **no SBC**, no brake-circuit diagnostic, and no dual-channel path to the coil. A single fault in the driver's brake output stage that leaves the coil energised holds the brake **released**, with no detection and no annunciation. Under ISO 13849-1 that is a single point of failure in the holding function — **not Category 3**.

The `MBC output` (opman_fun **p.190**) exists precisely for host-controlled brake schemes — *"Use this signal when controlling the electromagnetic brake by the host controller"* — but it is a standard output, not a safety output, and on BLV-R the coil is not brought out separately from the motor cable.

**Mitigating context — and it is substantial.** Your requirements document assumed hold-on-gradient with 600 kg towed. At **0° gradient there is no gravity load on the brake**. The residual holding duty is: resist manual pushing, resist being struck, and hold during load transfer. The consequence of an undetected released brake on a flat floor is a stationary vehicle that can be pushed — materially less severe than the same fault on a ramp. **This is what turns F-02 from blocking into major.**

**Actions.**
1. **Record the justification in the risk assessment**, explicitly: gradient = 0°, no gravity-driven rollaway, holding function assessed at a lower PL, brake not claimed as a Category 3 element. State it — do not leave it implied.
2. **Do not claim SBC or a PL d holding function anywhere in the safety file.** The manual's disclaimer is unambiguous and an auditor will find it.
3. Confirm the RA covers **push and impact** load cases at 0° and that the held brake torque figure satisfies them.
4. If a gradient is ever introduced to the route — even a dock threshold or a ramped doorway — **this finding reverts to blocking** and requires an external safety-switched brake circuit or a separate brake. Make that a documented change-control trigger.

> ⚠️ **Track B caveat.** The 0°-gradient mitigation above is sound for Track A, where you set your own design basis. It does **not** survive contact with an ISO 3691-4 assessor unchanged: certification expects a *declared* maximum design gradient supported by a route survey, plus a parking-brake test at that gradient. "The floor is flat" is an operational assumption, not a design declaration, and real warehouse floors carry drainage falls, door thresholds and dock plates that are rarely 0°. See **F-12** and **§10.4**.

---

### F-03 — 🟠 MAJOR — Two states release the brake with no safety interlock

Two paths release the holding brake outside the safety chain (opman_fun **p.44** state table):

| Path | Behaviour |
|---|---|
| `FREE` input ON | Motor non-excitation **and brake released**. An ordinary direct input — assignable, non-safety-rated, and reachable over RS-485/CANopen (opman_fun p.45) |
| `Alarm (excitation)` | **Brake released** while an alarm is active |

The requirements document asks for a manual release that *"must be deliberate and clearly indicated, and must not be defeatable in normal operation."* `FREE` is none of those things: it is a software-assignable input that a firmware bug, a stray fieldbus write, or a wiring fault can assert, and it releases both drive wheels' brakes on a vehicle with 600 kg behind it.

**Actions.**
1. **Do not assign `FREE` to any direct input** in the running configuration. Leave it unassigned so it cannot be asserted by wiring.
2. Block `FREE` in the fieldbus command map at the vehicle controller. Record it as a forbidden command.
3. Provide the required manual release as a **deliberate physical action** — a keyed switch or a tool-required plug in a documented recovery procedure, interlocked so it cannot be actuated with the vehicle in automatic mode.
4. Enumerate which alarms report as `Alarm (excitation)` and confirm none of them can persist while the vehicle is unattended with the brake released.

---

### F-04 — 🟠 MAJOR — Stopping distance is not repeatable; it varies with battery state of charge

**Evidence.** opman_fun **p.510–511**, "Main power supply input voltage and output torque (output power limiting)": *"If the input voltage to the main power supply is dropped, the output torque is limited."* The 400 W BLVD-KRD curves are plotted at **48 V, 43.2 V, 36 V and 30 V**, and available torque falls materially as the bus sags. Undervoltage alarm fires at **29 V** (opman_fun p.454).

The requirements document requires stopping-distance repeatability *"because the protective-field geometry (ISO 13855) is calculated from it."* Available **braking** torque on the QSTOP ramp is bounded by the same voltage-dependent torque envelope as motoring torque. A pack at 80 % depth of discharge, or sagging under a simultaneous acceleration and steering load, will decelerate more slowly than the pack you measured the distance with.

**Actions.**
1. **Measure stopping distance at minimum operating bus voltage**, not at a full charge. Take that figure as the ISO 13855 input.
2. Set a **vehicle-level minimum operating voltage** well above the 29 V alarm — a cutout that parks the AGV before torque degradation invalidates the measured stopping distance. Derive the threshold from the p.511 curves for the deceleration you need.
3. Monitor bus voltage in the vehicle controller and inhibit automatic mode below the threshold.
4. Verify the FX3 `t_delay` still lets the ramp complete at minimum voltage — a slower ramp needs a longer timer, or the timer will expire mid-deceleration and hand the stop to the brake (F-01).

---

### F-05 — 🟠 MAJOR — No documented regenerative-energy path; the ELV assumption can be transiently violated

**Evidence.** The string "regenerat" appears **zero times** across all three manuals (operating, function, CANopen). There is no brake chopper, no regeneration unit, and no documented braking-energy path. Braking energy returns to the 48 V battery, bounded only by the **overvoltage alarm at 63 V** (opman_fun p.454).

Two consequences:

**(a) BMS interaction.** Deceleration energy at 1.0 m/s with ~900 kg total is roughly:

```
KE = 0.5 × 900 × 1.0²  =  450 J
Over a 0.5 s stop  →  ~900 W total  →  ~450 W per drive  ≈  9.4 A each at 48 V
Over a 0.2 s stop  →  ~2250 W total  →  ~23 A total
```

If the BMS opens the charge FET — full pack, low temperature, or any protection trip — that current has nowhere to go and the bus rises. Overvoltage (22h) puts the drive into **non-excitation immediately** (opman_fun p.454). The controlled ramp is lost mid-stop and the brake takes the whole event: an abrupt, uncommanded stop with 600 kg behind the hitch. **This is the jackknife case the requirements document is trying to prevent, reached by a purely electrical route.**

**(b) The ELV claim.** The requirements document states *"All ELV (<60 VDC) — LVD does not apply."* The drive tolerates the bus up to **63 V** before tripping. Regeneration into a near-full pack can therefore legitimately push the bus **above 60 V** without the drive faulting. The <60 V basis for the ELV / touch-voltage assumption is not guaranteed by the drive's own thresholds.

opman_fun **p.458** confirms this directly and worked in BLVD-KRD terms: the factory overvoltage threshold sits at **63 V** above a 48 V allowable operating voltage, and the diagram's own example lowers it to **55 V** via `Overvoltage alarm (user setting)`. So the fix in action 3 below is a documented, supported adjustment — not a workaround.

**Actions.**
1. Confirm with the battery vendor that the pack accepts the peak regen current above, **at 100 % SoC and at the lowest expected operating temperature**. Get the charge-current limit and the charge-FET behaviour in writing.
2. If the BMS can block charge, **fit a dissipative path** — brake chopper and resistor across the bus, clamping below 60 V. It is not a BLV-R accessory; it is a vehicle-level design item.
3. Set `Overvoltage alarm (user setting)` (opman_fun p.454) **below 60 V** so the drive trips before the ELV boundary is crossed. This preserves the ELV claim at the cost of an earlier fault — the right trade.
4. Verify the DC-DC converters feeding vehicle electronics tolerate the bus excursion up to whatever clamp you set.
5. Re-state the ELV basis in the safety file against the *clamped* voltage, not against nominal.

---

### F-06 — 🟡 MODERATE — No safety-rated protection against left/right configuration divergence

There is no parameter write-protection, no safety-configuration CRC, and no lock on the drive parameter set. `Quick stop rate`, `QSTOP input action`, `Control resolution` and the rest are freely writable over RS-485/CANopen. Two drives with different deceleration rates produce uncommanded yaw under braking — with 600 kg towed, the jackknife case.

Note also the **gear-ratio direction rule**: for the parallel-shaft gearheads a 1:30 ratio reverses output direction relative to the motor shaft (opman p.7), and the hollow-shaft flat gearhead has its own direction convention. Left/right mounting is mirrored, so sign conventions must be verified physically, not assumed.

**Actions.**
1. Maintain a **controlled parameter file per drive** under version control alongside this repo. Treat it as a safety-relevant document.
2. Add a **startup parameter read-back and compare** in the vehicle controller: read the stop-relevant parameters from both drives, compare against the golden file, and refuse automatic mode on mismatch. This is cheap and closes the finding.
3. Add parameter verification to the 3-monthly proof test (§4.4) and to any driver-replacement procedure.
4. Verify direction sign convention on both wheels physically at commissioning, with the gearheads fitted.

---

### F-07 — 🟡 MODERATE — No brake wear or degradation detection

No SBT, no brake-current monitoring, no wear feedback. Brake degradation is silent. The requirements document notes correctly that *"a degraded brake silently erodes the anti-rollaway safety function"* — and after F-01, degradation also erodes the emergency stopping distance, which is the more urgent consequence here.

The manual notes brake disk sliding noise is normal (opman p.7), so audible cues are not a diagnostic.

**Actions.**
1. Define a **periodic brake capability test** — a manual SBT equivalent. Suggested: at each 3-monthly proof test, apply the brake and command a bounded torque against it, confirming no rotation on the safety encoders. The wheel encoders give you the measurement channel for free.
2. Record measured stopping distance at defined intervals and trend it. A lengthening trend is your wear signal.
3. Once Oriental Motor answers F-01, use their stated brake life to set a **replacement interval based on dynamic-stop count**, and count dynamic stops in the vehicle controller.

---

### F-08 — 🟡 MODERATE — Environmental ratings are marginal for the deployment

| Spec | Value | Concern |
|---|---|---|
| Motor IP rating | **IP40** (opman p.26) | No water/dust protection. Drive-wheel motors on a warehouse floor see dust and occasional wet floors. Requires an enclosure; rules out washdown |
| Ambient temperature | **0 to +40 °C** (opman p.26) | Indonesian non-airconditioned warehouses routinely reach 35–38 °C. Inside a closed AGV chassis with 2 × 400 W drives, a 10–15 °C internal rise is ordinary — **the 40 °C limit will be exceeded** |
| Main circuit overheat alarm | **85 °C** (opman_fun p.454) | Trips to non-excitation |
| Motor overheat alarm | **95 °C** (opman_fun p.454) | Non-excitation **after deceleration** |
| Altitude | ≤1000 m | Fine for the deployment |
| Motor–driver cable | **≤3.5 m total, including the motor's own lead** (opman p.25) | Hard layout constraint on driver placement |

A thermal trip is not merely an availability problem: it removes the controlled deceleration path and hands the stop to the brake — F-01 again, on a hot afternoon at the worst moment.

**Actions.**
1. Measure the in-chassis ambient at the motors and drivers during a **worst-case duty cycle in the hottest part of the day** with the full 600 kg towed. Not at idle, and not in the morning.
2. If it exceeds 40 °C, add forced ventilation or derate continuous torque. Do not run outside the datasheet ambient and then rely on the alarm to protect you.
3. Confirm the motor compartment's ingress protection is adequate for the floor conditions given IP40 motors.
4. Verify the 3.5 m cable limit against the actual chassis layout.

---

### F-09 — 🟢 MINOR — Integration checks on the STO wiring

These are verification items, not defects — but each one silently voids the PL d claim if wrong.

| Check | Requirement | Source |
|---|---|---|
| **FX3 test-pulse width** | Safety-output OFF test pulses must be **≤ 1 ms**. A pulse >1 ms trips power removal spuriously; the drive ignores ≤1 ms by design | opman_fun p.212 |
| **HWTO input voltage** | 12–30 VDC. FX3 24 V outputs are compatible | opman_fun p.212 |
| **Jumper removal** | The **included jumpers** linking `+V–HWTO1+`, `HWTO1−–HWTO2+`, and `HWTO2−–0 V` **must be removed** before wiring STO. If left in, STO is defeated and the drive looks entirely normal | opman_fun p.212 |
| **Independent contacts** | *"Provide the contacts individually for operating the HWTO1 input and the HWTO2 input."* Two independent FX3 outputs, not one output split | opman_fun p.212 |
| **EDM electrical fit** | 12–30 VDC, **≤10 mA**, V<sub>sat</sub> ≤2.0 V. Confirm the FX3 input's current draw and polarity against this — 10 mA is a low ceiling | opman_fun p.213 |
| **Minimum OFF/ON time** | HWTO OFF <15 ms may not trigger power removal; ON <15 ms may not release it | opman_fun p.214–215 |
| **`Occur alarm at HWTO input OFF`** | Default **Disable**. Decide deliberately whether an HWTO trip should raise a drive alarm, and record the choice | opman_fun p.219 |

**The jumper item deserves emphasis.** It is the classic BLV-R commissioning failure: the drive ships with STO jumpered out, everything runs perfectly, and the safety function does nothing. Put it on the commissioning checklist as a witnessed step.

---

### F-10 — 🟢 MINOR — Residual rotation on inverter failure is negligible here

opman_fun **p.211**: *"If the inverter circuit is failed, the motor output shaft may rotate up to 180 degrees in an electrical angle (36 degrees in a mechanical angle) even when the power removal function is activated."*

Through the 1:30 gearhead this becomes **1.2° at the wheel = 0.0209 rad**:

```
wheel travel ≈ r_wheel × 0.0209
  r = 75 mm   →  1.6 mm
  r = 100 mm  →  2.1 mm
```

**Not a hazard.** The 1:30 reduction is doing useful safety work here. Record the calculation in the risk assessment against the manual's warning so the closure is visible — that is all this needs.

---

### F-11 — 🟢 MINOR — Gearbox and wheel coupling need ISO 13849-2 fault exclusion

The safety encoders are on the **wheels** — correct, and it covers gearbox faults for *speed sensing*. But the **brake is on the motor shaft, upstream of the 1:30 gearhead**. A gearbox tooth failure, a pinion failure, or a slipping hollow-shaft-to-axle connection means the brake cannot hold or stop the wheel at all, and no electrical diagnostic will see it.

Under ISO 13849-2, mechanical fault exclusion is normally acceptable for a correctly sized industrial gearhead, but it must be **claimed and justified**, not assumed.

**Actions.**
1. Document fault exclusion for the GFS6G30FR gear train and the motor pinion, justified against the permissible-torque data you hold, with an explicit safety factor at the worst-case braking torque — not the running torque.
2. Document fault exclusion for the **hollow-shaft-to-wheel-shaft connection** (key, clamp, or shrink disc). This is the weakest link in the chain and the easiest to get wrong at assembly.
3. Confirm braking torque at the gearhead input stays inside the permissible-torque envelope, including the shock factor for an emergency stop.
4. Add gear-train and coupling inspection to the periodic maintenance schedule.

> **Findings F-12 … F-18 are in §10.5.** They arise only under an ISO 3691-4 certification track and are held separately so this register stays valid for Track A. §10.4 also reassesses the severity of F-01 … F-11 above under that track — **F-11 moves from minor to major**, and F-09 from minor to major.

---

## 7. Vendor documentation checklist

| Document | Status | Reference |
|---|---|---|
| Drive **safety manual** | ✅ **Received** | opman_fun Part 4, "Power removal function", p.209–220 |
| **Functional-safety certificate** (STO) | ✅ **Received** — TÜV SÜD Z10 106467 0002 Rev. 00, valid to 2029-08-12 | certifications-compliance/TUVSUD_Z10_106467_0002.pdf |
| Functional-safety certificate for **SS1 / SBC / SLS** | ❌ **Does not exist** — the certificate lists STO only | — |
| **PFH<sub>D</sub>** per safety function | ✅ **Received** — 2.90×10⁻⁷ /h for STO | opman_fun p.210 |
| **Achieved PL / Category** statement | ✅ **Received** — PL d (Cat 3), SIL 2, conditional on EDM monitoring | opman_fun p.210 |
| Motor datasheet — continuous torque, stall, thermal | ⚠️ **Not in this folder** — manual defers to the Oriental Motor website (opman p.26). Torque-vs-voltage curves are in opman_fun p.510–511 | **Obtain and archive** |
| **Brake specification** — holding torque, spring-applied, response | ✅ **Held by the team** (per project) — not in this folder | **Archive into this folder** |
| Brake **response / reaction time** | ✅ **Held** — and cross-confirmed: ≤35 ms to Hold after HWTO OFF | opman_fun p.214 |
| Drive **STO reaction time** | ✅ **Received** — 15 ms or less | opman_fun p.210 |
| **EMC test report** (drive) | ⚠️ **Only the DoC.** DoC-9205 declares EN 61800-3, EN 55011, EN 61000-6-2/-6-4 — but the **report itself is not here** | **Request from Oriental Motor** |
| Encoder-interface / compatibility spec | ✅ **N/A by architecture** — SIL 3 encoders go to the FX3, not the driver. The driver's internal ABZO encoder is not safety-rated and is not claimed | — |
| **Regen handling specification** | ❌ **Does not exist** — zero mentions across all three manuals. See F-05 | **Vehicle-level design item** |
| Mission time / proof-test interval | ✅ **Received** — T<sub>M</sub> = 20 years; proof test **at least once every 3 months** | opman_fun p.210, p.218 |
| **RoHS declaration** | ✅ **Received** | DoC-9205 p.3 (Certificate of Non-Usage of RoHS Restricted Substances) |
| Gearhead GFS6G30FR data | ✅ **Held by the team** | **Archive into this folder** |
| **Written position on emergency dynamic braking** | ❌ **Missing — highest priority.** See F-01 | **Request from Oriental Motor** |

**Also on file:** UL Recognized Component certificates — UL-US-2127748-0 (UL 62368-1) and UL-CA-2122856-0 (CSA C22.2 No. 62368-1), both report reference E208200-A6005-UL, for BLVD-KRD. Not required for Indonesia, but useful if the product is ever exported to North America. Note these are **Recognized Component**, not Listed — *"incomplete in certain constructional features... intended for installation in complete equipment."*

---

## 8. Screening outcome — §7 of the requirements document

| Criterion | Outcome |
|---|---|
| ❌ No certified STO → reject | ✅ **Passes.** Certified, third-party, PL d / Cat 3, certificate number and issuing body on file |
| ❌ No SS1 → reject unless proven at PL d by other means | ⚠️ **Conditional.** External FX3 SS1-t is a valid construction and the integrity is sound. But the fallback deceleration when the ramp fails depends on prohibited brake use — **F-01 must close** |
| ❌ Enable input presented as STO | ✅ **Passes.** HWTO is a genuine dual-channel hardware inverter-gate shutoff with EDM diagnostics, third-party certified. Not an enable input |
| ❌ No safety manual → reject | ✅ **Passes.** opman_fun Part 4 is a proper safety manual with PFH<sub>D</sub>, conditions of use, verification procedure and proof-test interval |
| ❌ Power-applied brake → reject | ✅ **Passes.** Spring-applied, engages on power loss |
| ⚠️ No SLS → acceptable if safe speed achieved otherwise | ✅ **Acceptable.** FX3 + SIL 3 wheel encoders |
| ⚠️ No SBC → acceptable only if brake controlled safely by other certified means | ❌ **Not satisfied as wired.** Brake is driver-controlled and explicitly disclaimed. Mitigated — not eliminated — by the 0° gradient. See F-02 |

**Screening verdict: KEEP, with two conditions.** Nothing here warrants replacing the motors or drivers. The STO is exactly what the requirements document demanded and it is properly certified. The gaps are architectural and procedural, and both are closable without new hardware — provided F-01 comes back favourably from Oriental Motor.

---

## 9. Priority actions

**Before the protective field geometry can be frozen:**

1. **F-01** — Get Oriental Motor's written position on emergency dynamic braking. Everything about the stopping distance depends on it. *Start this now; vendor turnaround is the long pole.*
2. **F-04** — Re-measure stopping distance at **minimum** bus voltage and set a vehicle-level voltage cutout above 29 V.
3. **F-05** — Confirm BMS charge acceptance at 100 % SoC and low temperature; clamp the bus below 60 V; set `Overvoltage alarm (user setting)` accordingly.

**Before commissioning sign-off:**

4. **F-09** — Witness the STO wiring checklist. **Remove the jumpers.** Verify FX3 test-pulse width ≤1 ms and EDM electrical compatibility.
5. **F-03** — Leave `FREE` unassigned; block it in the fieldbus map; build a proper deliberate manual-release mechanism.
6. **F-06** — Golden parameter files under version control, plus startup read-back and compare across both drives.
7. **F-02** — Write the 0°-gradient justification into the risk assessment explicitly, and register "any gradient introduced to the route" as a change-control trigger that reopens this finding.

**Into the standing maintenance schedule:**

8. **§4.4** — STO verification test **every 3 months**, results recorded. Non-negotiable; the PL d claim depends on it.
9. **F-07** — Brake capability test at the same interval, using the wheel safety encoders as the measurement channel. Trend stopping distance.
10. **F-08** — Thermal survey under worst-case duty in the hottest ambient. Ventilate or derate.
11. **F-11** — Document gearbox and wheel-coupling fault exclusions; add to inspection schedule.

**Housekeeping:**

12. Archive the brake, motor and gearhead datasheets you hold into `certifications-compliance/` so the evidence pack is self-contained.
13. Request the drive EMC test report from Oriental Motor. The whole-vehicle EMC test is what actually closes EMC — the component DoC only feeds it.
14. Diarise the TÜV certificate expiry: **2029-08-12**.

> **Track B adds a second, larger action list.** See **§10.8**. Items 1–3 above stay first in either track — F-01 is on the critical path for both.

---

# 10. Track B — uplift to a real ISO 3691-4 certification

> ⚠️ **Read this caveat first.** Clause numbers and edition designations below are given by **requirement topic**, not by verbatim citation. ISO 3691-4 has had more than one edition (a 2020 first edition, superseded by a later one; EN ISO 3691-4 is the European harmonised version), and clause numbering moved between them. **Pull every clause reference from a controlled copy before you cite it in a technical file.** What is reliable below is the *substance* of what certification demands and how it lands on this drive subsystem — not the numbering.

## 10.1 The framing change: the certified item is the truck

This is the single most important thing to internalise, and it reframes everything in §1–§9.

**You cannot certify a motor and drive to ISO 3691-4.** It is a whole-vehicle standard for driverless industrial trucks. No amount of Oriental Motor paperwork — not the TÜV STO certificate, not the DoC, not the safety manual — certifies your AGV. Those documents are *component evidence that feeds a vehicle-level file*.

Practically:

| | Track A (current) | Track B (ISO 3691-4) |
|---|---|---|
| Certified item | Nothing — internal engineering record | **The AGV** |
| This document's role | The deliverable | **An annex** to the vehicle technical file |
| Who decides "compliant" | You | You (self-declaration) or a third-party body |
| Scope boundary | Drive subsystem only | Whole vehicle — the boundary in §0 dissolves |
| Governing evidence | Vendor datasheets | **Type test reports on your vehicle** |

The consequence: several things this document currently treats as out of scope — the lidar, the FX3, the encoders, stability, the battery, the chassis — come back inside the boundary. This section covers **what ISO 3691-4 demands of the drive subsystem**, and lists the vehicle-level packages compactly so you can see the full shape of the work.

## 10.2 Standards stack you must add

| Standard | Covers | Status |
|---|---|---|
| **ISO 3691-4** | Driverless industrial trucks — the governing standard | ❌ Not yet applied |
| **ISO 3691-1** | General industrial-truck requirements; parts are invoked by Part 4 | ❌ Not yet applied |
| **EN 1175** | Electrical/electronic requirements for industrial trucks | ❌ Not yet applied — **see the note below** |
| **EN 12895** | EMC for industrial trucks — whole vehicle | ❌ Not done (F-16) |
| **ISO 12100** | Risk assessment methodology | ⚠️ RA referenced but not seen |
| **ISO 13849-1** | SRP/CS design, PL calculation | ⚠️ Partially — §4.1 here |
| **ISO 13849-2** | **Validation** — a separate, mandatory activity | ❌ Not done (F-15) |
| **ISO 13855** | Protective equipment positioning / approach speeds | ❌ Not done |
| **IEC 61800-5-2** | Drive safety functions | ✅ Satisfied for STO |
| **ISO 22915 series** | Stability — relevant part for tow tractors | ❌ Not addressed (vehicle level) |
| **ISO 13857 / ISO 13854** | Guarding, reach distances, crushing gaps | ❌ Not addressed |

> **A correction to the requirements document's standards basis.** [agv-motor-drive-stopping-requirements.md](agv-motor-drive-stopping-requirements.md) §5 cites **IEC 60204-1** for overcurrent protection, regen path, protective bonding and IP rating. For industrial trucks that is the wrong instrument: **EN 1175 is the truck-specific electrical standard** and takes precedence over the general machinery electrical standard. IEC 60204-1 is not normally applied to industrial trucks. Under Track B, re-baseline §5 of the requirements document onto EN 1175 — the technical content largely carries over, but an assessor will expect the truck standard to be named. *Verify the applicability statement in the controlled copies.*

## 10.3 What ISO 3691-4 demands of *this* drive subsystem

### (a) Braking — three duties, one component

ISO 3691-4 / EN 1175 distinguish three braking functions, and require braking to remain effective **on loss of the energy supply**:

| Duty | Purpose | On this AGV |
|---|---|---|
| **Service brake** | Normal operational stopping | QSTOP deceleration ramp (non-safety-rated) |
| **Emergency / secondary brake** | Stops the truck on fault or emergency demand | **Motor electromagnetic brake** |
| **Parking brake** | Holds the stationary/unattended truck | **Motor electromagnetic brake** |

Two of the three duties land on a single component, and that component is on the motor shaft **upstream of the 1:30 gearhead**. This is the structural weakness of the architecture — see **F-13**.

It also **hardens F-01 rather than softening it**. Under Track A you could argue the brake's dynamic use is a rare fallback when the SS1 ramp fails. Under ISO 3691-4 the brake is *normatively* an emergency brake: dynamic application at speed is its designated job, not an edge case. Oriental Motor's prohibition (*"Do not use the brake mechanism… for braking the motor rotation"*, opman_fun p.211) then sits in direct conflict with a normative requirement. **You cannot reason your way past that — it needs the vendor statement.**

### (b) Restart after a protective stop

ISO 3691-4 requires that a truck **does not restart automatically** after a protective stop without a defined, deliberate condition. The drive can be configured to violate this — see **F-17**.

### (c) Automatic stop triggers

The truck must stop automatically on loss of guidance, safety-system fault, loss of a relied-upon communication link, and low energy supply. The drive side of this is straightforward (QSTOP + HWTO), but each trigger needs a specified reaction time and a documented path from detection to torque removal. None of this is specified yet — it belongs in the SRS (**F-15**).

### (d) Speed and speed-dependent protective fields

Speed monitoring is already correctly placed (FX3 + SIL 3 wheel encoders). What is missing is the **ISO 13855 calculation record** that turns measured stopping distance into field geometry, using the documented latencies from §5.1 (drive 15 ms, brake ≤35 ms) plus lidar and FX3 response.

### (e) Manual movement of a disabled truck

ISO 3691-4 requires provisions for moving a disabled truck. This is exactly **F-03** — and under Track B the deliberate, non-defeatable manual release stops being good practice and becomes a checked requirement.

## 10.4 Findings that change severity under Track B

| Finding | Track A | Track B | Why it moves |
|---|---|---|---|
| **F-01** Prohibited dynamic braking | 🔴 Blocking | 🔴 **Blocking, and harder to close** | The brake is normatively an emergency brake. The "rare fallback" argument is unavailable |
| **F-02** No SBC | 🔴 Major | 🔴 **Major → approaches blocking** | Flat-floor mitigation weakens; a declared gradient + parking-brake test is expected regardless (F-12) |
| **F-04** Stopping distance not repeatable | 🟠 Major | 🟠 **Becomes a formal type test** | Moves from "measure it carefully" to a witnessed protocol with recorded spread (F-14) |
| **F-05** No regen path / ELV breach | 🟠 Major | 🟠 **Major, now under EN 1175** | Becomes a named clause compliance item, not an engineering judgement call |
| **F-06** Config divergence | 🟡 Moderate | 🟠 **Major** | Assessors treat unprotected safety-relevant configuration as a validation gap |
| **F-07** No brake wear detection | 🟡 Moderate | 🟠 **Major** | Periodic brake performance re-verification becomes a mandatory scheduled activity |
| **F-08** Environmental margins | 🟡 Moderate | 🟡 **Moderate, but binding** | The certified envelope must match the declared operating conditions — you cannot certify to 40 °C and operate at 45 °C |
| **F-09** STO wiring checks | 🟢 Minor | 🟠 **Major** | Becomes witnessed commissioning evidence and part of ISO 13849-2 validation |
| **F-10** Residual rotation | 🟢 Minor | 🟢 Minor | Calculation just needs to be in the file |
| **F-11** Gearbox fault exclusion | 🟢 Minor | 🔴 **Major** | With the brake upstream of the gearbox, fault exclusion is load-bearing for the *braking* function, not just for speed sensing |

**F-11 is the biggest mover.** Under Track A it was a paperwork item about speed sensing. Under Track B, because the brake acts through the gear train, an unexcluded gearbox fault removes the emergency brake and the parking brake simultaneously. Read it together with F-13.

## 10.5 New findings under Track B

### F-12 — 🔴 MAJOR — No declared maximum design gradient or route survey

"Flat floor only" is an **operational assumption**, not a design declaration. ISO 3691-4 certification expects a declared maximum design gradient, supported by evidence, with a **parking-brake hold test performed at that gradient**.

Real warehouse floors are not 0°. Drainage falls of 1–2 % are normal in wash-down and loading areas, door thresholds and expansion joints create short local slopes, and dock plates are steep. An assessor will ask how the truck is prevented from encountering a gradient it was not designed for.

**Actions.**
1. **Survey the actual route with an inclinometer.** Record worst-case local slope including thresholds, joints and any dock approach.
2. **Declare a maximum design gradient with margin** above the surveyed worst case. Do not declare 0° — it is not defensible and it gives you no headroom.
3. Perform and record a **parking-brake hold test at the declared gradient with the full 600 kg towed**, using the brake holding-torque figure you hold as the prediction and the test as the proof.
4. Implement a **means to prevent operation outside surveyed routes**, and document it. This is what makes the declared gradient credible.
5. Re-run **F-02**'s justification against the declared gradient rather than against 0°. If the declared gradient is non-zero, F-02 becomes blocking and needs a safety-switched brake circuit or an independent brake.

### F-13 — 🔴 MAJOR — One brake serves emergency and parking duties, with no independent secondary

The motor's electromagnetic brake is the sole emergency brake **and** the sole parking brake, and it acts through the 1:30 gearhead. A single mechanical failure — brake disc, brake spring, motor pinion, a gear tooth, or the hollow-shaft-to-axle connection — removes **both** duties at once, on that wheel, with no electrical diagnostic.

ISO 3691-4's braking requirements expect braking to remain available under a single fault. On a differential-drive tugger the situation is worse than a simple loss of retardation: losing the brake on *one* wheel while the other holds produces a yaw moment with 600 kg behind the hitch.

**Actions.**
1. Either **document rigorous mechanical fault exclusion** for the full torque path (brake → motor shaft → pinion → gear train → hollow shaft → wheel), per ISO 13849-2, with justified safety factors at emergency-braking torque including shock factor — **or**
2. **Fit an independent brake acting on the wheel or axle downstream of the gearhead.** This is the clean answer. It resolves F-13, removes the gearbox from the braking fault path in F-11, and — if it is a separately-coilable brake — enables a proper safety-switched brake circuit that closes **F-02** as well. **One change closes three findings.**
3. Whichever route: analyse the **asymmetric (single-wheel) brake failure** case explicitly and show the resulting yaw is tolerable or detected.

### F-14 — 🟠 MAJOR — No braking performance type test protocol

Certification requires **measured** braking performance, not calculated. You need a written, repeatable protocol and a witnessed report. Minimum matrix:

| Variable | Conditions to cover |
|---|---|
| Load | Unladen **and** laden at 600 kg towed |
| Speed | Maximum design speed (1.0 m/s) |
| Direction | Forward and reverse |
| Floor | Worst-case surface in scope — including dusty and, if in scope, wet |
| Bus voltage | **Minimum operating voltage**, per F-04 — not full charge |
| Repeats | Enough to establish spread, typically ≥5 per condition |
| Modes | Service stop (QSTOP ramp) **and** emergency stop (HWTO drop) separately |

The emergency-stop rows are the ones that feed ISO 13855. The spread — not the mean — sets the protective field.

**Actions.** Write the protocol, agree it with your certification body *before* testing, instrument with the existing wheel encoders, and record raw data. Re-run after any change to brake, tyre, mass or speed.

### F-15 — 🟠 MAJOR — No Safety Requirement Specification or ISO 13849-2 validation

ISO 13849 is two standards and you have engaged with one. **Part 1 is design and PL calculation; Part 2 is validation, and it is mandatory.** Missing:

1. **Safety Requirement Specification**, per safety function — trigger, response, required PL, reaction time, behaviour on fault, reset condition. Functions to specify at minimum: protective stop (SS1), emergency stop, safe speed limiting, anti-rollaway/parking, and safe direction if used.
2. **SISTEMA project file** with real component data — the §4.1 SICK figures are placeholders and must be replaced.
3. **Validation plan and report** per ISO 13849-2, including **fault injection**: open each HWTO channel independently, short channels together, disconnect EDM, and confirm the FX3 latches correctly in every case.
4. **Fault-exclusion register** with justifications (feeds F-11, F-13).

This is a genuine work package, not a document to backfill in an afternoon. Budget for it.

### F-16 — 🟠 MAJOR — Whole-vehicle EMC to EN 12895 not performed

DoC-9205 declares the *drive* to EN 61800-3, EN 55011 and EN 61000-6-2/-6-4. That is component evidence. The vehicle needs testing to **EN 12895**, and the drives are the dominant emitter on it.

Also required: demonstration that the **safety functions do not fail under EMI**. Helpfully, the TÜV certificate already cites **IEC/EN 61000-6-7** (immunity for equipment intended to perform functions in a safety-related system) for the drive — good evidence to carry into the file, but it must be complemented at vehicle level with the real cable runs, the real ground topology and the lidar and FX3 in place.

**Actions.** Book vehicle EMC early — slots are long-lead and failures need redesign time. Request the drive's underlying EMC **test report** (not just the DoC) from Oriental Motor to support the vehicle campaign.

### F-17 — 🟠 MAJOR — Restart after a protective stop can be made automatic by configuration

opman_fun **p.219**: the `ETO reset action (ETO-CLR)` parameter accepts **1: ON edge (positive edge)** — the default — or **2: ON level**.

If set to **2** and the vehicle controller holds ETO-CLR asserted, the drive re-excites **automatically** the moment HWTO returns. That is an automatic restart after a protective stop, which ISO 3691-4 prohibits without a deliberate condition.

It gets broader. opman_fun **p.44** footnote: *"If the parameter is changed, the 'ETO' status can be released by the ALM-RST input, the S-ON input, or the STOP input."* So there are **four** configurable paths out of ETO, any of which could be driven continuously by the controller.

**Actions.**
1. **Lock `ETO reset action` to 1 (ON edge)** and place it in the golden parameter file (F-06) with startup verification.
2. Confirm ETO is **not** configured to clear on ALM-RST, S-ON or STOP.
3. Drive ETO-CLR from a **deliberate operator acknowledgment**, never from an automatic sequence. Verify at the vehicle controller that no code path asserts it unconditionally after a field clears.
4. **Test it explicitly**: trigger a protective stop, clear the obstruction, and confirm the truck does **not** move until acknowledged. Put this in the validation plan (F-15) — it is a favourite assessor check.

### F-18 — 🟡 MODERATE — Information for use, marking, and residual-risk traceability

Certification requires an instruction handbook, capacity/identification marking, and a maintenance schedule — and an assessor will **trace every vendor warning to a mitigation**. That includes the manual's own warnings quoted throughout §6: the standstill-before-STO note (p.210), the 36° residual rotation (p.211, closed by F-10), the brake prohibition (p.211, F-01), and the "not safety-related parts" disclaimers (p.214–215, F-02).

**Actions.** Build a residual-risk traceability matrix mapping each vendor warning → hazard → mitigation → evidence. Publish the maintenance schedule including the 3-monthly STO verification (§4.4), the brake capability test (F-07), and gear-train inspection (F-11).

## 10.6 Type tests required on the vehicle

| Test | Feeds | Status |
|---|---|---|
| Braking performance — service and emergency, laden/unladen | ISO 13855 field geometry | ❌ F-14 |
| Parking brake hold at declared gradient, laden | Anti-rollaway claim | ❌ F-12 |
| Protective field verification with the actual truck | Personnel detection | ❌ |
| Stopping distance at minimum bus voltage | F-04 | ❌ |
| Restart-inhibit after protective stop | ISO 3691-4 restart rule | ❌ F-17 |
| STO fault injection — each channel, EDM open | ISO 13849-2 validation | ❌ F-15 |
| Whole-vehicle EMC to EN 12895 | EMC compliance | ❌ F-16 |
| Thermal survey at worst-case duty and ambient | Declared operating envelope | ❌ F-08 |
| Stability per ISO 22915 (relevant part) | Vehicle level | ❌ |

## 10.7 Certification route

**Driverless industrial trucks are not Annex IV machinery**, so in the EU a manufacturer may self-declare against harmonised EN ISO 3691-4 and compile a technical file — no notified body is legally required. In practice, customers, insurers and integrators frequently demand a **voluntary third-party type examination** (TÜV SÜD, TÜV Rheinland, DEKRA, SGS, Bureau Veritas). Given the words "real certification", assume the third-party route.

What a body will ask for, in roughly this order:

1. Risk assessment to ISO 12100, with the AGV's operating environment and route defined
2. Safety Requirement Specification per safety function (F-15)
3. SRP/CS architecture, SISTEMA file, PFH<sub>D</sub> per function with real component data
4. Component certificates and safety manuals — **you already hold these for the drive**
5. ISO 13849-2 validation plan and report, incl. fault injection (F-15)
6. Type test reports (§10.6)
7. ISO 13855 protective-field calculation with measured stopping distances
8. EMC test report to EN 12895 (F-16)
9. Instruction handbook, marking, maintenance schedule (F-18)
10. Change-control and configuration management evidence (F-06)

**Engage the body before testing, not after.** Agreeing the test protocols up front is the difference between one test campaign and three.

## 10.8 Track B work package summary

**Critical path — start immediately:**

1. **F-01** — Oriental Motor written position on emergency dynamic braking. *Longest lead item in either track, and under Track B there is no fallback argument if the answer is restrictive.*
2. **F-13 decision** — independent wheel/axle brake, or documented fault exclusion. Fitting a separate brake closes F-13, F-11 and F-02 together and de-risks F-01. **Make this decision early**; it is the highest-leverage change available and it is far cheaper now than after the chassis is finalised.
3. **F-12** — Route survey and declared design gradient. Gates F-02's justification and the parking-brake test.
4. Engage a certification body and agree test protocols.

**Design and build:**

5. **F-16** — Book vehicle EMC (long lead).
6. **F-05** — Regen clamp below 60 V under EN 1175; `Overvoltage alarm (user setting)` reduced.
7. **F-17** — Lock ETO reset to ON edge; deliberate acknowledgment for restart.
8. **F-03** — Non-defeatable manual release for disabled-truck movement.

**Documentation — the largest single package:**

9. **F-15** — SRS, SISTEMA with real data, ISO 13849-2 validation plan and report, fault-exclusion register.
10. Re-baseline requirements-document §5 from IEC 60204-1 onto **EN 1175** (§10.2).
11. **F-18** — Instruction handbook, marking, residual-risk traceability matrix.
12. **F-06** — Configuration management with golden parameter files and startup verification.

**Test campaign:**

13. **F-14** — Braking performance protocol and execution across the full matrix.
14. Remaining type tests per §10.6.

**Honest assessment of the gap.** Track A is a handful of vendor questions, some wiring discipline and a maintenance schedule. **Track B is a project.** The drive hardware itself is not the obstacle — the STO is properly certified and the PFH<sub>D</sub> budget works. What is missing is the vehicle-level engineering evidence: specification, validation, and measured type tests. Expect the documentation and test packages to dominate the effort, and expect **F-01 and F-13 to determine whether the current single-brake architecture survives at all.** If the F-13 decision goes toward an independent wheel brake, most of the hard findings in this document resolve together.

---

*Assessment covers the motor, gearhead, drive amplifier and brake subsystem only for §1–§9; §10 extends to the vehicle boundary where ISO 3691-4 requires it. For §1–§9, stop commands originate from the FX3 safety controller and are outside scope; under Track B the FX3, lidar, encoders, battery and chassis all come back inside it. Page references are to the printed page numbers of the Oriental Motor manuals in this repository: `opman/` (BLV Series R Type Motor, HM-5308-3), `opman_fun/` (Function Edition, HP-5142-6), `opman_can/` (CANopen Communication Profile). PFH<sub>D</sub> values for non-Oriental-Motor subsystems in §4.1 are indicative placeholders — substitute the figures from your actual SICK datasheets before running SISTEMA. **ISO 3691-4, ISO 3691-1 and EN 1175 requirements in §10 are stated by topic, not by verbatim clause citation — edition and numbering must be confirmed against controlled copies before anything from §10 enters a technical file.** Reconfirm all clause references against controlled copies of the standards before design freeze.*
