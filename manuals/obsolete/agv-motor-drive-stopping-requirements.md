# AGV Motor & Drive — Stopping Functions and Compliance Requirements

**Scope:** the drive subsystem only — motor, drive amplifier/controller, holding brake, and their integrated safety functions.
**Out of scope:** safety lidar, safety encoder, safety PLC, navigation. Those form the safety controller that *commands* the stop; this document covers whether the motor/drive can *execute* it to the required integrity.

**Machine context:** tow-tractor AGV, differential drive (2 independently driven + braked wheels), 0.2–1.0 m/s, up to 600 kg towed, unfenced pedestrian area, target **PL d / Category 3**.

**How to use:** check each row against the candidate motor/drive datasheet and safety manual. A "yes" needs a **certificate or safety manual statement**, not a marketing claim.

---

## 1. Stop categories — IEC 60204-1 (what kind of stop)

These define *how* power is removed. Terminology comes from IEC 60204-1 §9.2.2.

| Stop category | Behaviour | Needed on this AGV? |
|---|---|---|
| **Category 0** | Immediate removal of power to the actuator — uncontrolled coast/stop | ✅ Required — final state of the E-stop chain |
| **Category 1** | Controlled stop with power retained to achieve the stop, **then** power removed | ✅ Required — primary stop; controlled deceleration then power removal |
| **Category 2** | Controlled stop with power **retained** to the actuator after stopping | ⚠️ Optional — only if a powered-hold operational stop is used |

**Design intent for this machine:** protective stops are **Category 1** (controlled deceleration, then power removal + brake hold). Emergency stop resolves to **Category 0 or 1** per risk assessment. Category 0 alone is generally unsuitable as the sole protective stop on a tugger — an uncontrolled coast with 600 kg towed risks jackknife/trolley over-run.

---

## 2. Drive safety functions — IEC 61800-5-2 (what the drive must implement)

These are the certified safety functions the drive amplifier must provide. Check the drive's safety manual for each.

| Function | Meaning | Required? | Notes for this AGV |
|---|---|---|---|
| **STO** — Safe Torque Off | Power that can cause rotation is removed; no torque | ✅ **Mandatory** | The baseline. Corresponds to Stop Category 0. Must be certified, not just "enable input" |
| **SS1** — Safe Stop 1 | Controlled deceleration, then STO after delay or below speed threshold | ✅ **Mandatory** | Corresponds to Stop Category 1 — the primary protective stop |
| **SS2** — Safe Stop 2 | Controlled deceleration, then Safe Operating Stop (position held with power) | ⚠️ Optional | Only if powered hold is used instead of the mechanical brake |
| **SOS** — Safe Operating Stop | Holds position, motor energised | ⚠️ Optional | Pairs with SS2 |
| **SLS** — Safely Limited Speed | Speed kept below a safe limit | ✅ **Strongly recommended** | Supports speed-dependent protective fields; if speed selects the field, that speed is part of a PL d function |
| **SDI** — Safe Direction | Motion permitted in one direction only | ⚠️ Optional | Useful for no-reverse zones or gradient anti-rollback |
| **SBC** — Safe Brake Control | Safe control of an external holding brake | ✅ **Required if a fail-safe brake is used** | Needed for anti-rollaway and hold-on-gradient |
| **SBT** — Safe Brake Test | Periodic automated brake capability test | ⚠️ Recommended | Provides diagnostic coverage for the brake; supports Cat 3 DC |
| **SSM** — Safe Speed Monitor | Signals when speed is below a set limit | ⚠️ Optional | Useful as a status output to the safety controller |
| **SLA / SAR** — Safely Limited Acceleration / Safe Acceleration Range | Bounded acceleration/deceleration | ⚠️ Optional | Relevant to towed-load stability and jackknife control |

**Minimum acceptable set for this AGV: STO + SS1 + SBC, with SLS strongly recommended.**

---

## 3. Integrity requirements (ISO 13849-1 / IEC 62061)

| Requirement | Target | What to check in the documentation |
|---|---|---|
| Performance Level | **PL d** (unless RA raises it) | Safety manual states achieved PL per function |
| Category | **Category 3** | Single-fault tolerance; a single fault must not cause loss of the safety function |
| PFH<sub>D</sub> | Stated value per safety function | Needed for the SISTEMA calculation |
| SIL (if declared to 62061) | **SIL 2** equivalent | Cross-reference to PL d |
| Certification | Third-party certificate (TÜV or equivalent) | Certificate number + issuing body, not self-declaration |
| Safety manual | Must exist | Mandatory — contains conditions of use, PFH<sub>D</sub>, fault-exclusion assumptions, wiring requirements |
| Mission time / T<sub>M</sub> | Typically 20 years | Affects PL calculation validity |
| Proof-test / diagnostic interval | Stated | Some functions require periodic testing to retain the claimed PL |

**Both drives must be identical in safety configuration** — asymmetric drive/brake behaviour between the two wheels causes uncommanded yaw and, with 600 kg towed, jackknife risk. Symmetry is a safety requirement, not a convenience.

---

## 4. Braking and holding requirements (ISO 3691-4 / EN 1175)

| Requirement | Notes |
|---|---|
| **Service brake** | Controlled deceleration to a stop within the calculated stopping distance |
| **Fail-safe holding brake** | Spring-applied, electrically released — must engage on loss of power. Not a power-applied brake |
| **Anti-rollaway / hold on gradient** | Must hold the vehicle **plus 600 kg towed load** on the maximum design gradient, indefinitely, unpowered |
| **Brake sizing basis** | Worst case = max towed mass × max gradient × dynamic factor — not vehicle mass alone |
| **Automatic brake application** | Brake must apply automatically on: E-stop, power loss, safety-function trip, guidance loss, controller fault |
| **Stopping distance repeatability** | Must be consistent — the protective-field geometry (ISO 13855) is calculated from it |
| **Brake wear / degradation detection** | Recommended (SBT) — a degraded brake silently erodes the anti-rollaway safety function |
| **Manual brake release** | Required for recovery/pushing a disabled vehicle; must be deliberate and clearly indicated, and must not be defeatable in normal operation |

---

## 5. Motor and drive electrical / general requirements

| Requirement | Standard | Notes |
|---|---|---|
| Voltage class | — | 48 VDC nominal, ≤ ~52 VDC. All ELV (<60 VDC) — LVD does not apply |
| Overcurrent / thermal protection | IEC 60204-1 | Motor thermal protection; drive overload protection |
| Regenerative braking energy path | IEC 60204-1 | Defined path for braking energy — battery charge-current limit or brake chopper. Bus must not over-volt into the DC-DC converters |
| Bidirectional current capability | — | Drive and its branch protection must handle regen current direction |
| EMC | EN 12895 / EN IEC 61000-6-2/-6-4 | Drives are the dominant EMC source. Component certificate collected; **whole-vehicle test is what closes it** |
| Protective bonding | IEC 60204-1 | Motor frame and drive chassis bonded to the equipotential star point, paint-free lands |
| IP rating | IEC 60204-1 | Appropriate to the compartment / indoor industrial environment |
| Enclosure / guarding of rotating parts | ISO 13857 / ISO 13854 | Drive wheels and shafts guarded; openings sized per reach tables |
| Ambient temperature / duty rating | Datasheet | Continuous duty at max towed load, not peak rating |
| Encoder interface | — | Drive must accept the safety-rated encoder; **coupling is a fault-exclusion item** (mechanical, ISO 13849-2) |

---

## 6. Vendor documentation checklist

Request these for the candidate motor + drive. Missing items are gaps, not formalities.

| Document | Purpose | Received? |
|---|---|---|
| Drive **safety manual** | The load-bearing document — PL/SIL, PFH<sub>D</sub>, conditions of use, wiring | ☐ |
| **Functional-safety certificate** (TÜV or equivalent) for STO/SS1/SBC/SLS | Third-party proof, with certificate number | ☐ |
| **PFH<sub>D</sub> values** per safety function | SISTEMA input | ☐ |
| **Achieved PL / Category** statement per function | Confirms PL d / Cat 3 | ☐ |
| Motor datasheet — continuous torque, stall, thermal | Gradient + towed-load sizing | ☐ |
| **Brake specification** — holding torque, spring-applied confirmation, response time | Anti-rollaway sizing | ☐ |
| Brake **response / reaction time** | Stopping-distance calculation | ☐ |
| Drive **STO reaction time** | Stopping-distance calculation | ☐ |
| **EMC test report** (drive) | Feeds vehicle EMC campaign | ☐ |
| Encoder-interface / compatibility spec | Safe-speed feedback integration | ☐ |
| Regen handling specification | Braking energy path | ☐ |
| Mission time (T<sub>M</sub>) and proof-test interval | PL validity | ☐ |
| RoHS declaration | Machine RoHS file | ☐ |

---

## 7. Quick pass/fail screening

Use this to reject unsuitable candidates fast:

- ❌ **No certified STO** → reject. Non-negotiable.
- ❌ **No SS1** → reject, or the controlled stop must be proven achievable another way at PL d.
- ❌ **"Enable input" presented as STO without a certificate** → reject. An enable input is not a safety function.
- ❌ **No safety manual** → reject. Without PFH<sub>D</sub> and conditions of use the PL d calculation cannot be completed.
- ❌ **Power-applied (non-fail-safe) brake** → reject for holding/anti-rollaway duty.
- ⚠️ **No SLS** → acceptable only if speed-dependent protective fields are achieved by other safe-speed means.
- ⚠️ **No SBC** → acceptable only if the brake is controlled safely by other certified means.

---

*Scope note: covers the motor, drive amplifier, and brake subsystem only. Stop commands originate from the safety controller (FX3) fed by the safety lidar and safety encoder — outside this document. Numeric values (stopping distance, gradient, deceleration, holding torque) are project load cases from the risk assessment, not standard constants. Reconfirm all clause references against controlled standard copies before design freeze.*
