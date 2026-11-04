# Emergency suite plan (MCQ4-4)

Branch `kyb-trackless` · 2026-10-30 · plan only, nothing implemented

## 1. Goal

When the safety PLC signals an emergency, the controller must **know** it, rather than infer it from the drives losing torque. It reacts in the same predictable way in every mode, shows it on every web page and on the panel outputs, and leads the operator through recovery: **release the E-stop, then press Reset, then press Start.**

The safety function itself does not change. The Flexi Soft still takes STO (HWTO1/HWTO2) in hardware, independent of the PC. This plan only makes the PC's reaction, its messages and its recovery correct. Nothing here may delay, mask or replace the hardware stop.

## 2. The signal

| Input | Level | Meaning |
|---|---|---|
| DI08, Flexi Soft Q3 (`io-mapping.md` §2) | HIGH | no emergency |
| | LOW | emergency |
| DI02 PB RESET | rising edge | wired to Flexi Soft I8 **and** to DI02, so one press resets the PLC and the PC sees it too |

**Fail-safe:** LOW, an invalid panel image and a stale DIO image all count as EMERGENCY. A cut wire or a dead DIO island must never read as "OK".

### Bench questions to settle first (they change the state machine, §3)

| # | Question | Why it matters |
|---|---|---|
| Q1 | Does Q3 stay LOW after the E-stop is released, until Reset (latched in the PLC logic), or go HIGH as soon as it is released? | Latched means the PC cannot tell "still pressed" from "released, waiting for Reset". The screen then shows one combined instruction. |
| Q2 | Is Q3 the **E-stop only**, or the whole safety chain (E-stop + scanner OSSD + EDM)? | If OSSD drives it, a protective-field stop also reads as EMERGENCY, and the auto-resume after a field stop would break. Q3 must be the E-stop (and EDM) only. Otherwise the Flexi Soft logic needs changing first. |
| Q3 | After Reset, how long until the drives leave "Switch on disabled"? | Sets the "resetting…" timeout before RESET_FAILED is raised. |
| Q4 | Does a Reset press while the E-stop is still pressed do anything in the PLC? | Decides whether a premature Reset only needs a hint, or an alarm. |

Test: press and release the E-stop, press Reset, and log DI08, DI02 and both drives' statuswords at 20 Hz. `candump` plus a read-only Modbus poll is enough.

## 3. Controller state: one shared "emergency" picture

A new pure module `amr_base/amr_base/emergency.py`, clock-fed and table-tested like `arm_policy`, owns the states. Everything else reads its output.

```
            DI08 LOW (or panel invalid/stale)
  NORMAL ─────────────────────────────────────▶ EMERGENCY
    ▲                                              │ DI08 HIGH (released; with a latched Q3
    │ drives leave "Switch on disabled"            │ this only happens after Reset)
    │ (or no drive power is wanted)                ▼
    │                                         RELEASED_NEEDS_RESET ──(timeout, Q3)──▶ RESET_FAILED
    └──────────── Reset edge, DI08 HIGH ────── RESETTING
```

- **EMERGENCY:** "EMERGENCY STOP. Release the E-stop button, then press Reset."
- **RELEASED_NEEDS_RESET:** "E-stop released. Press the blue Reset button." With a latched Q3 (Q1) this state is only visible to the PC once the drives show STO still held after DI08 went HIGH.
- **RESETTING:** a short window after the Reset edge while the drives come out of STO. Its timeout is set from the Q3 measurement.
- **NORMAL:** cleared. A run that was stopped still needs **Start**; nothing moves on Reset alone (§4.4).

The state is published so every consumer sees the same picture. Two options:

- **(a) Recommended:** new fields in `PanelState`: `bool estop_ok` (debounced DI08, False while invalid) and `uint8 emergency` (the state above).
- **(b)** A separate `EmergencyState` message.

(a) keeps one panel image and one freshness rule. It is an interface change, so it needs a clean rebuild and a full `amr.service` restart (Humble has no type hash, removal plan R6).

## 4. Behaviour per layer

### 4.1 Panel (`panel_io.PanelAdapter`, `panel_node`)

- Read DI08 as a debounced level, like DI09: `panel.di_estop_ok: 8`, a new profile key.
  - Debounce only the HIGH direction: going to EMERGENCY must be immediate (one scan), and leaving it must persist for `debounce_scans`.
  - Amended 2026-11-02: LOW is now filtered too, by `panel.estop_low_scans` (2 on agv-01). Q3 test pulses read as single LOW scans every 400 ms in bursts and kept powering the drives off. An invalid image or a comms gap is still an emergency at once. STO is the Flexi Soft's and is not delayed.
  - Add `config._validate` rules: the channel is distinct from the panel/pendant/arm channels, and is required while `panel.enabled`.
- Run `emergency.py`, set `PanelState.estop_ok` / `emergency`, and log every transition.
- **Outputs during EMERGENCY:**
  - DO00 rotary lamp: flashing (1 Hz) or steady, operator to choose.
  - DO01 horn: off.
  - DO02 alarm horn: a short burst on entry (e.g. 2 s), then off, so a held E-stop does not sound forever. Operator to choose.
  - Outside EMERGENCY, today's `motion_on` / `alarm_on` logic is unchanged.

### 4.2 Drive power (`arm_policy`, `drive_node`)

- `want_power`: anything other than NORMAL means **no power wanted**, checked before the arm and mode rules. Reason text "E-stop (DI08)" / "waiting for Reset".
- That ends today's re-engage loop, which fails against STO every 2 s, raises SAFETY_RESET_NEEDED and floods the log.
- The drives are released (zero target, Shutdown) to STANDBY. STO has already cut torque; the PC is only catching up.
- After NORMAL the drives re-engage at a **zero target** only if power is wanted (DI09 high, plus a run asking for power in AUTO), exactly as after boot.

### 4.3 Command mux (`gating.select`, `cmd_mux_kinematics_node`)

- `gating.Panel` gains `estop_ok`. While not NORMAL, `select()` returns `NONE` with `code="ESTOP"`, inhibited, ahead of every source.
- Reset the slew and S-curve state to zero (`_v = 0`, `_a = 0`, yaw too) while inhibited. Otherwise the first command after Reset would ramp from the pre-E-stop speed, which is a jump.
- **Pendant held through Reset:** nothing moves until every pendant button and the jog have been released once after NORMAL (a re-press rule). This is the same idea as "no motion on Reset".

### 4.4 Run executor (`route_executor_node`)

- **EMERGENCY during a run:** hold with cause `estop` straight from `PanelState`, with no torque guessing. Keep the torque inference as the fallback when the panel is stale.
- **Start while not NORMAL:** refuse it outright, with no pre-move lamp and horn and no 8 s wait. Reason "E-stop active - release it and press Reset first".
- **Recovery:** EMERGENCY → NORMAL leaves the run BLOCKED(estop). The operator then presses **Start**, the pre-move warning runs (lamp + DO02 for 2 s), and the run resumes. `auto_resume_estop` stays `false` and is refused in the profile on MCQ4-4.
- Physical Reset keeps its present job (acknowledging an executor FAULT at rest) and gains none: it never resumes a run.

### 4.5 Supervisor (`supervisor_node`, `mode_fsm`)

- No mode change on EMERGENCY. NAVIGATION stays NAVIGATION, so the map, the localisation and the loaded route survive and the run can resume after Reset + Start.
- The lease is withheld while not NORMAL (`_allowed()` returns 0). This adds a second, independent no-motion layer above the mux.
- Refuse a survey start and mode changes that would move the vehicle while not NORMAL.

### 4.6 Survey moves

- Abort an active survey move on EMERGENCY. Never resume it; it must be started again. (Commissioning and pp moves were removed with the commissioning page, 2026-10-30.)
- The blind run, restored 2026-11-02 on `/blindrun` (PV only), is treated the same way: an EMERGENCY aborts a running job, never to resume (owner, 2026-11-02). A single measured move cut by a stop measures the stop.

## 5. Web app

### 5.1 Data

- The adapter puts `emergency: {state, estop_ok, since_s, reset_seen}` into `state()`, from `PanelState`.
- `alarms.standing()` puts the matching row **first**, ahead of everything (see §6).
- When the panel is stale the row is PANEL_STALE, as today, not a false "emergency".

### 5.2 What every page shows

A **full-width red banner** pinned under the nav, in `base.html`, on every page including Manual, Run, Maps, IO and Status. It shows the state, one numbered instruction list and the time since the stop:

```
 ■ EMERGENCY STOP                                      since 00:42
   1. Make the area safe.
   2. Release (twist) the E-stop button.
   3. Press the blue RESET button on the cabinet.
   4. Press START to continue the run.      (shown only if a run was stopped)
```

- Each step is ticked when the PC can see it:
  - step 2 when DI08 goes HIGH (needs a non-latched Q3, §2 Q1)
  - step 3 on the Reset edge and the drives leaving STO
  - step 4 after Start
- **RESET_FAILED:** "Reset did not take - check the E-stop is released and that the safety circuit is closed, then press Reset again. If it repeats, call the engineer."
- The banner clears only at NORMAL. A run waiting for Start then shows the ordinary "STOPPED (needs you): press Start" headline.
- Kiosk 1280×720: the banner must not push the jog pad below the fold. Shrink the pad, as was done for `--lt-h`.

### 5.3 Page specifics

- **Home:** the headline becomes **EMERGENCY STOP** (a new headline level above NEEDS SERVICE).
- **Manual:** the jog pad is greyed with "E-stop active". Web jog presses are refused server-side too: `/api/jog` returns 409 with the reason, and the mux inhibit is the real guard.
- **Run:** the Start/Resume buttons are disabled, with the same reason. The hold name `estop` already maps to "E-stop" in `run.js`.
- **IO page:** DI08 is already named "E-stop OK (safety PLC)". Show it red when LOW, and add the derived state next to it.
- **Alarms / event history:** one event per transition: entered, released, reset, reset failed, cleared, with timestamps.

## 6. Alarm catalogue (`agv_core/alarms.py`, RUNBOOK §4 regenerated)

| Code | Severity | Clears by | Title | Action |
|---|---|---|---|---|
| `ESTOP` (reworded) | ERROR | `reset_button` (new CLEARS value) | Emergency stop | Release the E-stop button, then press Reset on the cabinet. |
| `ESTOP_RESET_NEEDED` (new) | WARN | `reset_button` | E-stop released: press Reset | Press the blue Reset button on the cabinet. |
| `ESTOP_RESET_FAILED` (new) | ERROR | `engineer` | Reset did not take | Check the E-stop is released and press Reset again; call the engineer if it repeats. |
| `SAFETY_RESET_NEEDED` (kept) | WARN | `reset_button` | Motors off: safety circuit needs a reset | Shown only when DI08 is HIGH but the drives sit in "Switch on disabled" (a field or EDM trip), so it no longer doubles as the E-stop message. |

The test `test_every_code_emitted_by_a_node_has_a_catalogue_row` keeps this honest.

## 7. Tests

Pure and table-driven where the logic lives:
- `test_emergency.py`, every transition:
  - a LOW glitch enters at once
  - HIGH needs N scans
  - a Reset edge with DI08 still LOW does nothing
  - an invalid or stale panel counts as EMERGENCY
  - the timeout reaches RESET_FAILED
- `test_arm_policy.py`: no power while not NORMAL, in both modes, whatever DI09 and the run say.
- `test_gating.py` / `test_mux_*`: ESTOP inhibits every source, the ramp state is zeroed, and the pendant must be released once after NORMAL.
- Executor:
  - hold `estop` from the panel field
  - Start refused while not NORMAL, with no pre-move
  - Reset alone never resumes
  - Start after NORMAL resumes with the pre-move warning
- `test_alarm_horn.py`: the output pattern in EMERGENCY.
- Web:
  - the banner on every page (`test_operator_ux`)
  - jog/start refused with 409
  - `standing()` order: EMERGENCY first, PANEL_STALE rather than a false emergency
- `test_config.py`: `panel.di_estop_ok` is required and distinct, and `auto_resume_estop: true` is refused on this profile.
- Sim:
  - `fake_panel_node` gets an E-stop toggle
  - `test_unified_sim` drives a route, trips the E-stop mid-route, and checks that the vehicle holds and the banner text is right
  - Reset alone does not move; Reset then Start resumes

## 8. On-vehicle acceptance (witnessed, wheels on the ground, area clear)

1. Idle, MANUAL: press the E-stop. The banner shows on every page within 0.2 s, and the jog does nothing. Release the E-stop and press Reset. The banner clears, and the jog works again only after the pendant has been released once.
2. MANUAL, jogging with the pendant held: press the E-stop. It stops on STO. Keep the pendant held through release and Reset; the vehicle must **not** move.
3. AUTO, mid-route: press the E-stop. The run is BLOCKED(estop) and the banner appears. Reset only: nothing moves. Press Start: the 2 s lamp + DO02 warning, then the run resumes.
4. Start pressed with the E-stop still in: refused at once, with no pre-move warning.
5. Reset pressed with the E-stop still in: nothing changes, and the banner keeps step 2.
6. Unplug DI08's wire: EMERGENCY (fail-safe). Unplug the DIO Ethernet: PANEL_STALE, no motion.
7. Protective field stop (if §2 Q2 shows OSSD is not on Q3): the field auto-resume still works and **no** E-stop banner appears.

## 9. Order of work

1. Settle the bench questions (§2), and change the Flexi Soft Q3 logic if Q2 needs it. The software design depends on both.
2. `emergency.py` + tests, the profile key + config rules.
3. `PanelState` fields, then `panel_io` / `panel_node`, plus the output pattern.
4. `arm_policy`, the mux inhibit and ramp reset, the supervisor lease.
5. Executor (hold, refuse Start, recovery).
6. Alarm catalogue, RUNBOOK §4.
7. Web: adapter, `standing`, headline, the `base.html` banner, the page specifics.
8. Clean rebuild and full service restart, then the §8 acceptance.

## 10. Owner decisions

| # | Question | Recommended |
|---|---|---|
| E1 | DO00 lamp during EMERGENCY: steady or 1 Hz flash? | 1 Hz flash (distinct from "moving") |
| E2 | DO02 alarm horn on entry: duration? | 2 s burst, then silent |
| E3 | After Reset, should an AUTO run resume with Start, or be aborted? | Resume with Start (keeps the map/route; Start is a deliberate act) |
| E4 | Should the web offer a "Reset" button? | **No.** Reset stays the physical cabinet button only; the PLC needs it anyway, and a remote reset is a restart from out of sight |

### Owner answers (2026-10-30), these override the text above

- **E1 / E2:** no lamp and no alarm horn in an emergency. **Every DIO output is held LOW** while the state is not NORMAL (DO00 lamp, DO01 horn, DO02 alarm horn, any `motion_on`/`alarm_on`/`horn` coil).
- **E3:** an emergency **aborts** the run, in any state including pre-move. The route is cleared, nothing resumes, and a new run needs a deliberate Load/Start after NORMAL. The executor therefore no longer *holds* with cause `estop` when the panel says EMERGENCY; it aborts with reason "emergency stop".
- **E4:** physical Reset only. No web Reset button, no reset API.
- **Contract (landed first):**
  - `PanelState.estop_ok`, `PanelState.emergency` with `EMG_NORMAL/ACTIVE/RESET_NEEDED/RESETTING/RESET_FAILED = 0..4`
  - profile `panel.di_estop_ok: 8`, `panel.estop_reset_timeout_s: 3.0`
  - alarm codes `ESTOP` (ERROR, clears_by `reset_button`), `ESTOP_RESET_NEEDED`, `ESTOP_RESET_FAILED`

### Owner policy update (2026-10-30, later): replaces E3

"Emergency doesn't stop the sequence or any program. It focuses on a physical stop like what the safety chain does. This applies to waiting as well."

- An emergency **never ends the run**:
  - A moving run (EXECUTING, pre-move included) goes into the `estop` hold (BLOCKED).
  - Any other hold is re-labelled `estop`, so it cannot auto-resume.
  - READY, PAUSED and a waiting step keep their state untouched.
- **Resume:** after release + Reset (NORMAL), the operator presses **Start**. The pre-move warning runs and the route continues from the step where it stopped. The hold never resumes by itself (alarm row `ESTOP_HOLD`, "press Start to continue").
- **Unchanged:**
  - the physical reaction: all outputs LOW, no drive power, mux inhibited, lease withheld
  - Start/Load refused while not NORMAL
  - physical Reset only
- **Survey moves** still end on an emergency. They are single blind moves that the safety chain's torque loss ended before the emergency suite existed. The mapping session itself is not ended. (Commissioning and pp moves no longer exist, 2026-10-30.)
