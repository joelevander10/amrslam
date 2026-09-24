# Coding plan: resume a route by itself after a safety stop

> **Status 2026-09-19: implemented** (see "Implemented" at the end). Decisions taken 2026-09-19:
> auto-resume only after protective-field (lidar) stops; the E-stop button waits
> for Start (`auto_resume_estop` false); software BLOCKED also auto-resumes; 2.0 s.
> Operator request: "upon emergency stop (via lidar OSSD to safety PLC and all),
> the AMR continues its task when the obstacle is cleared. No button needed.
> Ignore safety/CE requirements for this. Just want quick recovery to driving mode."

## What happens today (from the code and `~/.amr/logs`)

| Cause | What the software sees | What the run becomes | How it continues today |
|---|---|---|---|
| **nanoScan3 protective field → OSSD → FX3 → HWTO (STO)** | Both drives leave Operation enabled. `drive_node` logs `disarming: drive left Operation enabled (ETO?)` (9 times in the logs), then `cannot arm … Switch on disabled` every 2 s until HWTO returns (654 times). Wheel feedback goes stale. | **FAULT**, from whichever check fires first: `wheel feedback stale`, `localisation not READY (stale: wheels)`, or `action aborted` (Nav2 gives up) | Acknowledge, then abort, reposition and start the route again from the beginning |
| **E-stop button** | Same HWTO path, same signature | FAULT | Same |
| **RPP "detected collision ahead!"** (22 times in the logs) | The FollowPath goal ends `ABORTED` | **FAULT** `action aborted` | Same |
| **Executor obstruction check** (scan points in the swept envelope) | `N scan points inside the <step> envelope` | **BLOCKED** | Resume on the Run page, then a physical **Start** |

So a lidar stop today costs the whole run. A software BLOCKED costs two button presses.

**The hardware is already fine.** The drives re-arm by themselves once HWTO
returns (`armed (targets zero)`), with a zero setpoint. Nothing new is needed on
the drive side.

## The design

Any stop whose cause can clear by itself becomes **BLOCKED with auto-resume**
instead of FAULT. Once the cause has cleared and stayed clear, the executor
continues on its own: no Resume, no Start.

### 1. Know which stop it was (executor inputs)
- Subscribe to `/drives/status` (`DriveStatus`): armed / disarmed / fault, and the reason.
- Subscribe to `/output_paths` (`sick_safetyscanners2_interfaces/OutputPaths`,
  already published and read by nobody today):
  - `status[0]` is the protective field's OSSD, `true` = clear;
  - checked live on 2026-09-19: `status[0] true, is_safe[0] true` with the field clear.
- A **safety stop** is: the drives drop out of Operation enabled (or disarm)
  while EXECUTING. The cause is recorded:
  - `protective field` if `status[0]` was false within `safety_window_s` (1.0 s) of the dropout;
  - `e-stop / safety chain` otherwise.

### 2. Stop, don't fault (`route_executor_node.py`, `run_fsm.py`)
- A safety stop → `fsm.block("safety stop: protective field")` + `_interrupt`,
  **before** the prerequisite check runs, so stale wheels and localisation
  don't turn it into a FAULT.
- While BLOCKED with a safety-stop cause, these prerequisite failures are
  *expected* and hold the run instead of faulting it:
  - `wheel feedback stale/invalid`;
  - `localisation not READY` when its reason is `stale: wheels`.

  Every other prerequisite failure still faults.
- A FollowPath or Spin result `ABORTED` within `safety_window_s` of a safety
  stop, or while the protective field is not clear → BLOCKED, not FAULT.
- **RPP collision abort** → BLOCKED with the cause `controller: collision ahead`.
  - How to tell it apart: the goal ends ABORTED while the drives are armed and
    the executor's own envelope shows points ahead.
  - Any other ABORTED stays a FAULT.
- The FSM gains `auto_resume(reason)`, allowed only from BLOCKED. It is the same
  transition as `start()` from a prepared resume, but without the Start edge.
  PAUSED (the operator's own Pause) never auto-resumes.

### 3. Resume by itself
Every tick in BLOCKED with `auto_resume_enabled`, it continues once all of these
have held for `auto_resume_clear_s` (2.0 s):
- the existing `_resume_checks()`: prerequisites, selector AUTO, wheels still,
  on the segment and inside the corridor, turn-centre drift for rotates,
  envelope clear for `clear_stable_s`;
- the drives are armed;
- `/output_paths status[0]` is true (field clear).

Then `fsm.auto_resume("resumed after <cause>")` and `phase = PHASE_INIT`, so the
remaining path is sent as today (straights resume from the current projection,
rotates from the travelled angle, chains carry on).
- Any failure resets the 2 s clock. The Run page shows why it is waiting.
- There is **no timeout**: it waits until the cause clears, the operator aborts,
  or the selector goes to MANUAL. MANUAL still aborts the run, as today.

### 4. Parameters (`route_executor_node`, set in the navigation launch)
| Parameter | Default | Meaning |
|---|---|---|
| `auto_resume_enabled` | `true` | Master switch. `false` gives today's behaviour. |
| `auto_resume_clear_s` | 2.0 | Everything must be clear this long first. |
| `auto_resume_estop` | `false` | Also auto-resume when the cause was not the protective field (the E-stop button). See decision 1. |
| `safety_window_s` | 1.0 | Links an abort or a stale input to a drive dropout. |

### 5. UI (Run page) and events
- The state tile shows `BLOCKED — auto-resume: waiting (field not clear)` /
  `… resuming in 1.2 s`.
- The Resume button stays, and so does Start (as manual overrides).
- Events: `SAFETY_STOP` (cause) and `AUTO_RESUMED` (stop duration) on `/amr/events`.
- `RunState` gains `auto_resume` (bool) and `hold_cause` (string). This needs a
  colcon build of `amr_interfaces`.

### 6. Tests
- `test_run_fsm.py`:
  - `auto_resume` works only from BLOCKED;
  - never from PAUSED, FAULT or READY;
  - clears `resume_prepared`.
- `test_route_executor_logic.py`:
  - a drive dropout while EXECUTING → BLOCKED with the cause `protective field`
    when `status[0]` was false, and `e-stop / safety chain` when it was not;
  - stale wheels and `stale: wheels` localisation during the hold don't fault,
    but any other prerequisite does;
  - an ABORTED goal within the window → BLOCKED; outside it → FAULT;
  - RPP collision abort with points ahead → BLOCKED;
  - auto-resume fires after 2 s of all-clear, and a failure in between resets
    the clock;
  - auto-resume does not fire off the segment, with the selector on MANUAL, or
    with `auto_resume_enabled` false;
  - a chain resumed mid-arc keeps the chained path.
- Sim (opt-in, short): fake drive dropout → BLOCKED → re-arm → run completes.

### 7. Vehicle check (operator present)
1. A straight route at the base speed. Step into the protective field. The
   vehicle stops (STO), then drives on about 2 s after you step out. The log
   shows `SAFETY_STOP protective field` and then `AUTO_RESUMED`.
2. The same on an arc, and during a rotate.
3. An obstacle left in the path: it waits and does not creep.
4. Switch to MANUAL during the hold: the run aborts, as today.
5. Watch that the supervisor keeps NAVIGATION through the drive dropout.
   `supervisor_node` reads `/drives/status`; confirm it does not drop the mode.

## What this plan does not change
- The FX3, OSSD and STO wiring. The physical stop is exactly as today.
- **To confirm first:** if the FX3 project has a *restart interlock* (a manual
  reset after a protective-field stop), HWTO stays off until that reset and no
  software can skip it. The logs show re-arms without anyone at the panel,
  which suggests automatic restart. Check it in the Flexi Soft project.

## Decisions (recommendation in bold)
1. **E-stop button:** a deliberate press should not restart the vehicle by
   itself. The software can only tell it apart through `/output_paths` (the
   field was clear at the dropout).
   - **Recommendation: auto-resume only after protective-field stops;** after a
     button stop, wait for Start (no Resume click needed).
   - The alternative, `auto_resume_estop: true`, resumes after both.
2. Executor BLOCKED (software envelope) also auto-resumes: **yes**. It is the
   same "obstacle gone → continue" situation.
3. Wait time before driving off: **2.0 s** of all-clear.

## Implemented (2026-09-19)
- `route_executor_node.py`:
  - hold causes `field` / `estop` / `obstacle` / `controller` / `pending`;
  - `_hold`, `_classify_pending`, `_excused_in_hold`, `_auto_resume_tick`;
  - subscriptions to `/drives/status` and `/output_paths` (the import is guarded;
    without the SICK package a dropout counts as an E-stop);
  - Start resumes any hold whose resume checks pass, with no Prepare resume needed.
- `run_fsm.auto_resume()`: from BLOCKED only.
- `RunState`: `hold_cause`, `auto_resume` (`amr_interfaces` rebuilt).
- Run page: the State tile shows e.g. `lidar stop · resumes by itself`.
- Tests: `test_auto_resume.py` (12), `test_run_fsm.py`, `test_readiness.py`.

**Differences from the plan:**
- **Found while building: localisation.** Once READY, the localisation monitor
  latched **LOST** after 0.1 s of stale wheel feedback, which an STO always
  causes. Without a fix, no auto-resume could ever pass the READY check.
  `readiness.evaluate(..., excused)` and `localization_monitor_node` now:
  - excuse the wheels while `/drives/status` reports torque off (not
    operational and neither drive in Operation enabled);
  - give READY 0.4 s of grace for the wheels alone (`wheels_grace_s`), so the
    drive report can arrive.
  - A feedback loss with a drive still enabled is still a loss.
- **Wheel feedback can vanish before the drive report arrives.** The executor
  stops at once (hold `pending`). It then classifies the stop as a safety stop
  when the drive report says torque off, or FAULTs after `safety_window_s` if
  the report never says so.
- **A safety stop during an obstacle hold** takes the safety cause. **A safety
  stop during an operator Pause** stays PAUSED, and neither faults.
- **Controller aborts** are not told apart by reason: Humble's FollowPath result
  carries no error code. Every ABORTED outside a safety stop is a `controller`
  hold, with at most `controller_abort_retries` (3) per step, then FAULT.
