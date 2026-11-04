# Port plan: kyb-trackless SLAM/AMCL fixes into gvievo-main

Status: PLAN, not implemented (2026-11-03). Target branch `gvievo-main`.
Sources: kyb-trackless `287f67f`, gvievo-main `c41afb3` (`origin/gvievo-main`), split at `233c07a`.

## 1. Summary

Port five SLAM/AMCL fixes from kyb-trackless to gvievo-main as hand-written changes on a
new branch. None of them touches tape following, and gvievo-main keeps its own rule that a
blocked path waits for a physical Start.

| # | Item | Effort | Main file on gvievo-main |
| --- | --- | --- | --- |
| 1 | Automatic local-costmap clear | Moderate (new trigger point) | `amr_mission/route_executor_node.py` |
| 2 | RPP collision look-ahead 1.6 s to 1.07 s | Easy (one value) | `amr_navigation/config/nav2_params.yaml` |
| 3 | Resume corridor 0.30 m | Easy | `amr_mission/route_executor_node.py` |
| 4 | Start-refusal text with limits, blocked-for time | Easy | `amr_mission/route_executor_node.py` |
| 5 | Smooth autonomous speed changes, matched corner slowdown, 0.30 m/s corners | Moderate | `amr_base/cmd_mux_kinematics_node.py`, `amr_navigation/route.py`, `amr_bringup/launch/base.launch.py` |

The owner decided on 2026-10-30 that kyb-trackless and gvievo-main are separate projects
with no merge (`trackless-only-removal-plan.md`, D24), and gvievo-main has since
restructured the same files (`hold_policy`, `LeanNode`, `wire`). So each item is re-written
against gvievo-main's code, not cherry-picked.

Out of scope: unlimited automatic retries, wait steps, E-stop as a hold, destination
buttons, floor labels, and the doubled ramps made on kyb-trackless after the 3 November
runs (not yet tested).

## 2. Why: the 3 November field runs

Four autonomous runs of the same 32-step, 396 m route on MCQ4-4 (kyb-map-02 rev 3,
route-itg-02 rev 1) showed each problem these items fix. Only run 3 completed.

| Run | Ended | What happened | Item |
| --- | --- | --- | --- |
| 4 (15:11) | Manual takeover in s26 | Held 3 min 11 s at the 1 m arc s12 behind a phantom costmap obstacle, 72 retries; moved on 1 s after a hand clear of the local costmap | 1 |
| 1 (13:47) | Fault in s24 | RPP reported "collision ahead" three times on a 116 m straight while someone crossed the aisle | 2 |
| 2 (14:13) | Aborted in s21 | After a hold on an arc it stood 0.13 m off the line, past the 0.10 m resume limit, and sat 7.5 min until an E-stop | 3 |
| Before run 1 (13:36 to 13:44) | 17 refused Starts | "0.10 m / 0.2 deg from the route start" with no allowed limit shown; operators could not tell how close was close enough | 4 |
| 3 and 4 | Corners at 0.30 m/s | Step timing confirms the slowdown finished before each corner; the old mux stepped the braking from 0 to 0.5 m/s² at once | 5 |

The phantom obstacle comes from the scanner's ±95° view: marks left beside the vehicle are
never ray-traced clear, and a tight arc swings the footprint into them. The full analysis
is in `field-logs/2026-11-03/report/` (run comparison PDF, run-data workbook, costmap
illustration).

## 3. Item 1: Automatic local-costmap clear

Clear Nav2's local costmap every time Start begins motion, just before the 2 s pre-move
warning. That removes phantom marks without changing gvievo-main's policy that a blocked
path waits for Start.

On kyb-trackless the clear fires 10 s into a blocked hold and at Continue of a wait.
gvievo-main has neither trigger: a controller abort holds until Start
(`hold_policy.AUTO_CAUSES = ("field",)`), then faults after `controller_abort_retries` (3).
So the trigger moves to the Start press. Anything really in front is re-marked within
0.2 s (5 Hz updates), long before the warning ends.

Changes in `amr_mission/route_executor_node.py`:

1. Import `ClearEntireCostmap` from `nav2_msgs.srv` and declare `costmap_clear_service`,
   default `/local_costmap/clear_entirely_local_costmap`.
2. Create the client on the `io` callback group, next to the `/follow_path` and `/spin`
   action clients.
3. Copy `_clear_costmap(why)` from kyb-trackless: it calls the service asynchronously,
   warns once if the service is down, and logs an event.
4. Call it from `_begin_premove()`, which every Start reaches: a new run from READY and a
   resume from BLOCKED or PAUSED.

Also needed:

- A `COSTMAP_CLEARED` INFO row in `agv_core/alarms.py`, so the event shows on the Alarms page.
- Two tests: Start after a controller hold calls the clear before the warning, and a
  missing service warns once without blocking the Start.
- Confirm on gvievo-main that Nav2 runs without a namespace, so the default service name
  resolves (true on kyb-trackless; not yet checked on gvievo-main).

The trade-off: a clear forgets a real object beside or behind the vehicle that the forward
scanner cannot re-mark. Under gvievo-main's Start policy an operator looks before every
Start, which limits that risk.

## 4. Item 2: RPP collision look-ahead 1.6 s to 1.07 s

Shorten Regulated Pure Pursuit's collision check from 0.96 m to 0.64 m ahead at 0.60 m/s,
so something passing well ahead no longer aborts a straight.

| File on gvievo-main | Change |
| --- | --- |
| `amr_navigation/config/nav2_params.yaml` | `max_allowed_time_to_collision_up_to_carrot: 1.6` to `1.07` |
| `amr_navigation/test/test_nav2_params.py` | The bound `0.9 <= ttc * VEHICLE_V_MAX <= 1.1` becomes `0.6 <= ttc * VEHICLE_V_MAX <= 0.7` |

The safety fields are not affected: the hardware protective field still stops the vehicle,
and warning field 2 still stops it within 0.40 m. Only the software's early abort changes.
Leave `approach_velocity_scaling_dist` at 1.0 m unless Item 5 lowers the braking limit.

## 5. Item 3: Resume corridor 0.30 m

Let a held run resume up to 0.30 m off its line instead of 0.10 m. The controller then
steers back, with the wider limit kept until it has converged.

Today gvievo-main's `_resume_checks()` refuses a resume once
`abs(cross) > route.limits.cross_track_limit_m` (0.10 m on route-itg-02). A small shove
during a hold therefore strands the run, as in run 2.

Changes in `amr_mission/route_executor_node.py`:

1. Declare `resume_corridor_m`, default 0.30, and add `_resume_corridor()` returning
   `max(cross_track_limit_m, resume_corridor_m)`.
2. Use `_resume_corridor()` in `_resume_checks()` in place of the cross-track limit.
3. Add `_mark_resume()`, recording the step and the along-track position, and call it
   where `_start_edge()` resumes a BLOCKED or PAUSED run.
4. In `_cross_track_allowed()`, allow `max(2 x limit, corridor)` for `converge_m` after the
   marked resume point, so the run does not fault while steering back.
5. At mission load, widen the clearance margin to
   `max(cross_track_limit_m, start_gate_m, resume_corridor_m)`, so validation warns when a
   route leaves less room than a resume may use.

Tests: resume at 0.13 m off an arc is accepted, a resume at 0.35 m is refused with the
limit in the message, and the wider allowance ends after `converge_m`.

Known limit, from `rejoin-plan.md` (kyb-trackless): a resume sends the remaining drawn
line to RPP, which steers back by itself. At 0.30 m off that is a swerve of about 1.07 m
radius (curvature 2 x 0.3 / 0.8² ≈ 0.94 rad/m) that nothing checks against the map first.
Until a rejoin path exists on either branch, keep routes at least the corridor's width
away from fixed structure (step 5 warns when they are not).

## 6. Item 4: Start-refusal text and blocked-for time

Make the start-gate refusal print millimetres and the allowed limits, and show how long a
blocked path has been blocked. Both are message changes with no effect on motion.

gvievo-main's `_start_gate()` prints `f"{d:.2f} m / ..."`. At 0.104 m this reads "0.10 m",
which looks inside a 0.10 m gate, and that is what operators saw on 3 November. The
kyb-trackless text reads: "0.104 m / 0.2 deg from the route start (allowed 0.10 m /
5.0 deg); reposition manually".

| Change | Where | Note |
| --- | --- | --- |
| Refusal text: `{d:.3f}` plus the allowed limits | `_start_gate()` | Copy the kyb-trackless f-string |
| `start_gate_m` 0.10 to 0.15 | parameter default | Site choice: the ITG presses were refused at 0.10 to 0.13 m. Decide for gvievo-main's own routes |
| "blocked N s" in the hold text | `_blocked_for()`, `controller_clear_m` 1.0 m | N counts from the first abort and resets after 1.0 m of travel or at the next step |

Under gvievo-main's Start policy the blocked-for time tells the operator how long the path
has been blocked before they press Start.

## 7. Item 5: Smooth autonomous speed changes and 0.30 m/s corners

Make every planned Nav2 speed change a jerk-limited S-curve, slow to 0.30 m/s before each
corner, and leave tape following, warning-field braking and emergency stops exactly as they
are. Port kyb-trackless's committed values (`287f67f`), not the doubled ramps made after
the runs.

| Setting | gvievo-main now | Ported value | Where |
| --- | --- | --- | --- |
| Auto acceleration `a_max` | 0.3 m/s² | 0.2 m/s² | `amr_bringup/launch/base.launch.py` |
| Auto braking `d_max` | 0.5 m/s² | 0.25 m/s² | same |
| Auto jerk limit `auto_jerk` | none | 0.4 m/s³ (new) | same |
| Warning-field braking `field_d_max` | uses `d_max` | 0.5 m/s² (new, keeps today's braking) | same |
| Corner speed | up to 0.60 m/s, turn-rate cap | 0.30 m/s, `ARC_V_MPS` | `amr_navigation/route.py`, `compiler.arc_speed()` |
| Corner slowdown | `(v² - v_next²) / (2 taper_decel)`, 0.4 m/s² | jerk-aware `_taper_dist()`, `taper_decel` 0.25, `taper_jerk` 0.4 | `amr_mission/route_executor_node.py` |
| Goal approach `approach_velocity_scaling_dist` | 1.0 m | 1.5 m (0.24 m/s², under the 0.25 limit) | `nav2_params.yaml` |

The mux change, in `amr_base/cmd_mux_kinematics_node.py`, sits in the non-manual branch
that today calls `slew_asym()` for everything:

1. Warning-2 stop (`_speed_ramp.stopping`): keep `slew_asym` at `max(d_max, stop_d_max)`.
2. Warning field slowing the run (`_speed_ramp.target < 1.0`): `slew_asym` at
   `max(d_max, field_d_max)`.
3. `gating.LINE`: keep today's `slew_asym` with `line_d_max` and `line_alpha_max`, and add
   `line_a_max` (default 0.3) so lowering `a_max` does not slow tape-following starts.
4. Everything else (FOLLOW, ROTATE): `scurve(self._v, self._a, sel.v, a_max, d_max,
   auto_jerk, dt)`. `scurve()` already exists in gvievo-main's `diff_drive.py`.

Keep `taper_decel` and `taper_jerk` equal to `d_max` and `auto_jerk`, and keep
`V² / approach_dist <= d_max`. The tests should pin both pairs. Update gvievo-main's
`test_nav2_params.py` bound from `<= 0.5` to `<= 0.25`.

Expected feel: 0 to 0.60 m/s in 3.4 s over 1.0 m, and 0.60 to 0.30 m/s before a corner in
1.7 s, starting 1.0 m before it. Known limit: the slowdown plans only one step ahead, so a
straight shorter than 1.0 m before a corner (s27 on route-itg-02) enters it too fast.

## 8. Sequencing and testing

Branch `port/kyb-slam-fixes` from `origin/gvievo-main`, one commit per item, lowest risk
first, so any item can be dropped without touching the others.

1. **Item 4**, message text and blocked-for time: no motion change.
2. **Item 2**, one Nav2 value and its test.
3. **Item 3**, resume corridor: executor only.
4. **Item 1**, costmap clear: executor plus one alarm row.
5. **Item 5**, speed profile: mux, launch, route, executor and Nav2 together, last because
   it changes how every route feels.

Tests per commit:

| Check | Runs on | Covers |
| --- | --- | --- |
| `pytest src/amr_navigation/test` | any machine (no ROS) | Item 2, Item 5 route and Nav2 bounds |
| `pytest src/amr_mission/test`, `src/amr_base/test` | Ubuntu 22.04 with ROS 2 Humble | Items 1, 3, 4, 5 executor and mux |
| `python3 tests/run_all.py` | Ubuntu (needs `fcntl`) | the pinned `agv_core` check count, raised for the new alarm row |
| `AMR_SIM_TESTS=1` sim launch tests | Ubuntu with ROS | a full sim route still completes, a tape run still follows |
| `ruff check` | any machine | style |

Update `amr_ws/RUNBOOK.md` with each item: the Start clear, the 0.30 m corridor, the new
refusal text and the speed profile.

## 9. Vehicle acceptance

Run one witnessed session on gvievo-main's vehicle with a ROS bag recorded
(`/cmd_wheel_vel`, `/wheel_states`, `/amr/run_state`, `/tf`), then read it with
`tools/bag_report.py`. Each check below passes or names what failed.

- [ ] Item 1: someone stands beside the vehicle during a hold and walks away; Start logs
      `COSTMAP_CLEARED` and the step continues without a new abort
- [ ] Item 1: with a box truly on the path, Start clears, the box is re-marked and the step
      holds again (expected)
- [ ] Item 1: with Nav2's clear service unavailable, Start still works and one warning is
      logged
- [ ] Item 2: a person crossing the aisle about 1.5 m ahead no longer aborts a straight; a
      box on the path still holds before the protective field trips
- [ ] Item 3: pushed 0.15 to 0.25 m off an arc during a hold, the run resumes on Start and
      converges without a fault; at 0.35 m Start is refused with the limit shown
- [ ] Item 4: parked 0.12 m off the route start, the refusal shows millimetres and the
      allowed limits
- [ ] Item 5: corners take about 10.5 s per 3.14 m (0.30 m/s), the slowdown ends before
      each corner, and every stop lands within 0.05 m
- [ ] Regression: one tape (LINE) run starts and stops as before, and one SLAM survey and
      save still work

## 10. Risks and open decisions

Three decisions belong to the owner before work starts; the rest are risks the plan
already handles.

| Decision | Recommendation |
| --- | --- |
| Clear the costmap on every Start, or only on a Start after a blocked path | Every Start that begins motion: a parked vehicle collects the same phantom marks |
| `start_gate_m` for gvievo-main's routes | 0.15 m if operators position by hand, as on MCQ4-4 |
| Corner speed 0.30 m/s on every arc, or only on tight ones | All arcs, as on kyb-trackless; a radius threshold is possible later |

| Risk | Handling |
| --- | --- |
| A clear forgets a real object beside or behind the vehicle, which the forward scanner cannot re-mark | The operator looks before each Start (gvievo-main's policy); add it to the RUNBOOK |
| A resume from up to 0.30 m off swerves back without a map check (`rejoin-plan.md`) | Keep route clearance at least the corridor's width; port the rejoin path once kyb-trackless has it |
| Lowering `a_max` would also slow tape-following starts | `line_a_max` keeps LINE at 0.3 m/s² (Item 5, step 3) |
| Hand ports drift from the kyb-trackless originals | One commit per item, with the kyb-trackless function named in each commit message |
| Item 1 is not yet proven on kyb-trackless either | Its first vehicle run on kyb-trackless comes first; port after that run shows `COSTMAP_CLEARED` working |
| Runs take longer with gentler ramps and 0.30 m/s corners | Measure the route time in the acceptance session and compare it with today's |

Not changed by this plan: gvievo-main's rule that a blocked path waits for Start and
faults after 3 tries. kyb-trackless retries without limit instead; that is a separate
decision.
