# Hardware tests to run on the vehicle

**To:** the coding agent working on the AGV PC (`gvipc-ce`, `~/agv_can`, branch `kyb-trackless`).
**From:** the development session on the Windows checkout, 2026-11-03.

These changes were written and unit-tested on a Windows PC with no ROS and no vehicle, by two
development sessions working in the same tree. Your job is to prove them on the real vehicle
and report back.
- Section 1: the automated tests that could not run off the vehicle.
- Sections 2–3: the route `home` step and the bag report's slip analysis.
- Section 4: the other session's (`gvievo-01-d4`) Foxglove removal and slower AUTO ramps.
- Section 5: how to report.

**Order of work: 4.0 deploy → 4A → 1 → 4B → 2 → 3.**
- 4A changes what the service starts, and every later test boots through it.
- 4B changes AUTO acceleration, braking and stop approach, so it must pass before any test
  that judges AUTO timing or stopping accuracy, the home step included.
- Section 3 runs on the bags recorded in 4B and 2.

## 0. Ground rules

- **You do not start motion.** Every autonomous start needs the selector in AUTO and a
  physical Start press. Tell the operator what to press and when, then observe. Never script
  around the panel.
- An operator stands within reach of the E-stop for every run in sections 2–3.
- Run on a clear test lane (or the ITG-02 area with nobody in the aisle) and keep a 1 m
  clearance on both sides of every S-curve.
- Do not commit from the vehicle account. Write results to the file named in section 5.
- If a step fails, stop that section, record what happened, and go on to the next section
  only if it does not depend on the failed one.

## 1. Automated tests (no motion)

```bash
cd ~/agv_can
git status --short          # record it: the tree may hold work from more than one session
git log -1 --oneline
cd amr_ws && colcon build --symlink-install && source install/setup.bash
cd ~/agv_can && python3 tests/run_all.py
cd amr_ws && colcon test --packages-select amr_navigation amr_mission amr_web \
    --event-handlers console_direct+ ; colcon test-result --verbose
```

Pass:

| Check | Expected |
|---|---|
| `tests/run_all.py` | `all checks passed`, **731** checks (new module `test_slip_events`, 40 checks) |
| `amr_navigation/test/test_home_leg.py` | 31 passed (they passed on Windows; confirm on the vehicle's Python 3.10 / pytest 6.2) |
| `amr_navigation/test/test_route.py` | all pass; the 7 store tests that fail on Windows must **pass** here |
| `amr_mission/test/test_route_executor_logic.py::test_home_leg_progress_chain_overshoot_and_turn_cap` | passes. **It has never run**: it needs ROS messages. A failure here is a real finding |
| the rest of `amr_mission`, `amr_web` | no new failures compared with the last known state |

Then restart the service (`sudo systemctl restart amr.service`), wait for the boot report,
and confirm the web UI loads.

## 2. Route `home` step (Go home)

**What changed.** The route editor has a new last step, **Go home**. From wherever the route
ends, up to 5 m behind the route's start pose (HOME) and within 20° of its heading, the vehicle
drives forward to HOME and stops on HOME's heading:
- a straight at 0.60 m/s if it is already on HOME's line;
- otherwise a quintic S-curve at 0.30 m/s, never tighter than R 1.0 m and never more than
  30° off HOME's line.

Code: `amr_navigation/scurve.py` (new), `route.py`, `compiler.py`, `validate.py`,
`footprint.py`, and the executor's `_along_cross`, `_follow_path` and `_step_w_max`.
RUNBOOK step 4 describes it for operators.

**Record every run.** Start `tools/run_log.sh` before the operator presses Start, stop it
after DONE, then run `python3 tools/bag_report.py ~/.amr/runs/<stamp>`.

### 2.1 Editor (no motion)

On the Routes page, active map kyb-map-02:
1. Draw a route that ends 2 m behind the start pose on its line; press **Go home**. Expect
   the log line `home leg, straight 2000 mm to HOME`.
2. Undo, end it 0.15 m to one side instead, press **Go home**. Expect an S-curve preview with
   a dashed footprint band, and a step-list row `home S-curve 0.30`.
3. Try each refusal and check that the log explains it:
   - the route ends in front of HOME: `not behind HOME`;
   - 6 m behind: `at most 5000 mm`;
   - 0.30 m sideways with only a 1 m run-up: `needs a … m radius … end the route at least … mm behind HOME`.
4. **Validate** a valid one. Expect valid, `Closes YES`. Save it.
5. Check the saved route on the Run page and under Measure: the curve is drawn, not a dot.

Pass: all of the above, and no errors in the browser console.

### 2.2 Straight home leg

A short route (out about 6 m, U-turn, back), ending about 2 m behind HOME **on its line**,
then Go home. One run.

Pass: DONE; stop error at HOME ≤ 0.05 m and ≤ 2° (from the run summary or `/amcl_pose`
against the route start); no `EXEC_OFF_PATH` or `EXEC_OVERSHOOT`.

### 2.3 S-curve home leg

Same shape, but the return straight is offset so the route ends 2 m behind HOME, offset:
- +0.10 m (left)
- −0.10 m (right)
- +0.25 m
- −0.25 m

Each needs a new route revision. One run each, without the trolley first.

Record for each: peak cross-track on the home step (run summary, `cross_track_m`), stop error
at HOME, any `PATH_BLOCKED` / controller abort on the home step, and the measured speed on
the curve (`bag_report` `wheels.csv`).

Pass: peak cross-track ≤ 0.10 m (the route's `cross_track_limit_m`); stop error ≤ 0.05 m and
≤ 2°; speed on the curve ≈ 0.30 m/s; no faults. Note anything RPP does at the start of the
curve (it should not jerk: the curve starts with zero curvature).

### 2.4 Repeat count 2

Take the ±0.25 m route from 2.3, set repeat count 2, save, and run once.

Pass: both passes complete without stopping at HOME between them; the second pass starts
cleanly from HOME.

### 2.5 ITG-02 migration

In the editor, load `route-itg-02` rev1, delete the last step `s32` (the hand-tuned straight
after the `ITG-RELEASE-HP` wait), press **Go home**, Validate, save it as a new revision, and
create its mission. Run one full ITG-02 cycle **with the trolley**, the normal way.

Pass: DONE; the final approach after the release wait ends at HOME within 0.05 m / 2°; the
next destination press is not refused by the start gate (0.15 m / 5°). Report the stop error
next to the old `s32` behaviour if known.

## 3. Bag report: torque in N·m and slip / impact detection

**What changed.** `tools/bag_report.py` adds `left_nm`, `right_nm` and `sum_nm` to
`torque.csv`, using 1.27 N·m rated × 30 gearbox / 100 % per percent. The per-step summary is now in N·m.
A new SLIP / IMPACT section (logic in `agv_core/slip_events.py`) lists:
- hitch snatches (torque jumps);
- torque collapses (tyre letting go);
- wheel-vs-gyro yaw-rate mismatches.

Each comes with the AMCL map→odom correction and localization state that followed it. Every
threshold is a first guess.

1. Run `bag_report.py` on every bag from section 2. It must finish without a traceback and
   print the SLIP / IMPACT section.
2. **Sanity:** sum torque at a steady 0.60 m/s with no trolley should be small and positive. The
   `wheel/gyro yaw gain` line should read about 1.00 (0.95–1.05). A **negative** gain means
   `gyro_sign` is wrong: report it, do not change it.
3. **Towing set** (only if the operator agrees, trolley hitched). Make 5 start-ups for each of
   these, with `run_log.sh` recording:
   - no trolley;
   - trolley with the pin tight;
   - trolley with the pin loose.

   Record the trolley mass and the pin slack in mm.
4. Report, for each set: the "largest torque jumps" and "largest yaw residuals" lines, the
   snatch / collapse events, and the map→odom shift after each event against the printed p95
   baseline.

Pass for this section: the tool runs on real bags and the numbers are plausible. The
thresholds get tuned from this data later; that is not a pass/fail item now.

## 4. Foxglove removal and slower AUTO ramps (from session `gvievo-01-d4`)

Two changes: (A) Foxglove removal, (B) twice-as-long auto speed ramps. Its other files
(`manuals/gvievo-main-port-plan.md`, `manuals/foxglove-removal-plan.md`,
`field-logs/2026-11-03/report/`) are documentation only.

Both sessions edited these shared files:
- `test_route_executor_logic.py`: the taper test about line 611, and the home-leg test at the end;
- `base.launch.py`: the mux block;
- `route_executor_node.py`: the taper parameters, and the home-leg methods;
- `nav2_params.yaml`;
- `RUNBOOK.md`.

Make sure both sets of edits are present after the pull.

### 4.0 Deploy (once, before everything else)

With `amr.service` stopped and the vehicle parked:

```bash
cd ~/agv_can && git pull        # or check out the commit holding these changes
cd amr_ws && colcon build --symlink-install --packages-select amr_bringup amr_mission amr_navigation amr_web
sudo systemctl start amr.service
```

`amr.env` is read straight from the repo (`EnvironmentFile=`), so the pull also removes
`AMR_FOXGLOVE`. (`amr_web` is added to the build list for the home step's editor changes.)

### 4A. Foxglove removal (no motion) — first

**Safety:** vehicle stationary and disarmed, E-stop reachable. Stopping the service drops the
drive heartbeat; that is expected and recovers on start.

1. **Offline tests.** The service may keep running: domain 89 keeps the tests off the vehicle graph.

   ```bash
   cd ~/agv_can/amr_ws && source install/setup.bash && source env/vehicle.sh
   env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/amr_bringup/test src/amr_navigation/test src/amr_mission/test src/amr_base/test
   python3 ~/agv_can/tests/run_all.py
   ```

   Pass: all green, including `src/amr_bringup/test/test_no_foxglove.py` and
   `tests/test_alarms.py`. `test_alarms.py` is the RUNBOOK alarm-table check; it fails on Windows
   only because of cp1252 decoding, so it must be judged here. These runs also cover most of
   section 1.

   Optional:
   `env AMR_SIM_TESTS=1 ROS_DOMAIN_ID=89 python3 -m pytest -q src/amr_bringup/test/test_unified_sim.py src/amr_bringup/test/test_fault_injection_sim.py`.
   Pass: the sim supervisor boots to IDLE with three groups.

2. **Boot check.**

   ```bash
   journalctl -u amr.service -b --no-pager | grep -E 'spawned|foxglove|STARTING -> IDLE'
   ss -ltnp | grep 8765
   pgrep -af foxglove
   curl -s -o /dev/null -w '%{http_code}\n' http://localhost:5001/
   ```

   Pass: `spawned base` and `spawned web` appear and the mode reaches IDLE. The word foxglove
   does not appear, `ss` and `pgrep` print nothing, and the web answers 200.

3. **Shutdown check.**

   ```bash
   sudo systemctl stop amr.service
   journalctl -u amr.service -n 30 --no-pager
   pgrep -af 'amr_|nav2|slam_toolbox'
   ```

   Pass: `stopped base: clean` and `stopped web: clean` (no foxglove line), then
   `teardown: done`, and `pgrep` prints nothing. Then `sudo systemctl start amr.service` again.

### 4B. Twice-as-long AUTO speed ramps — before section 2

**What changed:**
- mux: `a_max` 0.2 → 0.1 m/s², `d_max` 0.25 → 0.125 m/s², `auto_jerk` 0.4 → 0.1 m/s³;
- executor: `taper_decel` 0.25 → 0.125, `taper_jerk` 0.4 → 0.1;
- Nav2: `approach_velocity_scaling_dist` 1.5 → 3.0 m.

**Untouched:** warning-field braking (`field_d_max` 0.5), the warning-2 stop, E-stop/STO, the
manual and pendant ramps, and rotation (`alpha_max` / `delta_max`).

**Safety:**
- A second person with an E-stop in hand walks beside (not in front of) the vehicle for the
  whole run. The aisle is cleared and the operator is at the panel.
- Planned stops now start 3 m out and are gentler. Field and emergency stops are NOT
  softened, so normal clearances apply.
- If the vehicle overshoots a stop or faults at an end check, stop the session and report the
  log lines. Do not tune on the vehicle.

1. **Confirm the loaded values** (with `env/vehicle.sh` sourced):

   ```bash
   journalctl -u amr.service -b | grep 'cmd_mux_kinematics: r='        # a_max=0.100
   ros2 param get /cmd_mux_kinematics d_max                             # 0.125
   ros2 param get /cmd_mux_kinematics auto_jerk                         # 0.1
   # after NAVIGATION is selected on kyb-map-02 rev 3:
   ros2 param get /route_executor taper_decel                           # 0.125
   ros2 param get /route_executor taper_jerk                            # 0.1
   ros2 param get /controller_server FollowPath.approach_velocity_scaling_dist   # 3.0
   ```

   Pass: all six match. If any value is old, the build or the restart did not take: stop here.

2. **One full run of route-itg-02 rev 1** (mission `kyb-map-02_rev3_route-itg-02_rev1`), the
   normal procedure: localize to READY, AUTO, Start, Continue at s11, s20 and s31. Record the
   whole run with `tools/run_log.sh`, or at least:

   `ros2 bag record -o ~/bags/ramps-$(date +%F-%H%M) /cmd_wheel_vel /wheel_states /amr/run_state /amr/mux_state /drives/torque /odometry/filtered /tf /tf_static`

   Then run `python3 ~/agv_can/tools/bag_report.py <bag dir>`.

   Pass:
   - From rest to 0.60 m/s takes about 6.9 s / 2.0 m (`wheels.csv` `v_mps`; it was 3.4 s / 1.0 m).
   - Before each corner the speed falls from 0.60 to 0.30 over about 3.5 s and is at 0.30 before
     the corner starts.
   - The 3.14 m corners (s2, s4, s6, s23, s25) take about 10.5 s each.
   - After a corner it is back to 0.60 within about 3.9 s.
   - Every stop (the three waits and the end) passes the executor's end check: no FAULT, and
     each `step sNN (...) done: X m / Y deg` line in the layer log is within limits (position
     tolerance 0.05 m on this route).
   - No FAULT, and no controller hold that a real obstacle or a scanner warning doesn't explain.
   - Expected, not a failure: s27 is 0.2 m long, so the vehicle enters the s28 arc near full
     speed and slows inside it. Watch s28's cross-track; it must stay under the 0.10 m limit.
   - Write down the total run time, to compare with 3 Nov run 3 (20 min 34 s, at 0.50 m/s
     straights) and run 4.

3. **Warning-field behaviour unchanged** (one check). A second person stands still at a
   marked spot about 3 m ahead on a long straight (s5 or s24), off to the side of the protective
   field, and steps in only on a signal. They never walk toward the moving vehicle.

   Pass: warning field 1 slows the run to ×0.5 within about 1 s (a FIELD_WARNING event).
   Warning field 2 stops it within 0.40 m, braking as hard as before (`field_d_max` 0.5), not
   gently.

### 4C. Committed in 287f67f, never seen working on the vehicle (optional, for full coverage)

- **Automatic local-costmap clear.** Expect `COSTMAP_CLEARED` events at each Continue (s11,
  s20, s31) and after any blocked-path hold reaches 10 s. Check during the 4B run with
  `grep COSTMAP_CLEARED ~/.amr/logs/events.jsonl`.

  Physical check: during the s11 wait someone walks around the vehicle's left side, then
  presses Continue. s12 should start without a long "path blocked" hold.
- **Resume corridor 0.30 m.** During a hold, with the drives disarmed, push the vehicle
  0.15–0.25 m off the line, then Start. Pass: it resumes and converges without a fault. At
  0.35 m it should refuse, with the limit in the message.

## 5. Reporting

Write `field-logs/<date>/hardware-test-results.md` with:
- the `git log -1` and `git status --short` from section 1;
- one table row per test (ID, pass / fail, the measured numbers, the run_log stamp);
- anything surprising, quoted from the logs with time stamps.

Keep the run_log folders. Do not change thresholds, parameters or code to make a test pass;
report the failure instead.
