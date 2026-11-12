# Unified AMR operator and cutover runbook

`amr.service` is the intended production service. It starts the hardware base,
operator web app, then stays in **IDLE**. It does
not load a map, start a survey or run a route after boot. The operator selects mapping or navigation from Run.

The physical controls remain authoritative:

- **MANUAL** permits bounded browser jog, the pendant and preset survey moves.
- **AUTO** permits a loaded route only after a fresh physical **Start** edge.
- E-stop and the FX3/STO chain remain the immediate physical stop path.
- A stale panel, supervisor lease, drive state or command inhibits motion.

The web app is `http://192.168.2.20:5001/` and opens on **Home**. Its tabs are
**Home**, **Run** (with **Maps** for surveys and **Routes** for the route editor),
**Manual**, **Alarms** (events), **Monitor**, **I/O**, **Params** and **Blind run**. The
Status and Commission pages were removed on 2026-10-30: Home shows the mode, its reason,
the fault banner, Return to idle and Recover; Monitor has the rest. The blind run came
back on 2026-11-02 as its own page (§3, PV only). Every page has the same left
rail: tiles for Mode, Selector, Drives, Command, Localisation, Run, Wheels and
Generation, with the event log below. A tile with a yellow bar needs attention and
a red bar means a fault or stale data. If the whole rail is greyed out and the
header pill says `DISCONNECTED`, the values are last known, not live. After a
software update, hard-refresh the browser so it does not keep old styles or
scripts: the **↻** button at the top right of every page (next to ✕ on Demo) does
it from the touch panel; Ctrl+Shift+R with a keyboard.

## 1. Normal service operation

```bash
systemctl status amr.service
journalctl -u amr.service -f
sudo systemctl start amr.service
sudo systemctl stop amr.service
sudo systemctl restart amr.service
```

The unit is `Type=notify` with `WatchdogSec=30`: the supervisor pings systemd on
every loop tick, so a WEDGED supervisor (alive process, dead loop) is restarted
like a crash. The web process group is respawned by the supervisor itself, 10 s
apart, three times, after which the Alarms page shows `WEB_DOWN` and the Home
page offers **Restart**.

### Who sees what: operator and engineer

The pages open in the **operator** view: **Home · Run · Manual · Alarms**, and
nothing else. Home is one line of state, one sentence of action and one button,
all chosen server-side from the alarm catalogue (`agv_core/alarms.py`, section 4).

The **engineer** view adds Maps, Routes, Monitor, I/O, Params and Blind run. Press `Op` in the header and type the PIN; press `Eng` to go back.
The PIN is **2809** by default and lives in `~/.amr/web.json` (0600, created on
first run) or in `AMR_WEB_ENGINEER_PIN`, never in the repository. It is a mistake
guard, not security: every endpoint is still reachable directly on the network,
and the vehicle is protected by the physical panel and the supervisor's lease, as
it always was.

A healthy start reaches `IDLE`, with the base and web processes running and no
mapping/navigation layer. The Home page must read `READY` with nothing standing.
Home and the rail must show:

- Mode `IDLE`, with no red fault banner on Home;
- Selector `MANUAL` or `AUTO` (not `STALE`/`INVALID`) and Drives `ARMED` or `OFF`, without a red bar;
- Command `none` while untouched;
- Localisation and Run `–` (and, on Monitor, Active map `—`).

Stopping the service revokes the control lease first. The supervisor then stops
the active layer, base and web process groups. The base shutdown
zeros wheel commands, clears the drive heartbeat consumer and de-energizes the
drives; the panel owner drives its output coil low. After stop, check that no
child remains:

```bash
systemctl show amr.service -p ActiveState -p SubState -p MainPID -p Result
pgrep -af 'amr_|nav2|slam_toolbox' || true
```

Do not use process-name kills during normal operation. The supervisor owns the
child process groups and performs bounded SIGINT → SIGTERM → SIGKILL cleanup.

### Demo page (visitors)

`Demo` (last tab, both views, no PIN) is a full-screen page for when the vehicle is
on display, **stationary**: product story, fixed spec strip, and a live top-down view
of what the safety laser scanner sees around the vehicle outline. When someone comes
within `near_m` (default 1.0 m from the outline) it says "It sees you". It is
read-only and says nothing technical: no scan, no pose or any fault reads as
"Standing by". It has no tabs; **tap the ✕ in the corner** to return to Home.
Wording, specs and `near_m`: `amr_web/static/demo/content.json` (restart the
service after editing). The view needs `/scan` and the odom TF (EKF) running.

## 2. Survey → draw a route → run it

The complete workflow, in order. Only step C moves the vehicle on its own, and
only after a physical Start. Everything else is browser jog under **MANUAL**,
or it moves nothing.

Before you begin:

- The service is in `IDLE` (Home) and the drives are operational.
- Someone is at the vehicle with the E-stop in reach, and the area is clear.
- The selector is on **MANUAL**.
- Mark a start position and heading on the floor (tape an arrow). The survey
  begins and ends there, and it is the easiest place to start a route from.

**Header (top right).** The Wi-Fi readout shows the robot's link on `wlp1s0`
(SSID and dBm; red `no link` when `agv_field` is out of range). Next to it, **NET**
shows whether the ROBOT reaches the internet (green dot = online, red = offline,
rechecked every 4 s by a TCP connect to 1.1.1.1 / 8.8.8.8 on 443; hover for the
round trip). Both are information only: nothing on the vehicle needs Wi-Fi or the
internet. ROS runs on loopback, the lidar and the I/O island are wired, and the
pages are served by the robot itself. Without `agv_field` the tablet cannot reach
the pages (use a wired or USB connection), and boot waits at most about 30 s for
NetworkManager before `amr.service` starts.

**Jogging.** A jog pad appears on the Manual, Maps and Run pages. Hold
a pad button, or hold W/A/S/D or the arrow keys, to drive. The held button turns
blue. Releasing stops the vehicle, even if you let go before the page has
received its jog session. So do Space, Esc, the centre **Stop** button, switching
browser tabs or windows, and typing in a text field. The speed menu offers 0.10, 0.20, 0.30
and 0.40 m/s (2026-09-19; the server caps jog at 0.40 m/s and 0.39 rad/s). The diagonal
buttons drive like the pendant: the fast wheel at the selected speed, the slow one at 75 %
of it. Left/Right spin at 1.95 × the selected speed in rad/s, at most 0.39 rad/s.
Surveys may use any setting; check the survey's return review (see below) after the first
fast one.

**Drive power (2026-10-08).** The drives are not powered while idle: they sit in
standby (CANopen up, wheel counts read, power stage off). Under **MANUAL** they are
powered only while the **Manual Arm** switch (DI08) is on; jog pad, pendant, survey
moves do nothing with it off (Monitor and the rail show Drives `OFF`,
reason "Manual Arm (DI08) off"). Under **AUTO** they are powered by a run: every
move after a Start first sounds the alarm horn for 2 s,
holds that wait for Start drop to standby, holds that clear by themselves keep power.

**Pendant.** The hard-wired jog pendant on the DIO island works under the same
authority as the jog pad (selector **MANUAL**, Manual Arm on, no fault) and
needs no browser. Hold a direction button to drive; releasing it stops the
vehicle. Opposing buttons pressed together cancel each other. The pendant
outranks the jog pad while a direction is held. Speeds (2026-09-19, mux
parameters in `base.launch.py`): FWD/RVS 0.50 m/s (`pendant_v_m_s`; 0.60 was tried and
reverted on 2026-09-19 after a survey at that speed came out rotated); FWD/RVS
with LEFT/RIGHT arcs with the slow wheel at 75 % of the fast one
(`pendant_turn_ratio`, fast wheel 0.50 m/s, slow 0.375); LEFT/RIGHT alone
spins in place at 0.39 rad/s (`pendant_w_rad_s`). **While surveying** (mode MAPPING)
every manual spin, pendant and jog pad alike, is capped at 0.27 rad/s (mux
`survey_w_max_rad_s`): a fast spin smears the scans the map is built from. Manual sources follow an
S-curve: acceleration builds at 1.0 m/s³ to 0.3 m/s² (`manual_jerk`,
`manual_a_max`), so 0 → 0.50 m/s takes about 1.9 s; releasing a button still
stops at the drives' ramp. Wiring
(profile `pendant`, 0-based DI channels): FWD DI04, RVS DI05, LEFT DI06,
RIGHT DI07. The IO page shows the live bits under those names.

### A. Survey a map (`Maps` page)

1. Park the vehicle on the floor mark, facing the marked heading, and keep it
   **still**.
2. Fill in **Map id**, using letters, digits, `_` and `-` only (for example
   `line_section`). Fill in **Start mark description** (for example "tape
   arrow A, facing the rack ends").
3. Press **New map (start survey)**. The supervisor switches to `MAPPING`, and
   this takes a few seconds. The state box then shows
   `MAPPING map <id>  surveying '<id>' from (x, y, heading)`.
   - `not ready: vehicle is moving`: stop and press again.
   - `not ready: IMU not calibrated`: the IMU calibrates during the first
     ~2.3 s of standing still after the service starts. Wait and press again.
   **Preset moves** (2026-09-19, Maps page under the jog pad, only while surveying):
   type a distance and press *Forward* / *Reverse* (0.05–10 m at 0.40 m/s, 0.30 until 2026-11-03), or a
   *Left* / *Right* spin of 45, 90, 135 or 180° (0.20 rad/s). Press once: the vehicle
   drives the whole move by itself under the same authority as the jog pad
   (selector **MANUAL**). **Any pendant button, the E-stop, the selector, the jog
   pad or *Stop move* ends it at once.** After each move the page reports how far
   SLAM corrected the vehicle's position during it; a red "the map may have
   slipped here" (a heading correction of 2° or more) means the scan matcher
   probably took a wrong alignment: back up and drive that stretch again, slowly.
   Smooth preset moves and slow spins are what the mapping copes with best in
   large open areas.
4. Drive the area slowly with the jog pad. Watch **Live map**: grey and black
   cells grow, the green arrow is the vehicle and red points are the current
   scan.
   - Drive every aisle the routes will use, and a little beyond.
   - Pass junctions and loop back over areas you have already mapped, so SLAM
     can close the loop.
   - Avoid fast turns in place. If walls start to look doubled, slow down.
5. Drive back to the **same floor mark and heading**, stop, and press
   **Returned to start**. The page shows the closure: how far SLAM thinks the
   vehicle is from where the survey started (`dx`, `dy`, `dyaw`). The state is
   `RETURN_REVIEW`.
   - Small values, a few cm and 1–2°, with walls that look single and straight
     mean a good map.
   - If the values are large or walls look doubled, you can drive another loop
     and press **Returned to start** again, or abort.
6. Optionally fill in **Review note** (for example "span 12.00 m measured
   11.96"), then press **Save map revision**. The state goes `SAVING` → saved,
   and the supervisor returns to `IDLE` on its own.
7. The map appears under **Saved map revisions** as `<id>`, `rev N`, with a
   short SHA. A revision never changes after it is saved. Surveying the same
   map id again creates `rev N+1`.

**Abort survey** discards the survey and returns to `IDLE`. You cannot switch
to navigation while an unsaved survey exists: save it or abort it first. A new
survey can start straight away, with no service restart.

**`SLAM STALLED N s`** in the session message (also refused by "Returned to
start" and "Save") means slam_toolbox stopped consuming scans while the scanner
is still publishing: its process has hung. The live map freezes and the pose
falls back to `odom` a few seconds later. Nothing recovers it: abort and start
the survey again (a fresh slam_toolbox is launched). SLAM and AMCL read
`/scan_gated` (scan_gate_node), which releases each scan only once its odometry
transform exists, to keep them off the tf2 code path that hung on 2026-09-17.

### A2. Trolleys and other temporary objects (`Maps` → **Edit areas**)

The survey cannot wait for an empty floor, so a saved map contains trolleys,
parked forklifts and pallets that later move. Without a mark, the map treats
them as walls, and a route through a spot where a trolley stood is refused.
Mark such places as **dynamic areas** (2026-09-19, dynamic-mapping plan
Increment 1). Nothing on this page moves the vehicle, and the mode can stay
`IDLE`.

> **A dynamic area is a statement about the MAP, not a guard during the run.**
> Since 2026-09-20 the executor makes no scan check of its own, so marking an
> area only lets a route be validated through it. If the trolley is still
> standing there when the run comes past, the only thing that will stop the
> vehicle is the nanoScan3 protective field. **Check the area is clear before
> you Start.**

1. On **Maps**, press **Edit areas** on the revision. The Review page opens
   with that revision. Areas already marked are drawn with yellow hatching.
2. Pick a tool and drag a rectangle with the left mouse button. Right-drag
   pans the map.
   - **Dynamic area**: the scan here may change. Draw it around the object
     with some floor, not over walls or racks.
   - **Clear dynamic**: removes the mark.
   - **Erase to unknown**: safe erase. Unknown cells still block routes unless
     they lie inside a dynamic area.
   - **Paint free**: you state that this is open floor (the object is gone for
     good). Use it rarely.
   - **Remove noise** (2026-11-02): inside the rectangle, every occupied blob of up
     to 4 cells (8-connected, about 10 × 10 cm at 5 cm) becomes free. Walls and
     racks are long connected runs and stay. CLI: `--despeckle-rect`
     (`--despeckle-cells N` for a larger limit).
   - **Static obstacle** (2026-11-02): the cells become occupied - a wall the
     survey missed, or a block routes must never cross. CLI: `--occupied-rect`.
   Each rectangle is one edit. Edits apply in order, so a later one wins where
   rectangles overlap. Undo, Redo and × remove edits.
3. Add a note and press **Save as new revision**. The robot writes `rev N+1`.
   The files `map.pgm`, `dynamic.pgm`/`.yaml` and `edits.json` (the parent and
   the edits) are listed and hashed like every bundle file. The old revision
   is not changed.
4. Routes are saved per revision. On **Routes**, select the new revision, load
   each route, **Validate**, **Save revision**, and create a new mission. On
   **Run**, select the new revision.

What a dynamic area does:

- **Validation**: mapped occupied or unknown cells inside a dynamic area do not
  block a route. The result shows a yellow **provisional** line for the step
  ("passable only if the live scan agrees"). Keepout still blocks, and cells
  outside the mark still block.
- **During a run**: nothing. The executor no longer compares the scan against
  the route (operator decision 2026-09-20: obstacles belong to the safety
  chain), so an object still standing in a marked area is stopped only by the
  protective field, and only if it is in front of the vehicle. The run log
  warns at load which steps cross dynamic areas.
- **Localisation monitor**: beams that end in a dynamic area, or where the map
  expects a wall inside one, are left out of the scan match.
- **AMCL**: with `AMR_LOC_BLANK_DYNAMIC=true` in `amr.env` (the AMR QR's
  setting since 2026-11-09; `false` = the full map) it uses a copy with the
  dynamic areas set to unknown, served on `/map_loc`. The web view keeps
  showing `/map`. A map without dynamic areas is used as it is. The
  navigation layer refuses to start if fewer than 500 occupied cells remain
  outside the dynamic areas: shrink them, or set it back to `false`.
- **Smoothed map->odom**: with `AMR_LOC_SMOOTH_POSE=true` (the AMR QR since
  2026-11-12) AMCL does not broadcast map->odom; `pose_smoother_node` does,
  working each correction in over a second or two so the controller does not
  chase AMCL's jitter where blanked areas leave few beams (the wobble).
  AMCL's own map->odom is on `/amr/localization/map_odom_raw`; the
  localization monitor judges AMCL by it. Parameters (time constant, rate
  limits, snap limits) are the node's; see `amr_localization/pose_smoother.py`.
- The rear blind spot is unchanged: a dynamic area behind the vehicle is not
  seen during a reverse.

From the shell (same result as the page):

```bash
ros2 run amr_mission map_edit --map tool-center-00 --rev 1 \
    --dynamic-rect 4.0 -1.5 6.5 -0.4 --note "trolley bay by rack C"
```

### B. Draw a route (`Routes` page)

The route editor moves nothing. The mode can stay `IDLE`.

1. **Map**: pick the saved `<id> rev N`. **Route id**: a plain name, for
   example `route_a`. **Load saved** stays on `— new —`, or pick an existing
   route to edit it.
2. **Start pose**: click the tool, then click on the map where the route
   starts and **drag towards the heading**, then release. A green arrow and the
   vehicle's footprint appear.
   - Put the start on a spot you can mark on the floor. The survey start mark
     is the easiest.
   - When the run starts, the vehicle must be within **0.10 m and 5°** of this
     pose, or Start is refused.
3. Build the route from steps. They chain from the start pose:
   - **Straight**: select the tool, then click a point ahead. The straight runs
     along the current heading, as far as that point projected onto the
     heading. It cannot go backwards or sideways; turn first.
   - **CCW / CW 45, 90, 180, 270**: turn in place, as seen from above. CCW is
     to the left.
   - **Undo**, **Redo** and **Clear steps** edit the list. The step list
     (`s1`, `s2`, …) is shown under **Steps**.
   - **Measure** (2026-11-03, view only): click point A, then point B. The
     panel and the map show the distance, ΔX and ΔY in mm, in map
     coordinates (B − A). A click snaps, in this order, to a wait point, an
     arc centre, a step end (or turn point, route start, survey start), or
     else onto a route line. Snapping covers the route being drawn and the
     newest revision of every saved route on this map (drawn faint while
     Measure is on). Untick **Snap**, or hold Shift, to measure from exactly
     where you click. A third click starts a new measurement; Esc or
     switching the tool off clears it. Dragging still pans. Measuring never
     changes the route, its undo history or the last validation.
4. **Repeat count**: how many times the route runs back to back, 1–100. Use 1
   unless the route returns to its own start pose. A repeated run shows
   `[pass p/P]` in the run reason. **Speed** is not a route setting
   (2026-10-02): every forward straight runs at 0.60 m/s
   (`route.VEHICLE_V_MAX`), every arc at 0.30 m/s (`route.ARC_V_MPS`,
   2026-11-03), and a reverse at half the straight speed, 0.30 m/s.
   Since 2026-11-03 every autonomous speed change is a smooth S-curve (mux
   `auto_jerk` 0.1 m/s³, at most `a_max` 0.1 m/s² up and `d_max` 0.125 m/s²
   down; twice as long as the first smoothing, after the field runs that
   day): a start reaches 0.60 m/s after about 2.0 m (6.9 s). Field and
   emergency stops are not softened: while a warning field is in force the
   vehicle brakes at `field_d_max` 0.5 m/s² (warning 1) or the drive's ramp
   (warning 2), and loss of authority still stops at once.
   Consecutive straights and arcs are driven as ONE continuous move (no
   stop between them); a rotate or a reverse ends such a chain. Before an
   arc the speed tapers early enough for the S-curve to reach 0.30 before
   the boundary (executor `taper_decel` 0.125 m/s², `taper_jerk` 0.1 m/s³,
   `taper_lead_s` 0.3): 0.60 → 0.30 starts 1.82 m before the arc, takes
   1.59 m (3.5 s) and is done about 0.2 m before it; after the arc it speeds
   back up over about 1.7 m (3.9 s). At the end of a chain the controller's own ramp over
   the last 3.0 m (`approach_velocity_scaling_dist`) brings it to the stop. **Reverse** (2026-09-18): type a distance in mm and press *Reverse*
   to back up along the current heading, facing forward: at most 2000 mm
   and at half the speed cap (never the boost). The nanoScan3 does not
   cover the rear, so a reverse is driven blind: keep it short, keep the
   area behind clear, and stay within reach of the E-stop for the first
   ones. Shown dashed with `◂` in the editor. **Arc** (2026-09-18): radius
   (≥ 1.0 m, twice the track) and angle (45–180°), *Arc L* / *Arc R*; the
   vehicle drives forward along the circle and leaves with the heading turned
   by the angle;
   an arc runs at the route's arc speed (0.40 m/s, whatever the straight
   before it ran at), or less where v/R would exceed 0.9 × the arc turn
   ceiling of 0.45 rad/s (never below R 1.0 m). Spins run at 0.37 rad/s.
   Pure
   pursuit cuts the entry of a 1 m arc inside by a few cm (0.8 m lookahead);
   watch the cross-track on the first arcs. Above 0.40 m/s the nanoScan3 protective field must be validated for
   the speed first (stopping from 0.85 needs about 0.72 m at 0.5 m/s² plus
   the reaction time).
   **Go home** (2026-11-03): the last step. End the route up to 5 m behind the
   start pose (HOME), roughly facing it, and press *Go home*: the vehicle drives
   forward to HOME and stops on HOME's heading. Already on HOME's line it is a
   straight; otherwise the editor draws an S-curve (never tighter than R 1.0 m,
   never more than 30° off HOME's line, at most 20° heading error at its start)
   that runs at 0.30 m/s. The route then always closes, so a repeat count above 1
   works without hand-tuning the last straight. When it cannot be built the log
   says why and how far back to end the route instead. A wait (a release point)
   may come right before it.
5. Press **Validate**. The robot checks the route against the map, including
   the footprint along every straight and the area every turn actually sweeps
   (a 90° turn does not check behind the vehicle; a 180° does). The footprint
   is grown by the 0.05 m margin (footprint.yaml `margin_m`, one cell) and every cell under it must be free:
   unknown (grey) cells count as blocked. The margin is smaller than the 0.10 m cross-track the
   executor tolerates while running, so leave visible clearance to walls when drawing. The dashed outlines on the map are exactly
   these areas; a failed one is red. A start pose or turn next to the wall the
   survey began against is the usual failure: move the start forward.
   A route that leaves the map, even partly, is refused. Problem steps turn red
   and the reasons are listed under **Result**, with the length, total turning,
   and whether the route closes on its start.
6. Press **Save revision**. The result shows `saved <route> rev M`. An invalid
   route is refused.
7. Press **Create mission from saved revision**. The result shows
   `mission <id> created`. The id is `<map>_rev<N>_<route>_rev<M>`. A mission
   ties this exact route revision to this exact map revision, and the Run page
   lists missions. Creating it again with the same references is harmless; an
   existing id with different references is refused.

### C. Run the route (`Run` page)

1. **Activate the map.** Under **View / activate**, pick the same
   `<id> rev N` and press **Use this map on the vehicle**. The vehicle must be
   stopped and the selector on MANUAL. The mode goes `TRANSITIONING` →
   `NAVIGATION`, and the **Active** tile shows `<id> rev N`. Nothing moves.
2. **Tell it where it is.** Keep the active map selected in **View / activate**.
   **Set initial pose on map** and **Confirm** are greyed out while another map is
   in view, and a pose drawn on a map that is no longer active is refused. Press
   **Set initial pose on map**, then on the map click the vehicle's real position
   and **drag towards the way it faces**, and release. Localisation shows
   `CHECKING`. The red scan points should now lie
   on the map's walls.
   - If they don't line up, set the pose again more carefully.
   - Jog a short distance (about 0.5 m forward and back, a small turn) with the
     pad on this page. This helps it converge.
3. **Confirm.** Wait until the **Can confirm** tile shows `YES`: covariance
   small, a recent scan comparison with **Scan match** ≥ 0.6, sensors fresh
   (sensor ages over 1 s turn yellow). A new initial pose or **Reset** clears all
   earlier evidence, so it can take a moment to become `YES` again. Check yourself that the
   red scan overlays the walls, then press **Confirm: scans align**. The state
   becomes `READY`. **Reset** starts localisation over.
4. **Position the vehicle** on the route's start pose, within 0.10 m and 5°,
   using the jog pad. The dark-blue arrow and outline show where the vehicle
   thinks it is, and the planned route is drawn in steel blue. A red
   `STALE SCAN` or `STALE POSE` label means that overlay is old; do not position
   against it.
5. **Load.** Pick the mission in **Mission**. Only missions for the active map
   revision are listed. Press **Load (READY)**. The run state becomes `READY`.
6. **Go.** Clear the area. Switch the physical selector to **AUTO**, then press
   the physical **Start** button once. The run state becomes `EXECUTING`.
   - `Start refused: 0.23 m / 7.0 deg from the route start; reposition
     manually`: switch to MANUAL, jog onto the start, switch back to AUTO and
     press Start.
   - `localisation not READY`: go back to steps 2–3.
7. **While running**:
   - The physical controls always win: E-stop, or switching to MANUAL, stops
     it.
   - **Pause** stops it and keeps its progress. To continue: **Prepare resume**,
     then physical **Start**.
   - `BLOCKED` is a hold with a cause, shown on the State tile (2026-09-19,
     auto-resume). It keeps its progress and continues from where it stopped:
     - **lidar stop**: a person or object entered the nanoScan3 protective
       field and the safety chain took the drives' torque. Once the field is
       clear and the drives are back, the run **continues by itself** after
       2 s of all-clear. No button.
     - **E-stop**: the drives lost torque with the field clear (the E-stop
       button). Release it; the run waits for a physical **Start** (no
       Prepare resume needed).
     - **controller stop**: Nav2 gave up on the move (usually "collision
       ahead"). Continues by itself when clear, at most 3 times per step,
       then FAULT.
     - There is no **obstacle** hold any more (2026-09-20). The executor does
       not look at the scan, so nothing but the protective field and the
       E-stop will stop the vehicle for something in the way — and the field
       looks forward only, so a reverse step or a spin has no cover at all.
     - Every hold waits until the vehicle is still, on its segment and in
       the corridor, the drives have torque, the field is clear and
       localisation is READY; the reason says what it is waiting for. MANUAL
       during a hold aborts the run. **Prepare resume** + Start still works
       as an override.
     - Executor parameters: `auto_resume_enabled` (false = the old
       Prepare resume + Start), `auto_resume_clear_s` 2.0,
       `auto_resume_estop` false, a blocked path (controller abort) is retried
       until it clears, never a FAULT,
       `prereq_grace_s` 0.5.
   - **Abort** ends the run. The mission must be loaded again.
8. **Finished.** The run state is `DONE`. Load the same or another mission and
   press Start again for another run. Every run needs its own physical Start.
   To leave navigation, **Abort** any active or paused run, then press
   **Return to idle**.

Mode changes (a new survey, another map) are refused while a run is `READY`,
`EXECUTING`, `PAUSED` or `BLOCKED`. Abort the run first.

| Run state | Meaning, and what to do |
|---|---|
| `READY` | Mission loaded, waiting for AUTO + physical Start. |
| `EXECUTING` | Driving the route. |
| `PAUSED` | Operator pause. **Prepare resume**, then physical Start. |
| `BLOCKED` | A hold: lidar stop or controller stop continue by themselves once clear; after the E-stop button press Start. See "While running". |
| `FAULT` | Sensor, drive, panel or localisation problem, or a tolerance was exceeded (for example "passed the endpoint"). A sensor prerequisite must fail continuously for `prereq_grace_s` (0.5 s) before it counts, so one dropped frame is not a fault; losing wheel feedback while moving still stops at once (a `BLOCKED` hold, not a fault). Read the reason and the Monitor/Alarms pages, fix the cause, press **Acknowledge fault** (or the physical **Reset** button, with the vehicle at rest: it does exactly the same and nothing more - no motion, no resume, no drive or supervisor fault clearing). That does not resume: localise again if needed, reposition, load, Start. A run whose Nav2 goal never reported an outcome stays barred until navigation is stopped and started again, acknowledgement or not. |
| `DONE` | Route complete. Load a mission again for another run, or Return to idle. |

| Localisation | Meaning |
|---|---|
| `UNLOCALIZED` | No initial pose yet. Set one. |
| `CHECKING` | Pose given, converging. Confirm once `can_confirm` is true and the scans align. |
| `READY` | Confirmed. Runs can load and start. |
| `LOST` | A sensor went stale, uncertainty grew, or the pose jumped. Set the initial pose and confirm again. |

Wheel feedback that arrives but is flagged invalid counts as missing: localisation
treats the wheels as stale, and a loaded or running route faults with
`wheel feedback invalid`.

## 3. Diagnostics (Monitor)

Use `/monitor`, `/io`, `/alarms` and `/params` for read-only diagnosis. The
pages consume owner-published snapshots; opening more browser clients must not
create another CAN or Modbus poller. Drive-alarm reset remains a physical power
cycle where required by the drive procedure.

`/monitor` (engineer view) shows, top to bottom:

- the active map, the last saved survey and the operation id;
- **Standing now**: every standing alarm with its code and what clears it;
- **Readiness**: base ready, panel, drives, command source, localisation, manual
  and autonomous available;
- **Restart service** (the same server-side refusals as on Home);
- the `/diagnostics` cards: `drive/left`, `drive/right`, `can/bus` and `imu/mls`.

### Blind run (`/blindrun`, engineer view)

Encoder-only test moves, to measure the encoders and the vehicle on the floor
(restored 2026-11-02; the other branches had it on their Commission page). It is
an exclusive IDLE substate: while a move is held or running,
ordinary jog and mapping/navigation mode requests are refused. These are not the
preset survey moves on **Maps**, which stay as they are.

1. Select **MANUAL**, switch the drives on (DI09), make sure the vehicle is
   stopped and the test area is clear.
2. Pick the move and fill in its numbers:
   - **Straight**: forward/reverse and distance (m).
   - **Rotate**: CCW/CW and angle (°).
   - **Arc**: left/right, angle (°) and radius (m). An arc always goes forwards,
     and its radius must be at least half the track (0.244 m); tighter, use Rotate.
   - **Speed**: 0.05–0.80 m/s (`blind_run.max_speed_mps`). Above 0.40 m/s the
     page warns and shows the ramped stopping distance. Confirm the
     nanoScan3/FX3 protective field is sized for the speed before going above 0.40.
3. Tick the checklist (area clear, E-stop in reach, speed within the field
   sizing), then **Hold move**. This validates the move and moves nothing.
   Holding is refused unless the supervisor is in IDLE, the wheels are known to
   be still and the encoder scale is fresh. Several segments can be held as
   JSON under **Advanced**. A plan `id` may use letters, digits, `_`, `.` and
   `-` only, starts with a letter or digit, max 64.
4. Review **Held move**: the wheel count targets and the commanded dx/dy/heading.
5. Press physical **Start** once to execute. Only a Start pressed after the plan
   was held counts. Browser input cannot start it.
6. **Clear / abort** stops and invalidates the job. So do the selector to AUTO,
   an **E-stop**, panel loss, a supervisor restart or generation change, an
   encoder-scale change and stale wheel counts. An aborted run still writes its
   evidence, including the interrupted segment. `DONE` or `ABORTED` cannot
   replay on another Start; hold the move again. An E-stop aborts a blind run
   like a survey move, never to resume (owner, 2026-11-02): a move cut by a
   stop measures the stop, not the move. Runs and waits on a route are held
   instead (§2 C).
7. Measure where the vehicle ended up, in the start frame: x forward, y left,
   heading counter-clockwise +. Measure the axle-midpoint mark, and take the
   heading from a second mark ~1 m ahead. Enter dx/dy (mm) and heading (°)
   under **Measured result** and **Save measurement**. **History** then lists
   commanded, measured and error per run. The measurement is stored beside the
   evidence (`~/.amr/commissioning/<run>.measured.json`); saving again replaces it.
8. Keep the displayed evidence path with the test record. If the reason says
   `evidence NOT written`, the file is missing; do not record an old path.

Profile velocity only: the software position loop runs in the commissioning
node and goes through the mux like any other source (`blind_run.*` in the
profile). The profile-position (PP) backend was removed on 2026-10-30 and is
not part of this; 607Ah/6081h stay unwritable (`agv_core/drivers/canbus/guard.py`).
Lease bit 4 and mux source 5 (COMMISSIONING) carry the job again, with their old
meaning.

## 4. Fault recovery

Every code below is a row in `agv_core/alarms.py`; the operator sees the title and
the action, this table adds the engineer's note. Regenerate after changing the
catalogue: `python3 -m agv_core.alarms --md` (tests/test_alarms.py fails if this
block and the catalogue disagree). The laminated card is `--card`.

<!-- BEGIN GENERATED: python3 -m agv_core.alarms --md -->
<!-- generated by python3 -m agv_core.alarms --md; do not edit by hand -->
Every code the vehicle can show, what the operator does about it, and the
engineer's note. Generated from `agv_core/alarms.py`.

### Cleared by: auto

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `FIELD_PROTECTIVE` | warn | Stopped: something is in the protective field | Clear the area in front of the vehicle; it continues by itself. | The mux zeroes every AUTO source while /output_paths says the protective field is violated - the software side of a stop the scanner's OSSD pair already makes. |
| `GENERATION_MISMATCH` | warn | Motion is held after a mode change | Wait. If it stays, press Restart on the Home page. | A command or permit carries a superseded generation; the mux drops it by design. |
| `GLS_DOWN` | warn | The floor-label camera is not answering | Nothing to do now: it never stops the vehicle. Tell the engineer if it stays. | gls_node: no reply to the read-only keepalive for 3 x gls.keepalive_s (cable, power, enp3s0 192.168.4.2, or SOPAS left in Live mode, which mutes the host port). Outside the safety path: no position fix from the camera until it answers again. |
| `LOC_CORRECTED` | warn | The position was corrected; the vehicle kept going | Nothing to do. If it repeats at the same place, see the report on the Alarms page. | map->odom correction over the jump trigger but within jump_verified_max_m, with the scan fitting the map before and after it: kept READY, reported (loc_losses). |
| `LOC_RELOCALIZED` | warn | Position found again; the run continues | Nothing to do. If it repeats at the same place, see Localisation losses on the Alarms page. | Automatic relocalisation confirmed: converged within reloc_max_shift_m of the stop, scan fit >= reloc_fit_min. |
| `LOC_RELOCALIZING` | warn | Position lost during the run: finding it again | Keep clear; the AMR finds its position where it stopped and continues by itself. | Route executor hold 'loc' after LOC_JUMP / LOC_SCAN_MISMATCH / LOC_COV_GREW while executing; the monitor set the initial pose at AMCL's estimate (relocalize.py). |
| `NOT_LEASED` | warn | Waiting for the supervisor | Wait. If it stays, press Restart on the Home page. | Mux has no fresh ControlLease (supervisor down, starting or in a transaction). |
| `PATH_BLOCKED` | warn | Stopped: the way ahead is blocked | Clear the route. The vehicle tries again by itself. | Nav2 controller aborted the step (hold cause 'controller'). Retried every few seconds for as long as the path is blocked, never a FAULT (owner 2026-11-03). |
| `SERVICE_CRASHED` | warn | The vehicle software restarted itself | Nothing to do. Tell the engineer if it keeps happening. | A marker from THIS boot survived: the supervisor died and systemd's Restart=on-failure or the 30 s watchdog brought it back. The drives were disarmed by the teardown or by their own 1016h, so no power cycle is needed. journalctl -u amr.service has the reason. |
| `SOURCE_TIMED_OUT` | warn | The command stopped arriving | Press and hold again. If a page is jogging, check the Wi-Fi link. | The selected stream went stale (cmd_timeout_s 0.2 s): closed tab, dropped Wi-Fi or a released button - indistinguishable to the vehicle, by design. |
| `WEB_BUG` | warn | A page did not work | Reload the page. Save a report if it keeps happening. | Unhandled exception in the Flask app or the page script; the reference id is in the web log. |
| `BOOT_READY` | info | The vehicle is ready | Nothing to do. | End-of-boot report from the supervisor; the text lists anything missing. |
| `COSTMAP_CLEARED` | info | The vehicle refreshed its view of nearby obstacles | Nothing to do. | route_executor cleared the controller's local costmap: at a Continue that releases a wait, or after costmap_clear_after_s (10 s) of a blocked path. Marks behind the scanner's view never clear by themselves (2026-11-03, route-itg-02 s12). |
| `FIELD_BLOCKED` | info | Stopped: something is in the safety field | Clear the area. The vehicle starts again by itself. | Scanner OSSD -> FX3 -> STO. Executor hold cause 'field'; auto-resume after the hold window. |
| `FIELD_WARNING` | info | Slowed or stopped: something is in a warning field | Clear the area ahead; auto speed returns by itself when the warning fields clear. | cmd_mux scales every AUTO source while a warning field is occupied: warning 1 (outer, 1.77 m ahead) x0.5, warning 2 (inner, 0.97 m) a deceleration stop within 0.40 m, the stricter winning; the slowdown and the recovery ramp over 1 s. Paths and factors: amr_bringup/config/scanner_fields.yaml. |
| `GLS` | info | The floor-label camera changed | Nothing to do. | gls_node: the GLS621 answers again after a GLS_DOWN. |
| `INHIBITED` | info | Motion is held by the supervisor | Wait for the mode change to finish. | Lease allowed = 0: a transaction, a save, STARTING, FAULT or STOPPING. |
| `LABEL_RECORDED` | info | A floor label was recorded on the map | Nothing to do. | Record label here, or HOME at survey start: a new label-set revision (gls-wait-coding-plan §C). New missions bind it; existing ones keep theirs. |
| `MODE_CHANGE` | info | The vehicle changed mode | Nothing to do. | Supervisor FSM transition; the text carries from -> to. |
| `MUX_SOURCE` | info | The vehicle is taking commands from somewhere else | Nothing to do. | Command-source edge in the mux (pendant, manual, follow, rotate...). |
| `NO_PERMIT` | info | Waiting for a mission step | Nothing to do; the executor drives this. | AUTO with no MotionPermit, or a permit whose source has no fresh /cmd_vel. |
| `NO_SOURCE` | info | Nothing is asking the vehicle to move | Nothing to do. Press Start, or jog from the Manual page. | Selector position is fine but no fresh command stream is selected. |
| `PANEL` | info | The control panel changed | Nothing to do. | Selector, Start/Reset edge or pendant change from panel_node. |
| `RUN_SUMMARY` | info | A mission finished | Nothing to do. | One line per run at DONE/ABORT/FAULT: duration, distance, holds by cause. |
| `WAITING_FOR_PREREQ` | info | Waiting: a sensor or the panel is not ready yet | Wait a moment. If it stays, look at the Alarms page. | Hold cause 'pending': a prerequisite went stale inside the grace window. |

### Cleared by: start button

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `DRIVES_NOT_READY` | warn | Stopped: the motors are not powered | Switch the pendant enable (arm) on. AUTO: then press Start (after the cabinet's safety reset if the E-stop was used). | DriveStatus not operational while the mux had a command to apply. Since 2026-10-08 the drives are powered only on demand (DriveStatus.power_reason says why not). |
| `ESTOP_HOLD` | warn | Stopped by the E-stop: press Start or Continue | Make sure the area is clear, then press Start or Continue. The route continues where it stopped. | The run's 'estop' hold (owner policy 2026-10-30: an emergency is a physical stop and never ends the run). Shown once the emergency is NORMAL again; never resumes by itself. |
| `SAFETY_RESET_NEEDED` | warn | Motors are off: the safety circuit needs a reset | Press the blue Reset button on the cabinet. | Both drives in 'Switch on disabled' (statusword 0x1270): STO held by the FX3, arming retries every 2 s and succeeds the moment the chain closes. |

### Cleared by: continue button

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `CONTINUE_REFUSED` | warn | Continue did not start the vehicle | Fix what the detail says, then press Continue again. | A Continue/Start at a wait failed its gate: E-stop not reset, MANUAL, localisation not READY, wheels moving, or the vehicle moved off the wait pose (verify_position_m / verify_heading_deg). RunState.continue_refused carries the reason. |
| `RUN_WAITING` | info | Waiting at a stop: press Continue | Hitch or release the trolley, step clear, then press Continue (or Start). | The run is WAITING at a route wait step (gls-wait-coding-plan §A.4). Drives disarmed; Continue re-checks localisation, AUTO, still wheels and the wait pose, then the pre-move warning runs. Survives an emergency (owner, 2026-10-30). |

### Cleared by: reset button

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `ESTOP` | error | EMERGENCY STOP | Make the area safe. Release (twist) the E-stop button, then press the blue Reset button on the cabinet. | DI08 (panel.di_estop_ok, Flexi Soft Q3) LOW, or the panel image invalid/stale. STO is the Flexi Soft's; the PC withholds power and the lease, inhibits the mux and holds a moving run (emergency-suite plan; it never ends a run or a wait). Also the executor's 'estop' hold when it can only infer it. |
| `ESTOP_RESET_NEEDED` | warn | E-stop released: press Reset | Press the blue Reset button on the cabinet. | DI08 is HIGH again but no Reset edge has been seen since the emergency. |

### Cleared by: ack

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `EXEC_ACTION_FAILED` | error | The navigation software refused a step | Press Acknowledge on the Run page. If it repeats, call the engineer. | Nav2 action aborted/rejected/lost, or its server was unavailable. |
| `EXEC_ENDPOINT_MISSED` | error | The step finished in the wrong place | Press Acknowledge on the Run page, then reload the mission. | Endpoint position/heading error over tolerance at step completion. |
| `EXEC_OFF_PATH` | error | The vehicle drifted off its route | Press Acknowledge on the Run page, then reload the mission. | Cross-track over the route's allowance; the step was interrupted. |
| `EXEC_OVERSHOOT` | error | The vehicle went past the step's end | Press Acknowledge on the Run page, then reload the mission. | Travel past the straight's length, or a turn overshoot, beyond tolerance. |
| `EXEC_POSE_INJECTED` | error | The position was changed while the vehicle was moving | Press Acknowledge on the Run page, then reload the mission. | An /initialpose arrived during EXECUTING: progress is no longer trustworthy. |
| `EXEC_PREREQ_LOST` | error | A sensor or the panel dropped out mid-step | Press Acknowledge on the Run page, then check the Alarms page. | A prerequisite stayed bad past prereq_grace_s, or wheel feedback stopped without a safety stop. |
| `EXEC_TURN_FAILED` | error | The turn did not come out right | Press Acknowledge on the Run page, then reload the mission. | Turn centre drift, wrong direction of rotation, or remaining angle out of tolerance. |

### Cleared by: operator

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `EXEC_ROUTE_INVALID` | error | This mission cannot be run | Pick another mission, or call the engineer to fix this one. | Route/mission failed to compile or validate against the active map revision. |
| `LOC_COV_GREW` | error | The vehicle is unsure where it is | Set the initial pose on the Run page and confirm the scans line up. | AMCL covariance over the limit for longer than cov_hold_s. |
| `LOC_JUMP` | error | The vehicle's position jumped | Set the initial pose on the Run page and confirm the scans line up. | map->odom correction over the jump trigger since the last accepted pose, not confirmed by the scan (over jump_verified_max_m, a poor fit before it, or no fit after it). |
| `LOC_LOST` | error | The vehicle lost its position | Set the initial pose on the Run page and confirm the scans line up. | Umbrella code for a LocalizationState LOST without a more specific cause. |
| `LOC_RELOC_FAILED` | error | Could not find the position again by itself | Set the initial pose on the Run page, confirm, then Continue route. | Automatic relocalisation timed out, converged too far from the stop, or the attempts per run (auto_reloc_max) are used up; the run FAULTed. |
| `LOC_SCAN_MISMATCH` | error | The vehicle is not where it thinks it is | Set the initial pose on the Run page and confirm the scans line up. | Too many beams pass THROUGH mapped obstacles (scan_long over the limit): the unambiguous wrong-pose signature. |
| `MAP_SAVE_FAILED` | error | The map was not saved | Try Save again on the Maps page. If it fails twice, call the engineer. | mapping_session save returned an error; the survey is still open. |
| `AUTO_LOAD_REFUSED` | warn | The last mission could not be loaded by itself | Load a mission on the Run page; it is then loaded by itself each time. | route_executor auto_load: the remembered mission (last_mission.json) was refused by the load checks (another active map, an invalid route, a missing file). |
| `DEST_REFUSED` | warn | The destination button did not start a run | Fix what the detail says, then press the destination button again. | A UBC/ITG/PISTON press (panel.destinations, 2026-11-03) started nothing: MANUAL, a run already under way (moving, held or WAITING), a FAULT, the E-stop, no route assigned in the profile, no saved mission for the route on the active map, or the Start refusal (off the route start pose, localisation not READY, ...). |
| `LABEL_RECORD_FAILED` | warn | The HOME floor label was not saved with the map | Park on HOME, localise, then use Record label here. | mapping_session saw the HOME label at survey start but could not write the new map's label set (the text says why). The map itself was saved. |
| `LOC_NOT_SET` | warn | The vehicle does not know where it is yet | On the Run page: pick the map, set the initial pose, then confirm. | LocalizationState UNLOCALIZED: no accepted initial pose since the layer started. |
| `SAVE_UNKNOWN_OUTCOME` | warn | The map may or may not have been saved | Check the Maps page for a new revision before saving again. | The save RPC passed its deadline after the request was accepted; a late answer is ignored. |
| `SURVEY_NOT_READY` | warn | The vehicle is not ready to survey | Fix what the Alarms page lists, then start the survey again. | mapping_session refused: sensor ages or mode preconditions not met. |
| `SURVEY_START_REFUSED` | warn | The survey would not start | Fix what the Alarms page lists, then start the survey again. | mapping_session refused the start request. |
| `LOC_NOT_CONFIRMED` | info | Waiting for you to confirm the position | On the Run page, check the scans line up with the map and press Confirm. | LocalizationState CHECKING with can_confirm true. |

### Cleared by: recover

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `COORDINATOR_UNAVAILABLE` | error | The survey software is not answering | Press Recover on the Home page, then start the survey again. | mapping_session service missing when the supervisor needed it. |
| `LAYER_EXITED` | error | The mapping or navigation software stopped | Press Recover on the Home page, then load the mission again. | The mode layer's process group exited unrequested; the base is untouched. |
| `LAYER_NO_READINESS` | error | The new software never reported ready | Press Recover on the Home page, then try the mode change again. | The layer produced no readiness signal at all: it probably died during entry. |
| `LAYER_START_TIMEOUT` | error | The new software did not start in time | Press Recover on the Home page, then try the mode change again. | The new layer did not report ready inside layer_start_s; see ~/.amr/logs/layer.log. |
| `LIFECYCLE_STARTUP` | error | The new software did not become ready | Press Recover on the Home page, then try the mode change again. | A lifecycle node failed to configure/activate during a transition. |
| `LOOP_ERROR` | error | Internal error in the vehicle software | Press Recover on the Home page. If it repeats, save a report and call the engineer. | The supervisor caught an exception in its loop and failed the running operation. |
| `MUX_ACK_TIMEOUT` | error | The mode change did not finish | Press Recover on the Home page. | The mux never acknowledged the new generation inside the barrier budget. |
| `SURVEY_RPC_TIMEOUT` | error | The survey command got no answer | Check the Maps page for a new revision, then press Recover. | returned/abort/save RPC passed its deadline; a late answer is ignored. |
| `BASE_NOT_READY` | warn | Not ready: the vehicle's basics did not come up | Press the safety reset on the cabinet. The vehicle becomes ready by itself. | Boot budget expired with drives/panel/mux/wheel feedback missing. Since 2026-09-22 the supervisor leaves FAULT on its own once the base reports ready. |

### Cleared by: power cycle

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `DRIVE_ALARM` | error | Drive alarm | Switch the drives off and on, then press Recover. | CANopen EMCY latched; 40C0h (alarm reset) is on the write deny-list, so only a power cycle clears it. See the code in the detail (e.g. 8130h = heartbeat lost). |
| `DRIVE_FAULT` | error | A drive is in fault | Switch the drives off and on, then press Recover. | CiA-402 Fault state on a node while armed. |
| `DRIVE_SILENT` | error | A drive stopped answering | Switch the drives off and on, then press Recover. | No TPDO/heartbeat from a node for driver_timeout_s while armed. Check CAN wiring and drive logic power before blaming the PC. |
| `DRIVE_STOP_UNCONFIRMED` | error | The vehicle cannot confirm it stopped | Stay clear. Switch the drives off and on, then press Recover. | Fault stop without positive standstill evidence: the PC heartbeat is withheld on purpose so each drive's own 1016h trips. |
| `UNCLEAN_SHUTDOWN` | warn | The vehicle lost power last time | Switch the drives off and on, then press the safety reset on the cabinet. | A running.json marker survived from before this boot (amr_bringup/uptime.py). The drives lost the PC heartbeat, so 8130h is latched and only a power cycle clears it. A marker from the SAME boot is a service crash instead and needs no drive action. |

### Cleared by: restart service

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `BASE_EXITED` | error | Needs service: the vehicle software stopped | Switch the drives off and on, then press Restart on the Home page. | The base process group died. Recovery cannot rebuild it: the drives must be re-armed. |
| `BOOT_ERROR` | error | Needs service: the vehicle software failed to start | Press Restart on the Home page. If it repeats, call the engineer. | Exception while spawning the base/web groups; see journalctl -u amr.service. |
| `LAYER_NOT_EMPTY` | error | Needs service: old software would not go away | Press Restart on the Home page. | STOP_OLD found surviving members of the previous layer's process group. |
| `LAYER_STOP_TIMEOUT` | error | Needs service: software would not stop | Press Restart on the Home page. | The layer did not exit inside its stop budget. |
| `SPAWN_FAILED` | error | Needs service: a program would not start | Press Restart on the Home page. If it repeats, call the engineer. | Group.spawn failed (missing executable, bad environment). |
| `STOP_TIMEOUT` | error | Needs service: shutdown did not finish | Press Restart on the Home page. | Teardown exceeded its budget; systemd's TimeoutStopSec (55 s) is the backstop. |
| `SUPERVISOR_DOWN` | error | Needs service: the vehicle software is not running | Press Restart on the Home page. If it repeats, call the engineer. | No ModeState/lease reaching the web. systemd restarts on failure and on a watchdog timeout. |
| `WEB_DOWN` | error | Needs service: this page's server keeps stopping | Press Restart on the Home page. | The supervisor respawned the web group three times and gave up. |

### Cleared by: engineer

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `CAN_DOWN` | error | The motor controllers cannot be reached | Call the engineer: check the CAN adapter (USB) and its cable. | can0 cannot be opened (the CANable unplugged or not enumerated). drive_node retries every 2 s and reports link_state 'no bus'; the row clears itself once the bus opens. |
| `DISK_FULL` | error | No storage left: new maps cannot be saved | Call the engineer: the disk is full. A route already running is not affected. | Under 200 MB free. Survey start and map save are refused; navigation is deliberately NOT gated - a full disk must not stop a vehicle that is already moving. |
| `ESTOP_RESET_FAILED` | error | Reset did not take | Check the E-stop is released and press Reset again. If it repeats, call the engineer. | Reset edge with DI08 HIGH, but the drives stayed in 'Switch on disabled' (STO held) for panel.estop_reset_timeout_s. DI08 dropping again is a new emergency, not this. |
| `GLS_CONFIG_MISMATCH` | error | The floor-label camera's settings changed | Call the engineer. The vehicle still runs; the camera is ignored. | gls_node read back OFtplstr1 / GlsCodeSel / DCDMVerISO15415 and they differ from the profile (gls.expect_template) or REQUIRED in gls_link.py - e.g. the device was reset to factory settings. Every frame is dropped until the next connect verifies. Re-apply the saved SOPAS parameter file; never write settings from the vehicle. |
| `LOC_GATE_FAILED` | error | The position check stopped running | Call the engineer: the map/scan check failed. | Scan-consistency gate or the transform it needs failed; no evidence either way, so LOST. |
| `LOC_STREAM_STALE` | error | A sensor the vehicle navigates by went quiet | Call the engineer: a sensor stopped (see the detail for which). | scan/wheels/imu/tf/amcl age over the spec 2.4 limits. |
| `PANEL_STALE` | error | The control panel is not answering | Call the engineer: the panel wiring or the I/O island is down. | No fresh, valid PanelState: Modbus DIO at 192.168.1.40 lost, or panel_node down. No motion authority at all without it. |
| `DISK_LOW` | warn | The vehicle is running out of storage | Call the engineer: old maps and reports need deleting. | Under 1 GB free on the state or maps filesystem. Surveys and saves still run; at 200 MB they are refused (DISK_FULL). |
| `FIELD_UNKNOWN` | warn | Stopped: no safety-scanner field data | Check the scanner cable and that its driver is running (Monitor page). | Supervised AUTO is zeroed with no fresh /output_paths: a field state nobody can see is not clear. The OSSD stop path does not depend on this data link. |
| `SCANNER_SILENT` | warn | The safety scanner is not sending data | The vehicle can still be driven by hand. Call the engineer before running a mission. | No /scan at all: the nanoScan3 driver could not reach 192.168.3.1:6060 (cable, power, netplan) or AMR_LIDAR=false. The driver is an OPTIONAL launch member, so nothing faults and nothing else reports it - this row is the only place it shows. The vehicle's STOP is the scanner's OSSD pair into the FX3 and is unaffected by the data link; mapping, navigation and localisation are dead without it. |
| `SCANNER_STALE` | warn | The safety scanner stopped sending data | The vehicle can still be driven by hand. Call the engineer before running a mission. | /scan arrived and then stopped: driver died mid-run, or the Ethernet link dropped. Expected rate is 34 Hz. |

<!-- END GENERATED -->

### Notes that are not codes

- `8130h` on both drives: the drive lost the PC heartbeat. Stop the service,
  correct the PC/CAN cause, perform the required drive power-cycle procedure,
  then start to IDLE. The drive owner also withholds the heartbeat on purpose
  when it cannot confirm a fault stop (Monitor shows `stop_unconfirmed` and
  `heartbeat_withheld`); treat that as a drive/CAN fault, not a PC crash.
- A drive faults while its heartbeat is still arriving: check Monitor for
  `tpdo1_age_s` / `tpdo2_age_s`. Losing either feedback PDO alone faults the
  drive owner.
- Where to look: `~/.amr/logs/base.log` (drives, panel, mux, EKF),
  `~/.amr/logs/layer.log` (the current mapping or navigation layer),
  `~/.amr/logs/events.jsonl` (the operator event log, kept across restarts),
  `journalctl -u amr.service` (the supervisor itself). **Save report** on the
  Alarms page packages all of these.
- Panel invalid: check the DIO island at `192.168.1.40`. Reconnection must not
  create a Start edge or replay a jog.
- No ROS discovery in an engineering shell: source `env/vehicle.sh`. Hardware
  refuses any domain other than 10.

## 5. Install rehearsal and validation

Build a coherent overlay before installing unit files:

```bash
cd ~/agv_can/amr_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
./deploy/validate.sh
```

`validate.sh` checks script syntax, required paths and systemd unit syntax. It
does not touch service or hardware state. The installer backs up installed AMR
unit files, installs the source units and runs `daemon-reload`; it deliberately
does not enable, disable, start, stop or restart anything:

```bash
sudo ./deploy/install.sh
```

Record the printed `/var/backups/amr-units/<timestamp>` path. Confirm the
installer preserved current state with:

```bash
systemctl is-enabled amr
systemctl is-active amr
```

### Deploying a build that removes nodes or interfaces

colcon never prunes an overlay, and Humble does not refuse a peer whose message
definition differs (there is no type hash), so a build that deletes nodes,
launch files or message fields is deployed with a **full clean rebuild** and a
**witnessed restart of the whole service**, never a partial one. The trackless-only
build of 2026-10-27 is one (`manuals/trackless-only-removal-plan.md`, Phase 7):

```bash
sudo systemctl stop amr.service
cd ~/agv_can/amr_ws && rm -rf build install log && colcon build --symlink-install
sudo ./deploy/install.sh && ./deploy/validate.sh
ls install/*/lib/*/ install/*/share/*/launch/ | grep -E 'rfid|tape|line_layer|p0_loc|web_panel'   # must be empty
sudo systemctl restart amr.service      # witnessed; supervisor and web together
```

On a vehicle that ran the tracked product, archive the tape-era data to a
tarball first, then purge it and rotate the two journals so the event history
does not show retired codes as UNKNOWN rows: `~/amr_maps/*/tapegraph`,
`~/.amr/teach`, `~/.amr/line_graph.json`, `~/.amr/reports/tape_health`,
`~/.amr/mls_cal`, the tracked-era `~/.amr/runs/*`, and `~/.amr/logs/events.jsonl*`
and `~/.amr/operations.jsonl` (moved aside after the tarball is verified). The
exact commands are in the removal plan, Phase 7.

### Two sudo rules the operator pages need (optional, install once)

Without them the vehicle runs exactly as before; the Home page's **Restart**
answers "not permitted" and a saved report carries a note where the journal
would be. With them, an operator can restart the service and collect evidence
without SSH. The file grants those two exact commands and nothing else:

```bash
sudo install -o root -g root -m 0440 \
  ~/agv_can/amr_ws/deploy/amr-web.sudoers /etc/sudoers.d/amr-web
sudo visudo -c
```

The **Restart** button is refused by the page while the wheels are turning or a
run is live; the sudo rule itself does not check that, which is why the command
list is kept this narrow.

### Power-loss hardening (optional, install once)

`sudo ./deploy/install.sh` now also installs two drop-ins and reports them:

| File | Where | What it does |
|---|---|---|
| `deploy/sysctl-amr.conf` | `/etc/sysctl.d/60-amr.conf` | dirty pages expire after 5 s instead of 30 s, `swappiness=0` |
| `deploy/journald-amr.conf` | `/etc/systemd/journald.conf.d/amr.conf` | journal capped at 300 M (it was uncapped, 896 M on 2026-09-22) |

Two manual steps the installer deliberately leaves alone:

```bash
sudo systemctl restart systemd-journald       # apply the cap
sudo journalctl --vacuum-size=300M            # shrink what is already there
sudo swapoff -a && sudo sed -i '/swap.img/d' /etc/fstab && sudo rm -f /swap.img
```

### Data on its own volume (W8, optional)

The VG has ~130 G unallocated, so `/` does not need resizing. With the service
**stopped**:

```bash
sudo lvcreate -L 40G -n amr-data ubuntu-vg
sudo mkfs.ext4 -L amr-data /dev/ubuntu-vg/amr-data
sudo mkdir -p /data/amr
echo 'LABEL=amr-data /data/amr ext4 defaults,data=journal,commit=5,nofail,x-systemd.device-timeout=10 0 2' \
  | sudo tee -a /etc/fstab
sudo mount /data/amr
sudo install -d -o $USER -g $USER /data/amr/state /data/amr/maps
rsync -a ~/.amr/ /data/amr/state/ && rsync -a ~/amr_maps/ /data/amr/maps/
mv ~/.amr ~/.amr.pre-volume && mv ~/amr_maps ~/amr_maps.pre-volume
ln -sfn /data/amr/state ~/.amr && ln -sfn /data/amr/maps ~/amr_maps
sudo install -o root -g root -m 0644 -D deploy/amr-data-volume.conf \
  /etc/systemd/system/amr.service.d/data-volume.conf
sudo systemctl daemon-reload && sudo systemctl start amr.service
findmnt -no OPTIONS /data/amr        # must contain data=journal
./deploy/validate.sh                 # prints "state on /data/amr with data=journal"
```

The drop-in is separate from `amr.service` on purpose: `RequiresMountsFor` on a
mount that does not exist stops the service from starting at all, so it must be
installed **after** the volume is mounted. To undo, delete the drop-in and
`daemon-reload`; the `~/.amr` and `~/amr_maps` symlinks keep every path working.

### Pull-the-plug acceptance (W9)

Three cuts at the wall switch, each followed by a boot and a read of the Home line
and the Alarms history. PASS means the operator can say what happened without help.

1. **Mid-mission** (NAVIGATION, moving in a clear aisle). Expect: drives stop within
   the heartbeat timeout; after boot Home says **Needs service / The vehicle lost
   power last time** naming the mission, the drives need an off/on, and Recover or
   Restart reaches IDLE.
2. **During a map save** (press Save, cut within 1 s). Expect: no revision directory
   without a manifest; the previous revision still loads on the Run page; a
   `.staging-` or `.draft-` directory is left behind, which is evidence, not damage.
3. **Idle with the web open.** Expect: boot to IDLE inside the budget with no operator
   action, and the Alarms history still holds the events from before the cut —
   including the last error, which is the one that is fsynced.

Record each in the acceptance file with the Home line quoted verbatim.

### After changing the alarm catalogue

Section 4 of this runbook is generated. Regenerate it and the operator card:

```bash
python3 -m agv_core.alarms --md     # paste between the GENERATED markers in section 4
python3 -m agv_core.alarms --card   # the laminated one-pager for the vehicle
pytest tests/test_alarms.py         # fails if the catalogue and this file disagree
```

## 6. Cutover history and retirement (U11, 2026-09-21)

The first witnessed cutover from the legacy `agv_controller` unit to
`amr.service` was done during U10, and the legacy controller (`main.py`,
`app/`, `canworker.py`) and the interim `amr_nav.service` / `amr_mapping.service`
units were deleted at U11 (`manuals/obsolete/vehicle-reports/2026-09-21-restructure-and-line-layer.md`).
The last deployable legacy revision is commit `aa02ec7`, tagged `legacy-final`
(removal plan D18).

The tracked product (tape follower, MLS tape reading, RFID, tape missions) was
removed on 2026-10-27 (`manuals/trackless-only-removal-plan.md`); its last
build is the tag `tracked-final`. Bags recorded before then decode only against
that build. The Foxglove bridge was removed on 2026-11-03
(`manuals/foxglove-removal-plan.md`): nothing listens on :8765, and remote views go
through the web app.

On a vehicle that still carries the old units, retire them once:

```bash
sudo systemctl disable --now agv_controller.service amr_nav.service amr_mapping.service 2>/dev/null
sudo rm -f /etc/systemd/system/agv_controller.service /etc/systemd/system/amr_nav.service /etc/systemd/system/amr_mapping.service
sudo systemctl daemon-reload
systemctl list-unit-files 'agv_controller*' 'amr*'    # amr.service is the only one left
```

`amr.service` is the only AGV application boot service. Nothing else on the PC
opens `can0` or the I/O island; the bench tools take the owner lock and refuse
while the service runs.

## 7. Rollback

There is no second controller to fall back to. A rollback is a rollback of the
*build*: engage E-stop, stop the service, check out the previous known-good
commit (tags and `manuals/obsolete/vehicle-reports/` record which builds were accepted),
rebuild, and start again:

```bash
sudo systemctl stop amr.service
cd ~/agv_can && git checkout <known-good>
cd amr_ws && rm -rf build install log && colcon build --symlink-install
sudo ./deploy/install.sh        # if the unit or env files changed
sudo systemctl start amr.service
```

If unit contents themselves must be restored, copy them from the backup path
printed by `install.sh`, then reload systemd:

```bash
sudo cp /var/backups/amr-units/<timestamp>/*.service /etc/systemd/system/
sudo systemctl daemon-reload
```

Re-run the boot check (section 2) before releasing E-stop. A rollback does not
authorize automatic motion or resume any interrupted job. `nav.launch.py` and
`mapping.launch.py` are diagnostics and simulation entry points only; the
unified web app needs the supervisor for any lease, mode or map context.
