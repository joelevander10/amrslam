# Unified AMR operator and cutover runbook

`amr.service` is the intended production service. It starts the hardware base,
operator web app and optional Foxglove bridge, then stays in **IDLE**. It does
not load a map, start a survey, resume a commissioning job or run a route after
boot. The operator selects mapping or navigation from the web app.

The physical controls remain authoritative:

- **MANUAL** permits bounded browser jog or a held commissioning plan.
- **AUTO** permits a loaded route only after a fresh physical **Start** edge.
- E-stop and the FX3/STO chain remain the immediate physical stop path.
- A stale panel, supervisor lease, drive state or command inhibits motion.

The web app is `http://192.168.2.20:5001/`. Its tabs are **Status**, **Manual**,
**Maps** (survey), **Routes** (route editor), **Run**, **Monitor**, **I/O**,
**Alarms** (events), **Params** and **Commission**. Every page has the same left
rail: tiles for Mode, Selector, Drives, Command, Localisation, Run, Wheels and
Generation, with the event log below. A tile with a yellow bar needs attention and
a red bar means a fault or stale data. If the whole rail is greyed out and the
header pill says `DISCONNECTED`, the values are last known, not live. After a
software update, hard-refresh the browser (Ctrl+Shift+R) so it does not keep old
styles or scripts.

## 1. Normal service operation

```bash
systemctl status amr.service
journalctl -u amr.service -f
sudo systemctl start amr.service
sudo systemctl stop amr.service
sudo systemctl restart amr.service
```

A healthy start reaches `IDLE`, with the base and web processes running and no
mapping/navigation layer. The Status page and the rail must show:

- Mode `IDLE`, with no red fault banner on the Status page;
- Selector `MANUAL` or `AUTO` (not `STALE`/`INVALID`) and Drives `ARMED` or `OFF`, without a red bar;
- Command `none` while untouched;
- Active map `—`, Localisation and Run `–`.

Stopping the service revokes the control lease first. The supervisor then stops
the active layer, base, Foxglove and web process groups. The base shutdown
zeros wheel commands, clears the drive heartbeat consumer and de-energizes the
drives; the panel owner drives its output coil low. After stop, check that no
child remains:

```bash
systemctl show amr.service -p ActiveState -p SubState -p MainPID -p Result
pgrep -af 'amr_|nav2|slam_toolbox|foxglove' || true
```

Do not use process-name kills during normal operation. The supervisor owns the
child process groups and performs bounded SIGINT → SIGTERM → SIGKILL cleanup.

## 2. Survey → draw a route → run it

The complete workflow, in order. Only step C moves the vehicle on its own, and
only after a physical Start. Everything else is browser jog under **MANUAL**,
or it moves nothing.

Before you begin:

- The service is in `IDLE` (Status page) and the drives are operational.
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

**Pendant.** The hard-wired jog pendant on the DIO island works under the same
authority as the jog pad (selector **MANUAL**, drives armed, no fault) and
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
   type a distance and press *Forward* / *Reverse* (0.05–10 m at 0.30 m/s), or a
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
them as walls. A route through a spot where a trolley stood is refused, and a
scan return from a trolley that is still there counts as the map, not as an
obstacle. Mark such places as **dynamic areas** (2026-09-19, dynamic-mapping
plan Increment 1). Nothing on this page moves the vehicle, and the mode can
stay `IDLE`.

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
- **During a run**: any scan return inside a dynamic area, within the step's
  envelope, counts as an obstacle, even where the map shows a trolley. A
  trolley that is still there therefore stops the run (**BLOCKED**). Move it,
  then press Start.
- **Localisation monitor**: beams that end in a dynamic area, or where the map
  expects a wall inside one, are left out of the scan match.
- **AMCL** by default still localises against the full map. With
  `AMR_LOC_BLANK_DYNAMIC=true` in `amr.env`, it uses a copy with the dynamic
  areas set to unknown, served on `/map_loc`. The web view keeps showing
  `/map`. Leave it `false` until AMCL covariance and scan match have been
  compared on the same route with both settings on a trolley-heavy map. The
  navigation layer refuses to start if fewer than 500 occupied cells remain
  outside the dynamic areas.
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
4. **Repeat count**: how many times the route runs back to back, 1–100. Use 1
   unless the route returns to its own start pose. A repeated run shows
   `[pass p/P]` in the run reason. **Speed cap**: 0.05–0.60 m/s, new routes
   0.55 (2026-09-19). Use 0.15–0.20 m/s for a first run. **Long straight
   speed**: a straight strictly longer than *Long straight from* (default
   4 m) runs at this speed, up to 0.85 m/s (new routes 0.85); leave the field
   empty for no boost. Boosted straights show `▲0.85` in the step list and
   the result shows how many. **Arc speed** (new routes 0.40 m/s): never
   above the speed cap. Existing routes keep their stored speeds until they
   are edited and saved again (re-save them after the first vehicle run at
   the new speeds). Autonomous acceleration is 0.15 m/s² (mux `a_max`), so a
   straight reaches 0.85 only after about 2.4 m.
   Consecutive straights and arcs are driven as ONE continuous move (no
   stop between them); a rotate or a reverse ends such a chain. Before a
   slower chained step (a normal straight after a boosted one, an arc) the
   speed tapers early enough for the mux to reach the lower speed at the
   boundary (executor `taper_decel` 0.4 m/s², `taper_lead_s` 0.3: 0.96 m for
   0.85 → 0.40, 0.34 m for 0.55 → 0.40); at the end of a chain the
   controller's own ramp over the last 1.0 m brings it to the stop.
   Routes saved before 2026-09-18 never boost until re-saved with the field
   set. **Reverse** (2026-09-18): type a distance in mm and press *Reverse*
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
     - **obstacle**: points in the path ahead (the executor's own check).
       Continues by itself 2 s after the path is clear.
     - **controller stop**: Nav2 gave up on the move (usually "collision
       ahead"). Continues by itself when clear, at most 3 times per step,
       then FAULT.
     - Every hold waits until the vehicle is still, on its segment and in
       the corridor, the path is clear and localisation is READY; the reason
       says what it is waiting for. MANUAL during a hold aborts the run.
       **Prepare resume** + Start still works as an override.
     - Executor parameters: `auto_resume_enabled` (false = the old
       Prepare resume + Start), `auto_resume_clear_s` 2.0,
       `auto_resume_estop` false, `controller_abort_retries` 3.
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
| `BLOCKED` | A hold: lidar stop, obstacle or controller stop continue by themselves once clear; after the E-stop button press Start. See "While running". |
| `FAULT` | Sensor, drive, panel or localisation problem, or a tolerance was exceeded (for example "passed the endpoint"). Read the reason and the Monitor/Alarms pages, fix the cause, press **Acknowledge fault** (or the physical **Reset** button, with the vehicle at rest: it does exactly the same and nothing more - no motion, no resume, no drive or supervisor fault clearing). That does not resume: localise again if needed, reposition, load, Start. A run whose Nav2 goal never reported an outcome stays barred until navigation is stopped and started again, acknowledgement or not. |
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

## 3. Diagnostics and commissioning

Use `/monitor`, `/io`, `/alarms` and `/params` for read-only diagnosis. The
pages consume owner-published snapshots; opening more browser clients must not
create another CAN or Modbus poller. Drive-alarm reset remains a physical power
cycle where required by the drive procedure.

`/commissioning` reuses the encoder-only straight, arc, pivot and pulse planner.
It is an exclusive IDLE substate:

1. Select **MANUAL**, ensure the vehicle is stopped and the test area is clear.
2. Pick the move and fill in its numbers:
   - **Straight**: forward/reverse and distance (m).
   - **Rotate**: CCW/CW and angle (°).
   - **Arc**: left/right, angle (°) and radius (m). An arc always goes forwards, and its radius must be at least half the track (0.244 m); tighter, use Rotate.
   - **Speed**: 0.05–0.80 m/s. Above 0.40 m/s the page warns and shows the ramped stopping distance. Confirm the nanoScan3/FX3 protective field is sized for the speed before going above 0.40.
   - **Backend**: **PV** (profile velocity, the default) or **PP** (profile position, executed inside the drives). PP is greyed out with the reason while it is locked (see below).
3. Tick the checklist (area clear, E-stop in reach, speed within the field
   sizing), then **Hold move**. This validates the move and moves nothing.
   Holding is refused unless the wheels are known to be still and the encoder
   scale is fresh. Several PV segments can still be held as JSON under
   **Advanced**. A plan `id` may use letters, digits, `_`, `.` and `-` only,
   starts with a letter or digit, max 64.
4. Review **Held move**: the wheel count targets, the commanded dx/dy/heading,
   and for PP the per-wheel velocity and ramps.
5. Press physical **Start** once to execute. Only a Start pressed after the plan
   was held counts. Browser input cannot start it.
6. Use **Clear / abort** to stop and invalidate the held job. An aborted run
   still writes its evidence, including the interrupted segment.
7. Measure where the vehicle ended up, in the start frame: x forward, y left,
   heading counter-clockwise +. Measure the axle-midpoint mark, and take the
   heading from a second mark ~1 m ahead. Enter dx/dy (mm) and heading (°)
   under **Measured result** and **Save measurement**. **History** then lists
   commanded, measured and error per run, PV and PP side by side. The
   measurement is stored beside the evidence as `<evidence>.measured.json`;
   saving again replaces it.
8. Save the displayed evidence path with the commissioning record. If the reason
   says `evidence NOT written`, the file is missing; do not record an old path.

**Profile position (PP) is locked** (`pp.enabled: false` in the profile). The
motor is a BLMR6400SKM-GFV-B (400 W, 1:30 gearhead). The BLV-R manual requires
motion-extension mode for that combination, and no positioning type offers it.

There is no vendor confirmation. Running PP is an internal decision to accept
that risk, bounded by the drive settings below, the 0.30 m/s PP speed cap for
the first sessions, and a bench test on blocks before the floor.

Unlock PP only after all of these:
- **Drives:** a person has set, in both drives with MEXE02, then saved and power-cycled:

  | MEXE02 parameter | Object | Value |
  |---|---|---|
  | Max torque | 6072h | 10000 (default) |
  | Position deviation alarm (p6) | 6065h | 36000 |
  | IN-POS positioning completion signal range (p7) | 6067h | 1000 |
  | Halt option | 605Dh | 1 (default) |
  | Stopping method at alarm generation (p6) | 605Eh | 2 (default) |
  | Quick stop rate (p7) | 6085h | 1600 (also shortens alarm stops in PV) |
- **Profile:** those values are entered under `pp.expect`, and `pp.vendor_ref` records who decided, when and on what basis (`manuals/agent-prompts/pp-profile-fill.md` does both, reading the drives back first).

Software never writes those objects. Before every PP move the drive owner
reads them back from both drives and refuses the move on any difference.

A PP move is held, not latched. It halts (controlword Halt, on the profile ramp) on any of:
- the page's job stops re-sending it for 0.2 s;
- the supervisor lease loses COMMISSIONING or changes generation;
- the panel is not a fresh, valid MANUAL;
- a drive raises a following error (6041h bit 13);
- twice the planned time plus 2 s passes.

A halt not confirmed at rest within 3 s faults the drive owner and withholds
the PC heartbeat, so the drives' own 1016h reaction takes over. Bench first,
on blocks:
1. PP refused while locked.
2. A deliberately mismatched value refused.
3. One wheel turn at 0.1 m/s lands within the position window.
4. Selector to AUTO mid-move halts.
5. Closing the browser mid-move halts within 0.2 s.
6. `kill -9` of drive_node trips 1016h.

While a plan is prepared or running, ordinary jog and mapping/navigation mode
requests are refused. Selector change, lease loss, a supervisor restart or
generation change, an encoder-scale change, stale counts or panel loss aborts
the job, or drops a held plan. `DONE` or `ABORTED` cannot replay on another Start edge; upload
a new plan.

## 4. Fault recovery

- `BASE_NOT_READY`: inspect `~/.amr/logs/base.log`, CAN, panel/DIO, scanner and
  drive state. Correct the cause, then use supervisor recovery.
- `LAYER_EXITED` or readiness timeout: inspect `~/.amr/logs/layer.log` (the
  current mapping or navigation layer) and `~/.amr/logs/base.log`. Recovery cleans the old process group and returns to IDLE;
  it does not automatically restart the failed operation.
- `BASE_EXITED`, `BOOT_ERROR`, or a dead base for any other reason: **Recover
  refuses** with "restart required". Recovery replaces the mode layer and needs a
  live mux to acknowledge the new generation; it cannot rebuild the base (drives,
  mux, panel, EKF). Correct the cause, then `sudo systemctl restart amr.service`.
  Motion stays inhibited throughout.
- `8130h` on both drives: the drive lost the PC heartbeat. Stop the service,
  correct the PC/CAN cause, perform the required drive power-cycle procedure,
  then start to IDLE. The drive owner also withholds the heartbeat on purpose
  when it cannot confirm a fault stop (Monitor shows `stop_unconfirmed` and
  `heartbeat_withheld`); treat that as a drive/CAN fault, not a PC crash.
- A drive faults while its heartbeat is still arriving: check Monitor for
  `tpdo1_age_s` / `tpdo2_age_s`. Losing either feedback PDO alone now faults the
  drive owner.
- `LOOP_ERROR` or `BOOT_ERROR`: the supervisor caught an internal exception and
  finished the operation that was running as failed. Save `journalctl -u
  amr.service` for the report; restart the service if Recover does not reach IDLE.
- `SURVEY_RPC_TIMEOUT` / `SAVE_UNKNOWN_OUTCOME`: the survey coordinator did not
  answer in time. After a save timeout, check the Maps page for a new revision
  before saving again.
- Panel invalid: check the DIO island at `192.168.1.30`. Reconnection must not
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
systemctl is-enabled agv_controller amr_nav amr_mapping amr
systemctl is-active agv_controller amr_nav amr_mapping amr
```

## 6. Witnessed first cutover

Perform this only after the U10 K1–K5 vehicle session passes on the same build.
Keep a person at the vehicle, clear the area, select MANUAL and engage E-stop
before changing services.

```bash
sudo systemctl disable --now agv_controller.service amr_nav.service amr_mapping.service
sudo systemctl enable --now amr.service
systemctl status amr.service --no-pager
```

Expected observations:

1. Only `amr.service` is enabled and active.
2. The web app opens on port 5001 and reaches `IDLE`.
3. No mapping/navigation layer or prior job is active.
4. Exactly one drive owner, panel owner and scanner owner exist.
5. Release E-stop only for the short K1 boot/ownership recheck.

Do not delete the legacy source or unit files during this step. Full retirement
is U11 and starts only after accepted hardware evidence.

## 7. Rollback during the migration window

Engage E-stop and stop the unified service. Restore the previously approved
unit choice; `amr_nav` is shown below. The legacy units read
`deploy/amr_legacy.env` (map selection) since 2026-09-16: if the installed copy
predates that, run `sudo ./deploy/install.sh` first or `amr_nav` fails at
launch with `malformed launch argument 'map_id:='`. Starting a conflicting unit stops
`amr.service` as an additional guard.

```bash
sudo systemctl disable --now amr.service
sudo systemctl enable --now amr_nav.service
systemctl status amr_nav.service --no-pager
```

If unit contents themselves must be restored, copy them from the backup path
printed by `install.sh`, then reload systemd:

```bash
sudo cp /var/backups/amr-units/<timestamp>/*.service /etc/systemd/system/
sudo systemctl daemon-reload
```

Re-run the prior service's documented boot check before releasing E-stop. A
rollback does not authorize automatic motion or resume any interrupted job.

What a rollback gives you is the LEGACY controller and its own operator
interface, as a set. The unified web pages are not part of it: without the
supervisor they have no lease, mode or active-map context, so jog, survey,
initial-pose and mission controls are refused by design. Do not try to pair the
unified web app with a legacy or standalone launch; `nav.launch.py` and
`mapping.launch.py` are diagnostics and simulation entry points only.
