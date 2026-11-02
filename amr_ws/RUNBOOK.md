# Unified AMR operator and cutover runbook

`amr.service` is the intended production service. It starts the hardware base,
operator web app and optional Foxglove bridge, then enters **LINE** (tracked, the
default) unless the profile is trackless-only, which stays in **IDLE**. It does
not load a map, start a survey, resume a commissioning job or run a route after
boot, and LINE moves nothing until the physical Start under AUTO. The operator
switches to trackless on Home, then selects mapping or navigation from Run trackless.

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

The **engineer** view adds Status, Maps, Routes, Monitor, I/O, Params and
Commission. Press `Op` in the header and type the PIN; press `Eng` to go back.
The PIN is **1805** by default and lives in `~/.amr/web.json` (0600, created on
first run) or in `AMR_WEB_ENGINEER_PIN`, never in the repository. It is a mistake
guard, not security: every endpoint is still reachable directly on the network,
and the vehicle is protected by the physical panel and the supervisor's lease, as
it always was.

A healthy start reaches `IDLE`, with the base and web processes running and no
mapping/navigation layer. The Home page must read `READY` with nothing standing.
In the engineer view, the Status page and the rail must show:

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

### Demo page (visitors)

`Demo` (last tab, both views, no PIN) is a full-screen page for when the vehicle is
on display, **stationary**: product story, fixed spec strip, and a live top-down view
of what the safety laser scanner sees around the vehicle outline. When someone comes
within `near_m` (default 1.0 m from the outline) it says "It sees you". It is
read-only and says nothing technical: no scan, no pose or any fault reads as
"Standing by". It has no tabs; **tap the ✕ in the corner** to return to Home.
Wording, specs and `near_m`: `amr_web/static/demo/content.json` (restart the
service after editing). The view needs `/scan` and the odom TF (EKF) running.

### Tape or trackless: the profile's `tracked` key

One vehicle, two products, chosen by a single top-level key in
`profiles/<name>.json` and read once at boot:

| `"tracked"` | boots to | offered | refused with |
|---|---|---|---|
| `false` (default, the SLAM AMR) | `IDLE` | surveys, maps, routes | LINE: "this vehicle is trackless (profile tracked=false)" |
| `true` (the magnetic-tape AGV) | `IDLE` → `LINE` by itself | the tape follower (physical Start under AUTO; no software arm) | surveys, NAVIGATION: "this vehicle is a tape AGV (profile tracked=true)" |
| `null` (both, agv-01) | `IDLE` → `LINE` by itself (since 2026-10-08) | both: the **Navigation** switch on Home picks tracked (LINE) or trackless (IDLE, then mapping → navigation on Run trackless) | nothing by product |

The Status header's mode shows `LINE` and `/api/state` carries `product`
(`"tape"` / `"slam"`). Switching = edit the key, then
`sudo systemctl restart amr.service` (the key is missing → the profile refuses
to load and the service does not start; the error names it). The boot entry
into LINE happens once; leaving it afterwards is honoured and not undone. Jogging
needs no mode change (MANUAL jogs in LINE too). On a `null` vehicle the Home
**Navigation** switch is the way between the two: Tracked requests LINE, Trackless
requests IDLE. Like every mode change it moves nothing and is refused unless the
vehicle is stopped, nothing is running and there is no emergency.

**The LINE follower's rules (after the 2026-09-21 review fixes):**

| Event | Follower | To continue |
|---|---|---|
| physical Start under AUTO from IDLE or DONE (2026-10-07: no Arm button) | ARMED if every prerequisite holds: the drives power up and the alarm horn sounds for `timing.auto_start_delay_s` (2 s), then RUNNING with the movement horn; otherwise stays IDLE, "Start refused: …" in the event log | fix the cause, press Start again (a refused press is dropped, never remembered) |
| drives not powered within `timing.premove_timeout_s` (8 s) of a Start | back where the Start came from, "Start cancelled: drives did not power up" | release the E-stop, Reset the safety relay, Start |
| selector leaves AUTO, or the supervisor withdraws LEASE_LINE | FAULT `authority` at once, not after the 0.5 s grace | Reset, Start |
| Reset (any state but IDLE) | IDLE | Start |
| field violated | HOLD `field` | resumes by itself 2 s after clear (`auto_resume_hold_s`) |
| torque lost with the field clear — also while already held for `drives`/`rate`/`track` | HOLD `estop`, drives to standby | physical Start (the 2 s warning again) |
| no fresh `/output_paths` (0.5 s) | Start refused ("protective field state unknown"); a torque loss counts as `estop` | scanner driver up |
| tape samples under `line_min_track_hz` (profile), or fewer than 3 seen, or SDO | Start refused / HOLD `rate` | MLS TPDO stream (`nmt_starts` in diagnostics) |

> **Branching (2026-10-08):** every run takes the LEFT track at every diverter and merge:
> `branch_default` is `left` in both `missions/line-a.json` and `missions/empty.json` (plain
> line following), with no branch-latch tags. The RIGHT trial flag is gone.

Speed: two tracked speeds from the profile, cruise `autopilot.auto_rpm`
(2387 r/min = 0.75 m/s) and slow = `auto_rpm × auto_slow_ratio` (0.5 → 0.375 m/s);
no separate LINE ceiling. A warning field halves either. In the sim the launch passes
`field_source:=assume_clear` (no scanner there); never on a vehicle.

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

**Drive power (2026-10-08).** The drives are not powered while idle: they sit in
standby (CANopen up, wheel counts read, power stage off). Under **MANUAL** they are
powered only while the **Manual Arm** switch (DI08) is on; jog pad, pendant, survey
moves and commissioning moves do nothing with it off (Status shows Drives `OFF`,
reason "Manual Arm (DI08) off"). Under **AUTO** they are powered by a run: every
move after a Start first sounds the alarm horn for 2 s (tape and trackless alike),
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
   type a distance and press *Forward* / *Reverse* (0.05–10 m at 0.30 m/s), or a
   *Left* / *Right* spin of 20, 30, 45, 90, 135 or 180° (0.20 rad/s). Press once: the vehicle
   drives the whole move by itself under the same authority as the jog pad
   (selector **MANUAL**). **Any pendant button, the E-stop, the selector, the jog
   pad or *Stop move* ends it at once.** After each move the page reports how far
   SLAM corrected the vehicle's position during it; a red "the map may have
   slipped here" (a heading correction of 2° or more) means the scan matcher
   probably took a wrong alignment: back up and drive that stretch again, slowly.
   **"the map is broken from here"** (5° or more in one move) is a wrong scan
   match that is already part of the map, and driving it again does not undo it.
   **Abort the survey** and start again, or continue from the last saved revision
   (A1). On 2026-10-23, two 20° slips went unnoticed and the loop never closed
   (2.9 m at the start mark).
   Smooth preset moves and slow spins are what the mapping copes with best in
   large open areas. On a large site, survey in parts: close each part at its
   start mark, save it, and **Continue survey** from there (A1). A bad slip then
   costs one part, not the whole site.
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

### A1. Continue a saved survey (`Maps` → **Continue survey**)

A large area does not have to be surveyed in one go (2026-10-23). Save part of it,
and later keep surveying **the same map** from wherever the vehicle stands, as long
as that spot is inside the saved map. slam_toolbox loads the saved pose graph, so
old and new parts close loops with each other like one survey. The save becomes
the next revision of the same map id; the saved revision is not changed.

1. Stop the vehicle somewhere **inside the saved map**, preferably where walls or
   pillars run in two directions. Selector **MANUAL**, mode `IDLE`.
2. On **Maps**, press **Continue survey** on the revision. Only revisions with a
   saved pose graph show the button (surveyed or continued ones, not merged ones).
3. Say where the vehicle is:
   - **Where its survey ended**: the vehicle has not moved since that save.
   - **At its start mark**: parked on the map's floor mark, at its heading.
   - **Drag pose on map**: drag from the vehicle's position towards its
     heading. About 1 m and 30° is close enough.
4. Optionally describe the spot (**Where you are now**), then press
   **Continue survey**. The robot matches the live scan against the saved map
   around your pose first:
   - `continue refused: the scan fits the saved map poorly here`: the pose is off,
     or the vehicle is outside the mapped area. Correct the pose, or drive further in.
   - `continue refused: ambiguous`: another pose fits almost as well (a long
     corridor, repeated rack rows). Move to where the walls run in two directions.
   - Nothing has loaded after a refusal: press again.
5. On a good match it loads the pose graph. This takes up to about two minutes for a
   large map. The Maps page then shows `MAPPING`, `continues revN`, and the scan
   match score.
   - `... loaded, but SLAM put the vehicle ...` or `... could not load ...`: the
     supervisor goes to FAULT. Press **Recover**, then start again.
6. Survey as in section A.
7. **Returned to start** means the map's **original** start mark, in the same frame.
   Driving back to it measures the closure of the whole map, old and new. You can
   also press it where you are and save, as with a partial survey; the closure
   numbers then mean nothing.
8. **Save map revision**. The revision list shows `continues revN (scan match x.xx)`.

- Dynamic areas, keepout and edits of the continued revision are **not carried
  over**: the new revision is a fresh map. Mark them again with **Edit areas**.
- Routes and the tape graph belong to a revision: re-validate or re-teach them on
  the new one.

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
       `auto_resume_estop` false, `controller_abort_retries` 3,
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

### MLS marker codes (bench, 2026-10-09)

Markers are short 15 mm strips of south-up tape beside the north-up track. The MLS
decodes them itself (SICK standard mode). Codes 1–3 are used: 30 mm, 60 mm, and 60 + 30 mm
from the track centre. The plan, the bench tests B0–B7 and the reasoning are in
`manuals/mls-marker-plan.md`, and the layout drawing is
`documentation/layout-reference/mls-marker-codes-standard.html`. Markers appear on Home
beside the RFID tag and in the event log (`LINE_MARKER`). Since 2026-10-09 **the markers
decide speed and RFID decides where to stop**: the mission's `high_zone` names an outer and an
inner marker code (`line-a`: outer 2, inner 1), and that pair grants HIGH (`manuals/rfid-tag-table.md`,
"Speed zones"). A zone marker counts only while RUNNING, line good and within 30 mm of the
tape centre; the marker stream going down or losing events drops to NORMAL. To run NORMAL
only, pick a mission without a `high_zone` (plain line following, `basic-right`). No marker
changes the run's state.

Every command below needs `amr.service` stopped, and every write goes through `guard.py`:

```bash
python3 -m agv_core.drivers.canbus.read_mls snapshot              # marker objects, polarity, teach lock
python3 -m agv_core.drivers.canbus.read_mls markers --nmt --seconds 60   # every marker / polarity change
python3 -m agv_core.drivers.canbus.read_mls marker-level           # Hall profile: marker peak vs the 30 % level
python3 -m agv_core.drivers.canbus.read_mls set-markers            # dry run; --go writes, resets node 10, reads back
```

- **Always on (2026-10-23):** there is no profile switch; `mls.markers_enabled` is retired and a
  profile that still has it is refused. When the sensor is not configured to match, drive_node
  publishes no marker events and Home shows "sensor not set up": zone missions run NORMAL and a
  marker run refuses Start.
- **Polarity lock:** `set-markers --polarity-lock` (202Dh:05) only after a full lap with
  `markers` has shown `north` everywhere. With the lock, south-up track is invisible to the
  sensor.
- **Teach key:** `set-markers` locks the sensor's capacitive teach key (2029h = 1). Any
  future zero-point, inversion or offset-calibration step done on the keypad must first
  unlock it with an SDO write of 2029h = 0. Don't run an offset calibration over tape
  (manual p. 44).

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
| `NOT_LEASED` | warn | Waiting for the supervisor | Wait. If it stays, press Restart on the Home page. | Mux has no fresh ControlLease (supervisor down, starting or in a transaction). |
| `PATH_BLOCKED` | warn | Stopped: the way ahead is blocked | Clear the route. The vehicle tries again by itself. | Nav2 controller aborted the step (hold cause 'controller'); bounded retries, then FAULT. |
| `PP_ABANDONED` | warn | A blind move was abandoned | Nothing to do. | Blind-run move dropped (authority lost or the owner disarmed). |
| `PP_REFUSED` | warn | A blind move was refused | Nothing to do; ask the engineer if it was expected. | Blind-run move refused by the drive owner. |
| `PP_VETOED` | warn | A blind move was stopped before it started | Nothing to do; ask the engineer if it was expected. | Blind-run move vetoed at start: authority re-read after the drive setup (audit R02) no longer allowed it; the veto's controlwords were sent. |
| `SERVICE_CRASHED` | warn | The vehicle software restarted itself | Nothing to do. Tell the engineer if it keeps happening. | A marker from THIS boot survived: the supervisor died and systemd's Restart=on-failure or the 30 s watchdog brought it back. The drives were disarmed by the teardown or by their own 1016h, so no power cycle is needed. journalctl -u amr.service has the reason. |
| `SOURCE_TIMED_OUT` | warn | The command stopped arriving | Press and hold again. If a page is jogging, check the Wi-Fi link. | The selected stream went stale (cmd_timeout_s 0.2 s): closed tab, dropped Wi-Fi or a released button - indistinguishable to the vehicle, by design. |
| `WEB_BUG` | warn | A page did not work | Reload the page. Save a report if it keeps happening. | Unhandled exception in the Flask app or the page script; the reference id is in the web log. |
| `BOOT_READY` | info | The vehicle is ready | Nothing to do. | End-of-boot report from the supervisor; the text lists anything missing. |
| `FIELD_BLOCKED` | info | Stopped: something is in the safety field | Clear the area. The vehicle starts again by itself. | Scanner OSSD -> FX3 -> STO. Executor/line hold cause 'field'; auto-resume after the hold window. |
| `FIELD_WARNING` | info | Slowed or stopped: something is in a warning field | Clear the area ahead; auto speed returns by itself when the warning fields clear. | cmd_mux scales every AUTO source while a warning field is occupied: warning 1 (outer, 1.77 m ahead) x0.5, warning 2 (inner, 0.97 m) a deceleration stop within 0.40 m, the stricter winning; the slowdown and the recovery ramp over 1 s. Paths and factors: amr_bringup/config/scanner_fields.yaml. |
| `INHIBITED` | info | Motion is held by the supervisor | Wait for the mode change to finish. | Lease allowed = 0: a transaction, a save, STARTING, FAULT or STOPPING. |
| `LINE_LOC` | info | Tape localization event | Nothing to do in shadow mode; a LOST or unexpected-fork entry is worth a look on the Teach page. | amr_line.tape_runtime (tracked plan 3, P1 shadow): where the locator would have stopped, ordered a junction or approached the U-turn, and whether its uncertainty allowed it; LOST and re-acquired; teach passes and stations. Nothing here changes motion yet. |
| `LINE_MARKER` | info | Track marker read | Nothing to do; this records a marker code the line sensor read beside the tape. | MLS marker codes (manuals/mls-marker-plan.md): the sensor decodes the code, drive_node publishes /amr/line_marker, the line follower logs it. The mission's high-zone codes also set the speed (2026-10-09); no marker changes the run's state. Also logged: the marker stream going up or down, and markers lost on the topic (warn). |
| `LINE_MISSION` | info | Tape mission event | Nothing to do; this records what the mission engine did with a tag. | amr_line.tape_run events: stops, passed destinations, speed toggles, branch orders, U-turns, the run's result at Home. The text names the tag and the action. |
| `MLS_FIELD_WEAK` | info | Weak tape signal under the line sensor | Nothing to do now; the engineer checks the tape where this keeps happening. | The MLS raised its magnetic-field event (2080h:04 = 1, manual 8.8): the field fell below the critical strength, so the unfiltered LCP is not within data-sheet spec. Worn, lifted or dirty tape, or the sensor too high. Only counted with a track under the sensor on the sample before; at most one per 2 s (the text counts the rest). History only, no motion change. Per-position record: ~/.amr/reports/tape_health (tools/tape_health_report.py). |
| `MODE_CHANGE` | info | The vehicle changed mode | Nothing to do. | Supervisor FSM transition; the text carries from -> to. |
| `MUX_SOURCE` | info | The vehicle is taking commands from somewhere else | Nothing to do. | Command-source edge in the mux (pendant, manual, follow, line...). |
| `NO_PERMIT` | info | Waiting for a mission step | Nothing to do; the executor drives this. | AUTO with no MotionPermit, or a permit whose source has no fresh /cmd_vel. |
| `NO_SOURCE` | info | Nothing is asking the vehicle to move | Nothing to do. Press Start, or jog from the Manual page. | Selector position is fine but no fresh command stream is selected. |
| `PANEL` | info | The control panel changed | Nothing to do. | Selector, Start/Reset edge or pendant change from panel_node. |
| `PP_START` | info | A blind move started | Nothing to do. | Profile-position (blind-run) move accepted by the drive owner. |
| `RUN_SUMMARY` | info | A mission finished | Nothing to do. | One line per run at DONE/ABORT/FAULT: duration, distance, holds by cause. |
| `WAITING_FOR_PREREQ` | info | Waiting: a sensor or the panel is not ready yet | Wait a moment. If it stays, look at the Alarms page. | Hold cause 'pending': a prerequisite went stale inside the grace window. |

### Cleared by: start button

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `AUTHORITY_LOST` | warn | Stopped: the vehicle lost permission to drive | Put the selector back to AUTO and press Start. | Line hold cause 'authority': lease/permit withdrawn or the selector left AUTO. |
| `DRIVES_NOT_READY` | warn | Stopped: the motors are not powered | MANUAL: switch Manual Arm (DI08) on. AUTO: press Start (after the cabinet's safety reset if the E-stop was used). | DriveStatus not operational while something wanted to move: the hold cause 'drives', or the mux gating a command. Since 2026-10-08 the drives are powered only on demand (DriveStatus.power_reason says why not). |
| `ESTOP` | warn | Stopped: emergency stop or safety chain | Release the E-stop, then press Reset and Start on the panel. | Hold cause 'estop'. Auto-resume only if auto_resume_estop is set in the profile. |
| `MARKERS_LOST` | warn | Stopped: the line sensor's markers are not reporting | Check where the vehicle is (the home marker may have been passed), then press Start. | Line layer hold cause 'markers': a marker run (mission marker_run, no RFID) lost the MLS marker stream, or marker events, while driving, so the U-turn or home marker may have been driven past. Never resumes by itself; Start is refused while the stream is down. |
| `RFID_LINK_LOST` | warn | Stopped: the RFID tag reader is not reporting | Check the reader and its cable; once it is back, check where the vehicle is, then press Start. | Line layer hold cause 'rfid': on a mission with tags the RFID stream broke while driving (link down, reconnect, encounter buffer overrun), so a stop or the U-turn may have been driven past. Never resumes by itself; Start is refused while the link is down. |
| `SAFETY_RESET_NEEDED` | warn | Motors are off: the safety circuit needs a reset | Press the blue Reset button on the cabinet. | Both drives in 'Switch on disabled' (statusword 0x1270): STO held by the FX3, arming retries every 2 s and succeeds the moment the chain closes. |
| `TRACK_LOST` | warn | Stopped: the tape is not under the sensor | Push the vehicle back onto the tape, then press Start. | Line layer hold cause 'track': MLS reports no track for longer than the loss grace. |
| `AT_STATION` | info | Parked at a station | Press Start when the station work is done; it moves after the start delay. | Tape mission stop row (role always, or the job's destination): a measured stop over stop_distance_m after the tag, then a LineState HOLD with cause 'station'. The Home stop ends the run (DONE) instead. |

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
| `PP_FAULTED` | warn | A blind move failed | Press Acknowledge on the Run page. | Blind-run move faulted mid-flight. |

### Cleared by: operator

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `EXEC_ROUTE_INVALID` | error | This mission cannot be run | Pick another mission, or call the engineer to fix this one. | Route/mission failed to compile or validate against the active map revision. |
| `LOC_COV_GREW` | error | The vehicle is unsure where it is | Set the initial pose on the Run page and confirm the scans line up. | AMCL covariance over the limit for longer than cov_hold_s. |
| `LOC_JUMP` | error | The vehicle's position jumped | Set the initial pose on the Run page and confirm the scans line up. | map->odom correction over the jump trigger since the last accepted pose. |
| `LOC_LOST` | error | The vehicle lost its position | Set the initial pose on the Run page and confirm the scans line up. | Umbrella code for a LocalizationState LOST without a more specific cause. |
| `LOC_SCAN_MISMATCH` | error | The vehicle is not where it thinks it is | Set the initial pose on the Run page and confirm the scans line up. | Too many beams pass THROUGH mapped obstacles (scan_long over the limit): the unambiguous wrong-pose signature. |
| `MAP_SAVE_FAILED` | error | The map was not saved | Try Save again on the Maps page. If it fails twice, call the engineer. | mapping_session save returned an error; the survey is still open. |
| `LOC_NOT_SET` | warn | The vehicle does not know where it is yet | On the Run page: pick the map, set the initial pose, then confirm. | LocalizationState UNLOCALIZED: no accepted initial pose since the layer started. |
| `SAVE_UNKNOWN_OUTCOME` | warn | The map may or may not have been saved | Check the Maps page for a new revision before saving again. | The save RPC passed its deadline after the request was accepted; a late answer is ignored. |
| `SURVEY_NOT_READY` | warn | The vehicle is not ready to survey | Fix what the Alarms page lists, then start the survey again. | mapping_session refused: sensor ages or mode preconditions not met. |
| `SURVEY_START_REFUSED` | warn | The survey would not start | Fix what the Alarms page lists, then start the survey again. | mapping_session refused the start request. |
| `LOC_NOT_CONFIRMED` | info | Waiting for you to confirm the position | On the Run page, check the scans line up with the map and press Confirm. | LocalizationState CHECKING with can_confirm true. |

### Cleared by: recover

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `COORDINATOR_UNAVAILABLE` | error | The survey software is not answering | Press Recover on the Status page, then start the survey again. | mapping_session service missing when the supervisor needed it. |
| `LAYER_EXITED` | error | The mapping or navigation software stopped | Press Recover on the Status page, then load the mission again. | The mode layer's process group exited unrequested; the base is untouched. |
| `LAYER_NO_READINESS` | error | The new software never reported ready | Press Recover on the Status page, then try the mode change again. | The layer produced no readiness signal at all: it probably died during entry. |
| `LAYER_START_TIMEOUT` | error | The new software did not start in time | Press Recover on the Status page, then try the mode change again. | The new layer did not report ready inside layer_start_s; see ~/.amr/logs/layer.log. |
| `LIFECYCLE_STARTUP` | error | The new software did not become ready | Press Recover on the Status page, then try the mode change again. | A lifecycle node failed to configure/activate during a transition. |
| `LOOP_ERROR` | error | Internal error in the vehicle software | Press Recover on the Status page. If it repeats, save a report and call the engineer. | The supervisor caught an exception in its loop and failed the running operation. |
| `MUX_ACK_TIMEOUT` | error | The mode change did not finish | Press Recover on the Status page. | The mux never acknowledged the new generation inside the barrier budget. |
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
| `BOOT_ERROR` | error | Needs service: the vehicle software failed to start | Press Restart on the Home page. If it repeats, call the engineer. | Exception while spawning the base/web/foxglove groups; see journalctl -u amr.service. |
| `LAYER_NOT_EMPTY` | error | Needs service: old software would not go away | Press Restart on the Home page. | STOP_OLD found surviving members of the previous layer's process group. |
| `LAYER_STOP_TIMEOUT` | error | Needs service: software would not stop | Press Restart on the Home page. | The layer did not exit inside its stop budget. |
| `SPAWN_FAILED` | error | Needs service: a program would not start | Press Restart on the Home page. If it repeats, call the engineer. | Group.spawn failed (missing executable, bad environment). |
| `STOP_TIMEOUT` | error | Needs service: shutdown did not finish | Press Restart on the Home page. | Teardown exceeded its budget; systemd's TimeoutStopSec (55 s) is the backstop. |
| `SUPERVISOR_DOWN` | error | Needs service: the vehicle software is not running | Press Restart on the Home page. If it repeats, call the engineer. | No ModeState/lease reaching the web. systemd restarts on failure and on a watchdog timeout. |
| `WEB_DOWN` | error | Needs service: this page's server keeps stopping | Press Restart on the Home page. | The supervisor respawned the web group three times and gave up. |

### Cleared by: engineer

| Code | Level | Operator sees | Operator does | Engineer's note |
|---|---|---|---|---|
| `DISK_FULL` | error | No storage left: new maps cannot be saved | Call the engineer: the disk is full. A route already running is not affected. | Under 200 MB free. Survey start and map save are refused; navigation and LINE are deliberately NOT gated - a full disk must not stop a vehicle that is already moving. |
| `LOC_GATE_FAILED` | error | The position check stopped running | Call the engineer: the map/scan check failed. | Scan-consistency gate or the transform it needs failed; no evidence either way, so LOST. |
| `LOC_STREAM_STALE` | error | A sensor the vehicle navigates by went quiet | Call the engineer: a sensor stopped (see the detail for which). | scan/wheels/imu/tf/amcl age over the spec 2.4 limits. |
| `PANEL_STALE` | error | The control panel is not answering | Call the engineer: the panel wiring or the I/O island is down. | No fresh, valid PanelState: Modbus DIO at 192.168.1.30 lost, or panel_node down. No motion authority at all without it. |
| `DISK_LOW` | warn | The vehicle is running out of storage | Call the engineer: old maps and reports need deleting. | Under 1 GB free on the state or maps filesystem. Surveys and saves still run; at 200 MB they are refused (DISK_FULL). |
| `FIELD_UNKNOWN` | warn | Stopped: no safety-scanner field data | Check the scanner cable and that its driver is running (Status page). | Supervised AUTO is zeroed with no fresh /output_paths: a field state nobody can see is not clear. The OSSD stop path does not depend on this data link. |
| `MLS_TEMP` | warn | The line sensor is outside its temperature range | Call the engineer: the line sensor is too hot or too cold. | 2070h:01 outside the sensor's own thresholds 2070h:07..2070h:06 (manual 8.7), read every 60 s and at once after a temperature event. Clears itself when back inside. |
| `SCANNER_SILENT` | warn | The safety scanner is not sending data | The vehicle can still be driven by hand. Call the engineer before running a mission. | No /scan at all: the nanoScan3 driver could not reach 192.168.3.2:6060 (cable, power, netplan) or AMR_LIDAR=false. The driver is an OPTIONAL launch member, so nothing faults and nothing else reports it - this row is the only place it shows. The vehicle's STOP is the scanner's OSSD pair into the FX3 and is unaffected by the data link; mapping, navigation and localisation are dead without it. |
| `SCANNER_STALE` | warn | The safety scanner stopped sending data | The vehicle can still be driven by hand. Call the engineer before running a mission. | /scan arrived and then stopped: driver died mid-run, or the Ethernet link dropped. Expected rate is 34 Hz. |
| `TRACK_RATE_LOW` | warn | Stopped: the tape sensor is too slow | Call the engineer: the tape sensor is not keeping up. | Line hold cause 'rate': track_hz below the PID's rate gate (SDO fallback reads ~10 Hz). |

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
systemctl is-enabled amr
systemctl is-active amr
```

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
3. **Idle with the web open.** Expect: boot to LINE (IDLE on a trackless-only profile) inside the budget with no operator
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
units were deleted at U11 after the ported LINE mode was accepted on the vehicle
(`manuals/vehicle-reports/2026-09-21-restructure-and-line-layer.md`). The last
deployable legacy revision is the git tag `legacy-final`.

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
commit (tags and `manuals/vehicle-reports/` record which builds were accepted),
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
