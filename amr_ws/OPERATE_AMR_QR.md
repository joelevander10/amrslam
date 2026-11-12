# Running the AMR QR: map, route, drive

Day-to-day operation of the AMR QR (profile `amr-qr-01`) once
[INSTALL_AMR_QR.md](INSTALL_AMR_QR.md) is done. This is the AMR QR version of
[RUNBOOK.md](RUNBOOK.md) §2; the RUNBOOK has the full detail for every page. The
differences: the AMR QR has **two buttons instead of a selector switch**, no Manual Arm
switch, no horn, no Reset or Continue button (the Run page has a **Continue** instead), a
**push mode** for the brakes, and routes run at 0.80 m/s on straights (slower where a straight has its own speed), 0.30 m/s on curves (or each curve's own speed).

The software is gvievo-01 **KYB trackless** (2026-11-03) with the AMR QR hardware on top:
route **wait steps** with Continue, a **Go home** last step, smooth **S-curve** speed
changes, the route editor's **Measure** tool and the new page layout (Home instead of
Status). The GLS floor-label camera, the destination buttons and the Flexi Soft emergency
reset of KYB are switched off: the AMR QR has none of that hardware.

## 0. The panel in one table

| Button | In MANUAL | In AUTO |
|---|---|---|
| **START** (green) | switch to **AUTO** (nothing moves); the last mission you loaded loads by itself (READY) | **Start**: run the loaded route / resume |
| **STOP** | **Reset**: acknowledge a route FAULT | back to **MANUAL** (a running route is aborted) |
| **E-stop** | motors 0 V + brake | same; the route is held (BLOCKED), continue with START after release |

The USB keypad has the same buttons: **G** = START, **ENTER** = STOP, **E** = E-stop, and
**F1–F6** = Go to POS 1–6 (§2a).

- **After power-up the vehicle is always in MANUAL.**
- To run a route: START once to switch to AUTO, then START again to start it. You **Load** a
  mission on the Run page only when you want a different one: the last mission you loaded is
  remembered (also after a restart) and loads by itself whenever the AMR is in AUTO with
  nothing loaded - after the AUTO press, after a run finished, after an Abort or an
  acknowledged fault - as soon as localisation is READY.
- A button already held at power-up, or when the I/O link comes back, does nothing until
  it is released and pressed again.
- The web page never starts a run by itself. Motion needs either a held jog key in
  MANUAL, or START in AUTO. The one exception is **Continue** on the Run page, which only
  releases a route that is already WAITING at a wait step (§5).
- **No Reset after the E-stop.** The red banner and the Alarms page may say "release it and
  press Reset" (KYB's text, for its Flexi Soft): on the AMR QR, releasing the E-stop is
  enough. The banner clears by itself; a held run continues with START.

**Drive power** (the gvievo power model, 2026-10-08):

| Situation | Drives tile | Wheels |
|---|---|---|
| MANUAL | `ARMED` | powered, brakes on while still; a held jog drives |
| AUTO, nothing running | `OFF` · "AUTO, no run asking for power" | **standby**: 0 V, brakes held - normal, not a fault |
| AUTO, after START of a run | `OFF` for the **2 s pre-move warning**, then `ARMED` | the route drives |
| Push mode (Manual page) | `OFF` · "push mode: brakes released" | brakes released, rolls freely |
| E-stop / FAULT | red | 0 V, brakes held |

---

## 1. Start of shift

1. Check the area is clear and the E-stop is released. Switch the vehicle on.
2. The service starts by itself (`amr.service`). Open `http://<robot-ip>:5001/` on the
   tablet or laptop. The page opens on **Home**. The **operator** view has Home · Run ·
   Manual · Alarms. The **engineer** view adds Maps and Routes (under Run), Monitor, I/O,
   Params and Blind run: press `Op` in the header and type the PIN (**2809** by default,
   kept in `~/.amr/web.json`). The old Status and Commission pages are gone: **Home**
   shows the mode, its reason, the fault banner, Return to idle and Recover; **Monitor**
   has the rest. After a software update press **↻** (top right) once so the browser
   loads the new pages.
3. **Home** and the left rail should show:

   | Tile | Should read |
   |---|---|
   | Mode | `IDLE`, no red banner |
   | Selector | `MANUAL` (not `STALE` / `INVALID`) |
   | Drives | `ARMED`, no red bar |
   | Command | `none` |

   Localisation and Run show `–`.
4. If Drives is not ARMED, check the **Alarms** page. Common causes:
   - E-stop pressed.
   - An I/O module is off: the DIO at 192.168.1.40 or the analog module at .30.
   - CAN is down (no encoders).

   Fix the cause and wait a few seconds; the drives re-arm by themselves. Push mode
   left on shows `OFF` · "push mode": press **Brakes ON** on the Manual page. If a drive
   FAULT is latched, clear it from a shell:
   ```bash
   ros2 service call /drives/ack_fault std_srvs/srv/Trigger
   ```
5. The IMU calibrates its bias during the first ~2.3 s of standing still. Don't
   touch the vehicle for a few seconds after the service starts.

From a shell (engineering):
```bash
systemctl status amr.service
journalctl -u amr.service -f
ros2 topic echo /drives/status --once
```

## 2. Driving by hand

On the **Manual**, **Maps** or **Run** page (selector must be MANUAL):

- **Hold** a pad button, or hold W/A/S/D / the arrow keys. Releasing stops the vehicle.
  So do Space, Esc, the centre Stop button, and switching browser tab.
- The speed menu offers 0.10–0.40 m/s; the AMR QR's top speed is 0.94 m/s
  (`vehicle.motor_max_rpm` 3000, `qr_base.v_max_v` 10 V: the drivers' full scale).
- While surveying, spins are capped at 0.27 rad/s.

### 2a. USB keypad (same keypad and keys as the KIM2A)

Plug the keypad's USB dongle into the AMR PC. It is found by itself within 2 s (also when
plugged in or replugged while running); the journal says `USB keypad connected: '…'`.

| Key | Does |
|---|---|
| **W** / ↑ | forward (hold) |
| **S** / ↓ | reverse (hold) |
| **A** / ← · **D** / → | spin left / right (hold) |
| **W+A**, **W+D**, **S+A**, **S+D** | curve |
| **G** | the **START** button: MANUAL → AUTO, then in AUTO starts / resumes the route |
| **ENTER** | the **STOP** button: AUTO → MANUAL (aborts a running route); in MANUAL = Reset |
| **Q** | switch MANUAL ↔ AUTO |
| **E** | **keypad E-stop** (latched). **ENTER** releases it; that press does nothing else |
| **F1** … **F6** | go to **POS 1** … **POS 6**: the same as **Go to POS n** on the Run page (§4b) |

- Driving keys work in **MANUAL only**, like the panel pendant: 0.50 m/s straight,
  0.39 rad/s spin, soft S-curve start and stop. Releasing every key stops the vehicle.
- A held key outranks the web jog. After any E-stop, let go of every key once before it
  drives again.
- **F1–F6** work in **AUTO** (press **Q** or hold **G** first) with no run under way and the
  position confirmed: the AMR loads the newest saved mission of that POS route and starts it
  (pre-move warning, start gate), exactly as the Run page POS button. In MANUAL, during a run,
  or with no saved mission for that POS, the press is refused and "What happened" says why
  (`DEST_REFUSED`). A press while ENTER is held or the panel is not trusted is dropped.
- The other KIM2A keys (1–5 speed, K/L, T/Y, R, F7–F12, the trolley keys) do nothing here.
- The keypad E is a **software** stop. The red E-stop button and the scanner are the real
  safety stops.
- The scanner fields do not slow a manual drive; the lidar's own safety output still stops
  the motors. If that cuts in while you keep holding a key, the wheels latch
  `wheel stalled` after 2 s: let go, clear the obstacle, then
  `ros2 service call /drives/ack_fault std_srvs/srv/Trigger`.
- Not reacting? `ros2 topic echo /diagnostics | grep -A12 qr/keypad` shows the device and
  the keys it sees held. `ros2 run amr_base qr_keypad --list` (service stopped) shows every
  input device and which one the profile uses (`qr_base.keypad_name_contains`, now
  `XING WEI`).

## 3. Make a map (Maps page)

1. Tape an **arrow on the floor** (the start mark). Park the vehicle on it, facing the
   arrow, and keep it still.
2. Fill in **Map id** (letters, digits, `_`, `-`) and **Start mark description**.
3. Press **New map (start survey)**. Mode becomes `MAPPING`.
   - `not ready: vehicle is moving`: stop, press again.
   - `not ready: IMU not calibrated`: wait a few seconds still, press again.
4. Drive the whole area **slowly** with the jog pad. Watch **Live map** grow.
   - Or **push** it - smoother than the web jog over a weak Wi-Fi: press **Brakes OFF
     (push)** (right there on the Maps page under the jog pad, or on Manual / Run; motors
     at 0 V, brakes released), push the vehicle slowly by hand, and press **Brakes ON
     (drive)** when done. Level floor only - it rolls freely. The encoders keep the
     odometry. Brakes OFF is refused in AUTO and while a run is under way.
   - Drive every aisle the routes will use, and a little beyond.
   - Loop back over areas you have already mapped, so SLAM can close the loop.
   - Avoid fast spins. If walls look doubled, slow down.
   - Optional: **Preset moves** (forward/reverse N m, spin 45–180°). START, STOP,
     E-stop or the jog pad ends a preset move at once.
5. Drive back onto the **same floor mark, same heading**, stop, and press **Returned to
   start**.
   - The closure `dx / dy / dyaw` should be a few cm and 1–2°, with single straight
     walls.
   - Otherwise drive another loop and press it again, or **Abort survey**.
6. Press **Save map revision**. The mode returns to `IDLE`, and the map appears as
   `<id> rev N`. A revision never changes; surveying the same id again creates `rev N+1`.

### 3b. Continue mapping: add a new area to a saved map

1. Maps page, the revision to extend: **Continue mapping** (only revisions that keep their
   SLAM pose graph show it: every surveyed or edited revision; not a **Rotate map** revision -
   continue from the one before the rotation, then rotate again).
2. The Survey session box says **Continue mapping … from `<map>` rev N**. Put the AMR **on that
   map's survey mark, facing as when it was mapped**, standing still, then press **Continue
   `<map>` rev N (start survey)**. SLAM loads the saved map and goes on from the mark.
3. Drive slowly at first so the scans lock onto the known walls, then drive the new area.
   Come back to the mark, **Returned to start**, check the closure, **Save map revision**.
4. The result is `<map>` rev N+1, **in the same coordinates**: the old area keeps every cell
   it had (your noise / obstacle edits too), the new area is added, dynamic areas are carried
   over. Routes, POS branches, stop points and the homepost keep their places; draw the new
   routes on rev N+1, re-validate the old ones there and **Create mission** again. Areas you
   had erased to unknown may come back if the AMR sees them on this run.
5. Continue mapping only **adds**: where the old map had cells it keeps them, also where the
   floor has changed since. To bring a changed stretch up to date, use **Re-map area** (§3d)
   on rev N+1: the drive's own scans are saved with it for that.

### 3a. Clean up the map: noise, static and dynamic obstacles

Engineer view → **Maps** → **Edit areas** on the revision. Pick a tool, drag a rectangle
on the map (right-drag pans, wheel zooms), repeat, then **Save as new revision**. The old
revision never changes; the new one is `rev N+1`. Nothing moves the vehicle.

| Tool | Use it for | What it does |
|---|---|---|
| **Remove noise** | Lidar dots / speckle in open floor | Inside the rectangle, every occupied dot or blob up to 4 cells (about 10 × 10 cm) becomes free. Walls, racks and bigger objects stay. You can drag it over the whole map |
| **Static obstacle** | A permanent thing the survey missed or drew badly (wall, fixed rack, a no-go block) | Cells become occupied: routes are refused through it, localisation uses it |
| **Dynamic area** | Things that move: trolleys, pallets, parked forklifts | The map keeps them, but routes may be validated through them (yellow "provisional"), and localisation ignores them (the scan check, and AMCL itself: `AMR_LOC_BLANK_DYNAMIC=true`). **Check the area is clear before Start** |
| **Clear dynamic** | Undo a dynamic mark (make it static again) | Removes the mark |
| **Paint free** | A big object that is gone for good | Cells become free floor |
| **Erase to unknown** | Safe erase when unsure | Cells become unknown (routes still refused unless dynamic) |

Each rectangle is one edit; later ones win where they overlap; Undo / Redo / × remove
edits. After saving: on **Routes**, open each route on the new revision, **Validate**,
**Save revision**, create the mission again; on **Run**, activate the new revision.

**Rotate map** (straighten it, same page, its own revision): press **Reference line** and
drag a line along a long, straight wall. The angle that lays that wall exactly along X or Y
(the nearest one, at most 45°) appears in **Rotate by**; you can also type an angle (+ =
counter-clockwise). **Save rotated revision** turns the map, its dynamic / keepout areas and
the survey start mark together about the map origin. Save or clear any area edits first.
The rotated revision has no slam pose graph (it described the old orientation). Routes stay
on the revision they were drawn on: draw the routes for the rotated map on it, then Use it
on the Run page and set the initial pose as usual.

From a shell, the same, e.g. remove the noise of a whole map:
```bash
ros2 run amr_mission map_edit --map block_c --rev 1 --despeckle-rect -50 -50 50 50 --note "noise"
```

### 3c. Changes seen during runs: where the map no longer fits

While the AMR runs routes localised (`READY`, the scan fitting), the localisation monitor
counts, per 20 cm block of the map, what its scan check sees: **mapped things it looks
through** (they are gone) and **objects where the map has floor** (they are new). Counted at
most once per 20 cm / 10° of travel; older sightings fade (half every 3 days). It is kept per
map revision (`~/.amr/map_changes/<map>-rev<N>.npz`): a new revision starts from nothing.

Maps → **Edit areas** on the revision you run on → **Changes seen during runs**:

- on the map: **red** blocks = on the map but gone, **yellow** = there now but not on the
  map, darker = seen more often; dashed boxes `C1`, `C2`, … = the changed areas;
- the list, strongest first, with what to do:

| Says | Means | Do |
|---|---|---|
| **gone** | seen through nearly every time | **Re-map box** (§3d), or Erase to unknown if it will not come back |
| **comes and goes** | sometimes there, sometimes not (trolleys, pallets) | **Mark dynamic**, then Save as new revision |
| **not on the map** | objects that are not on the map | Mark dynamic if they move; Re-map box if they stay (a new machine) |

**Show** zooms to the area; **Mark dynamic** adds a Dynamic area edit over it (pending, like
a drawn one: Save as new revision); **Re-map box** adds it to the Re-map boxes. It needs a few
runs before the picture means much, and knows only the stretches the AMR drives.

### 3d. Re-map area: bring a changed stretch up to date

For a stretch that really changed (a machine moved, a rack row gone) without mapping
everything again:

1. **Maps** → **Continue mapping** from the revision you use (§3b). Drive **slowly through the
   changed stretch** (both directions if you can), come back to the mark, Returned to start,
   Save. That saves rev N+1 **with the drive's own scans** (`survey_scan`): what the floor looks
   like now. (rev N+1's map itself is still the old one where it had cells.)
2. **Edit areas** on rev N+1 → **Re-map area**: **Re-map box** and drag boxes over the changed
   stretch (or use a `C` area's **Re-map box**). Let each box reach into machines or walls
   that did **not** change: they are what the alignment is checked against.
3. **Preview**: the server checks that the new scans line up with the map around the boxes
   (at least 60 % of the fixed structure there seen again in place, and no shift of a few cm
   fitting better) and shows **Show after**: the map as it will be. A survey that slipped is
   refused ("displaced about N cm": drive it again more slowly); with nothing fixed around
   the boxes it cannot check: the button says "not checked" and asks before saving.
4. **Save re-mapped revision** → rev N+2: inside the boxes every cell the drive saw is what it
   saw (free or occupied); cells it did not see, and everything outside the boxes, stay as
   they were. Dynamic and keepout areas, the survey mark and the homepost are unchanged.
5. As after any map edit: in **Routes** open each route on rev N+2, **Validate**, **Save
   revision**, **Create mission**; on **Run**, use rev N+2.

## 4. Draw a route (Routes page) — nothing moves

1. Pick the map `<id> rev N` and type a **Route id**.
2. **Start pose** (the route's homepost): click where the route starts and **drag toward
   the heading**. Use a spot you can mark on the floor; the survey mark is easiest
   (**Start = survey mark**). A route can have **its own homepost** anywhere on the map:
   park the AMR on it, localise it (§5 steps 1–3, `READY`), then press **Start = AMR
   here** - the start is exactly where the AMR stands. Mark that spot on the floor.
3. Add steps from the start pose:
   - **Straight**: click a point ahead of the vehicle.
   - **CCW / CW** 45 / 90 / 180 / 270: turn in place.
   - **Arc** (radius ≥ 1 m) or a short **Reverse** (driven blind: the scanner does not
     see the rear).
   - **Wait**: type a name the operator will read (for example `load pallet`) and press
     **Wait**. The route stops there at the end of the step before it, drives off
     (brakes held), and waits until someone presses **START** on the panel or **Continue**
     on the Run page. Use it for loading / unloading points.
   - **Go home** (last step): end the route up to 5 m behind the start pose, roughly
     facing it, and press **Go home**. The vehicle drives forward back to the start pose
     and stops on its heading: a straight if it is already on the start's line, otherwise
     a smooth S-shaped curve (never tighter than R 1 m, at most 20° heading error at its
     start). The route then always closes, so a **repeat count** above 1 works without
     hand-tuning the last straight. If it cannot be built, the log says why and how far
     back to end the route. A Wait may come right before it.
   - **Delete a step**: in the **Steps** list, **×** on its row deletes that step. The steps
     after it keep their shape: each straight keeps its **length** and the rest of the route
     simply follows on from where it now ends (so deleting a turn turns the rest of the route
     with it). **× to end** deletes that step and every step after it, to draw the end again.
     **Undo** brings deleted steps back. A whole route: **Clear steps**. On a POS route only
     its own branch steps can be deleted (the main part is the main route's: change it there,
     then **Rebase on newest main**); a closed branch needs its rejoin undone first. After
     deleting, **Validate** again: a route whose end moved may no longer close at its start
     (fix it with the last straight, or end it with **Go home**).
4. **Measure** (view only, never changes the route): press **Measure**, click point A,
   then point B. The panel and the map show the distance, ΔX and ΔY in mm. A click snaps
   to a wait point, an arc centre, a step end or a route line (of this route and the
   saved ones on this map); untick **Snap** or hold Shift to measure exactly where you
   click. A third click starts a new measurement; Esc clears it.
5. **Speed.** On the AMR QR every straight runs at **0.80 m/s** (`qr_base.route_v_mps`,
   2026-11-06), every arc and the Go home curve at **0.30 m/s** unless
   it has its own speed, a reverse at 0.30 m/s (never faster: the rear is not scanned), spins
   at 0.37 rad/s (`motor_max_rpm` 3000 = 0.94 m/s top, `v_max_v` 10 V; routes run at most at 95 % of it).
   Every speed change is a smooth **S-curve** (start 0.2 m/s², braking 0.25 m/s², jerk
   0.4 m/s³; 2026-11-06): from standstill it reaches 0.80 m/s in about 4.5 s and 1.8 m,
   slows 0.80 → 0.30 before an arc over about 1.7 m, and stops from 0.80 in about 1.6 m
   (the executor brakes for the end of a move at 0.2 m/s²). A straight of about 3.5 m
   between two stops reaches 0.80. **At 0.80 m/s the red protective field must be about
   1 m long** (§5b).

   **Curve speed** (2026-11-11): each **arc** and the **Go home** S-curve has its own speed box
   in the **Steps** list, like a straight. Empty = 0.30 m/s. Type e.g. 0.50 to take a curve
   faster, or 0.20 to take it slower. A curve can never go faster than the route speed, nor
   faster than its radius allows: v / R stays under 0.405 rad/s (R 1 m: 0.40 m/s, R 2 m:
   0.80 m/s). If you type more, the row shows **→** the speed it will really run at. The AMR
   slows down before a curve only to the curve's speed, so a faster curve also saves the
   slow-down before it. Validate, **Save revision**, **Create mission** again.

   **Slow zone on an existing route** (no redrawing): **Load saved** the route, type the speed
   under **Slow zone on map** (e.g. 0.30), press the button and click **A** on the route
   (where the AMR must already be slow), then **B** (where it may speed up again). The
   straights are cut at A and B and the part between gets the speed - across turns too (a
   turn or arc keeps its own speed). One **Undo** takes it back. The tool stays on for the
   next zone. **Every slow zone of the loaded route is listed** under it: change a zone's
   speed in its box, or **Remove** it (its straights go back to the route speed; the cut
   points stay as plain waypoints, which drive as one move). Then add a new one where you
   want it. Validate, **Save revision**,
   **Create mission** again. On a POS route only its own branch can be changed this way: put a
   slow zone on the main part into the main route, then **Rebase on newest main**.

   **Slow zones (speed waypoints).** A straight can have its own, lower speed: type it in the
   straight's box in the **Steps** list (empty = the route speed), or in **Straight speed**
   before adding straights. To slow down in the middle of a long straight, use **Speed
   waypoint**: pick the straight, type where the waypoint goes (mm from its start), **Split**;
   do it twice for a zone, then give the middle part e.g. 0.30. The parts drive as one move
   with **no stop**: the AMR slows *before* the slow part (about 1.7 m before it from 0.80 to
   0.30) so it enters it already slow, and speeds up again as soon as it leaves it (about
   1.7 m to get back to 0.80). Slow parts are drawn orange on the map; a dot marks each
   waypoint. Use them for doors, crossings, narrow aisles and wherever people walk. Consecutive straights and arcs are driven as one continuous move with
   no stop between them; a turn in place, a reverse or a wait ends such a chain. Field
   stops and the E-stop are **not** softened.
6. **Validate**. Red steps mean the footprint hits a wall or unknown (grey) cells. Move
   the step or the start pose away.
7. **Save revision**, then **Create mission from saved revision**.

## 4b. POS routes: one main route, six stop points (POS 1 .. POS 6)

Every POS uses the **same main route and homepost**; only its **branch** differs: it leaves
the main route at a junction, drives to its stop point, waits there, comes back and joins the
main route again, which takes it home.

1. **Draw the main route** once as a normal route (e.g. `main`) from the homepost, ending with
   **Go home**. Mark each junction as a step end (split a straight with **Speed waypoint**
   where a branch should leave). Save it.
2. **For each POS, the quick way** (Routes page, section **Branch**): pick **Main route**
   `main rev N`, type the **Route id** (`pos1` … `pos6`) and the **Wait** name (e.g. `POS 1`),
   choose how it comes back (**turn round, forward** or **reverse** ≤ 2 m), press **Stop point
   on map** and **click where the AMR must stop** (its centre). The map shows the main route,
   the junction and the stop point as you move the mouse. The branch is made for you: it leaves
   the nearest main straight at a right angle (the junction may be in the middle of a straight:
   that straight is cut there, the main route itself is not changed), turns 90°, drives to the
   point, waits, comes back to the junction, turns back onto the main heading and finishes the
   main route to the homepost. **Straight speed**, if filled in, is used for the branch's
   straights. The stop point must be square to a main straight and at least 300 mm from it.
   Then **Validate** (a turn that hits a rack shows red: pick another point or draw it by
   hand), **Save revision**, **Create mission**.
   **Or draw it by hand**, on the Routes page, section **Branch**:
   1. **Main route**: pick `main rev N`. **Leave after**: the main step whose end is the
      junction. **Start branch** - the draft is now main up to there.
   2. Type the **Route id** `pos1` (… `pos6`: the Go-to buttons use these names).
   3. Draw the branch with the normal tools: turn, drive to the stop point, add a **Wait**
      (name it, e.g. `POS 1`), then come back - turn round and drive back, or **Reverse**
      (≤ 2 m) - to the junction **with the main route's heading** there.
   4. **Rejoin after**: the main step whose end the branch is back on (the same junction, or
      a later one), **Rejoin main**. The rest of the main route is added, to the homepost. It
      is refused if the branch does not end on that step's end pose (1 cm, 0.5°) and says
      by how much it is off.
   5. **Validate**, **Save revision**, **Create mission from saved revision**.
3. **Changed the main route?** Save it as a new revision, then load each POS route and press
   **Rebase on newest main** (refused if the junction or the rejoin point moved: redraw that
   branch), Save, Create mission. Until then validation reports the POS route as differing.

**Go to POS n** (Run page, Mission tab): selector **AUTO**, AMR on the homepost, no run under
way; one press loads the newest mission of route `posN` on the active map and starts it after
the 2 s warning - exactly like pressing START with that mission loaded. A POS without a
mission yet is greyed. The USB keypad's **F1 … F6** are the same six buttons (§2a). At the
stop point the run WAITS: **Continue** (Run page) or **START**
sends it on, back along the main route to the homepost.

## 4a. Remove a map, a route or a mission (engineer)

- **Map**: Maps page, **Remove** on a revision row, or **Remove map** beside the map's name
  (every revision). A revision takes the route revisions drawn on it with it; the last
  revision is the whole map, routes and floor labels included.
- **Route**: Routes page, pick it under **Load saved**, then **Remove revision** (the one
  picked) or **Remove route** (all its revisions).
- **Mission**: Run page, pick it, **Remove mission** (its map and route stay).

The page first asks, listing everything that goes - **missions that use it are removed with
it**, so nothing is left pointing at something gone. Refused: the map the vehicle is using
(Return to idle first), anything while a survey runs, and the mission that is loaded
(abort or load another first). Nothing is erased: it moves to `.trash` in the maps folder
(`~/amr_maps/.trash/<date>-<what>/`, same layout as the maps folder) - move it back to undo,
or `rm -rf ~/amr_maps/.trash` to free the space.

## 5. Run the route (Run page)

Keep the vehicle in **MANUAL** until step 6.

1. **Activate the map.** Under **View / activate**, pick `<id> rev N`, then press **Use
   this map on the vehicle**. Mode becomes `NAVIGATION`.
2. **Set the initial pose.** Press **Set initial pose on map**, click the vehicle's real
   position, **drag toward where it faces**, and release. Localisation becomes
   `CHECKING`. The red scan points should sit on the map's walls. Jog about 0.5 m
   forward and back and turn a little to help it converge. Shortcuts with the AMR parked
   exactly: **Initial pose = survey mark** (the map's homepost), or **Initial pose = route
   start** (the homepost of the mission picked in the list - for a route with its own
   homepost).
3. **Confirm.** When **Can confirm** reads `YES` (scan match ≥ 0.6) and you can see the
   scan on the walls, press **Confirm: scans align**. Localisation becomes `READY`.
4. **Position the vehicle** on the route's start with the jog pad (or by pushing it in
   push mode, then **Brakes ON**). It must be within **0.20 m and 10°**.
5. **Load** the mission, then press **Load (READY)**. Run state becomes `READY`. Needed only
   the first time, or to change the mission: from then on the START that switches to AUTO
   loads it by itself (run state `READY`, "`<mission>` loaded by itself (AUTO): press START
   to run it"). If it cannot (localisation not READY yet, another map active) the run state
   says why: "AUTO: waiting to load …" or "could not load …".
6. **Go.** Clear the area and stay within reach of the E-stop.
   - Press **START** once: the Selector tile shows **AUTO**. The drives go to standby
     (brakes held).
   - Press **START** again: run state becomes **EXECUTING**. For the first 2 s
     (`timing.auto_start_delay_s`) it is the **pre-move warning**: the drives power up and
     nothing moves yet. Then the vehicle drives. The AMR QR has no horn: watch it.
   - `Start refused: ... from the route start`: press **STOP** (back to MANUAL), jog onto
     the start, then START twice again.
   - `localisation not READY`: repeat steps 2–3.
7. **While it runs**
   - **STOP** means stop now; the route is aborted (the vehicle goes to MANUAL).
   - **E-stop** puts the route in `BLOCKED`. Release the E-stop, then press **START** to
     continue from where it stopped.
   - **Pause** (web) keeps progress. To continue: **Prepare resume**, then **START**.
   - **At a wait step** the run state is `WAITING` with the wait's name and how long it
     has waited; the drives are off and the brakes held. Do the job, step clear, then
     press **Continue** on the Run page (bottom of the left column; it is only enabled
     while WAITING) or **START** on the panel. Everything is checked again first: AUTO,
     localisation READY, the vehicle still and within 0.15 m / 8° of the wait point (if it
     was nudged in MANUAL, put it back or Abort). Then the 2 s pre-move warning, then it
     drives on. A refused Continue says why under the run state.
   - Something in front: the scanner's warning fields slow the vehicle and the
     protective field stops it (**lidar stop**, `BLOCKED`); it continues by itself once
     clear. Nav2's own collision check can also stop it (**path blocked**, `BLOCKED`): it
     retries by itself until the path is clear, with no limit, and after 10 s blocked it
     refreshes its view of nearby obstacles (stale marks of someone who walked past) -
     see §6.
   - **Abort** (web), STOP in AUTO, or a FAULT ends the run - but the AMR **remembers the
     step** where it stopped, also across a restart or power cut (§5a).
8. **Finished**: run state `DONE`. For another run, **Load** again and press **START**
   (you are still in AUTO). To leave navigation, press **STOP** (MANUAL), **Abort** any
   open run, then **Return to idle**.

Mode changes (new survey, another map) are refused while a run is READY, EXECUTING,
PAUSED or BLOCKED. Abort it first.

### Where the AMR stops at the end of a loop

The route's **home is the start pose drawn on the map**, not the spot where the AMR was
put down. START accepts the AMR up to 0.20 m / 10° away from it, so an AMR placed 0.10 m
off ends its first loop 0.10 m from where it began, then at the same place on every loop
after. The run summary (Alarms page, `RUN_SUMMARY`) says both numbers: `started … from the
route start` and `last stop … from its end`, in mm ahead / left of the AMR's heading.
- **started** large, **last stop** small: the floor mark is not under the drawn start.
  Re-mark the floor where the AMR stops after a loop, or redraw the route from the mark.
- **last stop** large and mostly *left/right*: the AMR finished beside home after the Go
  home S-curve. End the route with a straight of at least 1.5 m on HOME's line, so the
  home leg is straight.
- both small but the AMR is visibly off: localisation at the homepost (few walls in view,
  or a dynamic area there). Map it again with the area empty, or keep fixed structure in
  the scanner's view.

**The homepost resets the obstacle map.** Every run clears the local costmap (the obstacle marks
the path follower keeps around the AMR) when it leaves the homepost, and again each time the
route comes back to it (a Go home step or a closed loop, every pass). A trolley that was
moved, or a person who walked past, cannot hold the next trip. The scanner still sees what is
really there; the Alarms page shows `COSTMAP_CLEARED` each time.

## 5a. Continue a route after Abort or FAULT

When a run is aborted (web **Abort**, or **STOP** in AUTO) or faults part-way, the Run page
shows a **Stopped route** box on the Mission tab: the mission, the step, why it stopped and
when. It is kept in `~/.amr/route_checkpoint.json`, so it is still there after a service
restart or power cut. Operator and engineer can both use it.

1. A FAULT: fix the cause and press **Ack fault** first (the button stays grey until then).
   After a restart: activate the same map and localise (initial pose + **Confirm**).
2. Press **Continue route from here**. The run state becomes `READY` at that step (not at
   step 1). Nothing moves.
3. **START** (AUTO, then START again, as for any run). Instead of the route-start check, it
   checks the AMR is **on the step where it stopped**:
   - straight / arc / reverse / go-home: at most **0.30 m** beside the path, not before the
     step's start, heading within **20°** of the path's there;
   - turn in place: within 0.15 m of the turn point and facing somewhere on that turn. It
     turns **only the part that is left** (stopped at 30° of a 90° turn: it turns 60°);
   - wait step: at the wait point, then it waits again.

   If that step was already finished (for example "endpoint missed" at its end), it starts
   on the next step. Too far off: the reason says how far ("0.45 m beside the path...");
   jog it back near the line in MANUAL and press START again.
4. Then the 2 s pre-move warning, and it drives on from there to the end of the route
   (and the remaining passes of a repeated route).

The stopped route is forgotten when the run **finishes** (`DONE`) or when a run is started
**from the route start** (a normal Load + START). It can't be continued - the button says
why - if the map, the route or its steps changed since it stopped; then Load and start from
the route start.

## 5b. Safety at 0.80 m/s (read before the first fast run)

The stopping distance grows with the square of the speed. Measured at 0.50 m/s the AMR QR
stops in about 0.30 m on a red-field stop; at **0.80 m/s** expect **about 0.7 m** (scanner
response, up to 0.3 s of coasting before the brake, then braking). The red field of 0.50 m is
**too short** for that: lengthen it in Safety Designer to about **1.0 m** for 0.80 m/s, and
yellow (slow to half) to about **1.8 m**, then **measure**: drive at a box at 0.80 m/s and
check the gap left. Until then keep `qr_base.route_v_mps` at 0.50 (profile, restart), or
put slow zones on every straight where something can step in.

## 6. When something stops

| You see | Meaning | Do |
|---|---|---|
| Run `BLOCKED`, E-stop | E-stop pressed during a run | Release E-stop, press **START** |
| Run `BLOCKED`, lidar stop | Something in the nanoScan3 protective field | Clear the area in front; continues by itself. Stops with nothing there: check the field set (§8) |
| Run `BLOCKED`, path blocked / controller stop | Nav2 saw something in the path ("collision ahead") | Clear it; it retries by itself until clear |
| Run `WAITING` | A wait step in the route | Do the job, step clear, press **Continue** (Run page) or **START** |
| Run `FAULT`, "turn centre drifted" | During a turn in place the AMR slid more than 0.15 m sideways (one wheel started late or slipped) | Check for a hitched trolley, something under a wheel, a slippery spot; put it back on the start, **Continue route** or Start again |
| Continue refused | AUTO off, not localised, vehicle moved off the wait point, or E-stop | Read the reason under the run state; fix it and press Continue again |
| Run `FAULT`, "drives did not power up" | No torque within 8 s of START (E-stop, push mode, I/O lost) | Fix it, then START again |
| Run `FAULT` | Sensor, drive, localisation or a tolerance exceeded | Read the reason, fix it, press **Ack fault** (or **STOP** in MANUAL). Then **Continue route from here** + START to go on from that step (§5a), or Load + START from the route start |
| Run aborted (Abort / STOP in AUTO) | The run was ended by hand | **Continue route from here** + START (§5a), or Load + START from the route start |
| Localisation `LOST` | Sensor stale, pose uncertain or jumped | **Alarms → Localisation losses** says why (below); set the initial pose again and **Confirm** |
| Drives `FAULT: ... stalled` | A wheel was commanded but did not turn for 2 s: blocked wheel, driver alarm or disabled, no motor power | Fix the cause, then `ros2 service call /drives/ack_fault std_srvs/srv/Trigger` |
| Drives `FAULT: ... AGAINST the command` | Wheel turned the wrong way: a sign in the profile is wrong | **Do not drive.** Recheck `invert_*` / `enc_invert_*` on blocks (INSTALL step 17) |
| Drives `FAULT: encoder feedback lost` / `I/O modules lost` | CAN or Modbus link dropped | Check cables, `can0`, the modules; then ack the fault |
| Selector `INVALID` / `STALE` | DIO module not answering | Check 192.168.1.40 and its cable; the mode falls back to MANUAL |
| Mode `BASE_NOT_READY` / `LAYER_EXITED` | A process died | See `~/.amr/logs/base.log` or `layer.log`; use **Recover**, or `sudo systemctl restart amr.service` |

**Why localisation was lost.** Every time Localisation drops from READY a report is written
(`~/.amr/loc_losses/`, the newest 50; the last 10 also go into **Save report**). The Alarms page
lists them under **Localisation losses**:

- what triggered it, the correction split into **ahead / left** of the AMR and its turn,
  the speed, where on the map;
- how the scan fitted the map in the last 3 s: % **on mapped walls**, % **on things that are
  not on the map** (trolleys, pallets, people), % **through mapped walls** (something mapped is
  gone);
- which sensor went quiet, if one did (lidar, wheel encoders, IMU), and how long;
- **likely causes**, most likely first - e.g. many unmapped objects → mark the area dynamic;
  a sideways correction while driving straight → lidar angle or a twisted map; a correction
  along the path → a long aisle or wheel slip; a quiet sensor → that sensor's cable/network.

**Show on map** opens the Run page with that moment drawn: the scan in green (on mapped walls),
red (on unmapped things) and orange (through walls), a blue arrow where the AMR was and a dashed
line for the correction. The event in "What happened" carries the same summary and the
report's name.

**Small corrections are not losses.** AMCL keeps correcting the wheel odometry; in a long aisle
or after a little wheel slip that correction can come in one step. A step over 0.15 m / 5° is
only a loss when the scan does not back it up. Up to **0.30 m** (and under 5°), when the scan
fitted the map (≥ 75 % on mapped walls; 80 % until 2026-11-11) just before it AND the next scan after it fits too
(within 1.5 s), the AMR keeps going: the step is logged as **`LOC_CORRECTED`** (yellow row in
**Localisation losses**, at most one report every 30 s). Over 0.30 m, over 5°, a poor fit before
it, or no fit after it is still **`LOC_JUMP`** and stops the AMR. Many `LOC_CORRECTED` rows with
the correction **along the path in the same direction** (always ahead -, or always ahead +)
mean the wheel scale is off: check the wheel diameter / gear ratio in the vehicle profile.
The limits are `jump_verified_max_m`, `jump_fit_min`, `jump_verify_s` in
`amr_localization/readiness.py`.

**Corrections are worked in smoothly (2026-11-12).** The AMR steers by its position on the map.
AMCL used to move that position at once at every update (every 5 cm); where most of the scan is
blanked - dynamic areas with trolleys on both sides - each update shifted it a few cm and a
fraction of a degree back and forth, and the AMR wobbled chasing it. With
`AMR_LOC_SMOOTH_POSE=true` (`amr.env`, on for the AMR QR) a correction is now worked in over
about 1 - 3 s of driving (a 0.18 m one in about 3.5 s), and AMCL's jitter is damped about five
times. Losses, `LOC_CORRECTED` and the scan check still look at AMCL itself, unsmoothed. Standing
still the AMR takes AMCL's position at once, so endpoint checks and relocalisation are as
before. If you ever need the old behaviour: set it to `false` and restart the service. The
layer log prints `working in a correction: ... still to go` when one is larger than 0.10 m / 2°.

**Lost during a run: it finds itself again and continues (2026-11-09).** When the position is
lost during a route because of a jump (`LOC_JUMP`), the scan passing through walls
(`LOC_SCAN_MISMATCH`) or AMCL becoming unsure (`LOC_COV_GREW`), the AMR does not FAULT at once:

1. It stops where it is (Run page: BLOCKED, `LOC_RELOCALIZING`, "relocalising, attempt 1/3").
2. After standing still 1 s it sets the initial pose itself, where it thinks it is, and lets
   AMCL settle in place (up to 30 s).
3. It confirms by itself only if the scan then lies at least 70 % on mapped walls, nothing goes
   through walls, and the position it settles on is within 0.50 m / 15° of where it stopped
   (`LOC_RELOCALIZED`).
4. It continues the route from where it stopped after 2 s, like after a safety-field stop.

It gives up and FAULTs (`LOC_RELOC_FAILED`) when it does not settle within 40 s, settles too
far away, or is lost a 4th time in the same run. Then do it by hand: set the initial pose,
Confirm, **Continue route**. A sensor that went quiet (`LOC_STREAM_STALE`) or a stopped scan
check (`LOC_GATE_FAILED`) is never retried: that is hardware. Switching to MANUAL during it
aborts the run, as for any hold. Keep clear of a stopped AMR: it can move off again by itself.
Limits: `auto_reloc_max` (3; 0 = off) in `navigation_layer.launch.py` executor_overrides,
`reloc_*` parameters of the localization monitor.

## 7. End of shift

1. Finish or **Abort** the run. Press **STOP** so the vehicle is in MANUAL, then **Return
   to idle**.
2. Park the vehicle.
3. Stop the service before cutting power:
   ```bash
   sudo systemctl stop amr.service
   ```
   The analog outputs go to 0 V and the direction coils drop. Brakes are released on a
   clean stop, as the old QR controller did, so the vehicle can be pushed.
4. Switch off.

Stopping the service first matters on this vehicle: the analog module keeps its last
voltage if the PC just loses power in the middle of a move. The hard-wired E-stop is the
real stop.

## 8. What the AMR QR does not have (read once)

- **Scanner fields: same as gvievo-01** since the broken nanoScan3 was replaced
  (2026-11-02). The warning fields slow or stop AUTO and the protective field stops it
  ("lidar stop", resumes by itself). There are no scan filters any more: the noise of
  September/October came from the broken unit. The AMR QR's own field set is
  `amr_bringup/config/scanner_fields.amr-qr-01.yaml`: **one warning field on output path 0**
  (LED yellow → AUTO at half speed, "Warning 1") and **the protective field on path 1** (LED
  red → stop, "lidar stop"). Measured 2026-11-05: green `[true, true]`, yellow
  `[false, true]`, red `[false, false]` (`ros2 topic echo /output_paths --field status`).
  If the fields are changed in Safety Designer, measure again and update that file.
  The analog drives have no torque feedback, so a stop the scanner's OSSD makes in
  hardware is seen as a stall (see AMR_QR.md §5 no. 8).
- **No Manual Arm switch, no horn, no pendant, no Reset / Continue / destination push buttons,
  no GLS floor camera** (`gls.enabled: false`; POS 1–6 are the Run page buttons and keypad
  F1–F6), no RFID and no tape mode (both removed
  from the code in KYB trackless).
