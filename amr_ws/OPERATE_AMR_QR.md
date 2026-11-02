# Running the AMR QR: map, route, drive

Day-to-day operation of the AMR QR (profile `amr-qr-01`) once
[INSTALL_AMR_QR.md](INSTALL_AMR_QR.md) is done. This is the AMR QR version of
[RUNBOOK.md](RUNBOOK.md) §2; the RUNBOOK has the full detail for every page. The
differences: the AMR QR has **two buttons instead of a selector switch**, no Manual Arm
switch, no horn, a **push mode** for the brakes, and it does not use the scanner's field
outputs (§6).

## 0. The panel in one table

| Button | In MANUAL | In AUTO |
|---|---|---|
| **START** (green) | switch to **AUTO** (nothing moves) | **Start**: run the loaded route / resume |
| **STOP** | **Reset**: acknowledge a route FAULT | back to **MANUAL** (a running route is aborted) |
| **E-stop** | motors 0 V + brake | same; the route waits (BLOCKED), continue with START after release |

- **After power-up the vehicle is always in MANUAL.**
- To run a route: START once to switch to AUTO, then START again to start it.
- A button already held at power-up, or when the I/O link comes back, does nothing until
  it is released and pressed again.
- The web page can never start motion by itself. Motion needs either a held jog key in
  MANUAL, or START in AUTO.

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
   tablet or laptop. The page opens in the **operator** view (Home · Run · Manual ·
   Alarms); **Home** must say `READY`. For Maps, Routes, Status and the rest, switch to the
   **engineer** view: press `Op` in the header and type the PIN (**2809** by default,
   kept in `~/.amr/web.json`).
3. The **Status** page and the left rail should show:

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
- The speed menu offers 0.10–0.40 m/s. The AMR QR is capped by `vehicle.motor_max_rpm`
  (about 0.28 m/s until the feedforward calibration raises it), so higher settings are
  clamped.
- While surveying, spins are capped at 0.27 rad/s.

## 3. Make a map (Maps page)

1. Tape an **arrow on the floor** (the start mark). Park the vehicle on it, facing the
   arrow, and keep it still.
2. Fill in **Map id** (letters, digits, `_`, `-`) and **Start mark description**.
3. Press **New map (start survey)**. Mode becomes `MAPPING`.
   - `not ready: vehicle is moving`: stop, press again.
   - `not ready: IMU not calibrated`: wait a few seconds still, press again.
4. Drive the whole area **slowly** with the jog pad. Watch **Live map** grow.
   - Or **push** it: on the **Manual** page press **Brakes OFF (push)** (motors at 0 V,
     brakes released), push the vehicle slowly by hand, and press **Brakes ON (drive)**
     when done. Level floor only - it rolls freely. The encoders keep the odometry.
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

### 3a. Clean up the map: noise, static and dynamic obstacles

Engineer view → **Maps** → **Edit areas** on the revision. Pick a tool, drag a rectangle
on the map (right-drag pans, wheel zooms), repeat, then **Save as new revision**. The old
revision never changes; the new one is `rev N+1`. Nothing moves the vehicle.

| Tool | Use it for | What it does |
|---|---|---|
| **Remove noise** | Lidar dots / speckle in open floor | Inside the rectangle, every occupied dot or blob up to 4 cells (about 10 × 10 cm) becomes free. Walls, racks and bigger objects stay. You can drag it over the whole map |
| **Static obstacle** | A permanent thing the survey missed or drew badly (wall, fixed rack, a no-go block) | Cells become occupied: routes are refused through it, localisation uses it |
| **Dynamic area** | Things that move: trolleys, pallets, parked forklifts | The map keeps them, but routes may be validated through them (yellow "provisional"), and the scan check of localisation ignores them. **Check the area is clear before Start** |
| **Clear dynamic** | Undo a dynamic mark (make it static again) | Removes the mark |
| **Paint free** | A big object that is gone for good | Cells become free floor |
| **Erase to unknown** | Safe erase when unsure | Cells become unknown (routes still refused unless dynamic) |

Each rectangle is one edit; later ones win where they overlap; Undo / Redo / × remove
edits. After saving: on **Routes**, open each route on the new revision, **Validate**,
**Save revision**, create the mission again; on **Run**, activate the new revision.

From a shell, the same, e.g. remove the noise of a whole map:
```bash
ros2 run amr_mission map_edit --map block_c --rev 1 --despeckle-rect -50 -50 50 50 --note "noise"
```

## 4. Draw a route (Routes page) — nothing moves

1. Pick the map `<id> rev N` and type a **Route id**.
2. **Start pose**: click where the route starts and **drag toward the heading**. Use a
   spot you can mark on the floor; the survey mark is easiest.
3. Add steps from the start pose:
   - **Straight**: click a point ahead of the vehicle.
   - **CCW / CW** 45 / 90 / 180 / 270: turn in place.
   - **Arc** (radius ≥ 1 m) or a short **Reverse** (driven blind: the scanner does not
     see the rear).
4. **Speed cap**: on the AMR QR use **0.15–0.20 m/s** for the first runs. Keep every
   speed at or below the vehicle's top speed (Params page, `MAX_SPEED_MPS`); anything
   above is only clamped.
5. **Validate**. Red steps mean the footprint hits a wall or unknown (grey) cells. Move
   the step or the start pose away.
6. **Save revision**, then **Create mission from saved revision**.

## 5. Run the route (Run page)

Keep the vehicle in **MANUAL** until step 6.

1. **Activate the map.** Under **View / activate**, pick `<id> rev N`, then press **Use
   this map on the vehicle**. Mode becomes `NAVIGATION`.
2. **Set the initial pose.** Press **Set initial pose on map**, click the vehicle's real
   position, **drag toward where it faces**, and release. Localisation becomes
   `CHECKING`. The red scan points should sit on the map's walls. Jog about 0.5 m
   forward and back and turn a little to help it converge.
3. **Confirm.** When **Can confirm** reads `YES` (scan match ≥ 0.6) and you can see the
   scan on the walls, press **Confirm: scans align**. Localisation becomes `READY`.
4. **Position the vehicle** on the route's start with the jog pad (or by pushing it in
   push mode, then **Brakes ON**). It must be within **0.20 m and 10°**.
5. **Load** the mission, then press **Load (READY)**. Run state becomes `READY`.
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
   - Something in front: the scanner's warning fields slow the vehicle and the
     protective field stops it (**lidar stop**, `BLOCKED`); it continues by itself once
     clear. Nav2's own collision check can also stop it (**controller stop**, `BLOCKED`,
     continues by itself, at most 3 times per step, then FAULT) - see §6.
   - **Abort** (web) ends the run; the mission must be loaded again.
8. **Finished**: run state `DONE`. For another run, **Load** again and press **START**
   (you are still in AUTO). To leave navigation, press **STOP** (MANUAL), **Abort** any
   open run, then **Return to idle**.

Mode changes (new survey, another map) are refused while a run is READY, EXECUTING,
PAUSED or BLOCKED. Abort it first.

## 6. When something stops

| You see | Meaning | Do |
|---|---|---|
| Run `BLOCKED`, E-stop | E-stop pressed during a run | Release E-stop, press **START** |
| Run `BLOCKED`, lidar stop | Something in the nanoScan3 protective field | Clear the area in front; continues by itself. Stops with nothing there: check the field set (§8) |
| Run `BLOCKED`, controller stop | Nav2 saw something in the path ("collision ahead") | Clear it; continues by itself (3 tries per step, then FAULT) |
| Run `FAULT`, "drives did not power up" | No torque within 8 s of START (E-stop, push mode, I/O lost) | Fix it, then START again |
| Run `FAULT` | Sensor, drive, localisation or a tolerance exceeded | Read the reason. With the vehicle still, press **STOP** in MANUAL (= Reset) or **Acknowledge fault**. Then localise / reposition / load / START again |
| Localisation `LOST` | Sensor stale, pose uncertain or jumped | Set the initial pose again and **Confirm** |
| Drives `FAULT: ... stalled` | A wheel was commanded but did not turn for 2 s: blocked wheel, driver alarm or disabled, no motor power | Fix the cause, then `ros2 service call /drives/ack_fault std_srvs/srv/Trigger` |
| Drives `FAULT: ... AGAINST the command` | Wheel turned the wrong way: a sign in the profile is wrong | **Do not drive.** Recheck `invert_*` / `enc_invert_*` on blocks (INSTALL step 17) |
| Drives `FAULT: encoder feedback lost` / `I/O modules lost` | CAN or Modbus link dropped | Check cables, `can0`, the modules; then ack the fault |
| Selector `INVALID` / `STALE` | DIO module not answering | Check 192.168.1.40 and its cable; the mode falls back to MANUAL |
| Mode `BASE_NOT_READY` / `LAYER_EXITED` | A process died | See `~/.amr/logs/base.log` or `layer.log`; use **Recover**, or `sudo systemctl restart amr.service` |

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
  September/October came from the broken unit. The new scanner's field set must match
  `amr_bringup/config/scanner_fields.yaml` (protective = output path 0, warning 2 = path 1,
  warning 1 = path 2). Check once: `ros2 topic echo /output_paths --field status` shows
  all `True` with the area empty and index 0 `False` with a hand 0.3 m in front.
  The analog drives have no torque feedback, so a stop the scanner's OSSD makes in
  hardware is seen as a stall (see AMR_QR.md §5 no. 8).
- **No Manual Arm switch, no horn, no pendant, no RFID, no tape mode** (profile
  `tracked: false`).
