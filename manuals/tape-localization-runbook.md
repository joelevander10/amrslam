# Tape localization runbook: AGV-01, site 103

Status 2026-10-17. What each section needs before it can be used:

| Section | Usable |
|---|---|
| 1 Rules | now |
| 1A The straight in front of MRU | now: where sections 2 and 3 run until the plant is free |
| 2 P0 measurement | now (T1–T3 built 2026-10-18; T1 checked on a private domain, not yet on the floor) |
| 3 Teach | now: P1 teach + shadow built 2026-10-18, not yet run on the vehicle |
| 4 Daily operation | after P2 + P3 (planned) |
| 5 Holds and faults | after P2 (planned) |
| 6 Changes to the site | after P1 (planned) |
| 7 Rollback | now |

Read with: `manuals/site-layout-103.md` (the site, ids such as `MW-1`, `J-N`, `HOME-N`) and
`manuals/tracked-plan-3-tape-localization.md` (why). General operation, survey and fault codes:
`amr_ws/RUNBOOK.md`. **Planned** sections describe pages and buttons that do not exist yet. When
they are built, the procedures move into `amr_ws/RUNBOOK.md` and this file keeps only P0.

Roles: **E** = engineer (PIN pages, terminal), **O** = operator at the vehicle with the E-stop in reach.

## 1. Rules

- **Every autonomous move is AUTO + a physical Start on the panel.** Nothing here, not a call, not
  a teach run, not a script, starts the vehicle. Jogging is MANUAL + Manual Arm (DI08).
- **P0 does not stop `amr.service`.** It starts an extra localization launch beside the running
  LINE layer. That launch publishes only AMCL's pose and its `map→odom` transform, which LINE does
  not read. Restart the service only on the operator's go, and only to load new code.
- **Bench tools that need `can0`** (`read_mls …`) refuse while the service runs. They are not part
  of this runbook.
- **One test at a time on the vehicle.** Never start a simulation in the vehicle's domain (10).
- **MLS markers:** always on (2026-10-23, no profile switch). The Home Marker tile shows whether the
  sensor is configured for them (`manuals/mls-marker-plan.md`).

## 1A. Where the tests run now: the straight in front of MRU

Until the plant is free, sections 2 and 3 run on the ~10 m straight in front of MRU (plan 3
section 7). Differences from the text below:

- **Profile:** `tape_loc.site_mission: test-mru-straight`, `tape_loc.shadow_circuit: C-STRAIGHT`
  (the current values).
- **T-HOME:** the east end of the free section, facing west. Mark it on the floor (the survey
  start too). **No RFID** (2026-10-18). The run is the job `mru-straight-markers` (Run tracked →
  Set job). It needs the marker stream up (Home tile).
- **Strips (plan 3 section 7.5; 15 × 100 mm, S up, one side), x measured from the west tape end:**
  (drawing: `documentation/layout-reference/mru-straight-test-layout.html`; west = toward the tape end,
  site-plan directions per `manuals/site-layout-103.md`)
  - code 2 at 1.0 m: starts the CW U-turn
  - code 1 at east limit − 2.0 m: home (the AGV parks 0.5 m past it, at T-HOME)
  - code 2 at east limit − 1.0 m: the backstop. Read on the way back, it means the home marker was missed: FAULT, jog back, Reset
  - no other code 1 or 2 strip anywhere on the straight
- **Holds:**
  - `MARKERS_LOST`: the marker stream dropped while driving. Check where the AGV is, then Start.
  - FAULT "home marker missed": check the code-1 strip, jog back to T-HOME, Reset.
- **Stations on the straight:** T-HOME, T-ATT, MW-4, ME-4 (only MRU4 lies inside). The U-turn at
  the west tape end is CW.
- **One lap:**
  1. T-HOME westbound
  2. the tape-end pivot (U-turn tag, automatic)
  3. eastbound back to T-HOME
  4. O turns the AGV round by hand: selector **MANUAL**, Manual Arm, spin in place, **empty**
- **2.2 strips:** 2–3 code-3 strips on the straight, at least 3 m apart and 1 m from any stop.
- **2.3 survey:** only the free area. Drive it both ways, slowly; turn at the ends by hand.
- **2.5 / 2.6:** `tools/run_log.sh mru-straight-markers` per lap. A lap: Start at T-HOME, the
  AGV runs, pivots at the tape end, comes back and stops at T-HOME (DONE). Then turn it by hand.
- **3.2:** circuit `C-STRAIGHT`.
- **3.3 stations:** T-HOME (facing west), T-ATT (**at least 2 m west of T-HOME**), MW-4 facing
  west, ME-4 facing east.
- **Not testable here:** junction orders, fork resyncs, wrong branch, crossings. They wait for the plant.

## 2. P0 measurement session

Goal: measure how repeatably AMCL knows where the AGV is along the tape (plan 3 section 5,
metrics M1–M6). Nothing about motion changes: the AGV runs `line-a` exactly as today.

### 2.1 Before the session

| # | Who | Check |
|---|---|---|
| 1 | E | Built: `cd ~/agv_can/amr_ws && colcon build --symlink-install && source install/setup.bash`. `tape_loc.enabled` is **false** (P0 refuses to start otherwise). `pidstat` installed (`sudo apt install sysstat`) |
| 2 | E | `python3 tests/run_all.py` passes (repo root) |
| 3 | E | `amr.service` runs the current code: `tools/run_log.sh` warns in its pre-flight if it doesn't. Restart on O's go |
| 4 | E | MLS marker configuration in place (`read_mls set-markers --go` done, marker bench B2 or later; Home Marker tile not "sensor not set up"). If not, P0 uses the RFID fallback (plan 3 section 5); note it in the session log |
| 5 | O | `line-a` runs cleanly from Home today; the tape and tags are undamaged |
| 6 | O | An ordinary production day for the hall: racks and machines where they normally are. Note anything unusual |

Session log: a text file `~/.amr/runs/p0-<date>.md`. Record each step's time, the map revision,
every run stamp, and anything unusual (people in the aisle, trolleys moved, lost localization).

### 2.2 Lay the code-3 reference strips

Code 3 = a 60 mm strip **and** a 30 mm strip, as on the drawing
`documentation/layout-reference/mls-marker-codes-standard.html`:
- each strip 15 mm wide, 100 mm long, south pole up
- both on the same side of the tape

1. Lay 4–5 strips along the laid circuit, numbered **A1…A5** on a floor label:
   - one on the trunk straight
   - one before MRU1
   - one between MRU2 and MRU3
   - one between MRU4 and the tape end
   - one between Home and the trunk
2. Keep each strip:
   - **≥ 2 m from any speed-zone pair** (codes 2 and 1)
   - **≥ 1 m from any diverter, merge or crossing**
   - away from where the vehicle stops, so it is crossed at running speed
3. Write the distance from each strip to a fixed reference (pillar, tag) in the session log, and take a photo.
4. Check: one ordinary `line-a` run recorded with `run_log.sh`. `bag_report.py` must list a code-3 marker at every strip, in both directions. A strip that is missed or read as another code is re-laid before going on.

### 2.3 Survey the map

1. O: put a floor mark (taped arrow) where the AGV parks at `line-a` Home. It is both the survey
   start and AMCL's starting pose.
2. O: Home page → **Navigation → Trackless** (mode IDLE). Selector **MANUAL**, Manual Arm on.
3. E/O: Maps page → Map id `site103`, start mark description "line-a Home arrow" → **New map**.
   Then follow `amr_ws/RUNBOOK.md` section 2.A, and in particular:
   - drive the whole laid circuit **in both directions**, on or next to the tape, slowly
   - include the HOME area, and as much of row 2, the left vertical and the middle line as is
     reachable, so that P4 can use the same map
   - use preset moves on the long straights; turn slowly; pass the same places twice
4. Return to the mark → **Returned to start**. The closure must be a few cm and 1–2°; otherwise
   drive another loop.
5. **Save map revision**. Write `site103 rev N` in the session log.
6. Maps → **Edit areas**: mark places where trolleys and stock stand and move (the trolley and
   tread supermarkets, trolley parking) as dynamic areas.
7. O: Home page → **Navigation → Tracked** (mode LINE).

### 2.4 Start localization beside LINE

1. O: drive or jog the AGV onto the Home floor mark, facing the arrow, and keep it still.
2. E, in a second terminal on the vehicle:

   ```bash
   source ~/agv_can/amr_ws/env/vehicle.sh
   ros2 launch amr_bringup p0_localization.launch.py map_id:=site103 revision:=N params:=default initial:=start
   ```

3. E checks:
   - `ros2 topic echo --once /amcl_pose`: a pose near the start, with covariance xx and yy below 0.01.
   - In Foxglove, the live scan lies on the map's walls.

   If the scan does not align: Ctrl-C, re-park on the mark, launch again.
4. Leave it running for the whole set. Do not relaunch between laps unless localization is lost.

### 2.5 Set A: default AMCL parameters, 10 laps

For each lap:

1. E, third terminal: `tools/run_log.sh line-a MRU4`. Start it **before** the Start.
2. O: selector **AUTO** → **Start** on the panel. The AGV runs Home → MRU4 → U-turn → Home.
3. E: after the AGV stops at Home, press Ctrl-C **once** in the `run_log.sh` terminal. Write the
   run stamp in the session log.

While laps run, E watches Foxglove. If the AMCL pose leaves the tape by more than ~0.2 m, or jumps:
- write the lap and place in the log
- after the lap, at Home: Ctrl-C the launch, re-park on the mark, launch again

### 2.6 Set B: lean parameters, 10 laps

At Home: Ctrl-C the launch, re-park on the mark, launch with `params:=lean`, check as in 2.4,
then 10 laps as in 2.5.

### 2.7 Report

```bash
python3 tools/p0_report.py ~/.amr/runs/<stamp> ...     # the 10 stamps of set A
python3 tools/p0_report.py ~/.amr/runs/<stamp> ...     # the 10 stamps of set B
```

Copy both M1–M6 tables into plan 3's **P0 results**, with the date, the map revision, the run stamps
and the session log's notes. Apply plan 3's decision rule (section 5).

### 2.8 Close

1. E: Ctrl-C the localization launch.
2. O: keep the code-3 strips if the decision keeps anchors; otherwise remove them.
3. The map revision is kept. P1 teaches on it.

## 3. Teach the tape graph (P1)

Teach when: a new map revision, newly laid tape, or a moved station (section 6). Teaching is done
empty, with no trolleys. All of it is on the **Teach** page (engineer nav, PIN). The page records
and publishes only; the AGV moves by the panel's Start, or by jogging under MANUAL.

### 3.0 Once: switch tape localization on

1. E: `profiles/agv-01.json` → `"tape_loc": {"enabled": true, ...}`.
2. E: `cd ~/agv_can/amr_ws && colcon build --symlink-install` (new messages and services), then
   restart `amr.service` on O's go.
3. Teach page → Map `site103`, rev `N` → **Select map**. This writes `~/.amr/line_graph.json`.
4. Load it into LINE: Home → Navigation → **Trackless** → **Tracked** (or a service restart on O's
   go). Teach → **Status** must say "teaching on site103 revN: no graph active". The LINE layer now
   runs map_server, AMCL and the localization monitor beside the follower.

### 3.1 Localise

1. O: AGV on the survey start mark (the `line-a` Home arrow).
2. E: give AMCL its initial pose:
   - first time, with no graph yet: Foxglove "2D Pose Estimate" on the mark, or
     `ros2 topic pub --once /initialpose …`
   - once HOME-N is taught: **At home** at HOME-N
3. E: **Scans align** once the live scan lies on the walls. The Readiness tile must show READY.

### 3.2 Record the circuit passes

A pass is one whole circuit: out of the HOME bay, through every junction and the U-turn, back
into the bay, ending where it started (within 0.5 m).

1. E: Circuit `C-MRU` → **Record pass**. The Graph tile shows the pass being recorded.
2. O: selector **AUTO** → **Start**: an ordinary `line-a` run. Until P2 the U-turn and the stops
   are still RFID's.
3. Back in the bay, standing: E **Stop pass**. The message gives the length, poses and forks seen.
   - A pass whose MLS fork episodes and U-turn do not match the circuit's junctions is refused,
     with both lists: an MLS that missed a fork, or a route that differs from the mission.
   - People or trolleys in the way during a pass: **Discard** and record it again.
4. Repeat until there are **3 good passes** (**Passes** lists them).
5. E: **Build**. Each leg is listed with its pass count and spread. A spread above 5 cm means
   recording that circuit again.

> **Open (input needed):** confirm that `line-a` drives exactly C-MRU: HOME-N → J-BMERGE → R717 →
> ATT → J-N right → X1 → J-ML → trunk → MRU → pivot → back → J-ML left → J-STEM → J-SPLIT left →
> HOME-N. If it does not, the passes are refused, and a temporary mission (or a supervised drive)
> has to drive that route.

### 3.3 Teach the stations

For each station: HOME-N, ATT, MW-1…MW-4, ME-4…ME-1 (and R2-1…R2-8 once row 2 is laid):

1. O: selector **MANUAL**, Manual Arm on. Jog the AGV to exactly where the operator wants it to
   stop, facing its travel direction:
   - MW = westbound
   - ME = eastbound
   - spin in place to turn it, empty only
2. If AMCL has not updated since the last move (it updates only on motion), nudge the AGV a few
   centimetres and stop again.
3. Hold still for **3 s**. E: Station → id → **Teach here**.
4. The message gives the leg, `s` and the distance from the taught tape. It refuses when:
   - the AGV is more than 10 cm from the tape
   - it faces the wrong way for the station's leg
   - localization is not READY, or there were fewer than 5 poses in the 3 s
5. **Build** again: it says "every circuit compiles", or lists what is missing. Today C-BTU stays
   incomplete: row 2 is not laid.

### 3.4 Publish and make active

1. E: **Publish** creates graph revision `g<N>` under the map. It never changes afterwards.
2. g `N` → **Make active**.
3. Load it: Trackless → Tracked, or a service restart on O's go. Status names the graph, and the
   Locator tile shows IDLE.

### 3.5 Shadow verification

1. O: AGV at HOME-N. E: **At home** → **Scans align** → READY. The Locator tile goes WAITING →
   HEALTHY.
2. Ordinary `line-a` runs, recorded with `tools/run_log.sh`. The locator only logs. At every
   station, junction order and U-turn approach, the event log (`LINE_LOC`) says "would stop / would
   order … : ok", or "would HOLD" with the reason. LOST, re-acquired and unexpected forks are logged
   too.
3. Acceptance: plan 3 section 4, P1. Any "would HOLD" or LOST gets looked at before P2.

## 4. Daily operation (planned, P2–P3)

### 4.1 After power-on

1. The service enters LINE. The Localisation tile shows UNLOCALIZED.
2. With the AGV parked at HOME-N: Calls page → **At home** → confirm that the scans align → READY.
3. If the AGV is **not** at HOME-N: jog it there under MANUAL first.

### 4.2 Calls and trips

1. **Calls.** A machine operator taps their machine on the Calls page (touch panel or laptop). The page lists pending calls per group (MRU, BTU).
2. **A trip forms** at two calls of one group: "Trip MRU: MRU2, MRU4: press Start at the bay". A call for the other group waits.
3. **Start at the bay.** O: selector **AUTO** → **Start** at HOME-N. Horn, then the AGV drives **empty** through the U-turn and stops at ATT.
4. **Attach.** O attaches the trolleys → **Start**.
5. **At each machine:**
   - **BTU:** swap the trolley → **Start**. Two calls at one shared point are one stop.
   - **MRU, westbound:** release the trolley → **Start**. After the last release the AGV pivots at the tape end, empty.
   - **MRU, eastbound:** it stops at the same machines in reverse order. Take the trolley back → **Start**.
6. **Back at HOME-N:** release the trolleys → Calls page **Trolleys off**. The trip is done, and the next trip can start.

### 4.3 What the screen shows

- **Localisation tile:** READY / CHECKING / LOST.
- **Locator line** in the rail: edge, `s`, σ, and HEALTHY / DEGRADED / LOST.
- **Run tile and message:** the next stop or junction, and why the AGV is held.

## 5. Holds and faults (planned, P2)

| Code / cause | Operator sees | Operator does |
|---|---|---|
| HOLD `position` before a station or junction | "position uncertain before MW-2" | Check that nothing blocks the scanner's view (people, trolleys parked in the aisle). When the Locator line shows HEALTHY, press **Start**. If it does not recover within a minute: MANUAL, jog to HOME-N, **At home**, and call E |
| HOLD `position` mid-route, localization LOST | "localization lost" | As above. After 30 s without READY it becomes a FAULT: MANUAL, jog to HOME-N, **At home**. Handle the trolleys by hand |
| HOLD `payload` before the MRU tape end | "MRU release not recorded: a trolley may still be attached" | Check the trolley. If it is attached, release it here → Calls page **Trolley released** → **Start**. The AGV never pivots with a trolley |
| Trip Start refused `payload` at HOME-N | "trolleys from the last trip not confirmed off" | Release them → **Trolleys off** → **Start** |
| FAULT `wrong_branch` | "took the wrong tape after J-…" | The AGV is stopped. MANUAL: jog back to HOME-N, **At home**, Reset. Call E: the junction or its taught geometry needs checking |
| HOLD `position`: unexpected fork | "a junction where none is expected" | Check where the AGV is and the tape under it. If all is in order, **Start**. If it repeats, call E (tape damage or a changed layout) |
| Site mission refused at layer start | "graph g3 is for map rev 4, active map is rev 5" | E: Teach page → make the matching graph active, or re-teach (section 6) |
| `LAYER_EXITED` | "the line software stopped" | **Recover** on the Status page (`amr_ws/RUNBOOK.md` section 4), jog to HOME-N, **At home** |

## 6. Changes to the site (planned, P1)

| What changed | Redo | Result |
|---|---|---|
| Tape re-laid on one section | record that circuit's edges again (3.2), then re-teach the stations on those edges (3.3) | new graph revision |
| A machine's stop position moved | re-teach that station (3.3) | new graph revision |
| Racks or stock moved a lot; frequent `position` holds on one section | first mark dynamic areas (`amr_ws/RUNBOOK.md` 2.A2); if that is not enough, a new survey | a new survey = new map revision → full re-teach (3.1–3.4) |
| New map revision for any reason | full re-teach | the old graph is refused for the new map |

A graph or map revision is never edited in place. The previous one stays and can be made active again.

## 7. Rollback

1. E: set `tape_loc.enabled: false` in `profiles/agv-01.json`.
2. Restart `amr.service` on O's go.

LINE then runs without AMCL, as before this plan: `line-a`, RFID stops. Taught graphs
(`~/amr_maps/<map>/tapegraph/g<N>`), passes (`~/.amr/teach/`) and the active pointer
(`~/.amr/line_graph.json`) are kept for when it goes back to `true`. In P1 nothing the locator says
reaches motion, so switching it off changes nothing about how the AGV drives.
