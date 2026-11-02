# Tracked plan 3: tape-graph localization (stops and junction orders without RFID)

Status: P0 tooling and P1 (teach + shadow) BUILT 2026-10-18, tested offline, in the unified sim
and on a private ROS domain; not yet run on the floor. Section 6 has the status and what is
still needed. **The plant is not free yet: the first floor tests run on the ~10 m straight in
front of MRU (section 7, stage 1).** Site and flow: `manuals/site-layout-103.md` (read it
first). Procedures: `manuals/tape-localization-runbook.md`. Scope: **AGV-01**. AGV-02 and two-AGV
traffic come later.

Supersedes, for the site:
- the RFID role of `manuals/rfid-tag-table.md`: stops, branch orders and the U-turn trigger
- D10 of `manuals/tracked-speed-plan-2-slam.md`: a taped path drawn in the route editor

Speed stays with the MLS markers (`tracked-speed-plan-1-no-slam.md`); this plan grants no HIGH.

## 0. Decisions

| # | Decision | Source |
|---|---|---|
| D1 | Localization replaces RFID for the site's stops, junction orders and the U-turn approach. RFID trials gave poor results | operator 2026-10-17 |
| D2 | Production localizes with **AMCL on a frozen map revision**. SLAM (slam_toolbox) only builds the map. Loop closure at runtime would move the map under the taught stations | this plan |
| D3 | **Teach and repeat; never draw.** The tape path and every station are recorded with the same AMCL on the same revision that runs production. Any fixed AMCL or map bias is present in both and cancels. What matters for ±10 cm is repeatability, not absolute accuracy | this plan |
| D4 | **The locator is one-dimensional:** which edge of the tape graph, distance `s` along it, and its uncertainty σ. The tape fixes lateral position and heading. AMCL's lateral output is used only as a health check (distance from the recorded path) | this plan |
| D5 | **No silent pass.** Every decision point (stop, junction order, U-turn) needs a healthy locator inside its lead window. Otherwise the vehicle HOLDs *before* the point. A missed RFID tag did nothing; a doubtful position stops the vehicle | this plan |
| D6 | **Not in the safety path.** The locator decides only where to stop and which tape to follow, the role RFID had. FX3/STO, the panel's Start and the mux are unchanged. Every new hold is more restrictive, never less. No safety-listed file (`gating.py`, `cmd_mux_kinematics_node.py`, `scanner_fields.*`, `guard.py`) changes | CLAUDE.md |
| D7 | **Designed data lives in the repo, taught data on the vehicle.**<br>- The mission (`missions/site-103.json`) holds groups, machines, circuits, junction orders and what happens at each stop.<br>- The tape graph (edge geometry, the `s` of every junction and station) is taught, bound to a map revision's sha256, and stored next to the maps.<br>- At load, every name the mission uses must exist in the graph | this plan |
| D8 | **Sensor events resync the locator, independently of AMCL:**<br>- a junction passed (the MLS track count back to one)<br>- a U-turn completed at the tape end<br>- optionally, MLS code-3 anchor strips (D9) | this plan |
| D9 | **Code 3 is the anchor code.** It is free: the zones use 2 and 1. One code serves every anchor, because the locator already knows which anchor comes next. P0 lays code-3 strips as ground truth; whether they stay as anchors is decided by P0's result (section 5) | this plan |
| D10 | **P0 measures before anything is built,** and its numbers decide the build (section 5) | this plan |

## 1. Accuracy budget

| Decision | Requirement | Locator limit at the decision |
|---|---|---|
| Station stop (machines, ATT, HOME-N) | operator ±10 cm | σ ≤ 2.5 cm when braking starts (2σ ≤ 5 cm); the stop control's own error ≤ 3 cm |
| Junction order | order latched before the fork | σ ≤ 0.20 m, order latched by `s_fork − 1.0 m` |
| MRU U-turn approach | slow down before the tape end | σ ≤ 0.20 m; the tape end itself (line loss) stops the vehicle, as today |
| Wrong branch | detected within ~2 m of the fork | path separation ≥ 0.15 m and the residual test (section 2.4) |

Braking from NORMAL (0.50 m/s, `ramp_accel_rpm_s` 950) takes about 0.5 m. Stops start braking
at `s_station − stop_distance_m`, as `TapeRun` does today after a tag read. Over the last
0.5 m, wheel odometry is precise, so the position at the moment braking starts dominates.

## 2. Architecture

### 2.1 Components

| File | Kind | Role |
|---|---|---|
| `amr_line/tape_graph.py` | pure, new | load and validate a graph; project a pose onto an edge → (s, lateral residual); follow a junction order to the next edge |
| `amr_line/tape_locator.py` | pure, clock-fed, new | the 1-D filter: predict, correct, resync, health (section 2.3) |
| `amr_line/trip.py` | pure, new | calls → trip (group, ≤2 machines) → ordered list of stops, junction orders and payload state. Replaces `TagTable` for site missions |
| `amr_line/tape_run.py` | changed | stops at an `s`; U-turn approach at an `s`; junction orders sent to `branch.Ladder` as set/clear pulses (the RFID tags were the old source of these pulses); new holds `position`, `payload`; new faults `wrong_branch`, `graph` |
| `amr_line/branch.py` | **unchanged** | the Ladder already takes pulses (`scan(set_left=…, clear_right=…)`); ForkPassed already consumes the order at the fork |
| `amr_line/job.py` | changed | the start rules for a site mission: READY localization, Start from HOME-N or ATT only; new `Inputs` fields (AMCL pose, readiness) |
| `amr_line/line_follow_node.py` | thin, changed | subscribes `/amcl_pose` and `/amr/localization_state`; ticks the locator; serves calls and teach |
| `amr_mission/tape_graph_store.py` | new | graph revisions under the map: staged, verified, published atomically, never rewritten (same transaction as `map_bundle.py`) |
| `agv_core/mission.py` | changed | schema v3 for site missions (section 2.5); v2.1 (`line-a`) stays loadable |
| `amr_bringup/launch/line_layer.launch.py` | changed | when the profile has `tape_loc.enabled: true`: map_server, AMCL, lifecycle manager and localization monitor on the graph's map revision (bundle hashes verified, as `navigation_layer.launch.py` does). EKF and `scan_gate` already run in base |
| `amr_bringup/launch/p0_localization.launch.py` | new (P0) | the same localization group, started beside a running LINE layer, for measurement only |
| `amr_interfaces` | changed | LineState: append locator and trip fields. New srvs: `LineCall`, `TeachStation`, `TeachEdge`, `AtHome`. **colcon build** |
| `amr_web` | changed | operator **Calls** page; engineer **Teach** page (PIN); a **Locator** tile in the rail |
| `profiles/agv-01.json`, `agv_core/config.py` | changed | block `tape_loc` (`enabled` + the locator thresholds): schema, validation, refusal tests in `tests/test_config.py` |
| `tools/run_log.sh`, `tools/bag_report.py`, `tools/p0_report.py` | changed / new | record and report the localization topics (section 5) |

The node stays thin and the decisions stay in pure modules, tested without ROS (CLAUDE.md conventions).

### 2.2 Tape graph

```
$AMR_MAPS_DIR/<map_id>/tapegraph/g<N>/graph.json, manifest.yaml
```

- **Binding:** `{map_id, map_revision, map_sha256}`. A graph whose map sha256 no longer matches is refused at load.
- **Edges:**
  - an id, a polyline resampled every 5 cm, its length, and its canonical direction
  - bidirectional edges (trunk, MRU line) are stored once; the locator carries the travel direction
- **Junctions:**
  - an id, matching the mission and `site-layout-103.md` section 4 (`J-N`, `J-ML`, …)
  - its kind: diverge / merge / crossing / T / tape end
  - the edge it is on and its `s`
  - which edge leaves on which side
  - `s_fork_end` for each direction it is passed: where the MLS saw a single track again, the median of the teach passes
- **Stations:** an id (`MW-1`, `ME-1`, `R2-3`, `ATT`, `HOME-N`), its edge, `s`, travel direction, and the teach record (pose, lateral residual, time).
- **Teach quality, per edge:** number of passes, and the largest lateral spread between passes.
- **Active graph:** a pointer `$AMR_STATE_DIR/line_graph.json` = `{map_id, map_revision, graph_revision, sha256}`. It is set from the Teach page and read once when the LINE layer starts. Teaching happens on the vehicle and changes too often to live in the profile.

### 2.3 The locator (`tape_locator.py`)

- **State:**
  - edge, `s`, travel direction (+1/−1), σ
  - health: `HEALTHY` / `DEGRADED` / `LOST`
  - the last accepted correction (when, how far ago in wheel travel)
- **Predict:**
  - from signed wheel travel (`/wheel_states` counts, as `TagOdometer` integrates them today)
  - σ grows by `k_odo · |Δs|`; `k_odo` comes from P0's M6, with 1 % as the starting value
  - at an edge end, the trip's junction order picks the next edge; a crossing continues on the same tape
- **Correct:**
  - **Latency:** take the AMCL pose at its stamp. Look up the odometry `s` at that stamp in a 2 s buffer and roll the correction forward to now. Without this, 0.1 s of latency at 0.5 m/s is 5 cm, half the budget.
  - **Projection:** project the pose onto the current edge → (`s_meas`, lateral `d`).
  - **Accept** only when all hold:
    - readiness is `READY`
    - the pose is no older than 0.5 s
    - `|d|` ≤ 0.10 m (set from P0's M2)
    - `|s_meas − s_pred|` ≤ max(3σ, 0.10 m)
  - **Update:** a 1-D Kalman update. The measurement σ is the AMCL covariance projected on the path tangent, floored at the along-track σ P0 measured (AMCL's own covariance is optimistic).
- **Resync** (D8), each a measurement with its taught spread as σ:
  - a fork passed at a known junction
  - a U-turn completed (`s` = the taught tape end)
  - a code-3 anchor, if laid: its taught `s`, applied only when exactly one anchor lies within 3σ of the prediction
- **Health:**
  - **HEALTHY:** a correction accepted within the last 2 m of travel, and σ under the station limit
  - **DEGRADED:** running on odometry; σ growing; decisions still allowed while σ is under their limit
  - **LOST:** any of
    - readiness LOST or UNLOCALIZED
    - 5 corrections rejected in a row
    - σ > 0.5 m
    - a wrong branch detected
- **Wrong-branch check:**
  - **When it runs:** after a diverge, once the two possible paths are ≥ 0.15 m apart.
  - **The test:** compare `|d|` against the ordered edge with `|d|` against the other edge.
  - **The verdict:** the other edge fitting better by 0.05 m for 0.5 m of travel → `wrong_branch`.

### 2.4 Decisions, holds and faults

| Point | Check, at its window | Not met | To continue |
|---|---|---|---|
| Station on the trip (MW, ME, R2, ATT, HOME-N) | σ ≤ 2.5 cm and not LOST when braking would start | HOLD `position` before the station: stop at `s_station − stop_distance_m − 0.3 m` | panel Start once the locator recovers (a no-motion AMCL update is requested while held); refused while still uncertain |
| Junction order | σ ≤ 0.20 m by `s_fork − 1.0 m` | HOLD `position`: the vehicle stops before the fork | as above |
| Fork passed somewhere unexpected | an MLS fork episode with no junction within 3σ | HOLD `position` | operator checks the vehicle's position and the tape, then Start |
| Wrong branch | section 2.3 | FAULT `wrong_branch` | jog back under MANUAL, Reset (section 6 of the runbook) |
| MRU U-turn | position past the last MW stop of the trip, **and every called MW release recorded as done** | HOLD `payload` before the tape end; the vehicle never pivots | operator releases or checks the trolley, confirms **Trolley released** on the Calls page, then Start |
| Leaving HOME-N | the previous trip ended with the trolleys released at HOME-N | trip Start refused (`payload`) | operator confirms the trolleys are off on the Calls page |
| Missed stop | a station's window passed while the locator was DEGRADED with σ above the limit | cannot happen silently: the station check holds first | — |
| Localization LOST while running | health LOST | HOLD `position` at once (normal braking) | Start after READY again; FAULT if it is not READY within 30 s |
| Graph or map mismatch | at layer start | layer refuses the site mission ("graph g3 is for map rev 4, active map is rev 5") | publish a re-taught graph or select the matching revision |

The holds follow job.py's existing pattern: holds that need a physical Start do not resume by
themselves. None of them extends a command's life, and none touches the lease or the mux.

### 2.5 Mission v3 (`missions/site-103.json`)

- **`groups`:**
  - `MRU` → `{machines: {MRU1: {release: MW-1, take: ME-1}, …}, circuit: C-MRU}`
  - `BTU` → `{machines: {BTU24: {swap: R2-1}, BTU16: {swap: R2-1}, …}, circuit: C-BTU}`
- **`circuits`:** an ordered list of items, `junction(id, order)` | `stop(id)` | `uturn(id)` | `section_end`. They are the circuits of `site-layout-103.md` section 4, with every junction order written out. `J-ML` is `left`, never `straight`.
- **`home`: HOME-N**, **`attach`: ATT**.
- **The loader refuses:**
  - a machine in two groups
  - a stop that is not on its group's circuit
  - a diverge without an order
  - an order of `straight` at a T
  - a U-turn before every release stop of its circuit
  - any id missing from the active graph

  Each refusal gets a test in `tests/`; update `EXPECTED_CHECKS` deliberately.

### 2.6 Start-up and initial pose

- **At boot:** AMCL has no pose (`set_initial_pose: false`). The AGV is normally parked at HOME-N.
  1. The operator presses **At home** on the Calls page. `/initialpose` gets the taught HOME-N pose.
  2. Readiness goes to CHECKING, and the operator confirms that the scans align (the existing confirm).
  3. Readiness goes READY, and the locator starts at HOME-N.
- **Trip Start** needs READY, a locator within 0.3 m of HOME-N (or of ATT for the second Start), and no payload hold.
- **After a FAULT away from home:** jog back to HOME-N under MANUAL, then **At home** again. Resuming mid-route after a *hold* is allowed: the locator keeps its state through holds.

### 2.7 AMCL in LINE, and CPU

- **Starting parameters:** those of `amr_localization/config/amcl.yaml`.
- **The lean set P0 also measures:** `max_particles` 800, `update_min_d` 0.10, `update_min_a` 0.05. The tape removes most of the search, so fewer particles may be enough.
- **Current load:** LINE uses ~2.6 of the N97's 4 cores (2026-10-08).
- **Go rule** (from plan 2): total < 75 % and < 75 °C on a warm day.
- **If it is tight:** under LINE, stop `commissioning_node` (~15 %) first.

## 3. Failure modes

| Failure | Detected by | Effect | Recovery |
|---|---|---|---|
| AMCL degraded (people, trolleys, occlusion) | lateral-residual or innovation gate, readiness scan match | DEGRADED: odometry carries `s`, σ grows; decisions continue while σ is under their limit | the next accepted correction or resync |
| AMCL jump | innovation gate rejects it; readiness jump check | rejected corrections; LOST after 5 in a row | HOLD `position`; READY again, then Start |
| Area changed (racks, stock) | scan match falls on one section; more rejected corrections there | DEGRADED or HOLD on that section | dynamic areas on the Maps page, or a new survey and re-teach (runbook section 7) |
| Wheel slip | innovation at the next correction or resync | σ covers it; a large innovation is rejected and counted | none needed; repeated slip is logged |
| Fork not detected by the MLS | junction passed with no fork episode | no resync; the order is still cleared at `s_fork + margin` (the backstop the exit tag used to be) | none needed; logged |
| Wrong branch taken | residual test (2.3) | FAULT `wrong_branch` | MANUAL jog back, Reset |
| Missed MRU release | payload guard | HOLD `payload` before the tape end | operator, Start |
| Localization stack down (CPU, crash) | AMCL pose age; readiness stale | LOST → HOLD `position`. If the AMCL process exits, the whole LINE layer is down: the supervisor reports `LAYER_EXITED` | Recover on the Status page (RUNBOOK section 4), jog to HOME-N, At home |
| Map revision changed | graph binding | site mission refused | re-teach on the new revision (runbook section 7) |
| Wrong initial pose | readiness CHECKING never confirms; scan match low | trip Start refused | jog to HOME-N, At home again |

## 4. Steps

Until the plant is free, P0 and P1 run on the straight in front of MRU (section 7, stage 1).
What needs corners and junctions waits for stage 2.

| Step | Content | Done when |
|---|---|---|
| **P0** Measure | tooling T1–T3 (section 5) and one session on the laid MRU circuit (runbook section 2) | the M1–M6 table is filled and the decision rule applied |
| **P1** Graph, teach, shadow | `tape_graph`, `tape_locator`, graph store, teach mode (edges by recorded runs, stations taught under MANUAL), Teach page, LineState fields, `tape_loc.enabled`, LINE-layer localization group. **Shadow mode:** the locator runs during ordinary `line-a` runs and logs where it would have stopped and branched. Motion is unchanged | 50 passes of each MRU station in shadow: predicted vs taught `s`, 2σ ≤ 5 cm, no LOST, no unexpected fork |
| **P2** MRU circuit live | `trip.py` (MRU group), stops and U-turn approach by `s`, the payload guard, ATT and HOME-N stops, the new holds and faults | floor acceptance: 20 MRU trips with 2 calls each, every stop within ±10 cm (measured on floor marks), every provoked failure (section 3) ending in its hold |
| **P3** Calls and trips | Calls page: queue, 2-call trip formation per group, "press Start at the bay", At home, trolleys-off confirm | operator acceptance of the flow in `site-layout-103.md` section 3 |
| **P4** BTU circuit | when row 2, the left vertical and the middle line are laid: teach the new edges and stations; junction orders through the Ladder; `J-ML` explicit LEFT; wrong-branch FAULT; floor checks of X1 and X2 (MLS bench B7 style) | 20 BTU trips; every junction order and stop right; a provoked wrong branch (temporary order swap on a test mission) ends in FAULT within 2 m |

Tests, all offline (pure modules):
- **`tape_graph`:** projection, edge succession, binding refusal.
- **`tape_locator`:** latency roll-forward, the gates, σ growth, resync, health transitions, wrong branch.
- **`trip`:** formation, circuit order, a shared point served once, the payload guard.
- **Mission v3 refusals.**
- **The new profile key.**

Unified sim once per milestone (mechanisms only; tolerances are settled on the vehicle).

## 5. P0: what is measured, and what it decides

Tooling, built before the session:

| Id | What |
|---|---|
| T1 | `p0_localization.launch.py map_id:=… revision:=… params:=default\|lean initial:=start`: map_server, AMCL and the lifecycle manager on a verified bundle in the vehicle domain. With `initial:=start` it publishes the manifest's survey-start pose once. It publishes only `/amcl_pose`, `/particle_cloud` and the `map→odom` transform, which nothing in LINE reads. It commands nothing |
| T2 | `run_log.sh`: also records `/amcl_pose` and `/tf` (`map→odom`), and runs `pidstat -u 5` and the CPU temperature beside the bag |
| T3 | `tools/p0_report.py <run> [<run>…]`: builds the reference path from all passes, then computes M1–M6 per direction and prints PASS/FAIL against section 1 |

Ground truth: **MLS code-3 strips** at 4–5 points on the laid circuit (runbook section 2.2). The
MLS reports a marker at a fixed point along the strip, so each read is a repeatable
reference. If the marker bench (mls-marker-plan B0–B7) is not far enough along, the fallback is
the RFID reads of `line-a`, with their known scatter subtracted. That fallback is weaker and is
recorded as such.

| Metric | Definition | Target |
|---|---|---|
| M1 along-track repeatability | AMCL `s` at each code-3 read (latency-compensated), spread per strip and direction over ≥ 20 passes | 2σ ≤ 5 cm |
| M2 lateral residual | AMCL distance from the reference path on straights | 2σ ≤ 5 cm |
| M3 jumps | AMCL vs odometry change > 0.15 m in one update | 0 in the session |
| M4 latency | now − stamp of `/amcl_pose`, p95 | ≤ 0.2 s |
| M5 CPU / temperature | total CPU, package temperature | < 75 %, < 75 °C |
| M6 odometry scale | wheel travel vs AMCL distance between strips | sets `k_odo`; > 2 % means checking the wheel diameter first |

**Decision rule:**
- **M1–M5 met (default or lean set):** build P1 as written. The code-3 strips may stay as extra anchors.
- **M1 between 5 and 10 cm:** tune AMCL once (`sigma_hit`, `max_beams`, `update_min_d`) and re-run. If it stays there, **anchors carry the stops**:
  - a code-3 strip before each station group
  - the stop `s` comes from odometry after the anchor (M6 decides whether that holds over the distance)
  - AMCL only decides which edge and which anchor
- **M1 > 10 cm or M3 > 0:** AMCL is used only for edge identity and the overdue checks; stops and junction windows come from anchors and fork resyncs. Rewrite section 2.3 before P1.
- **M5 fails:** free CPU (2.7), use the lean set, re-measure. Never trade accuracy for CPU silently.

Results are appended below as **P0 results**, with the session date, the map revision and the run stamps.

## 6. Implementation status (2026-10-18)

Nothing is committed. P0 tooling and P1 are built; P2–P4 are not started (P2 waits for the P0
results and P1 shadow acceptance, section 4).

### Built

| Area | Files | Checked |
|---|---|---|
| Pure core | `amr_line/tape_graph.py`, `tape_locator.py`, `tape_teach.py`, `tape_graph_store.py`, `trip.py`, `tape_runtime.py` | `amr_line/test/test_tape_localization.py`, `test_tape_runtime.py` (35 tests; amr_line suite 219 pass) |
| Site mission | `agv_core/mission.py` schema `site-3`, `missions/site-103.json` | `tests/test_mission.py` |
| Profile | block `tape_loc` (`enabled: false`, provisional thresholds; `site_mission`/`shadow_circuit` = the straight test) | `tests/test_config.py`; offline suite 725 checks |
| T1 | `amr_bringup/launch/p0_localization.launch.py`, `amr_bringup/tape_loc.py` | launched on a private domain against `evo-mru-map-02`: bundle verified, lean params applied, lifecycle active, start pose received |
| T2 | `tools/run_log.sh`: `/amcl_pose`, `/tf`, `/amr/localization_state`, `cpu_temp.csv`, `pidstat.txt` | CPU/temperature sampler run |
| T3 | `tools/p0_report.py` | `tools/test_p0_report.py` (synthetic laps: M1, M3, M4, M6) |
| P1 node | `line_follow_node`: `/amcl_pose`, `/amr/localization_state` in; `/amr/line/teach` (new `LineTeach.srv`), `/amr/line/at_home`, `/initialpose` out; LineState `loc_*`, `teach` fields; `LINE_LOC` events | LINE layer launched with `tape_loc.enabled` on a private domain: group up, services answer, fields published |
| P1 launch | `line_layer.launch.py` runs the group from `$AMR_STATE_DIR/line_graph.json`; skipped with a reason, never blocking line following | same; unified sim (2 tests) passes with it off |
| P1 web | Teach page (engineer nav; teach API 403 for the operator view), `/api/line/site`, `/api/line/at_home`, a Locator tile on Run tracked | `amr_web/test/test_teach_page.py`; web suite 301 pass. **Not yet looked at on the 1280×720 panel** |

### Changes from the plan text, made while implementing

- **Profile key:** `line.localization` became a profile block `tape_loc`, holding `enabled` plus every locator threshold (CLAUDE.md: every tunable lives in the profile).
- **Legs:**
  - a leg is directed and named `<from>:<order>><to>`
  - the two directions of a tape are two legs, each taught on its own passes (not "stored once")
  - a junction node is where the MLS sees a single track again
- **T junctions:** a T item in the mission carries `approach: stem | bar`. "Straight at a T" is refused only from the stem: J-R2E is a straight pass on the trunk.
- **Unlaid junctions:** `laid: false` takes a junction out of the legs until it is laid. J-LV and J-R2E are unlaid today, so C-BTU cannot be taught yet.
- **Store location:** `tape_graph_store.py` lives in `amr_line`, not `amr_mission`, with a `manifest.json`.
- **Re-acquiring after LOST:** needs readiness to leave READY and come back (a revalidation). AMCL that disagreed with the wheels is not trusted just because it stayed READY.
- **Teaching:** whole-circuit passes from the HOME bay back to the HOME bay, split at the circuit's nodes. A pass whose fork/U-turn events don't match the circuit is refused. Stations are taught from a 3 s standstill AMCL mean.
- **Shadow mode:** runs on one circuit (node parameter `shadow_circuit`, default C-MRU). A lap completes at HOME and the next one starts there.

### Inputs needed (not decidable from the repo)

- **P0 results:**
  - the `tape_loc` thresholds (`k_odo`, `amcl_sigma_floor_m`, `max_lateral_m`, …)
  - the M1 decision on whether code-3 anchors stay; anchor resync is not implemented because it depends on that decision
- **`site-103.json`:**
  - U-MRU pivot direction: **CW** (operator, 2026-10-18)
  - `lead_m` and the approach values
  - whether J-S and X2 are laid today
  - `laid: true` for J-LV and J-R2E when row 2 is laid
- **Teach route:** does `line-a` drive exactly the C-MRU route, starting and ending in the north bay? If not, the teach passes are refused (runbook 3.2).
- **Floor and map:** the code-3 strip positions (A1–A5) and the survey map id/revision.
- **On the vehicle:** `sudo apt install sysstat` (P0 M5 per-process CPU).
- **Panel check:** the Teach page and the extra Run-tracked tile, at UI scale `l` on the panel.

### Next steps

1. **Stage 1 on the straight in front of MRU** (section 7.3, runbook 1A): P0, teach, shadow,
   provoked failures. Its inputs are in section 7.4.
2. **Stage 2 when the plant is free:** re-survey, `site-103` / `C-MRU` in the profile, teach,
   shadow, and the junction features stage 1 cannot show.
3. **P2 design pass with the P0 numbers:**
   - the stop-by-`s` and junction-order wiring in `tape_run.py`/`job.py`
   - the holds `position` and `payload`, and the fault `wrong_branch`
   - the Calls page (P3)

## 7. Staged floor tests: the straight in front of MRU first (2026-10-18)

The plant is not free for testing. Only a straight of about 10 m in front of MRU is: no
corners, no junctions. Stage 1 proves most of the stack there. Stage 2 is the plant, when the
area is cleared.

### 7.1 What the straight can and cannot show

| Feature | Stage 1 (straight) | Waits for stage 2 (plant) |
|---|---|---|
| AMCL along-track repeatability, lateral residual, jumps, latency, CPU, odometry scale (P0 M1–M6) | yes. A straight aisle is the pessimistic case for along-track (fewest features along the travel) | M1 on corners, the trunk and the HOME complex |
| Teach: passes, averaging, stations, publish, activate, binding refusal | yes, on `missions/test-mru-straight.json` | the site-103 legs |
| At home, readiness confirm, locator start | yes (T-HOME) | HOME-N |
| Shadow stops: release westbound, take back eastbound | yes (MW, ME, T-ATT, T-HOME) | R2 stops |
| U-turn approach and resync at the tape end, payload logic | yes, if the section includes the MRU line's west tape end (input) | U-turn R717 |
| LOST, re-acquire after revalidation, DEGRADED on odometry (provoked) | yes | — |
| Junction orders, fork resyncs, unexpected fork, wrong branch, crossings X1/X2, J-ML | **no**: no junction on the straight | all |
| P2 live stops by `s` | only after the P2 code and P0's numbers; on the straight first | the full circuits |

### 7.2 The test layout

- **Mission `test-mru-straight`** (schema site-3, circuit `C-STRAIGHT`):
  T-HOME → T-ATT → MW-1…MW-4 → U-MRU pivot → ME-4…ME-1 → T-HOME.
  - It has no junctions. Its one leg runs from the pivot, east to T-HOME, and back west to the pivot.
- **T-HOME is the east end of the free section,** facing west. There is no tape end in the east, so:
  - the run ends there: a temporary RFID home tag stops it (input)
  - the operator turns the AGV round there by hand (MANUAL spin, empty)
  - the turnaround is part of the taught leg; the locator handles the leg doubling back on
    itself (tie-break by the predicted `s`, tested)
- **T-ATT at least 2 m west of T-HOME.** Its braking point must lie beyond the start, where
  AMCL has corrected the At-home sigma.
- **Stations:** MW-*/ME-* only for the machines inside the section. Remove the others from the
  test mission, otherwise the circuit does not compile.
- **Profile:** `tape_loc.site_mission: test-mru-straight`, `tape_loc.shadow_circuit: C-STRAIGHT`.
  - Stage 2 sets `site-103` / `C-MRU`.
  - The straight's map and graph are not reused: the plant gets its own survey and teach. Only
    the numbers carry over.

### 7.3 Stage 1 steps (runbook section 1A)

| Step | Content | Done when |
|---|---|---|
| S0 | Lay 2–3 code-3 strips ≥ 3 m apart, ≥ 1 m from any stop. Survey the free area. The temporary T-HOME tag mission | `bag_report` shows each strip both ways |
| S1 P0 | 10 laps default, 10 laps lean (one lap = T-HOME → pivot → T-HOME), `tools/p0_report.py` | M1–M6 table in "P0 results", marked **straight** |
| S2 P1 teach | 3 passes, 10 stations (fewer if fewer machines), publish, activate | Build says every circuit compiles; spread ≤ 5 cm |
| S3 P1 shadow | 30 laps with At home once per session | every shadow decision "ok"; no LOST; predicted vs taught `s` at the code-3 strips 2σ ≤ 5 cm |
| S4 provoked | people beside the lidar (DEGRADED, back to HEALTHY); localization reset (LOST → revalidate → re-acquired); a station taught facing the wrong way (refused); a graph for another map revision (refused) | each ends as section 3 says |
| S5 (later) | P2 live stops by `s` on the straight, payload HOLD before the pivot | after the P2 code; ±10 cm on floor marks |

### 7.4 Inputs for stage 1 (answered 2026-10-18)

| Question | Answer | Effect |
|---|---|---|
| West tape end inside the section; which U-turn | yes, **CW** (tag 0180) | `uturns.U-MRU.direction: cw` in both missions, as set |
| Which MRU machines lie inside | **only MRU4** | `test-mru-straight`: T-HOME, T-ATT, MW-4, U-MRU, ME-4, T-HOME; `trip_calls: 1` |
| East-end stop | **no RFID for now** | below |

### 7.5 Without RFID: marker triggers on the straight (agreed 2026-10-18)

Today only RFID starts the U-turn (tag 0180) and ends a run at home. On the straight, RFID is
omitted, so two MLS markers take over. The test needs no speed zone (10 m leaves no HIGH
budget), so codes 1 and 2 are free there. Code 3 stays the P0 reference.

MLS marker codes in use (profile `mls.marker_codes: [1, 2, 3]`; codes 4–7 cannot be read):

| Code | Strips | Elsewhere (`line-a`) | On the straight test |
|---|---|---|---|
| 1 | 30 mm | speed-zone inner marker; zones work on the floor (tested the week of 2026-10-09; 100 mm gap between the strips of a pair) | **home stop**, on the way back only |
| 2 | 60 mm | speed-zone outer marker | **U-turn approach**, on the way out only; on the way back, the **backstop** for a missed home marker |
| 3 | 60 + 30 mm | nothing at runtime | P0 reference strips |

**Behaviour: a marker run** (a new block in the v2 mission, `marker_run`, refused together with `high_zone`)

1. **Out:** the first code 2 after departure starts the U-turn exactly as tag 0180 does today:
   - slow to 0.1 m/s within 0.3 m
   - stop at once when the tape is lost
   - pivot CW
   - Code 1 is ignored on the way out (the departure passes the home marker again).
2. **Back,** after the pivot: the first code 1 ends the run at home (a measured stop over 0.5 m, DONE). Code 2 passed again behind the pivot point is ignored.
3. **Backstop:** a code 2 on the way back, while home is still pending, means the home marker was missed:
   - the command is zeroed at once (the drives' deceleration, about 0.13 m from 0.5 m/s)
   - FAULT "home marker missed"
4. **Missed U-turn marker:** the follower's line-loss stop at the tape end ends the run (safe, as a missed 0180 today).

The operator turns the AGV round at T-HOME by hand, then presses Start for the next lap.

**Layout** (x along the straight, 0 = the west tape end, eastward):

| x | What | Rule |
|---|---|---|
| 0 | tape end | |
| **1.0 m** | **code 2, U-turn** | ≥ 0.3 m (decel) + 0.1 m (sensor lead) + the creep; ≤ `max_approach_m` 2.0 m |
| MRU4 | MW-4 / ME-4 (shadow only) | where the machine is |
| | code 3 strips, 2 of them | ≥ 3 m apart, ≥ 1 m from every other strip and stop |
| T-HOME − 2 m or less | T-ATT (shadow only) | ≥ 2 m west of T-HOME |
| **east limit − 2.0 m** | **code 1, home** | the AGV parks 0.5 m east of it (stop distance): T-HOME = east limit − 1.5 m |
| **east limit − 1.0 m** | **code 2, backstop** | 1.0 m past the home marker; a missed home ends about 0.15 m past it, still 0.85 m inside the free area |

Strips 15 mm × 100 mm, south up, on the same side as the speed-zone strips. **No other code 1 or
2 strip may lie between the backstop and the tape end.** If speed-zone pairs are laid on this
straight, they would trigger the U-turn and the home stop. They have to be covered for the test,
or the layout has to be built around them.

No speed-zone pairs are laid on this straight (operator, 2026-10-18).

**Built 2026-10-18** (motion code; offline tests only, not yet on the floor):
- **`agv_core/mission.py`:** an optional `marker_run` block in the v2 schema. It is refused:
  - together with a `high_zone`
  - together with any RFID tag or branch row
  - with the same code twice, or a code not laid
  - with a missing key, or a U-turn or stop distance the drives cannot make

  Tests: `tests/test_mission.py` (7 checks).
- **`amr_line/tape_run.py`:** markers are routed by phase:
  - out: code 2 starts the U-turn through the same path a tag uses
  - back: code 1 ends the run at home; code 2 within the approach distance + 1.0 m after the pivot is the re-pass and is skipped; after that it is the backstop, a FAULT
  - reads during a stop or the U-turn are ignored
- **`amr_line/job.py`:**
  - hold cause `markers`: the stream down or an event lost while driving; needs Start
  - Start is refused without `mls.markers_enabled` or with the stream down
  - alarm `MARKERS_LOST` (catalogue and RUNBOOK section 4 regenerated)
- **`missions/mru-straight-markers.json`:** the straight's mission.
- **Tests:** `amr_line/test/test_marker_run.py` (7 tests):
  - a full lap with no RFID link
  - the re-pass is skipped
  - the backstop FAULT
  - a missed U-turn marker ends at the tape end
  - markers lost holds, and Start is refused while they are down
  - markers during the U-turn approach are ignored

The shadow locator and teaching are unchanged: the U-turn phases come from the engine, as with RFID.
