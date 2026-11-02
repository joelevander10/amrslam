# Tracked speed plan 1 of 2: NORMAL 0.5 / HIGH 0.85 without SLAM

Status: IMPLEMENTED in software 2026-10-08 (offline tests); floor acceptance (section 6) open. Drafted 2026-10-08, split 2026-10-08. Tracked (LINE)
mode only. **No SLAM, no AMCL**: speed is decided by RFID tag pairs, an encoder distance
budget and a curve guard. This is what goes on the floor first.
Plan 2 (`tracked-speed-plan-2-slam.md`) adds localization redundancy on top, after this
plan is proven on the vehicle. Nothing here depends on plan 2.

## 0. Decisions

| # | Decision | Source |
|---|---|---|
| D1 | NORMAL 0.50 m/s is the default; HIGH = 1.7 x NORMAL = 0.85 m/s | operator |
| D2 | No software gate on 0.85 m/s. The nanoScan3 field set is not validated for it (`lidar.zones_validated` false) and there is no encoder speed monitoring in the FX3 yet: protective field and STO stopping distance at 0.85 are a floor check before the first HIGH run | operator |
| D3 | Steering gains: NORMAL K 11.3 / Kd 0.95 (2026-10-08: the corner set K 25 / Kd 5 wobbled at 0.5 m/s), HIGH K 11.3 / Kd 0.94, interpolated on the current speed | operator |
| D4 | LINE deceleration at the follower's rate: mux `line_d_max` = drive decel (~1.0 m/s^2) | operator |
| D5 | HIGH is switched by outer/inner ~~RFID tag~~ **MLS marker** pairs at each straight end, with an encoder distance budget. The single toggle tag is retired. ~~Shared ids: every inner tag is `0040`, every outer tag `0060`~~ **2026-10-09: shared codes, every outer marker code 2, every inner code 1 (swapped from 1/2 the same day)** (mission `high_zone.outer_code`/`inner_code`); pair spacing 0.5 m; entry window 0.65 m. **Strip gap inside a pair 100 mm (was 50 mm; operator 2026-10-18), so the two reads are 0.20 m apart, well inside the window; zones tested working on the floor the week of 2026-10-09.** Drawing: `documentation/layout-reference/mls-zone-marker-placement.html`. RFID decides only where to stop. See section "2026-10-09" below | operator |
| D6 | Corner radii 0.5-1.0 m; HIGH straights > 15 m | operator |
| D7 | Curve guard on the IMU gyro (MLS), wheel odometry as fallback | operator |
| D8 | Warning field 2 `stop_m` 0.47 -> 0.40 (latency margin before the protective field) | operator |

**Principle: every failure ends at NORMAL.** HIGH needs every condition true at once;
any missed read, hold, stop, fault, stale input or guard trip drops to NORMAL, and
nothing but a fresh, correctly ordered zone entry raises it again. Nothing here is in
the safety path (scanner -> FX3 -> STO stays hardware); this only decides how fast the
follower asks to go.

## 2026-10-09: the zone cue moves from RFID to MLS markers

Operator decision: **MLS codes govern speed, RFID governs where to stop.** Speed zones must
work independently of the RFID setting. Everything else in this plan stands (budget, ramps,
curve guard, park/resume, every failure ends at NORMAL); only the cue changed:

| Item | Change |
|---|---|
| `agv_core/mission.py` | schema v2.2: `high_zone` = `{shortest_straight_m, pair_spacing_m, outer_code, inner_code}`; codes must be in `mls.marker_codes` and differ. `zone_outer`/`zone_inner` rows are refused with a pointer; the v2.1 block without codes is refused. `ZONE_ARM_TOLERANCE_M` 1.0 -> 0.15 m |
| `amr_line/speed_zone.py` | `SpeedZone.tag(id)` -> `cue(code)`; the state machine is unchanged |
| `amr_line/tape_run.py` | RFID scan no longer touches the speed. `markers()` feeds the zone: only the two codes, only while driving (not stopped, not in a U-turn), line good, \|LCP2\| <= 30 mm. Stream down or events lost: ARMED/HIGH -> NORMAL. `markers_enabled` false: the zone is off, a warning, the run stays NORMAL (the mission still loads: stops do not depend on markers) |
| `amr_line/job.py`, `marker_reader.py`, `line_follow_node.py` | the marker snapshot carries `line_good`; the job hands this tick's new markers to the run while RUNNING |
| `missions/line-a.json` | zone rows removed; `outer_code` 2, `inner_code` 1 (operator swap, 2026-10-09: code 2 nearer the corner) |

Before the first HIGH run, on top of section 6: marker plan B5 at 0.85 m/s (20 passes per
code, centred and 25 mm off). The RFID `0060`/`0040` clusters on the floor are now unknown
tags ("no tag rule"); lift them or leave them.

## 1. Why sensor-only curve detection is a backstop, not the trigger

- The MLS looks `sensor_lookahead_m` = 0.1 m ahead of the axle: ~0.12 s of warning at 0.85 m/s.
- 0.85 -> 0.50 m/s takes >= 0.35 s / 0.24 m at the drive limit, ~1.35 m with the 2 s comfort ramp.
- Entering radius R the tape leaves the sensor's +/-100 mm after s ~ sqrt(2 R 0.1): 0.32 m at R 0.5, 0.45 m at R 1.0.
- Steady corner error is kappa / K, independent of speed (autopilot docstring):

  | R | K 25 (NORMAL) | K 11.3 (HIGH) |
  |---|---|---|
  | 0.5 m | 80 mm of 100 | 177 mm - track lost |
  | 1.0 m | 40 mm | 88 mm - marginal |

So HIGH must end **before** a corner by tag placement (this plan; plan 2 adds position);
the curve guard only limits the damage when that fails. Note R 0.5 m is marginal even at
NORMAL with K 25, and NORMAL now runs K 11.3: corner tracking at 0.5 m/s on the tightest
real corner (0.8 m reported) is the first floor check (section 6); raise `k_ratio` in steps
(14, 16) with `kd` 1-1.5 if the line position nears 80 mm.

Literature behind the layers: error-magnitude speed adaptation (already in the
autopilot as `sr_*`), curvature from yaw rate kappa = omega/v, tape heading-angle sensing
(Naviq MTS160), course learning (Pololu LearnBot), curvature-based speed profiles.

## 2. Step A0 - speed reversal (no new behaviour beyond the numbers)

| Item | Change |
|---|---|
| Profile | `auto_rpm` 1500 -> **1592** (0.50 m/s); `auto_slow_ratio` -> **`auto_high_ratio` 1.7** (in (1, 2], `AUTO_HIGH_RPM` <= `motor_max_rpm`); `k_ratio`/`kd` = 11.3/0.95 (NORMAL; 25/5 from the old slow zone was tried 2026-10-08 and wobbled at 0.5 m/s - tuned at 0.24 m/s, yaw slew demand grows with v^2), new `high_k_ratio`/`high_kd` = 11.3/0.94; `slow_k_ratio`, `slow_kd`, `auto_slow_ratio`, `gain_blend_s` refused |
| Config | `AUTO_SLOW_RPM` -> `AUTO_HIGH_RPM` (derived); checks, comments, params view |
| `runtime.py` | name map: `AUTO_HIGH_RPM`, `HIGH_K_RATIO`, `HIGH_KD`; line-loss standstill ceiling keyed off NORMAL |
| `autopilot.py` (deliberate port edit) | `update(slow=)` -> `update(high=)`: cruise = `AUTO_HIGH_RPM` if high else `AUTO_RPM`. Gains **interpolated on the current base speed** between the NORMAL and HIGH pairs instead of the 0.4 s time blend: a 2 s high->normal ramp would otherwise carry K 25 near 0.85 m/s, past the yaw-slew limit the docstring warns about |
| Junction `slow_speed` (branch_latch) | meant "no HIGH through this junction"; refused since 2026-10-09 (no RFID tag sets the speed: keep the junction outside the marker zone) |
| U-turn creep | `approach_mps` <= NORMAL |
| Mission validator | stop rate checked from the fastest reachable speed (HIGH if the mission has high zones) against drive decel: 0.85 m/s in 0.5 m = 2300 r/min/s <= 3200 |
| Mux (`cmd_mux_kinematics_node.py`) | `line_d_max` param, 0 = drive decel; LINE source only (like `line_alpha_max`). **Failure mode:** any LINE command drop (stale, hold, fault) decelerates at up to ~1.0 m/s^2 instead of 0.5 - harder, never longer. **Recovery:** none needed; `line_d_max: 0.5` restores today. Test next to the `line_alpha_max` test |
| `scanner_fields.yaml` | warning_2 `stop_m` 0.47 -> **0.40**. Direct entry at 0.85: 0.90 m/s^2 (< drive 1.0); at 0.50: 0.31 m/s^2. Change the Safety Designer notes with it (this file is a copy). `scanner_fields.*` rule: adjust its test, state failure mode (a stop now ends 7 cm before the protective edge; latency up to ~0.08 s at 0.85 is absorbed) |
| Encoder distances | U-turn approach travel, U-turn re-pass re-arm and every distance budget use wheel travel (`/wheel_states` counts, as `TagOdometer` does), not commanded speed: a warning-field hold no longer burns `max_approach_m` |
| U-turn spin budget | paused while the mux reports a warning-field hold, instead of faulting the turn |
| Wire / UI | `LineState` append `bool high_speed`, `string speed_reason`; Run tracked Speed tile NORMAL/HIGH + reason; `colcon build` |
| README | status table speeds; known gap: 0.85 not validated against fields/STO |

## 3. Step A1 - fail-safe high zones (replaces the speed toggle)

### Layout

Each end of a HIGH straight carries a pair. All inner tags share id `0040`, all outer
tags `0060`; the read ORDER gives the direction, so no tag needs to know which straight
or which end it is on. Drawing: `documentation/layout-reference/corner-tag-placement.html`.

```
straight S1 ======== [40]--0.5 m--[60]--0.5 m--T1 ) corner R ( T2--0.5 m--[60]--0.5 m--[40] ======== straight S2
                     inner         outer                                   outer         inner
```

- T = tangent point (curve meets straight). O (`0060`) 0.5 m past T, I (`0040`) 0.5 m past O.
- Same pair at both ends of every HIGH straight. A corner between two NORMAL-only
  straights needs no tags.
- Any NORMAL area is a "corner" here, not only a curve: the busy Home area (Home,
  other AGVs' homes, release and hitch points) ends its HIGH straight with the same
  pair, outer (`0060`) on the busy side. Leaving Home: read 60 then 40 -> HIGH;
  arriving: 40 then 60, the budget already spent. That straight counts toward
  `shortest_straight_m`.
- Read zones (2026-10-08): each id is a cluster of 4 redundant tags in a 10 cm square,
  read over the antenna's field, so the 60 and 40 clusters may be in the field together
  (reads alternate 60/40/60...). Tolerated by distance, not time: a zone id seen again
  within `arm_m` (1.5 m since 2026-10-08; was 0.7, then 1.2) of wheel travel is the same pass and ignored, and a 60 within
  `arm_m` after a 40 is a straight being left and never arms (else a 40 re-read after the
  60 would grant HIGH into the corner). First reads stay 0.5 m apart whatever the field
  length; the entry window allows +0.2 m of first-read jitter. Limit: exit-60 and
  entry-60 must be > 1.5 m apart counted from the end of the exit cluster's field
  (1.0 m + arc − field length).
- Departure speed (2026-10-08): a run from Home always starts NORMAL. A station stop
  (MRU, `always` stops) departs at the speed it arrived at: arrived HIGH, the budget
  left at the stop tag (less the stop distance and any travel while parked) is kept
  and the departure ramps back to HIGH. A protective-field trip while parked keeps it
  (2026-10-23). Any other hold, a zone tag, missing counts or Reset in between forgets
  it -> NORMAL (`SpeedZone.park/resume`).
- Tags at the same lateral offset as the station tags (the reader antenna's track).

### Mission schema (v2.1)

```json
"high_zone": {"shortest_straight_m": «measure», "pair_spacing_m": 0.5},
"tags": [
  {"tag": "0040", "action": "zone_inner", "ignore_s": 1},
  {"tag": "0060", "action": "zone_outer", "ignore_s": 1}
]
```

- `shortest_straight_m`: the shortest HIGH straight, surveyed inner tag to inner tag.
  Shared ids cannot tell straights apart, so there is ONE budget for all of them:
  `high_for_m = shortest_straight_m - brake_m - margin_m`, `brake_m` = travel of the
  comfort ramp ((0.85+0.50)/2 x `high_ramp_s` = 1.35 m at 2 s), `margin_m` 1.0 m. Longer
  straights fall back to NORMAL early - harmless. Refused when `high_for_m` < 3 m.
  A straight much longer than the rest can later get its own ids (e.g. 41/61) with its
  own budget; not in this phase.
- `ignore_s` <= 2 s for zone tags (refused above; 1 s until 2026-10-08, line-a uses 2):
  through one corner the same id is read twice (exit O, then entry O) only 1.79 m apart
  (0.5 + pi R/2 + 0.5 at R 0.5) - 3.6 s at 0.5 m/s. The old toggle's 5 s window would swallow the entry O and the next
  straight would never go HIGH. Repeat reads of one pass are merged by the driver's
  `tag_clear_s` anyway.
- `speed_toggle` rows refused with a pointer to `zone_inner`/`zone_outer`; `ramp_s` moves
  to the profile (`autopilot.high_ramp_s`).

### Engine (`amr_line/speed_zone.py`, pure, clock- and odometry-fed)

| State | Event | Next | Note |
|---|---|---|---|
| NORMAL | O (`0060`) | ARMED | arm expires after **1.5 m** of encoder travel (spacing 0.5 + tolerance 1.0; 0.2 was too tight: first reads 0.65 and 0.99 m apart on the floor, 2026-10-08) |
| ARMED | I (`0040`) within the window | HIGH(budget) | ramp up over `high_ramp_s`, starting 1.0 m past T |
| ARMED | window expired (station tags have no effect, 2026-10-08) | NORMAL | |
| NORMAL | I alone | NORMAL | the normal exit (budget already ended HIGH) or a missed entry O |
| HIGH | budget used up | NORMAL, comfort ramp (2 s) | the planned exit: done >= 1.0 m before the far I |
| HIGH | I | NORMAL at **0.5 m/s^2** | late exit (budget too long: survey or encoder scale wrong) - WARN event |
| HIGH | O | NORMAL at **drive decel ~1.0 m/s^2** | I also missed - WARN event |
| HIGH | stop begins (parked, departs HIGH), U-turn, any HOLD but a protective stop or RFID link loss (parked, resume HIGH, 2026-10-09; also a protective stop first reported as torque loss, and a field trip while parked at a station, 2026-10-23), a real E-stop (parked, resumes NORMAL), Reset, new run, curve guard trip, IMU+odometry stale | NORMAL | reason recorded. Warning fields 1/2 never: the mux slows or stops the vehicle, the budget counts wheel travel |

The entry window (1.5 m) is shorter than the shortest exit-O to entry-I distance
(0.5 + pi R/2 + 0.5 + 0.5 = 2.29 m at R 0.5), so an O left armed on the way OUT of a
straight always expires inside the corner.

### Corner entry, every case (0.85 -> 0.50 m/s, 0.1 s read latency = 0.085 m)

| Case | HIGH ends at | Decel | Distance to NORMAL | NORMAL reached before T |
|---|---|---|---|---|
| Planned | budget, >= 2.35 m before I | comfort 2 s ramp | 1.35 m | >= 2.0 m (1.0 margin + I-O-T 1.0) |
| Already NORMAL (an error slowed it mid-straight: curve guard, a hold, early budget) | - | - | - | tags only arm/no-op; corner at NORMAL; next straight enters normally |
| Budget too long | I read while HIGH | 0.5 m/s^2 | 0.47 + 0.085 = 0.56 m | 0.44 m |
| Budget too long AND I missed | O read while HIGH | ~1.0 m/s^2 (needs D4 `line_d_max`) | 0.24 + 0.085 = 0.32 m | 0.18 m |
| All three missed | - | curve guard (section 4) | - | none: the guard limits damage only |

At 0.3 m spacing the O backstop would need ~1.1 m/s^2 - beyond the drive's 1.0 - which
is why the spacing is 0.5 m. A warning-field slowdown inside a straight (mux scaling)
does not end HIGH; the budget is encoder travel and keeps counting truthfully.

Missed reads on the way IN all end at NORMAL: O missed -> I alone; I missed -> the arm
expires.

### Site table

`missions/line-a.json` (as implemented 2026-10-08): the `0040` toggle row is removed
and `high_zone` is `null`, so the site runs everything at NORMAL - stops, U-turn, jobs -
while the pairs are placed. Once the shortest HIGH straight is surveyed, add the two
rows and the block:

```json
"high_zone": {"shortest_straight_m": «measured», "pair_spacing_m": 0.5},
{"tag": "0040", "action": "zone_inner", "ignore_s": 1},
{"tag": "0060", "action": "zone_outer", "ignore_s": 1}
```

The loader refuses zone rows without the block and a block without both rows: HIGH is
never run on a guess.

## 4. Step A2 - curve guard (backstop)

- **Signal:** kappa_hat = omega / v with omega from `/imu/data` (bias-corrected MLS gyro,
  `imu_bias_node`), fallback omega = (v_r - v_l) / track from `/wheel_states` when the IMU
  is older than 0.1 s; v from wheel odometry. Inactive below 0.2 m/s. Both stale -> HIGH
  refused (D-principle).
- **Second signal:** |lateral error| from the follower (> `curve_guard_e_mm`, default 40 mm).
- **Trip:** |kappa_hat| > `curve_guard_kappa` (default 0.25 1/m, R 4 m: well clear of
  straight-line wobble, well below the 1.0-2.0 1/m corners) sustained over 0.05 m of
  travel, or the error trip, while HIGH or ramping down.
- **Action:** NORMAL **at drive deceleration**, bypassing the 2 s comfort ramp; latched
  until the next valid O->I entry. Event + `speed_reason` "curve guard".
- **Independence note:** the gyro lives in the MLS, the same CAN device as the tape
  reading. An MLS failure already stops the follower (`sensor_lost`), so the guard is not
  needed then, but it is not independent of the MLS link.
- Profile keys (`autopilot.curve_guard_kappa`, `curve_guard_e_mm`) with schema, validation
  and refusal tests. Thresholds settled on the floor (section 6), not in sim.

## 5. Failure modes and recovery

| Failure | Effect | Recovery |
|---|---|---|
| Missed zone tag | NORMAL earlier or budget expiry before the corner | none; next entry pair |
| Toggle-style inversion | impossible: no toggle state survives a miss | - |
| Curve entered at HIGH anyway | guard trips -> drive-decel to NORMAL; worst case tape lost -> `line_lost` -> DONE | operator re-places the vehicle; check tag placement |
| IMU and odometry stale | HIGH refused | automatic when fresh |
| Warning field at HIGH | mux factor ramp; warning-2 stop ends 0.07 m before protective | automatic when clear |
| Protective field at HIGH | STO (hardware) + software zero; stopping distance at 0.85 **unvalidated** | HOLD field, auto-resume 2 s |
| `line_d_max` drop to zero | stop at up to ~1.0 m/s^2 | - |


## 6. Tests and floor acceptance

Offline (focused, per CLAUDE.md): config refusals; mission v2.1 refusals; `speed_zone.py`
table above row by row, plus every corner-entry case (planned, late at I, late at O, arm
expiry through a corner with shared ids); curve guard thresholds, fallback and staleness; autopilot gain
interpolation and `high=` default-equivalence; mux `line_d_max` vs other sources;
`scanner_fields` stop_m; encoder-distance U-turn approach under a simulated warning hold.
Unified sim once at the end of the plan.

Floor, in order - each step PASS when the operator saw it:

1. Corners at NORMAL 0.5 m/s with K 11.3 on the tightest radius (R 0.5 m is marginal by the
   kappa/K estimate).
2. Protective field + STO stopping distance from 0.85 m/s (D2) - before any HIGH run.
3. HIGH on the longest straight: ramp up/down 2 s, end before the corner by budget.
4. Each missed-tag case from section 3 by covering one tag at a time; the late-I and
   late-O exits by temporarily setting a budget longer than the straight.
5. Curve guard: enter a corner deliberately at HIGH at a safe test corner; record kappa_hat
   and error traces; set thresholds.
6. Warning 1 / warning 2 at both speeds: stop position vs the protective edge.

## 7. Open items («measure» / «placeholder»)

- Pair positions on the floor (0.5 m spacing from each tangent point); the shortest HIGH
  straight, inner tag to inner tag (`shortest_straight_m`).
- Reader read-zone length vs the 0.5 m spacing (each read must end before the next tag).
- Radius of each corner (tightest first).
- FX3/nanoScan3 validation at 0.85 m/s; later, encoder speed monitoring and speed-switched
  fields (then revisit D2 and D8).
