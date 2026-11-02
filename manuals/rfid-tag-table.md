# RFID tag table — tracked (LINE) product

Status 2026-10-08. Source of truth: `missions/<name>.json` (validated by `agv_core/mission.py`,
schema v2.1) and the `autopilot` block of `profiles/agv-01.json`. This page describes them;
if they disagree, the files win.

One tag id has one meaning per mission. Tags only do three independent things: **stop**,
**speed** (NORMAL / HIGH zone) and **branch** (plus the U-turn).

## Site tags (mission `line-a`)

| Tag | Action | Meaning | Stops / acts when | Ignore after read | Restart |
|---|---|---|---|---|---|
| `0010` | stop, role `home` | Home: decelerate to a stop over 0.5 m | every pass; ends the run (DONE) | 2 s | new job, then panel Start |
| `0020` | stop, role `always` | Trolley release: stop over 0.5 m | every pass | 2 s | panel Start (+0.6 s start delay) |
| `0180` | `u_turn`, cw | U-turn CW: slow to 0.1 m/s within 0.3 m, stop the moment the tape is gone (creep reached or not), pivot CW until the tape is back and centred | every pass, **also with no mission**; its own tag is ignored once on the way back | 2 s | continues by itself |
| `0190` | `u_turn`, ccw | U-turn CCW: as `0180`, pivoting CCW | as `0180` | 2 s | continues by itself |
| `0110` | stop, role `destination` | MRU1: stop over 0.5 m | only when MRU1 is the job's destination, once | 4 s | panel Start (+0.6 s start delay) |
| `0120` | stop, role `destination` | MRU2 | as MRU1 | 4 s | as MRU1 |
| `0130` | stop, role `destination` | MRU3 | as MRU1 | 4 s | as MRU1 |
| `0140` | stop, role `destination` | MRU4 | as MRU1 | 4 s | as MRU1 |

`branch_default` for `line-a` (and `empty`) is `left` (2026-10-08); no branch-latch tags yet.

## Speed zones: MLS markers, not RFID (2026-10-09)

**RFID decides where to stop; the MLS marker codes decide speed.** The RFID zone tags
(`0060` outer, `0040` inner, `zone_outer`/`zone_inner` rows) are retired and refused by the
mission loader: one weak read at 0.49 m/s (2026-10-09) and first reads of a 0.5 m pair landing
0.65-0.99 m apart (2026-10-08) made HIGH unreliable. The zone works whatever the RFID setting.

`line-a`: `high_zone` = shortest straight 10 m, pair spacing 0.5 m, **outer code 2, inner
code 1** (swapped by the operator 2026-10-09; code 3 stays free), HIGH budget 7.65 m.
"Outer" and "inner" are positions ALONG the tape: outer nearer the corner, inner further
into the straight. Both strips lie on the same side of the tape
(`documentation/layout-reference/mls-zone-marker-placement.html`).

| Marker | Placement (from the tangent point) | Effect |
|---|---|---|
| code 2 (outer) | 0.75-0.85 m, strip centred 60 mm from the track centre | arms the zone; HIGH only if code 1 follows within 0.65 m |
| code 1 (inner) | 0.90-1.00 m (50 mm gap after code 2), strip centred 30 mm from the track centre | after code 2: HIGH for the budget. Read while HIGH: late exit at 0.5 m/s² |

- Same codes at every straight end; the read order gives the direction (2→1 enters, 1→2 leaves).
- HIGH budget = `shortest_straight_m` (inner to inner: code 1 to code 1, straight-side edges) − brake 1.35 m − margin 1.0 m, counted in wheel travel.
- Code 2 read while HIGH (inner missed): urgent drop to NORMAL.
- A zone marker counts only while RUNNING, with the line good and within 30 mm of the tape
  centre (a pivot over a marker read code 2 at +121 mm, 2026-10-09). Marker stream down, or
  marker events lost on the topic: ARMED/HIGH drop to NORMAL.
- Entry window = 0.5 m spacing + 0.15 m: the MLS reports a marker at a fixed point
  (FailSafe: after the 100 mm marker plus ~35 ms filter delay, the same for both markers).
- The two codes are the zone's only: lay no other marker with them.
- Detail: `manuals/tracked-speed-plan-1-no-slam.md`, `manuals/mls-marker-plan.md`. The drawing
  `documentation/layout-reference/corner-tag-placement.html` still shows the RFID pairs.

## Speeds

| | Speed | Gains |
|---|---|---|
| NORMAL (default, corners, busy areas) | 0.50 m/s (1592 r/min) | K 11.3 / Kd 0.95 |
| HIGH (inside a zone only) | 0.85 m/s (+70 %) | K 11.3 / Kd 0.94 |

- A run from Home always starts NORMAL.
- A station stop (`always`, `destination`) departs at the speed it arrived at, with the budget
  left; any other hold, a zone marker, missing encoder counts or Reset in between makes it NORMAL.
- Warning fields (slow, slow-and-stop) keep HIGH: the mux scales the command and the budget
  counts wheel travel, so the vehicle speeds back up to HIGH when the field clears.
- **No RFID tag or RFID event sets the speed (2026-10-09).** Every stop tag (Home included)
  parks the budget and the departure restores it; an RFID-link hold and a protective stop do the
  same, less any wheel travel while held. Junction slow zones (`branch_latch[].slow_speed: true`)
  are refused by the loader.
- The U-turn is the one RFID-triggered speed change: it creeps in and pivots, and the run goes on
  NORMAL until the markers grant HIGH again (the vehicle now faces the other way).
- Any other hold (E-stop, drive feedback, track, authority) or a curve-guard trip drops HIGH.

## Rules that apply to every tag

- **Start**: with a mission, anywhere on the line (2026-10-09; Home no longer required) and a
  destination chosen when the mission has destinations; then the physical panel Start under
  AUTO. No web page starts the vehicle. Started past the destination on the outbound leg, the
  U-turn reports it missed and the run ends at Home.
- **No mission** (plain line following, `missions/empty.json`): only the U-turn tags `0180`/`0190`
  act; every other tag is ignored. NORMAL speed, no Home check, no RFID link needed to start
  (a missed U-turn tag ends at the tape end, where the line-loss stop halts the vehicle).
- **U-turn end**: the encoders only gate the pivot (tape accepted from 150°, fault at 225°, or
  past twice the nominal spin time); it ends when the tape is back under the sensor and
  centred within ±10 mm for 1 s. With the axle on the tape line that is 180° by geometry.
- **Departure**: the stop being left is not acted on again, however slowly the AGV pulls away.
- **Held / stopped**: reads cannot start a stop, U-turn or zone entry (branch exits still count).
- **Destination missed**: if the U-turn tag comes first, the destination is reported missed and
  not served on the return leg.
- **RFID link lost while driving** (mission with tags): hold `rfid`, panel Start to resume.
