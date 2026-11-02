# Tracked speed plan 2 of 2: SLAM / RFID / line-sensor redundancy

Status: PLAN, nothing implemented, **deferred**. Drafted 2026-10-08, split 2026-10-08.
2026-10-17: D10 (taped path drawn in the route editor) is superseded by `tracked-plan-3-tape-localization.md`
(path recorded, never drawn). A later speed gate here consumes plan 3's locator `s`.
Starts only after plan 1 (`tracked-speed-plan-1-no-slam.md`) runs on the vehicle: it
reuses plan 1's NORMAL/HIGH speeds, zone markers (MLS codes, outer 2 / inner 1, since 2026-10-09; RFID `0040`/`0060` before), distance budget and curve
guard unchanged, and only ADDS a localization opinion. With localization off the vehicle
behaves exactly as plan 1.

## 0. Decisions

| # | Decision | Source |
|---|---|---|
| D9 | Localization optional inside the LINE layer (profile flag); no Nav2 controller, behaviors or executor | operator |
| D10 | Zones authored as a taped path in the route editor, bound to the map revision | operator |

**Principle (inherited): every failure ends at NORMAL.** Localization can only take HIGH
away, never grant it on its own: HIGH needs plan 1's tag entry AND, when localization is
on, a healthy position inside a HIGH zone.

## 1. Why add it

- Plan 1's last line of defence when every zone tag is missed is the curve guard, which
  only limits damage. An independent position knows a corner is coming regardless of tags.
- A position finally gives the overdue-tag check (`rfid_tag_overdue`, described in
  `agv_core/drivers/rfid.py`, never implemented): a stop or U-turn tag not read by its map
  position + margin -> HOLD.
- Loop closure is not a runtime risk: production localizes with AMCL on a frozen,
  reviewed map revision (AMCL has no loop closure). It is a map-management risk: a new
  revision can shift the zones, hence D10 and the revision check below.

## 2. Roles

| Decision | Line sensor | RFID | Localization | Rule |
|---|---|---|---|---|
| Steering | owner | - | - | unchanged |
| Exact stops | - | owner | cross-check | tag not read by its map position + margin -> HOLD (`rfid_tag_overdue`) |
| HIGH allowed | curve guard | zone pairs | zone [s_start, s_end] | HIGH only if **all available** agree |
| Slow before corner | - | exit pair, budget | zone end - brake - k*sigma | earliest wins |

## 3. Mechanics

- **Layer:** profile `line.localization: true` + a bound map revision adds `map_server`,
  AMCL and `localization_monitor_node` to the LINE layer. No Nav2 controller, behaviors or
  executor. Flag off = today's LINE exactly.
- **AMCL for a vehicle on tape:** `max_particles` 2000 -> ~800, `update_min_d` 0.05 -> 0.15 m,
  `update_min_a` 0.03 -> 0.1 rad. Lateral is fixed by the tape; only along-track matters.
- **Taped path:** a polyline drawn on the map revision in the route editor, RFID tags placed
  on it; stored in the map bundle, so it moves with its revision. Loader refuses a path
  whose tags disagree with a new revision (loop closure / re-mapping protection).
- **Position along route s:** AMCL pose projected onto the path; direction from ds/dt;
  re-synced at each RFID read. Along-track sigma from the AMCL covariance projected on the
  path tangent.
- **Jump detection:** AMCL delta vs odometry delta > 0.15 m in one update -> untrusted until
  the next RFID re-sync.
- **Gate:** `amr_line/speed_gate.py`, pure: inputs s, sigma, jump flag, zone list, RFID
  zone state, curve guard; output NORMAL/HIGH + reason. A zone's HIGH end is pulled
  forward by `brake_m + k*sigma + latency`; when sigma makes a zone shorter than 3 m it
  stays NORMAL. Stale localization -> NORMAL, so CPU overload costs speed only.

## 4. CPU

LINE today: ~2.6 of 4 N97 cores, 58 C (2026-10-08 sample). Estimated add: AMCL 10-25 % of a
core (tuned lower), monitor 5-10 %, gate negligible -> ~70-75 %. Free first:
`commissioning_node` (15 % idle in LINE), profile `web_node` (38 %). Go rule: measured
< 75 % total and < 75 C on a warm day.

## 5. Measurement before building

One stationary-then-slow LINE run with AMCL running beside it (temporary launch; needs a
service stop/start, only on the operator's go): `pidstat 5` for 15 min + AMCL along-track
error at every RFID read in the longest straight. sigma <= 0.2 m -> build as described;
worse -> SLAM is only the overdue-tag detector and corner backstop.

## 6. Failure modes and recovery

| Failure | Effect | Recovery |
|---|---|---|
| AMCL jump / lost / along-track sigma too high | NORMAL (plan 1 rules still apply underneath) | RFID re-sync |
| Map revision changed | zones refused until the tape path is re-checked against the tags | operator re-binds |
| CPU overload | localization stale -> NORMAL | automatic |
| Stop / U-turn tag overdue by map position | HOLD for a person | operator checks the tag and the position, then Start |
| Localization disabled (flag off) | exactly plan 1 | - |

## 7. Tests and floor acceptance

Offline: `speed_gate.py` row by row (agree / disagree / stale / jump / sigma too large);
projection of the pose onto the taped path; overdue-tag HOLD; the loader's tag-vs-map
check on a shifted revision; flag off reproduces plan 1 exactly. Unified sim once.

Floor: the section 5 measurement first; then HIGH with localization on, a zone tag
covered (localization must still end HIGH before the corner); a pose jump provoked by
occluding the scanner (must drop to NORMAL); an overdue stop tag (cover it: HOLD).

## 8. Open items

- CPU and along-track accuracy measurement (section 5) - decides whether this plan is built
  as written or reduced to the overdue-tag detector and corner backstop.
- Taped-path drawing in the route editor for the current map revision.
- Profile/product question: a `tracked: true` vehicle with localization needs a survey;
  the mode FSM refuses MAPPING for `tracked: true` today.
