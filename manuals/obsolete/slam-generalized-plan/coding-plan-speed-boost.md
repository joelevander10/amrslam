# Coding plan: route speed 0.50 m/s, long straights 0.70 m/s

> Implemented 2026-09-18 (Parts A and B, plus manual: pendant 0.50 m/s, S-curve
> 0.3 m/s², 75 % turn ratio, spin unchanged). Decision 1 amended: `angular_rad_s`
> above the ceiling is clamped, not refused, because five stored routes carry 0.30.
> Rotation later raised 0.24 -> 0.34 rad/s (+40 %) with `rotational_acc_lim` 0.17, same day.
> Vehicle acceptance below is still to do.

Three requirements: the general autonomous speed goes from 0.40 to 0.50 m/s;
a straight longer than 4 m may run at 0.70 m/s; rotation stays at 0.24 rad/s.

The speed chain today has one number in four places: the route file's
`limits.linear_mps` (default 0.40, `route.py`), the editor's speed input
(max 0.40), RPP `desired_linear_vel: 0.40` (the vehicle-wide cap in
`nav2_params.yaml`), and the MotionPermit `v_max` the executor publishes per
run, which the mux clamps to (`gating.select` → `capped`). Nothing in the
chain is per step. The boost needs a per-step speed, so the executor becomes
the place that decides the speed of the *current* step, and it tells both
Nav2 (so the controller plans its approach with that speed) and the mux (the
hard clamp) the same number.

Hard prerequisite, not code: the nanoScan3 protective field must be sized
for 0.70 m/s before any run above 0.40 (RUNBOOK §commissioning already says
so for blind runs; `lidar.zones_validated` is still `false` in the profile).
Functional stopping from 0.70 at the mux `d_max 0.5` is 0.49 m plus ~0.5 m of
latency (scan gate hold ≤0.5 s, 20 Hz controller, mux slew), so the field
needs ≥1.0 m ahead at 0.70. Rear blind spot unchanged: forward only, as now.

---

## Part A — 0.40 → 0.50 vehicle-wide

### A1. Route schema — `amr_ws/src/amr_navigation/amr_navigation/route.py`
- `Limits.linear_mps = 0.50`. Bounds (0..5) unchanged.
- New vehicle constants next to the bounds: `VEHICLE_V_MAX = 0.70`,
  `VEHICLE_W_MAX = 0.24`; the loader (`Route.from_dict`, the `LIMIT_BOUNDS` loop) refuses a file whose
  `linear_mps` / `long_linear_mps` / `angular_rad_s` exceeds them with a
  `RouteError` naming the field. Today a file could ask for 5 m/s and only
  the mux clamp would stop it.
- Existing route files carry explicit `limits`, so they keep 0.30/0.40; only
  routes saved after this change get 0.50.

### A2. Nav2 — `amr_ws/src/amr_navigation/config/nav2_params.yaml`
- `FollowPath.desired_linear_vel: 0.70` — the ceiling. The per-step speed
  comes from the `/speed_limit` topic (Part B), so this number is what a
  step may reach, never what every step runs at. Update the comment.
- `approach_velocity_scaling_dist: 0.6 → 1.0`. RPP scales the approach
  linearly with distance; from 0.70 the 0.6 m ramp demands ~0.6 m/s² in its
  first half, above the mux `d_max 0.5`, and the lag lands past the endpoint.
  1.0 m keeps it at ≤0.37 m/s² from 0.70 and ≤0.19 from 0.50. Cost: every
  straight starts slowing ~0.4 m earlier (~0.3 s per step at 0.50).
- `max_allowed_time_to_collision_up_to_carrot: 1.0 → 1.5` (costmap collision
  projection: 1.05 m ahead at 0.70 instead of 0.70 m).
- Lookahead: `lookahead_time 2.5 × 0.70 = 1.75` clamps to `max_lookahead_dist
  1.2`; fine for straight lines, no change.
- `behavior_server.max_rotational_vel 0.24`, `rotational_acc_lim 0.12`,
  `angular_rad_s 0.24` untouched (requirement 3).

### A3. Mux and drives — no change, checked
- 0.70 m/s = 2 228 motor r/min (`RPM_PER_MPS 3183`), under `motor_max_rpm
  4000` and the mux `wheel_vel_max_rad_s`; auto ramp 2400 r/min/s unaffected.
- `a_max 0.15` stays (the gentle start asked for on 2026-09-17). 0→0.70 takes
  4.7 s over 1.63 m, so a 4 m boost step spends 1.6 m accelerating, 1.0 m
  approaching and only 1.4 m at 0.70: the gain over 0.50 on a 4 m straight is
  about 0.6 s; on 10 m it is ~4 s. If the boost feels pointless on short
  straights, `a_max 0.20` (0→0.70 in 3.5 s, 1.2 m) is the knob; measure first.
- `d_max 0.5` stays: it sets the functional stopping distance quoted above.

### A4. Editor, docs
- `editor.html`: `#ed-speed max="0.50" value="0.50"`; `editor.js` defaults
  `linear_mps: 0.50` (new draft and map-change reset, two places).
- RUNBOOK §route (line ~158) "Speed cap: 0.05–0.40" → 0.50; add the long-straight
  sentence from Part B. `amr_implementation_spec.md` / `unified_amr_service_plan.md`
  0.40 defaults → 0.50, note 0.70 long straights.
- Tests: `amr_navigation/test/test_route.py` defaults; `test_editor_js.py`
  expects 0.50 on new draft / reset; `test_launch_layers.py` or a small
  `test_nav2_params.py` asserting `desired_linear_vel == VEHICLE_V_MAX`
  (one source of truth for the ceiling).

---

## Part B — 0.70 m/s on straights longer than 4 m

### B1. Route schema — `route.py`
- `Limits.long_linear_mps: float | None = None` and
  `Limits.long_min_length_m: float = 4.0`. `None` = no boost: a route file
  that predates this (all current files) never speeds up by itself; the
  editor writes `long_linear_mps: 0.70` explicitly into new routes.
- `LIMIT_BOUNDS`: `long_linear_mps` in (0..VEHICLE_V_MAX], `long_min_length_m` in
  [1.0, 100]. Loader refuses `long_linear_mps < linear_mps` (a "boost" that
  slows down is a typo).
- `CompiledStep` gains `v_mps`: `long_linear_mps` if the step is a straight
  with `length_m > long_min_length_m` and `long_linear_mps` is set, else
  `linear_mps`; rotations get `None`. `compile_route` fills it, so validation
  output and the executor read the same decision.

### B2. Executor — `amr_mission/amr_mission/route_executor_node.py`
- Permit: `m.v_max = float(st.v_mps)` for a straight (was the route-wide
  `linear_mps`); rotation keeps `w_max = angular_rad_s`. The mux clamp is the
  hard limit per step, as today per run.
- Nav2 speed limit: new publisher `nav2_msgs/SpeedLimit` on `/speed_limit`
  (controller_server's default `speed_limit_topic`; RPP implements
  `setSpeedLimit(absolute)` in Humble). Publish `{percentage: false,
  speed_limit: st.v_mps}` right before every `_send_follow` and again at each
  tick while EXECUTING a straight (the setting persists in the controller,
  so re-sending is cheap insurance against a restart of controller_server).
  RPP then plans lookahead scaling and the approach ramp from the step's
  speed, so a 0.50 step approaches from 0.50 and a 0.70 step from 0.70.
  Without this the controller would command 0.70 everywhere and the mux
  clamp would flatten it to 0.50: correct speed, but the approach ramp would
  be computed for 0.70 and start too late.
- Obstruction horizon scales with the step speed:
  `horizon = max(stopping_horizon_m, st.v_mps * stopping_time_s)` with a new
  param `stopping_time_s 3.0` → 1.5 m at 0.50, 2.1 m at 0.70. The BLOCKED stop
  is a permit revoke through the mux slew, so at 0.70 the envelope must be
  checked ~1 m further ahead than today.
- Resume after BLOCKED/PAUSED mid-step: unchanged, the step's `v_mps` is
  re-sent with the FollowPath, and `_follow_path` still starts at the current
  projection.
- Run page / `RunState`: add `step_v_mps` so the operator sees "s3 straight
  0.70" while it runs.

### B3. Editor — `editor.html`, `editor.js`
- New field under Limits: "Long straight speed m/s (over 4 m)" number input
  `#ed-speed-long`, `step 0.05 min 0.05 max 0.70 value 0.70`, and
  "Long straight from m" `#ed-long-min` `min 1 value 4`. Empty long speed =
  no boost (`long_linear_mps: null` in the payload).
- Step list and canvas label mark boosted straights: `s2 4 850 mm ▲0.70`.
  The decision uses the same rule as `compile_route` (`length > long_min`),
  computed in JS from `stepLength(i)`.
- Loaded routes show their stored values; a route without `long_linear_mps`
  shows the field empty (Q15 rule: controls follow the model, never rewrite it).
- `refresh()` clears/sets both inputs; `payload()` includes them.

### B4. Validation feedback — `validate.py`, `showResult`
- `Validation.compiled` already carries steps; the result panel lists
  `n straights at 0.70`, so the operator sees what the boost applies to
  before saving.

### B5. Tests
- `test_route.py`: `v_mps` per step (4.0 m exactly is not boosted, 4.01 is);
  `long_linear_mps` absent → no boost; `long_linear_mps < linear_mps` refused;
  values above `VEHICLE_V_MAX` refused.
- `test_route_executor_logic.py`: permit `v_max` follows the step; SpeedLimit
  published with the step speed before FollowPath; horizon 2.1 m on a boosted
  step, 1.5 m otherwise.
- `editor_harness.js`: draft payload carries `long_linear_mps 0.70` and
  `long_min_length_m 4`; a loaded route without them shows empty and saves
  `null`; the row for a 4.5 m straight contains `0.70`.
- Sim (opt-in, short): route with a 2 m and a 6 m straight; `/cmd_vel` peak
  ≤0.50 on the first, reaches 0.70 on the second, endpoint tolerance still
  met (the approach ramp change is what this checks).

---

## Vehicle acceptance (before enabling on a real route)

1. Safety first: nanoScan3 field set validated for 0.70 (stopping ≥1.0 m
   ahead) and `lidar.zones_validated` set true in the profile by the person
   who did it. Until then the editor's long-speed max is 0.50 by config
   (`VEHICLE_V_MAX` read from the profile? — see decision 2).
2. Existing route at 0.50 (Part A only): endpoint error, turn entry heading,
   cross-track on the first metre, no approach overshoot. Compare with the
   0.40 baseline from 2026-09-17.
3. One long straight (≥ 6 m) at 0.70, cones only: peak speed reached, endpoint
   within 0.05 m, BLOCKED test with a box at 2.5 m ahead (must stop short).
4. Only then set the boost on production routes, one at a time.

Rollback per part: A2 `desired_linear_vel 0.40`, `approach 0.6`; B: leave
`long_linear_mps` unset in the route file (no code rollback needed).

## Decisions (recommendation in bold)

1. Boost threshold "over 4 m": **`length_m > 4.0` strictly, per route, default
   4.0**, editable in the editor so a site with short aisles can lower it.
2. Where the 0.70 ceiling lives: **`VEHICLE_V_MAX` constant in `route.py`,
   mirrored by a test against `nav2_params.yaml`** — one code change to raise
   later; the profile is for hardware facts, not policy.
3. Per-step speed transport: **`/speed_limit` + permit `v_max` together**;
   a SetParameters call on `desired_linear_vel` per step would also work but
   is slower (service round-trip) and not what the topic exists for.
4. `a_max`: **keep 0.15 for the first runs**, revisit to 0.20 with data.
