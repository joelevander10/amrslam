# Coding plan: autonomous speed 0.40 m/s + Pareto approach smoothing

Two scoped changes to autonomous route execution, one document, one branch.
Part A is the speed change from `coding-agent-speed-0.4-prompt.txt`, kept
verbatim in scope. Part B is the smallest change that removes most of the
visible end-of-segment jitter: two controller parameters and one executor
offset. Nothing else in the navigation behaviour changes.

Working tree: the pending web restyle, pendant and Wi-Fi changes stay as they
are. Do not touch `static/fonts/`, `jogpad*`, `wifi.py`, the pendant files.

---

## Part A — route speed 0.30 → 0.40 m/s, rotation 0.30 → 0.24 rad/s

### A1. Schema default — `amr_ws/src/amr_navigation/amr_navigation/route.py:97-98`
```
linear_mps: float = 0.40
angular_rad_s: float = 0.24
```
Bounds on lines 37–38 (0..5) already admit both. Nothing else in the loader
changes: explicit values in a file win over the default, a file that omits
`limits` gets the new defaults on load only (files are never rewritten).

### A2. Nav2 config — `amr_ws/src/amr_navigation/config/nav2_params.yaml`
- `FollowPath.desired_linear_vel: 0.30 → 0.40` (line 30; keep the comment,
  update the number in it).
- `behavior_server.max_rotational_vel: 0.30 → 0.24` (line 114).
- Leave `rotate_to_heading_angular_vel: 0.3` (dead: `use_rotate_to_heading:
  false`), `failure_tolerance: 0.3`, `transform_tolerance: 0.3` — not speeds.
- Sanity, no edit needed: at 0.40 m/s the velocity-scaled lookahead is
  2.5 s × 0.40 = 1.0 m, inside [0.8, 1.2]; approach decel from 0.40 to 0.05
  over 0.6 m is 0.13 m/s², well under the mux `a_max` 0.5 and the drives'
  800 rpm/s ramp.

### A3. Route editor — `amr_web/templates/editor.html:29`, `amr_web/static/editor.js`
- `editor.html`: `<input id="ed-speed" ... max="0.40" value="0.40">`.
- `editor.js:5` and `:206` (new draft / reset): `limits: { linear_mps: 0.40 }`.
- `editor.js:122-124` (`refresh`): this block was written for a `<select>`
  (`[...sp.options]`) but the template is a number input, so `refresh()`
  throws `TypeError` in the browser today (uncommitted restyle). Replace the
  three lines with `sp.value = route.limits.linear_mps;` — that is exactly the
  "show the loaded route's stored value, never overwrite it" requirement.
  Keep `onchange` (line 68) as is.
- No angular control is added; new routes get 0.24 from the schema.

### A4. Authority chain — verify, no code change expected
`Route.limits` → `compiler` → executor `MotionPermit.v_max/w_max`
([route_executor_node.py:860-861](amr_ws/src/amr_mission/amr_mission/route_executor_node.py#L860))
→ mux `gating.select` caps FOLLOW/ROTATE with `capped(v, permit.v_max)`
([gating.py:285-290](amr_ws/src/amr_base/amr_base/gating.py#L285)) → `clamp_wheels`.
The mux enforces the *route's* caps, so 0.40/0.24 arrive through the schema
default and a route at 0.15 m/s stays at 0.15. Confirm with the tests in A6;
if `capped()` turns out to drop sign or ignore `w_max`, fix it there and
nowhere else.

### A5. Documentation
Update only text describing the **autonomous route default**:
- `manuals/slam-generalized-plan/amr_implementation_spec.md:217, 225, 293`
- `manuals/slam-generalized-plan/unified_amr_service_plan.md:283`
- `nav2_params.yaml` comments touched in A2.
- `amr_ws/RUNBOOK.md:158` speed-cap range: `0.05–0.40 m/s`.

Leave unchanged (classify in the handoff): survey ≤ 0.30 m/s
(`amr_implementation_spec.md:246`, RUNBOOK jog paragraph), browser jog
`jog.py V_MAX/W_MAX = 0.30`, pendant `pendant_v/w` 0.30, RUNBOOK:83 pendant,
`test_jog.py` 0.3 refreshes, covariance `0.3**2` fixtures, `test_run_fsm.py`
"0.3 m from route start", `test_server.py:100` explicit `linear_mps: 0.3`
(this one is *the* "stored value survives" fixture — keep 0.3 on purpose).

### A6. Tests
- `amr_navigation/test/test_route.py`
  - new: route without `limits` → 0.40 / 0.24.
  - new: explicit `0.15 / 0.10` preserved through parse → compile.
  - line 45: `time_allowance_s > abs(expected) / 0.24` (compiler uses the
    route's `w`, so recompute the expectation from the new default, not 0.30).
  - line 379 `-0.3` rejection case: unchanged (a negative limit is still invalid).
- `amr_base/test/test_gating.py` (or `test_authority.py`): FOLLOW twist
  (0.9, 0.9) with permit `v_max=0.40, w_max=0.24` → (0.40, 0.24); negative
  input → negative output; permit `v_max=0.15` wins over 0.40; `w_max=0.10`
  wins over 0.24.
- `amr_mission/test`: executor publishes `MotionPermit.v_max/w_max` equal to
  the route limits (extend the existing permit test if present, else a small
  unit on the publisher helper).
- `amr_web/test/test_server.py`: saving a route without limits stores 0.40/0.24;
  loading the existing `linear_mps: 0.3` fixture returns 0.3.
- `amr_web/test`: new `editor_harness.js` + `test_editor_js.py`, same shape as
  `jogpad_harness.js` / `test_jogpad_js.py` (node, skip if absent): new draft
  = 0.40, reset = 0.40, loading `{limits:{linear_mps:0.3}}` shows 0.3 and
  does not mutate the model. Assert `jog.py` presets untouched.

---

## Part B — approach smoothing, Pareto scope

### What we are and are not fixing
The jitter is pure pursuit at a shrinking lookahead (curvature = 2·offset/L²)
answering AMCL pose steps (updates every 5 cm / 0.03 rad) at crawl speed while
hunting into a 2.5 cm goal circle. The complete fix (odom-frame final
approach, or executor-driven last 0.6 m) is deferred. This part only:

1. stops sub-visible steering commands reaching the wheels, and
2. stops the vehicle hunting for a 2.5 cm circle it cannot see.

No lookahead, frequency, collision, acceleration-limit or speed-floor change.
Both changes are parameters with a stated rollback value.

### B1. Angular deadband — `nav2_params.yaml`
- `controller_server.min_theta_velocity_threshold: 0.001 → 0.03` rad/s.
  Corrections below 0.03 rad/s (≈0.3 cm/s wheel differential) are zeroed by
  Nav2 before they reach the mux. Rollback: 0.001.
- `FollowPath.max_angular_accel: 1.0 → 0.5`. Halves the rate at which a
  correction can build; the mux `alpha_max` (1.0) stays as the hard limit.
  Rollback: 1.0.
- Deliberate exception to A2's "no controller retuning": these two are the
  Part B change and are called out as such in the handoff.

### B2. Goal placement — executor + goal checker
Today: `xy_goal_tolerance: 0.025` because 0.05 stopped ~4.5 cm short
([nav2_params.yaml:23](amr_ws/src/amr_navigation/config/nav2_params.yaml#L23)).
Instead of a tighter circle, aim past the point:
- `nav2_params.yaml precise.xy_goal_tolerance: 0.025 → 0.05` (spec §5.2 ±0.05).
- `route_executor_node.py`: parameter `goal_overshoot_m` (default **0.045**,
  clamped to `[0, route.limits.position_tolerance_m]`). In `_send_follow`
  ([:590-608](amr_ws/src/amr_mission/amr_mission/route_executor_node.py#L590)),
  after the sampled poses, append one extra pose at
  `end + goal_overshoot_m` along the step heading. The goal checker fires
  ~0.5 cm before the true endpoint and the vehicle stops on it.
- The executor's own arrival/fault logic is **unchanged**: the SETTLE phase
  and `along > length + 2·tolerance` fault still measure against the true
  endpoint, so an overshoot beyond tolerance is still a fault, and the clamp
  guarantees the extra pose lies inside the validated swept envelope
  (validated with the 0.20 m footprint margin).
- Rollback: `goal_overshoot_m:=0` and tolerance back to 0.025.

### B3. Tests
- `amr_mission/test`: `_send_follow` path's last pose is `overshoot` metres
  past the step end along its heading; with `goal_overshoot_m:=0` the last
  pose is the endpoint; a value above `position_tolerance_m` is clamped.
- Existing route-sim tests must still pass with the 0.05 tolerance (they use
  the fake base; note any tolerance they assert on).
- No unit test can prove smoothness; B is validated on the vehicle (below).

---

## Validation

```bash
cd /home/gvipc-evo-01/agv_can && python3 tests/run_all.py
cd /home/gvipc-evo-01/agv_can/amr_ws
colcon build --packages-select amr_navigation amr_mission amr_web --symlink-install
env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test
ruff check src
```
No hardware services, no can0, no drive commands, no nanoScan3 changes.

Vehicle session (after `sudo systemctl restart amr.service`), one route,
three straights with turns, in this order so each effect is seen alone:
1. Part A only (B params at rollback values): confirm 0.40 m/s on straights,
   0.24 rad/s spins, stop error within ±0.05 m, cross-track fault never trips.
2. Enable B1: watch `/cmd_wheel_vel` in the last 0.6 m — expect fewer, larger
   sign changes rather than a stream of ±0.01 rad/s.
3. Enable B2: measure stop error over ≥5 stops; tune `goal_overshoot_m` so the
   mean is ~0, then freeze it in the launch file.
Success criterion for B is "noticeably smoother and stop error unchanged or
better", not "no correction visible".

## Handoff report
- Files changed, exact autonomous values (0.40 / 0.24), and confirmation that
  manual, pendant, survey, browser jog and commissioning speeds are untouched.
- Every remaining `0.30 / 0.3` classified: intentionally unchanged / test
  fixture / historical doc / missed configuration.
- Part B parameter values and their rollback values, listed separately.
- Tests added or updated; validation commands and results; pre-existing
  failures separated from new ones (the `editor.js` `options` TypeError is
  pre-existing and fixed under A3).
