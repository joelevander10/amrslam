# Coding plan: speed boost 2 (auto 0.55 / arcs 0.40 / long 0.85, faster spins, faster manual)

> **Status 2026-09-19: implemented** (see "Implemented" at the end). It follows `coding-plan-speed-boost.md`
> (2026-09-18: 0.50 / 0.70 / arcs at 60 %, taper, pendant 0.50).
> Decisions taken 2026-09-19: existing routes re-saved by hand after the first
> vehicle run; the web jog is 0.40 everywhere, survey included; arc turn-rate
> ceiling 0.45 rad/s. A survey by pendant runs at the pendant speed (MAPPING grants
> the manual lease, with no survey-specific cap): see "Survey at pendant speed".

## Targets

| Motion | Now | Target | Where it lives |
|---|---|---|---|
| Auto, normal | 0.50 m/s | **0.55 m/s** | route `limits.linear_mps` default |
| Auto, arc | 0.6 × normal = 0.30 (max 0.9·ω·R) | **0.40 m/s** | new `limits.arc_linear_mps` |
| Auto, long straight (> 4 m) | 0.70 | **0.85 m/s**, ramping down before an arc | `limits.long_linear_mps`, `VEHICLE_V_MAX` |
| Auto, spin | 0.34 rad/s | **0.37 rad/s** (+10 %) | `limits.angular_rad_s`, `VEHICLE_W_MAX`, Nav2 Spin |
| Pendant, drive | 0.50 | **0.60 m/s**, slow wheel 75 % while turning (unchanged) | mux `pendant_v_m_s` |
| Pendant, spin | 0.30 rad/s | **0.39 rad/s** (+30 %) | mux `pendant_w_rad_s` |
| Web/screen jog, drive | 0.30 max | **0.40 m/s**, slow wheel 75 % on the diagonal buttons | `jog.py`, `jogpad.js` |
| Web/screen jog, spin | 0.30 max | **0.39 rad/s** (+30 %) | `jog.py`, `jogpad.js` |

The ramp-down before an arc **already exists**: the chain taper from 2026-09-18
(`_step_speed`, `taper_decel` 0.4 m/s², `taper_lead_s` 0.3 s). With the new
numbers it starts:
- **0.96 m** before the arc when coming off an 0.85 straight (0.85 → 0.40);
- 0.34 m before it at the normal speed (0.55 → 0.40);
- 0.78 m before the end of a boosted chain that ends in a stop (0.85 → 0.55,
  after which RPP's approach ramp takes over).

Nothing new is needed there, only tests with the new numbers.

## Part A — Auto

### A1. Route limits (`amr_navigation/route.py`)
- `VEHICLE_V_MAX` 0.70 → **0.85**; `VEHICLE_W_MAX` 0.34 → **0.37**.
- New `BASE_V_MAX = 0.60`: the bound on `linear_mps`. The boost is the only
  thing allowed near 0.85. This keeps RPP's approach ramp gentle (see A3).
- `Limits` defaults: `linear_mps` **0.55**, `long_linear_mps` **0.85** (was
  null = off; the editor already offered 0.70), `angular_rad_s` **0.37**, and
  new `arc_linear_mps` **0.40**.
- `LIMIT_BOUNDS`:
  - `linear_mps` (0, `BASE_V_MAX`);
  - `long_linear_mps` (0, `VEHICLE_V_MAX`);
  - `arc_linear_mps` (0, `BASE_V_MAX`).
  - The loader refuses `arc_linear_mps > linear_mps`, as it already refuses
    `long_linear_mps < linear_mps`.
- **Arc turn rate:** at 0.40 m/s on the minimum 1.0 m radius, ω = 0.40 rad/s.
  That is above the spin cap (0.37), and the mux clamps ω to the permit's
  `w_max`: it would cut ω, keep v, and the vehicle would run wide. So arcs get
  their own turn-rate ceiling:
  - `VEHICLE_ARC_W_MAX = 0.45` rad/s;
  - arc speed = `min(arc_linear_mps, 0.9 · 0.45 · R)` → 0.40 m/s for any
    R ≥ 1.0 m;
  - the arc step's permit `w_max` = 0.45;
  - wheel check at R = 1.0 m: fast wheel 0.40 + 0.40·0.2435 = 0.50 m/s, slow
    0.30 m/s. Far inside the 1.26 m/s motor limit.
  - `ARC_SPEED_RATIO` is removed; `ARC_YAW_RATE_RATIO` 0.9 stays.
- Old route files without `arc_linear_mps` load with 0.40 (clamped to their own
  `linear_mps` if that is lower).

### A2. Compiler and executor
- `compiler.arc_speed` uses `arc_linear_mps` and `VEHICLE_ARC_W_MAX`.
- In the executor permit, arc steps get `w_max = VEHICLE_ARC_W_MAX`; straights
  and rotates keep `w_mps`.
- `stopping_time_s` 3.0 → **2.0**. The obstacle envelope ahead at 0.85 would
  otherwise be 2.55 m (3 s of travel), inviting BLOCKED on anything near the
  path. At 2.0 s it is 1.7 m. A 0.85 → 0 stop at the mux's 0.5 m/s² takes
  0.72 m plus the latency, and `stopping_horizon_m` 1.5 stays the floor.
- Taper: unchanged code, re-pinned tests.

### A3. Nav2 (`nav2_params.yaml`)
- RPP `desired_linear_vel` 0.70 → **0.85** (= `VEHICLE_V_MAX`; each step's own
  speed still comes from `/speed_limit`).
- `approach_velocity_scaling_dist` stays **1.0**:
  - a chain always reaches its end at the base speed, because the taper takes a
    boosted chain down first;
  - the decel demand is `BASE_V_MAX²/1.0` = 0.36 m/s² ≤ 0.5.
  - The `test_nav2_params.py` check becomes `BASE_V_MAX**2 / approach ≤ 0.5`,
    plus a new check that the taper ends a boosted chain at the base speed.
- `max_allowed_time_to_collision_up_to_carrot` 1.5 → **1.2 s**. At 0.85 this is
  1.0 m ahead, about the same as today's 1.05 m at 0.70. Keeping 1.5 s would
  look 1.3 m ahead and trigger more "collision ahead" aborts.
- `regulated_linear_scaling_min_radius` 0.9 stays. Arcs are ≥ 1.0 m, so RPP does
  not slow them further.
- Spin `max_rotational_vel` 0.34 → **0.37**. `rotational_acc_lim` stays 0.17
  (2.2 s to full rate).

### A4. Route editor (`editor.html`, `editor.js`)
- Speed cap input max 0.60, default 0.55. Long-straight default 0.85, max 0.85.
- New field **Arc speed** (`#ed-speed-arc`, default 0.40, max = the cap).
- The step rows show the arc speed from the new rule.
- `arcSpeed()` mirrors `compiler.arc_speed`.

### A5. Existing routes
Route files store their own limits. **Existing routes keep 0.50/0.70/0.34 until
re-saved**, and the editor shows the stored values without rewriting them
(pinned by `test_editor_js`). Decision 1 below.

### A6. Prerequisite on the vehicle (not code)
`lidar.zones_validated` is still `false`.
- 0.85 m/s needs about 0.72 m of braking at 0.5 m/s², plus the reaction time.
- Check that the nanoScan3 protective field for the monitoring case used on
  straights covers that. If it doesn't, the vehicle stops too late, not early.
- Run the first 0.85 straights with someone at the E-stop.

## Part B — Manual

### B1. Pendant (`base.launch.py`, `gating.Params`, mux defaults)
- `pendant_v_m_s` 0.50 → **0.60**;
- `pendant_w_rad_s` 0.30 → **0.39**;
- `pendant_turn_ratio` 0.75 unchanged.

The S-curve (0.3 m/s², jerk 1.0) is unchanged. Reaching 0.60 takes about 2.3 s
and 0.8 m.

### B2. Web/screen jog (`jog.py`, `jogpad.js`)
- `jog.V_MAX` 0.30 → **0.40**, `jog.W_MAX` 0.30 → **0.39**. The server clamps to
  these, so they are the real cap.
- The speed selector offers 0.10 / 0.20 / 0.30 / **0.40**. The default stays 0.20.
- **Diagonal buttons (drive + turn): 75 % slow wheel**, the same rule as the
  pendant:
  - `w = 2·v·(1 − 0.75) / (track·(1 + 0.75))` = 0.587·v rad/s;
  - 0.235 rad/s at 0.40 m/s (fast wheel 0.457, slow 0.343).
  - Today the diagonals use `min(0.30, 1.5·v)`, a slow/fast ratio of 0.61 at 0.30.
- **Spin buttons:** `w = min(0.39, 1.95·v)`, which is +30 % on today's
  `min(0.30, 1.5·v)` at every speed setting (0.39 at 0.20 m/s and above).
- The web jog goes through the mux MANUAL path, so it gets the same S-curve
  profile as the pendant.
- The jog docstring's "survey caps (0.30 m/s, 0.30 rad/s)" becomes the new caps
  (decision 2).

## Tests (updated or new)
- `test_route.py`:
  - new defaults;
  - bounds: `linear_mps` 0.61 refused, `long_linear_mps` 0.86 refused,
    `arc_linear_mps` above `linear_mps` refused;
  - arc speed 0.40 at R 1.0 and at R 3.0;
  - arc `w_max` 0.45;
  - an old file without `arc_linear_mps` loads with 0.40.
- `test_nav2_params.py`:
  - `desired_linear_vel` = 0.85;
  - Spin = 0.37;
  - `BASE_V_MAX²/approach` ≤ 0.5;
  - collision time 1.2.
- `test_route_executor_logic.py`:
  - taper 0.85 → 0.40 = 0.96 m, and 0.55 → 0.40 = 0.34 m;
  - the arc permit carries `w_max` 0.45;
  - horizon at 0.85 = 1.7 m.
- `test_pendant_gating.py`, `test_mux_manual_profile.py`: 0.60 / 0.39 / 0.75.
- `test_editor_js.py` / `editor_harness.js`: the new defaults, the arc speed
  field, and 0.40 in the arc rows.
- `test_editor_js.py::test_browser_jog_presets_are_untouched` → (0.40, 0.39).
- A new `jogpad` harness case: the diagonal at 0.40 gives w 0.235 (75 % slow
  wheel), and spin at 0.40 gives 0.39.
- `test_server.py`: the default limits in `/api/routes` are 0.55/0.85/0.40/0.37.

## Rollout
1. Code and tests. Build `amr_interfaces` only if `RunState` changes (it doesn't
   here). Restart amr.service, with the operator's yes.
2. Vehicle, operator at the E-stop:
   - pendant 0.60 and spin, web jog 0.40 and diagonal;
   - a new route with a 5 m straight → arc R 1.0 → straight, watching the
     ramp-down before the arc and the cross-track on the arc (Run page).
3. Re-save the existing routes (decision 1).

## Survey at pendant speed (0.60 m/s, spin 0.39 rad/s)
- **Straight driving:**
  - the nanoScan3 scans at 34 Hz, so one scan smears 0.60 × 0.029 = 17 mm,
    about a third of a 5 cm map cell;
  - slam_toolbox still adds a node every 0.2 m of travel (`minimum_travel_distance`),
    now about 3 per second instead of 1.5;
  - the scan matcher searches ±0.25 m around the odometry guess, far more than
    the odometry drifts over 0.2 m.
- **Spinning** is the larger error: slam_toolbox does not correct for motion
  within a scan. One scan at 0.39 rad/s smears 0.65°, which is 11 cm at a wall
  10 m away (8.7 cm at today's 0.30). A timestamp offset between the scan and
  the odometry adds to it the same way. Spin slowly near long walls.
- **CPU:** asynchronous mode drops scans when the N97 falls behind. Twice the
  node rate must be checked in the first fast survey (`slam_toolbox` log,
  `/map` update gaps).
- **Acceptance:** the survey's own return review (`review` dx/dy/dyaw in the
  manifest). `tool-center-00` rev1, surveyed at ≤ 0.30, closed at 12/14 mm and
  0.4°. A fast survey whose closure error stays within about 2× of that, with no
  doubled walls in Maps → Review, is fine.
- If it isn't: add a survey cap. The mux would take `pendant_v_m_s` from the
  mode (0.30 in MAPPING), a small change to `gating.pendant_twist` plus the lease.

## Decisions (recommendation in bold)
1. **Existing routes:** re-save each one in the editor with the new speeds.
   **Recommendation: yes, by hand**, after the first vehicle run. A bulk script
   would create a new revision of every route without anyone looking at them.
2. **Web jog cap during a survey (MAPPING mode):**
   - **0.40 everywhere (recommended)**; slam_toolbox copes with 0.40 m/s. **Taken.**
   - The alternative keeps 0.30 while mapping, which needs the jog cap to
     depend on the mode.
3. **Arc turn-rate ceiling 0.45 rad/s,** separate from the spin cap. Without it,
   0.40 m/s arcs are only possible from R 1.20 m up (0.40 / (0.9 × 0.37)).

## Implemented (2026-09-19)
All of Part A and Part B, with these tests:
- `test_route.py`, `test_nav2_params.py`, `test_route_executor_logic.py`
  (`test_speed_boost_2_…`), `test_pendant_gating.py`;
- `test_editor_js.py` with `editor_harness.js`;
- `test_jogpad_js.py` with `jogpad_harness.js`;
- `test_server.py`.

**Differences from the plan:**
- **`arc_linear_mps` above `linear_mps` is not refused.** Every saved file
  carries the arc speed (0.40), so refusing it made a slow 0.30 route unloadable
  after its first save. The compiler and the editor take the lower of the two.
- **`Limits.long_linear_mps` stays `None` (off) when a file omits it.** Older
  files must not start boosting by themselves. New routes get 0.85 from the
  editor.
- **Web diagonal buttons follow the pendant rule** (fast wheel = the selected
  speed, slow = 75 %: body 0.35 m/s, 0.205 rad/s at 0.40). The plan's formula
  had the body at the selected speed, which puts the fast wheel at 0.457 m/s,
  above the selection.
- **The arc turn ceiling covers the whole chain.** The permit's `w_max` is 0.45
  for every step of a chain that contains an arc, including the straight
  leading into it, because RPP starts curving a lookahead before the arc
  begins. A straight outside such a chain, and every spin, keep the spin cap.

**Reverted 2026-09-19:** pendant drive speed back to 0.50 m/s (operator), after a pendant survey
(`IGP1_01`) at 0.60 m/s and 0.39 rad/s came out with a rotated duplicate of part of the map.
Pendant spin 0.39 rad/s and everything else stay as implemented.
Survey spin cap (operator, 2026-09-19): while the supervisor mode is MAPPING, the mux caps every
manual turn rate (pendant and browser jog) at 0.27 rad/s, 30 % below the 0.39 spin (mux
`survey_w_max_rad_s`, `gating.survey_spin_cap`; test_mux_manual_profile). Pendant arcs at
0.50 m/s (0.26 rad/s) and the jog diagonals are below the cap and unchanged.
