# Rejoin plan: getting back onto the route from up to 0.30 m off

Status: PLAN, not implemented (2026-11-03). Branch `kyb-trackless`.

## 1. Why

A held run (protective field, E-stop, blocked path) now resumes from up to
`resume_corridor_m` = 0.30 m off the drawn line (2026-11-03: a trolley pushed the vehicle
0.13 m off the s21 arc on route-itg-02 and the old 0.10 m corridor stranded the run).

Today a resume just sends the remaining drawn line to the controller (RPP). Pure pursuit
aims at a point 0.8 m ahead on the line, so it curves back by itself: fine at 0.1 m, but
at 0.3 m that is a curvature of 2·0.3/0.8² ≈ 0.94 rad/m (a 1.07 m radius) swerve that
**nothing checks against the map first**. The executor only widens its cross-track
allowance for the first metre (`_cross_track_allowed`, `_resumed_at`).

Goal: from up to 0.30 m off (and a modest heading error), plan a smooth, bounded-curvature
path back onto the route, **check its swept footprint against the map before driving**,
drive it slowly, and judge cross-track against that path until it has merged.

## 2. Shapes

### 2.1 Straight: S-curve (both sides)

In the line's frame, with lateral offset `d` and heading error `e` at the vehicle, a cubic
Hermite lateral profile over length `L`:

    y(0) = d, y'(0) = tan(e), y(L) = 0, y'(L) = 0        (x along the line)

It leaves along the vehicle's own heading, so no spin is needed, and it arrives tangent to
the line. Peak curvature with `e = 0` is `6d/L²`; `L` is the shortest length that keeps
the peak curvature at or below `1/R_min`, with `R_min` = 1.5 m (proposed, **owner decides**):

| offset d | L (e = 0) | L (e = 10° away from the line) | L (e = 10° towards it) |
|---|---|---|---|
| 0.10 m | 0.95 m | | |
| 0.20 m | 1.34 m | | |
| 0.30 m | 1.64 m | 2.26 m | 1.40 m |

### 2.2 Corner (arc): the tangent line, and why it is not the default

The tangent idea: from a point **outside** the arc, a straight line touching the circle
joins it further on. On route-itg-02's arcs (R = 2.0 m):

| offset d (outside) | tangent point ahead | line length | turn in place first |
|---|---|---|---|
| 0.10 m | 17.8° | 0.64 m | **17.8°** |
| 0.20 m | 24.6° | 0.92 m | **24.6°** |
| 0.30 m | 29.6° | 1.14 m | **29.6°** |

Two problems:

1. **It needs a spin first.** The vehicle is still facing roughly along the arc; the
   tangent line points up to 30° inward. That turn happens in place, and a spin has
   **no obstacle detection** (the scanner looks forward and nothing re-checks a rotation).
2. **It does not exist inside the arc.** From inside the circle no line touches it; a
   different shape is needed anyway.

**Recommended instead: the same S-blend, measured along the arc.** Use the arc as the
reference curve: the S-curve's lateral profile `y(s)` is laid along the arc's normal, so
the planned path is `ref(s) + n(s)·y(s)`. It works inside and outside, needs no spin, and
is literally the same function as 2.1. Curvature is then the arc's (1/2.0 m) plus the
blend's; `L` is lengthened until the combined peak is within `1/R_min` on the outside
and the vehicle's arc ceiling on the inside (`VEHICLE_ARC_W_MAX` 0.45 rad/s at 0.30 m/s
is 1.5 rad/m, not binding).

The tangent version stays as an **option** (§6, decision D2) for the outside case if you
still want it, with a swept-rotation check before the spin.

### 2.3 One algorithm on the remaining path

The executor already builds the remaining path as dense poses: the current step from the
vehicle's projection, every chained step after it, and the overshoot pose
(`_follow_path`). The rejoin works on that pose list, so straights, arcs and a join
point that falls past a step boundary into the next chained step are all the same case:

1. Arc-length parameterise the remaining reference poses; `s = 0` at the projection.
2. `d`, `e` = the vehicle's cross-track and heading error against the reference there.
3. `L` from the curvature bound (§2.1), searched in 5 cm increments.
4. Sample `ref(s) + n(s)·y(s)` every 2 cm for `s ∈ [0, L]`, then the reference from
   `s = L` onwards unchanged.
5. Refuse (stay held, with the reason) when:
   - `|d|` > `resume_corridor_m` (0.30);
   - `|e|` > `rejoin_max_heading_deg` (proposed 20°);
   - `L` + 0.3 m is longer than what is left of the chain (a ROTATE, WAIT or reverse
     ends a chain: the vehicle must be on the line by then);
   - the clearance check fails (§3).

Below `rejoin_min_m` (proposed 0.05 m) and 5° nothing is planned: RPP's own pull-in is
gentle enough there, and every normal resume stays exactly as today.

## 3. Clearance check before driving

`amr_navigation.footprint.swept_poses(grid, fp, poses)` then `footprint.check(...)`: the
same footprint, margin, unknown-is-blocked rule, keepout mask and dynamic-area
(provisional) rule as route validation. The executor already holds `self.grid` and the
footprint; it must also keep the **keepout** and **dynamic** masks it loads at mission load
(today they are used once, for validation, and dropped).

- Clear: drive it.
- Not clear: stay BLOCKED, reason `rejoin path not clear: N occupied cells (...)`. The
  operator moves the obstacle or the vehicle, or aborts.
- Provisional cells (trolley parking areas): allowed, with the same warning validation
  gives.

Limitation, stated in the plan and the RUNBOOK: the footprint is the vehicle's own. A
towed trolley is not in it. At `R_min` 1.5 m the trolley's off-tracking is small, but the
margin (0.05 m) is all it has.

## 4. Driving it

- **Path:** the FollowPath goal is the rejoin poses, then the remaining reference, chain
  and overshoot (a new `_rejoin_path`, used by `_send_follow` when a rejoin is active).
- **Speed:** `rejoin_v_mps`, proposed 0.25 m/s, until the merge point, then the step
  speed (published on `/speed_limit` and in the permit `v_max`, like the taper).
- **Turn rate:** 0.25 m/s on a 1.5 m radius is 0.17 rad/s, inside every permit `w_max`.
- **Cross-track while rejoining:** measured against the **planned rejoin path**, with the
  route's own `cross_track_limit_m` (0.10). After the merge point, the drawn line as
  usual. This replaces the blunt "allow 0.30 m for one metre" grace (`_resumed_at`),
  which is kept only as the fallback when no rejoin was planned.
- **Progress:** still measured along the drawn steps (`_along_cross`), so chain
  bookkeeping, tapers, settle and endpoint checks are untouched.
- **A new hold during the rejoin** (field, blocked path): the next resume plans a new
  rejoin from wherever it stopped.
- **Operator:** run reason `rejoining the line: 0.24 m over 1.6 m`; one INFO event
  `REJOIN` with offset, heading, length and the clearance result (for tuning).

## 5. Where the code goes

| Piece | File | Notes |
|---|---|---|
| Geometry: arc-length param, normals, Hermite blend, curvature search, refusals | **new** `amr_navigation/rejoin.py` | Pure, no ROS: unit-testable like `compiler.py` |
| Clearance | `amr_navigation/footprint.py` (reuse) | `swept_poses` + `check` |
| Keep keepout / dynamic masks | `route_executor_node._load_mission` | store beside `self.grid` |
| Plan at resume | `route_executor_node`: `_mark_resume` → `_plan_rejoin` | auto-resume and Start-resume |
| Path to RPP | `_follow_path` / new `_rejoin_path` | |
| Cross-track against the rejoin | `_cross_track_allowed` and the PHASE_GOAL check | |
| Speed during rejoin | `_step_speed` | |
| Parameters | executor: `rejoin_min_m` 0.05, `rejoin_max_heading_deg` 20, `rejoin_r_min_m` 1.5, `rejoin_v_mps` 0.25 | |
| Alarm/event codes | `agv_core/alarms.py` + RUNBOOK regen | `REJOIN` (info), `REJOIN_BLOCKED` (warn) |
| Docs | RUNBOOK §7 resume, this plan | |

## 6. Decisions for the owner

- **D1** `R_min` for the S-curve: 1.5 m proposed (gentle; 0.30 m needs 1.64 m of travel).
  1.0 m shortens it to 1.34 m but swings the trolley more.
- **D2** Arcs: S-blend along the arc (recommended, no spin) or the tangent line on the
  outside (needs a blind spin of up to 30°; still S-blend on the inside).
- **D3** Also rejoin at **Start** (the start check allows 0.15 m from HOME)? Proposed: yes,
  same code.
- **D4** Heading limit: 20° proposed. Beyond it, refuse (stay held) rather than spin.

## 7. Tests

**Unit (`amr_navigation/test/test_rejoin.py`, no ROS)**
- straight, d = ±0.1/0.2/0.3: starts at the pose with its heading, ends on the line and
  tangent, peak curvature ≤ 1/R_min, `L` matches §2.1;
- heading error toward / away from the line;
- arc inside and outside: ends on the circle, tangent;
- join point past a chained step boundary;
- refusals: too far, heading too large, chain too short (ROTATE next).

**Clearance:** a synthetic grid with an obstacle beside the S-curve but clear of the drawn
line, so a rejoin is refused that RPP's own pull-in would have driven into.

**Executor (`test_auto_resume.py` style):** a field hold, a push to 0.25 m, then
auto-resume: the goal path starts with the rejoin poses; cross-track is judged against
them; leaving the rejoin by more than 0.10 m faults; after the merge the line rule
applies; below 0.05 m no rejoin is planned (the path is exactly today's).

**Sim:** one case in `test_unified_sim.py`: hold, teleport 0.25 m sideways, resume,
merge within L + 0.5 m.

## 8. Vehicle acceptance

On kyb-map-02 (or a clear test lane), with and without the trolley:

1. Straight: stop with E-stop, push 0.10 / 0.20 / 0.30 m left and right, Reset, Start.
   Record the peak cross-track against the planned path and the merge distance.
2. Arc (R 2.0 m): the same, outside and inside.
3. An object placed beside the planned S-curve: the resume must be refused with the reason.
4. Pass: merges within L + 0.5 m, never more than 0.10 m off the planned path, no
   controller aborts, the trolley clears every fixture.

## 9. Effort

About 2–3 days of code and tests (rejoin.py and its tests are most of it), then half a
day on the vehicle. One service restart to deploy; no message changes.
