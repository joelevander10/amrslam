# Coding plan: dynamic objects on the factory floor (dynamic-mapping)

> **Status 2026-09-19:** Increment 1 is implemented but not yet vehicle-verified.
> Increments 2–4 are open. Before implementing, the plan was checked against the
> code, and these points were changed:
> - **AMCL and `/map`:** one `map_server` feeds AMCL, the web live view and the
>   localisation monitor, so §1.2's "point map_server at the derived map" and
>   "/map stays the reviewed map" contradict each other. Now a second
>   `map_server_loc` serves the blanked map on `/map_loc`, and AMCL is remapped
>   to it. This is opt-in (`blank_dynamic` launch arg, `AMR_LOC_BLANK_DYNAMIC`,
>   default false) until the Increment 4 comparison.
> - **Localisation monitor:** it has no `_load_grid`; it takes the grid from
>   `/map`. The launch passes `dynamic_yaml`, and the monitor checks alignment.
> - **Package layering:** `amr_maps` cannot import `amr_navigation.footprint`.
>   `rasterize_polygon` moved to `amr_maps.grid` (footprint imports it from
>   there), and `keepout_misalignment` became `grid.mask_misalignment`. The CLI
>   lives in `amr_mission` (`ros2 run amr_mission map_edit`) because it needs
>   `map_bundle`.
> - **Executor:** the obstruction rule changed after this plan was written
>   (`obstacle_map_tol_m`, 2026-09-18). Dynamic areas are removed from the
>   "explained by the map" mask (`explained_by_map`); the load call is now
>   around line 371, not 342.
> - **Edit format:** `derive_edit(maps_dir, map_id, rev, ops, note)`. Dynamic
>   polygons are ops (`dynamic` / `undynamic`) in the same ordered list as the
>   paint ops. The child inherits the parent's mask, and `edits.json` is never
>   inherited.
> - **Review UI:** it uses dragged rectangles (the operator's request) instead
>   of free polygons. The format still takes polygons.
> - **`Issue.severity`** is new (`error` / `info`); `Validation.ok` counts
>   errors only.

Trolleys, forklifts and other AMRs occupy floor space for hours or days and then
move. The survey cannot be done on an empty floor, so the saved map contains
them, and on later runs some are gone, some are new. Today that breaks the stack
in three places: route validation rejects a route through a mapped trolley that
has since left; the executor ignores scan points that the map "explains", so a
mapped trolley that is still there is never re-checked live; and AMCL and the
scan-consistency monitor treat the trolley cells as reliable structure.

This plan follows the design discussed with GPT-6 on 2026-09-18 (Codex session,
16:08–16:16): keep three products — survey evidence, a localisation reference,
and a navigation occupancy — and let the map improve across visits instead of
requiring a clean survey. It is cut into four increments that each ship on
their own. Increments 1 and 2 are enough for daily operation with trolleys;
3 and 4 make the map improve by itself.

Invariants that do not change:

- Revisions stay immutable; every consumed file is listed and hashed
  (`amr_mission/map_bundle.py`, review Q14). Anything new is a new revision via
  `derive_revision`. Missions keep referencing the revision they were made on.
- Unknown cells are blocked for route approval (spec §5.4, `footprint.check`).
  A dynamic area does not turn unknown into free; it turns "mapped clutter"
  into "check live".
- Live obstruction detection sees everything the scanner returns. Nothing is
  filtered out of `/scan_gated`; the Nav2 local costmap (`obstacle_layer` +
  `inflation_layer` only, no static layer) is untouched.
- Resume is still the operator's physical Start after a BLOCKED; nothing here
  adds automatic resumption or detours.

---

## Increment 1 — dynamic areas + map cleanup as a bundle revision

### 1.1 Bundle format — `amr_mission/amr_mission/map_bundle.py`

Two new optional files, both listed in the manifest and covered by the bundle
hash, same rule as `keepout.*`:

| file | content | consumers |
|---|---|---|
| `dynamic.yaml` + `dynamic.pgm` | mask in the map's grid geometry: ≥65 = dynamic area cell | validation, executor, nav layer (AMCL map), localisation monitor, evidence recorder |
| `edits.json` | ordered list of operator edits that produced this revision's `map.pgm` from its parent: `{"parent": {"revision": N, "sha256": ...}, "ops": [{"op": "paint", "value": "free"\|"unknown"\|"occupied", "polygon": [[x,y],...]}, {"op": "dynamic", "polygon": [...]}, ...]}` | provenance; replay when a map is regenerated (Increment 3) |

- `CONSUMED_OPTIONAL += ("dynamic.yaml", "dynamic.pgm", "edits.json")`; `verify()`
  applies the same "image of the yaml must be listed" check it does for keepout.
- New `derive_edit(maps_dir, map_id, from_revision, ops, dynamic_polygons) -> (path, rev)`
  in a new pure module `amr_maps/amr_maps/edit.py` + the bundle glue in `map_bundle.py`:
  load the parent grid, apply `ops` in order with `footprint.rasterize_polygon`
  (paint sets `data` to 0 / -1 / 100 inside the polygon), rasterise the dynamic
  polygons into a mask, write `map.pgm`/`map.yaml` (`gridio.write`), `dynamic.*`,
  `edits.json` into a stage and call the existing `derive_revision` path
  (copy parent files, overlay extras, verify, publish). The pose graph is copied
  unchanged: it is provenance and the seed for a continued survey, not the
  reference (see 3.4 for the regeneration caveat).
- `keepout_misalignment(grid, dynamic)` is reused for the dynamic mask (rename
  to `mask_misalignment` and keep the old name as an alias).

### 1.2 Semantics per consumer

**Route validation** — `amr_navigation/validate.py`, `footprint.py`
- `Clearance` gains `provisional: int`: cells in the sweep that are occupied
  or unknown in the map AND inside the dynamic mask. They do not count against
  `clear`, but `unknown` outside the mask still blocks as today.
- `check(grid, mask, keepout, outside, dynamic=None)`: `occ`/`unk` exclude
  `dynamic` cells; `provisional` counts them.
- `validate(...)` takes `dynamic` like `keepout` (`load_mask(rev_dir, "dynamic")`
  generalises `load_keepout`), checks alignment, and adds an informational
  `Issue(code="provisional", severity="info", step_id, message="N cells in a
  dynamic area: passable only if the live scan agrees")`. `Validation.ok`
  ignores info issues; the editor shows them in a distinct colour.
- Callers (`amr_web/server.py` ×3, `route_executor_node.py:342`) pass the mask.

**Route executor** — `amr_mission/route_executor_node.py`
- Load `dynamic` next to `keepout` at mission load.
- `_obstruction()`: today `hits = mask & (grid < 65)` drops points the map
  explains. New: `hits = mask & ((grid < 65) | dynamic)` — a scan point inside a
  dynamic area counts even if the map has an obstacle there. This is the change
  that makes a still-present mapped trolley BLOCK instead of being driven into.
- Nothing else changes in Increment 1. The "obstacle cleared" evidence is
  Increment 2.

**Navigation layer / AMCL** — `amr_bringup/launch/navigation_layer.launch.py`
- Follow the Q20 pattern (`_costmap_footprint_file`): at launch, if the revision
  has `dynamic.pgm`, write a derived localisation map
  `~/.amr/loc_map_gen{N}.pgm/.yaml` = `map.pgm` with dynamic cells set to unknown
  (205), and point `map_server` `yaml_filename` at it. AMCL then has no
  landmarks in dynamic areas; returns from a trolley fall under `z_rand`/`z_short`
  and beam skipping (`do_beamskip: true`, `likelihood_field_prob`) instead of
  pulling the pose towards a trolley outline that may have moved.
- Log the derived map's cell counts (`n dynamic cells blanked of M occupied`) so
  a map that is mostly trolleys is visible at launch. Hard refuse if fewer than
  `loc_min_occupied_cells` (param, default 500) remain: there is nothing to
  localise against.
- The `/map` topic the web live view shows stays the reviewed `map.pgm`
  (operators want to see the trolleys); the derived file is AMCL's only.

**Localisation monitor** — `amr_localization/scan_consistency.py`, `localization_monitor_node.py`
- `compare(...)` takes `dynamic` (bool array or None). Beams whose endpoint or
  whose expected wall lies in a dynamic cell are excluded from both fractions:
  they prove nothing about the pose. This keeps `min_scan_match 0.6` meaningful
  on a cluttered floor and stops a fresh trolley from counting as "long" beams.
- Monitor loads the mask with the grid (`_load_grid`), passes it through.

**Map editor UI** — `amr_web`
- New `review.html`/`review.js` page ("Maps → Review rev N") using `MapView`
  with the editor's token pattern (`mapToken`, `stillCurrent`):
  polygon tool with three actions — paint free / paint unknown / mark dynamic —
  undo/redo, dynamic areas drawn hatched, painted cells previewed on the canvas.
  Result: `POST /api/maps/<id>/<rev>/edit {ops, dynamic}` → `derive_edit` → new
  revision listed on the Maps page with a "derived from rev N (k edits)" line.
- Painting free is an operator assertion ("I know this is floor"); the page
  says so next to the tool. Painting unknown is the safe erase.
- `mapview.js` gains `fillPolygon(pts, color)` and a hatch pattern helper;
  `editor.js` draws dynamic areas hatched under the route so the operator sees
  why a step is "provisional".

### 1.3 Tests (Increment 1)
- `amr_maps/test/test_edit.py`: paint ops are applied in order; dynamic mask
  aligns with the grid; `edits.json` round-trips.
- `amr_mission/test/test_map_bundle.py`: `derive_edit` produces a revision whose
  manifest lists and hashes the three new files; a stray `dynamic.pgm` without
  `dynamic.yaml` is refused.
- `amr_navigation/test/test_route.py`: a straight through a mapped trolley in a
  dynamic area validates with one info issue; the same trolley outside a dynamic
  area still fails; unknown inside a dynamic area is provisional, outside is
  blocked.
- `amr_mission/test/test_route_executor_logic.py`: `_obstruction` counts scan
  points on a mapped-occupied cell inside a dynamic area (fixture grid + mask),
  ignores them outside.
- `amr_localization/test/test_scan_consistency.py`: beams into dynamic cells do
  not move `match`/`long`.
- `amr_bringup/test/test_launch_layers.py`: derived localisation map written
  with the right cells blanked; refusal below `loc_min_occupied_cells`.
- `amr_web/test/test_server.py`: `/edit` endpoint creates a revision; a
  `review_harness.js` for the polygon tool (same shape as `editor_harness.js`).

---

## Increment 2 — "observed clear" evidence in the executor

Today `clear_since` starts the moment fewer than `obstacle_points` endpoints
are inside the envelope. Endpoints vanish when the object leaves, but also when
a person stands in front of it, when the vehicle's own turn moves the object
into the rear blind spot, or when the object is beyond `range_max`. Resume
after BLOCKED should need positive evidence that the envelope was seen free.

### 2.1 Envelope visibility — new `amr_navigation/amr_navigation/visibility.py` (pure numpy)
- `seen_free(grid, mask, laser_xy, laser_yaw, scan, geom) -> (seen: np.ndarray, coverage: float)`:
  for the envelope cells in `mask`, a cell is *seen free* when some beam passes
  through it at a distance shorter than that beam's return (or the beam has no
  return / max range). Implementation: sample each beam at `resolution/2`
  steps up to its measured range (same construction as `raycast.cast`), mark
  the cells crossed; `coverage = seen.sum() / observable.sum()` where
  `observable` = envelope cells inside the scanner's 275° FOV from the laser
  pose (`raycast.NANOSCAN3_FOV`) and within `range_max`. Cells outside the FOV
  (rear blind spot) are excluded from the denominator: they cannot be observed
  and today's behaviour (not checked) is kept for them, stated in the RUNBOOK.
- Bounded cost: the envelope is at most `horizon` (straight) or the rotation
  sweep; at 0.05 m cells that is a few thousand cells, sampled at ≤10 Hz.

### 2.2 Executor states — `route_executor_node.py`
- `_obstruction()` returns a small dataclass instead of a string:
  `Obstruction(state: "occupied"|"clear"|"unobserved"|"stale", reason: str, n_hits, coverage)`.
  - `occupied`: `n_hits >= obstacle_points` (unchanged rule) — `clear_since = None`.
  - `stale`: scan or TF older than `scan_age` (unchanged) — `clear_since = None`.
  - `unobserved`: `n_hits < obstacle_points` but `coverage < clear_coverage_min`
    (param, default 0.9) — `clear_since = None`; reason names the coverage
    (`"envelope 62 % observed"`).
  - `clear`: hits below threshold AND coverage ≥ min — `clear_since` starts.
- `_resume_checks()` and the BLOCKED tick keep their structure; the reason string
  now distinguishes "obstacle" from "not yet seen clear", which is what the
  operator sees on the Run page under BLOCKED.
- While EXECUTING (not blocked), only `occupied` stops the vehicle, as today. A
  transiently unobserved envelope (someone walks past) must not stop a run.
- Diagnostics: publish `obstruction_state`, `obstruction_coverage` in the
  executor's `RunState` message (`/amr/run_state`, two new fields).

### 2.3 Tests (Increment 2)
- `amr_navigation/test/test_visibility.py`: an open envelope in front is fully
  seen; a wall halfway gives partial coverage; cells behind the FOV are excluded;
  a beam with no return sees its whole path.
- `test_route_executor_logic.py`: occluded obstacle → `unobserved`, resume
  refused with the coverage reason; object gone and floor seen → `clear` after
  `clear_stable_s`; a run in progress does not pause on `unobserved`.
- Sim (opt-in, short): `scan_synth` `AddObstacle` in front → BLOCKED; add a
  second box between the vehicle and the first, remove the first → still
  BLOCKED with "observed"; remove the second → resume allowed.

---

## Increment 3 — evidence across runs and a candidate map

The map improves from what the vehicle sees on ordinary missions. Evidence is
only recorded while localisation is READY and the map→odom stamp is fresh, so a
degraded pose can never rewrite the map that pose is estimated against.

### 3.1 Recorder — new node `amr_mission/amr_mission/evidence_recorder_node.py`
- Runs in the navigation layer next to the executor. Subscribes `/scan_gated`,
  `/amr/localization_state` (READY + fresh stamp gate), the mission's map
  identity from `/amr/run_state`.
- Per scan (≤10 Hz, decimated to 2 Hz — plenty for parked objects): with the
  laser pose at the scan stamp, mark `hits[r, c] += 1` at endpoints and
  `passes[r, c] += 1` along each beam up to its return (the same ray walk as
  2.1, so one implementation in `visibility.py`). Both arrays are `uint16` in
  the map's grid geometry.
- On mission end, or every 5 min, write
  `~/amr_maps/<map_id>/rev<N>/evidence/<run_id>.npz` with `hits`, `passes`,
  `scans`, `t_first`, `t_last`, `localization_ready_fraction`. Runs shorter than
  30 s or with ready fraction < 0.95 are dropped. `evidence/` is not a bundle
  file (revisions stay immutable; verify ignores the directory).
- Parameter `evidence_enabled` (default true), and an upper bound on files per
  revision (oldest deleted).

### 3.2 Candidate builder — new pure module `amr_maps/amr_maps/evidence.py` + CLI `amr_maps/scripts/build_candidate.py`
- Merge all `*.npz` of a revision by *session*, not by scan: each run votes once
  per cell (`hit_sessions`, `pass_sessions`), so a trolley parked for one long
  run cannot outvote ten short runs.
- Rules, all operator-tunable in the CLI:
  - **remove**: cell occupied in the reference, `pass_sessions >= P` (default 3)
    and `hit_sessions == 0` in those sessions → propose free. Applies inside
    and outside dynamic areas; outside, the CLI flags it as "structure went
    missing — check" because a wall does not move.
  - **reveal**: cell unknown in the reference and `pass_sessions >= P` → propose
    free (floor behind a trolley that has left).
  - **add**: cell free/unknown, outside dynamic areas, `hit_sessions >= S`
    (default 5) with hits in the last session too → propose occupied (new
    fixture). Inside dynamic areas nothing is ever promoted to occupied.
  - Everything else unchanged.
- Output: `candidate.pgm` + `diff.json` (cells changed per rule, per dynamic
  area) — nothing is published by the builder.

### 3.3 Review and publish
- The Review page (1.2) gets a "Candidate from evidence" mode: shows the diff as
  coloured overlays (removed = green, revealed = green hatched, added = red),
  lets the operator accept/reject per connected region, then publishes through
  `derive_edit` with `ops` recorded as `{"op": "evidence", "rule": ..., "cells": n}`
  plus the accepted region polygons. The new revision therefore carries a
  replayable history like a hand edit.
- Route revalidation: after publishing, the page lists routes of the parent
  revision and offers "re-validate on rev N+1" (existing `validate` API), so a
  route blocked by a removed trolley is usable again without redrawing.

### 3.4 Regeneration caveat
`mapping_session_node` builds `map.pgm` from `/map` at save time; the pose
graph is serialised but never re-rendered by us. If a survey is later continued
from a revision's pose graph (not supported today; would be a
`slam_toolbox` `map_start_at_dock`/deserialise start), the new `map.pgm` comes
from the graph and loses every edit. `edits.json` exists so that flow can
replay them (`edit.apply(ops)`), and the mapping session must refuse to
continue from a revision whose `edits.json` it cannot replay. Out of scope
until continued surveys are needed.

### 3.5 Tests (Increment 3)
- `amr_maps/test/test_evidence.py`: session voting; a single long session does
  not remove; occluded cells (no passes) never change; `add` never fires inside
  a dynamic area; reveal only from unknown.
- `amr_mission/test/test_evidence_recorder.py` (logic, no ROS): gate on
  READY/fresh; short runs dropped; bounded file count.
- Sim end-to-end (opt-in): survey with two boxes → run three missions with one
  box removed → candidate proposes exactly that box's cells → publish →
  the route through it validates on the new revision; the remaining box still
  BLOCKs.

---

## Increment 4 — operator visibility and hardening

- Run page: BLOCKED banner shows the obstruction state and coverage
  ("blocked: 14 points in the envelope" / "blocked: waiting to see the corridor,
  62 % observed").
- Maps page: per revision show `dynamic areas: k`, `edits: n`, `evidence runs: m`,
  and a "Build candidate" button that runs the builder in the web process with
  a bounded time.
- RUNBOOK: new section "Trolleys and other temporary objects" — how to mark
  dynamic areas, what provisional means, why resume can wait on visibility,
  how to accept a candidate. Bench checklist row for the rear-blind-spot
  limitation.
- Localisation acceptance on the vehicle before trusting dynamic areas: with a
  trolley-heavy map, compare AMCL covariance and `scan_match` between the
  plain map and the blanked map on the same route (K-series acceptance,
  `bench-checklists.md`). Blanking is a hypothesis about AMCL's behaviour and
  must be measured, not assumed.

---

## Order of work and what ships when

| step | ships | operator gets |
|---|---|---|
| 1.1 + 1.2 validation/executor/monitor + 1.3 tests | first | routes through provisional areas run; a mapped trolley that is still there BLOCKs |
| 1.2 nav layer (AMCL blanked map) + acceptance | after the measurement above | localisation tolerant of moved trolleys |
| 1.2 review UI | with the above | can mark areas and erase trolleys without a text editor |
| 2 | second | resume cannot be fooled by occlusion |
| 3.1 recorder | third | evidence accumulates silently |
| 3.2 + 3.3 | after enough runs exist | maps improve with operator approval |
| 4 | alongside 2–3 | status and documentation |

Until the review UI exists, dynamic areas can be authored with a small CLI
(`python3 -m amr_maps.edit --map tool-center-00 --rev 1 --dynamic poly.json`)
that calls `derive_edit`; ship it with 1.1 so the vehicle tests do not wait on
the page.

## Out of scope (deliberately)

- Detours / re-planning around obstacles: needs an approved corridor model
  first; Reeds–Shepp is unrelated to occupancy.
- Filtering moving objects out of the scan for SLAM during survey.
- Tracking moving objects (forklift prediction) and fleet reservations.
- Making the local costmap read the dynamic mask: it never uses the static map.

## Open decisions (recommendation in bold)

1. Dynamic areas as a raster (`dynamic.pgm`) or polygons only: **raster in the
   bundle, polygons in `edits.json`** — every consumer indexes cells, and the
   keepout code path already handles an aligned mask.
2. Paint-free as an operator assertion: **allowed, labelled as such**; the
   alternative (only evidence may free a cell) would make the first weeks
   unusable.
3. Evidence at 2 Hz on the N97 alongside AMCL and the executor: **measure in
   the first vehicle run with `evidence_enabled`; drop to 1 Hz if the executor
   tick or AMCL fall behind** (add a tick-overrun log to the executor first,
   as canworker has).
