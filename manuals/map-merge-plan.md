# Map merge plan: join independent surveys into one map

Status: plan, 2026-10-23. Nothing built yet.
Trigger: `boiacca-map-1` rev1 was saved part-way through the site (2026-10-23). The rest of the
area will be surveyed as separate maps and has to become one map that AMCL and Nav2 can use.

## 0. Summary

- **Flow:** survey map 1, survey map 2 (any start position, independent of map 1), then the engineer
  places map 2 on map 1 and saves the result as a **new revision of map 1**.
- **Base frame is kept.** The merged revision uses map 1's frame. Routes drawn on map 1 keep their
  coordinates, and its start reference stays valid. The grid grows; existing cells are copied unchanged.
- **Placement has three steps:**
  - manual coarse placement: pick the same two points on both maps, or drag and rotate
  - automatic refinement within a small window
  - quality gates on the overlap
- **Works with existing bundles.** The merge reads only `map.pgm`/`map.yaml` and the optional masks,
  which every revision has, including `boiacca-map-1` rev1. The manifest schema doesn't change, so
  older code still loads a merged revision.
- **Trust:** yes for AMCL, if both conditions hold:
  - the overlap passes the gates (§5)
  - the seam passes the vehicle checks (§6)
  It is a rigid join of two separately optimized maps, so it is weaker than one continuous survey
  (§7, §8).
- **Existing tools (§8):** slam_toolbox's continue mapping, already installed, gives a better map for
  extending `boiacca-map-1`. Build it first. This merge is for maps that already exist separately.

## 1. Goals and non-goals

| Goal | Non-goal |
|---|---|
| Join 2+ saved revisions into one revision usable by AMCL, Nav2, routes and the tape graph | Merging slam_toolbox pose graphs |
| Engineer-controlled placement, replayable from the record | Fully automatic place recognition |
| Revisions stay immutable; the merge is a new revision with full provenance | Editing the source revisions |
| Measurable quality: overlap score, conflicts, ambiguity | Correcting a source map's internal drift (a rigid join cannot bend a map) |

## 2. What the codebase already provides

| Fact | Where | Consequence for the merge |
|---|---|---|
| A bundle is `map.pgm`/`map.yaml` + `posegraph.*` + `manifest.yaml`, with optional `dynamic.*`, `keepout.*`, `edits.json` | `amr_mission/map_bundle.py` | The merge reads only the grid and the masks |
| The pose graph is written at save and **never read** by anything | grep: only `mapping_session_node.py:369` | The merged revision can drop it with no loss |
| `verify()` doesn't require a pose graph; only `stage_bundle` at survey save does | `map_bundle.py:238` | A merged revision without `posegraph.*` verifies |
| A derived revision is the pattern: lock, copy, add files, hash, verify, atomic publish; `edits.json` records the parent and the ops so it can be replayed | `derive_revision`, `derive_edit` | The merge is a new `derive_merge` with a `merge.json` that works the same way |
| Grids must be axis-aligned (rotated origin refused) | `amr_maps/grid.py:78` | Map 2 is resampled into map 1's cell lattice; the output is never a rotated grid |
| The `dynamic` mask must align cell-for-cell with the map | `verify()`, `grid.mask_misalignment` | Masks are rebuilt on the merged lattice |
| Routes are stored per revision, in map-frame metres | `amr_navigation/store.py` | Keeping map 1's frame keeps routes geometrically valid; they still need re-validating on the new revision |
| A tape graph is bound to (map_id, revision, sha256) | `amr_line/tape_graph.py:21` | Teach the tape graph **after** the final merge; each new merge needs a re-teach |
| Review page: canvas with pan, rectangle tools, save as new revision (`review.js`, `mapview.js`) | `amr_web` | The Merge page reuses it |
| `boiacca-map-1` rev1: 2319 × 1203 cells at 0.05 m (116 × 60 m), 44 k occupied, pose graph present, no masks; closure recorded away from the start mark (dx 0.44, dy 2.93 m: meaningless) | `~/amr_maps/boiacca-map-1/rev1` | This revision is the first base |

## 3. Data model

### 3.1 Output identity
- The output is a new revision under the **base** map id: `boiacca-map-1 rev N`. It isn't a new map id:
  - the lineage stays in one place
  - the start reference, routes and the engineer's mental model stay with map 1
- The manifest keeps the existing fields:
  - `start`: the base map's (unchanged)
  - `review`: the base map's, plus a `merge` note
  - `software`: `{"merge": "amr_maps.merge v1", "slam": <base value>}`
- `merge.json` is a new bundle file, added to `CONSUMED_OPTIONAL` so it must be listed and hashed,
  like `edits.json`:

```json
{
  "base":   {"map_id": "boiacca-map-1", "revision": 1, "sha256": "09ab1a68…"},
  "placed": [
    {"map_id": "boiacca-map-2", "revision": 1, "sha256": "…",
     "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
     "coarse": {"method": "two_point", "pairs": [[[x1, y1], [x2, y2]], [[x1, y1], [x2, y2]]]},
     "refine": {"accepted": true, "dx": 0.0, "dy": 0.0, "dyaw": 0.0, "score_m": 0.0,
                "inliers": 0.0, "ambiguity": "none"},
     "take_placed": [[[x, y], [x, y], [x, y]]]}
  ],
  "rule": "base_wins_v1",
  "stats": {"overlap_cells": 0, "agree": 0, "conflict": 0, "grown_cells": 0},
  "created": "…", "note": "…"
}
```

- `pose` is map 2's frame expressed in map 1's frame. It is the one number that defines the merge;
  the rest is evidence.
- The merge can be replayed: the same sources, pose, rule and polygons always give the same bytes
  (a test).

### 3.2 Pose graph
- The merged revision carries **no** `posegraph.*`. Copying map 1's would claim coverage it doesn't have.
- Consequence: a future "continue survey" (§8) starts from a survey revision, not from a merged one.

### 3.3 Merging further maps
- Map 3 is placed on the merged revision (base = `boiacca-map-1 rev N`) in the same way.
- `placed` is a list so one merge can place several maps at once; the UI places one at a time.

## 4. Merge algorithm (pure, `amr_maps/merge.py`, numpy only)

### 4.1 Grid growth
- The new extent is the union of map 1's extent and map 2's transformed corners, plus a 1 m margin.
- The new origin is snapped to map 1's lattice: origin = base origin − k × resolution, with k a
  whole number of cells.
  - This copies map 1's cells **one-to-one, with no resampling**.
  - A test asserts that every base cell is byte-identical in the merged grid.
- Size cap: refuse beyond 25 M cells (about 250 × 250 m at 5 cm), to bound AMCL's likelihood field
  and the Nav2 static layer on the N97. The cap is a profile value.

### 4.2 Resampling map 2
- Each output cell is mapped back into map 2 by inverse transform, with 2 × 2 supersampling per cell:
  - **occupied** if any sample hits occupied (stops thin walls breaking up under rotation)
  - **free** if all samples are known free
  - otherwise **unknown**
- There is no interpolation and no blending. Values stay trinary (0 / 100 / −1), as map_server's
  `trinary` mode expects.

### 4.3 Cell rule in the overlap (`base_wins_v1`)

| Base cell \ placed cell | unknown | free | occupied |
|---|---|---|---|
| **unknown** | unknown | free | occupied |
| **free** | free | free | free, counted as a **conflict** |
| **occupied** | occupied | occupied, counted as a **conflict** | occupied |

- The base wins where it has data, and the placed map fills only the base's unknown cells. The rule
  is simple, predictable, and never changes what map 1 already says.
- `take_placed` polygons (engineer-drawn) invert the rule locally, where map 2 is the better survey
  of a place (for example, a stretch map 1 only saw from far away).
- Conflict cells are shown as a red layer and counted. They are the main alignment evidence (§5).

### 4.4 Masks
- `dynamic`: the union of the base mask and the transformed placed mask, rebuilt on the merged lattice.
- `keepout`: the same.
- `edits.json` is never inherited, as in `derive_revision`.

### 4.5 Refinement (automatic, around the engineer's coarse pose)
- **Score:** the mean distance from map 2's occupied cells inside the overlap to the nearest occupied
  cell of map 1, truncated at 0.30 m.
  - The distance field is a bounded chamfer: iterative dilation as in `scan_consistency.occupied_near`.
    No scipy or cv2 dependency.
- **Search:**
  - coarse pass: ±0.5 m in 1-cell steps, ±3° in 0.25° steps
  - fine pass: ±2 cells, ±0.3° in 0.05° steps
- **Outputs:**
  - **score_m:** mean truncated distance
  - **inliers:** fraction of map 2's overlap occupied cells within 1 cell of map 1's occupied cells
  - **ambiguity:** compare the best score with the best score at least 0.3 m (or 1°) away along each
    axis. If they are within 10 %, the axis is flagged, e.g. "ambiguous along the corridor": the
    overlap only has walls in one direction.
  - **overlap structure:** the eigenvalue ratio of the overlap's occupied-cell spread. Near 0 means
    every wall runs one way, so the offset along that direction is not determined.
- The engineer accepts the refined pose or keeps the manual one. The choice is recorded either way.

## 5. Offline gates (the tool enforces them before Save)

| Gate | Start value | On fail |
|---|---|---|
| Overlap: map 2 occupied cells inside map 1's known area | ≥ 2000 cells (≈ 100 m of wall) | refuse |
| Overlap structure: walls in two directions | eigenvalue ratio ≥ 0.15 | refuse |
| Ambiguity | none flagged | refuse |
| Inliers | ≥ 0.80 | warn; engineer note required |
| Score | ≤ 0.05 m | warn; engineer note required |
| Conflicts / overlap known cells | ≤ 3 % | warn; check the red layer for moved trolleys and mark them as dynamic |

- The start values are placeholders, settled on the first real merge. As with other tolerances, they
  are decided on the vehicle, not tuned in sim.

## 6. Vehicle checks (before the merged revision carries routes or a tape graph)

| # | Check | Pass |
|---|---|---|
| V1 | Seam crossing: NAVIGATION on the merged revision; drive manually across the seam both ways at survey speed, 3 times | no `LOC_LOST`; the localization monitor's `match` doesn't drop at the seam compared with either side; map→odom jump at the seam ≤ 5 cm / 0.5° |
| V2 | Distance across the seam: two floor marks ≥ 10 m apart, one in each map's own area, tape-measured; park on each and read the AMCL pose | map distance within 1 % of the tape (the "span 12.00 m measured 11.96" convention) |
| V3 | Far side: relocalize from a cold start at a point ≥ 20 m inside map 2's area | AMCL converges without help; covariance settles as on map 1 |
| V4 | Route across the seam (trackless, when used) | runs with no recovery behaviours at the seam |

- V1–V3 are about 30 minutes with the operator.
- A step the operator saw pass is PASS.
- If V1 or V2 fails, the cause is in the placement or in a source map. Re-place, or re-survey the
  overlap; don't tune AMCL around a bad seam.

## 7. Can a merged map be trusted for localization?

**Yes, within limits.**

### 7.1 Why it can work
- AMCL compares each scan with the raster around the vehicle. It needs the map to be **locally**
  consistent, not globally true.
- Routes and the tape graph are drawn and taught **on the same merged raster**, so they agree with
  what AMCL sees even if the raster is slightly off from the real building.

### 7.2 What a rigid join cannot fix, and how the plan contains it

| Risk | Effect | Containment |
|---|---|---|
| Each survey keeps its own drift; a rigid join cannot spread the error across the seam | A long wall seen by both maps lines up at the seam and drifts apart away from it: 0.2° over 50 m is 17 cm, seen as doubled walls | `base_wins_v1` keeps one copy of each wall; conflict layer and gates; keep overlaps compact; V2 |
| Corridor aliasing: the overlap has walls in one direction only | Map 2 lands one rack pitch or pillar spacing off; looks plausible | Structure and ambiguity gates; V2 tape check |
| Different days: trolleys or stock moved | Conflicts in the overlap; AMCL sees clutter | Red layer → mark as dynamic areas (`RUNBOOK` §A2) |
| A poor source map (doubled walls, slipped heading) | Merging spreads a bad map | Each source must pass its own review before merging |
| Seam inside a precision area (tape stations, docking) | Small seam error where precision matters most | Put the seam ≥ 10 m from tape stations and teach points |

### 7.3 Verdict for plan 3 (tape localization)
- **Tape circuit:** keep it inside one source map, or check the seam with V1/V2 before S2 teach.
- **P0 measurement:** measure AMCL repeatability on both sides of the seam before trusting tracked
  localization there.

## 8. Existing tools (checked 2026-10-23, on the vehicle and online)

| Tool | Status here | What it does | Fit |
|---|---|---|---|
| **slam_toolbox continue mapping**: `deserialize_map` with `START_AT_GIVEN_POSE` (`LOCALIZE_AT_POSE` is for the localization node only) | Installed (2.6.10, service verified with `ros2 interface show`) | Loads a saved pose graph, matches the first scan near a given pose, and keeps mapping in the **same** pose graph and frame. Loop closures connect old and new scans. The upstream docs call this the supported multi-session flow; "lifelong" node decay is still experimental (don't use it) | **Best map quality.** Works from `boiacca-map-1` rev1, whose pose graph is saved. Needs our code: a deserializing mapping launch, a start-from-revision path in `mapping_session_node`, and a pose hint on the Maps page |
| **slam_toolbox `merge_maps_kinematic`** | Installed (executable present; services `slam_toolbox/add_submap`, `slam_toolbox/merge_submaps`) | Loads serialized pose graphs as submaps. Each is moved **by hand** with RViz interactive markers. Merge then **re-renders one grid from all raw scans** under the chosen transforms and publishes it on `/map` | Same manual rigid placement as this plan, but re-rendered from scans, which is cleaner than overlaying rasters. Gaps: needs RViz; no refinement, quality score or gates; nothing is saved (`map_saver` plus our own importer); poor error handling (a failed merge still publishes and reports success); needs pose graphs on both inputs |
| **m-explore-ros2 `multirobot_map_merge`** | Not installed; source only, targets Humble | Live multi-robot merge from `/map` topics. Uses known start poses, or estimates the transform by image feature matching (OpenCV). Combines with an element-wise max, so occupied wins in a conflict | Built for live exploration, not saved bundles. Its own users report that SLAM drift becomes visible at the seams with known poses. Borrowable idea: feature matching for automatic coarse placement |
| **RTAB-Map** multi-session | apt packages available | Native multi-session SLAM: each session has its own frame, and they connect at revisited places | Would replace the SLAM backend. Users report grid-export quirks in multi-session. Not worth the switch |
| **Cartographer** multi-trajectory + `load_state` | apt packages available | Several trajectories in one globally optimized graph | Also a backend swap; upstream is no longer actively developed |
| nav2_map_server | Installed | Loads and saves maps only | No merging |

### 8.1 Recommendation from the check
1. **Build continue-survey first.** Built 2026-10-23 (`RUNBOOK.md` §A1), not yet tried
   on the vehicle. Correction from reading the slam_toolbox Humble source:
   - The mapping node needs `START_AT_GIVEN_POSE`, not `LOCALIZE_AT_POSE`. The mapping
     node has no handler for `LOCALIZE_AT_POSE` and exits on the first scan.
   - Its first match only searches about ±0.25 m. So `amr_maps.relocalize` first
     matches the live scan on the saved map within ±1.5 m / ±30°, refuses poor or
     ambiguous matches, and hands slam_toolbox the matched pose.
   - It is slam_toolbox's own multi-session mechanism, with loop closures that remove the seam
     problem in §7.2.
   - `boiacca-map-1` rev1 can be continued, so the rest of this site may never need a merge.
   - Risks to check on the vehicle first:
     - load time and memory of a 94 MB pose graph on the N97
     - relocalization from the pose hint
2. **Keep the raster merge (this plan) for maps that already exist separately**: another day, another
   vehicle, or a survey that can't be continued. It also covers revisions with no pose graph.
   Changes after the check:
   - Coarse placement may use OpenCV feature matching on the two rasters, as in m-explore, next to the
     two-point pick, behind the same refine and gates. `cv2` is on the vehicle but not a dependency
     of `amr_maps`, so import it lazily.
   - Our refinement, gates, provenance (`merge.json`) and bundle integration are what the existing
     tools lack. That is the reason to build our own instead of wrapping one.
3. **Before M1 exists, `merge_maps_kinematic` is a zero-code trial.**
   - Load the pose graphs of rev1 and map 2, place them in RViz, merge, then save `/map` with
     `map_saver_cli`.
   - It shows how good a rigid join of these two surveys can be before any code is written.
   - Its output is not a bundle. Don't run routes on it.
   - Check before trusting it: whether submap 1's frame stays unchanged after the merge.

## 9. Field procedure for map 2 (so the merge has what it needs)

1. **Start inside map 1's area**, 10–15 m back from its edge, so the overlap is driven, not just seen
   from a distance.
2. **Drive the overlap slowly in both directions.** Include a corner or pillars, so it has walls in
   two directions.
3. **Survey on the same kind of day:** racks, machines and trolleys where they normally are.
4. **Put floor marks for V2:** two marks ≥ 10 m apart, one each side of the future seam, tape-measured.
   Write the distance in the session log.
5. **Map id `boiacca-map-2`.** Save at the start mark if possible, so its closure numbers mean something.

## 10. Increments

| # | Content | Tests |
|---|---|---|
| M1 | `amr_maps/merge.py`: lattice growth, resampling, `base_wins_v1`, masks, score, refine, ambiguity, stats | base cells byte-identical; identity merge = base; a known pose recovered on synthetic maps (`generate_sim_factory`); walls stay unbroken under a 7° rotation; conflict counting; corridor-only overlap flagged ambiguous; size cap refused |
| M2 | `map_bundle.derive_merge` → new base revision, `merge.json` in `CONSUMED_OPTIONAL`, `posegraph.*` dropped, sources verified by sha256 before use | replay gives identical bytes; changed source sha → refused; a fixture shaped like `boiacca-map-1` rev1 (pose graph, no masks) merges; merged revision verifies and loads in `navigation_layer` |
| M3 | CLI `ros2 run amr_mission map_merge --base boiacca-map-1 --rev 1 --place boiacca-map-2 --place-rev 1 --pose X Y YAW [--refine] --note …` (like `map_edit`) | CLI and web endpoints give the same bytes |
| M4 | Web **Merge** page (engineer PIN): base map + placed map overlay (placed walls in a second colour), two-point pick, drag/rotate with ±1 cell / ±0.1° nudges, Refine, conflict layer, gate table, Save as new revision. Endpoints `/api/maps/<id>/<rev>/merge/preview` and `/merge/save` | role check; gates enforced server-side; page fits 1280 × 720 at UI scale `l` |
| M5 | Vehicle checks V1–V3 on `boiacca-map-1` + `boiacca-map-2`; settle the §5 thresholds | operator observation |

- **Safety:** no increment touches motion, gating or the CAN path. The Merge page doesn't move the
  vehicle. A merged revision is used only after V1–V3, and the nanoScan3 / FX3 chain is unchanged.
- **Docs:** `RUNBOOK.md` gets an "A3. Merge maps" section at M4. `tape-localization-runbook.md`
  §2.3 notes the seam rule in §7.3.
