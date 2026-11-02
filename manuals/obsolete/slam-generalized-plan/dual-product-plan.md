# Plan: one codebase, two products — tape AGV and trackless AMR

> **Status 2026-09-20:** approved, nothing built yet. Decisions below were taken with
> the operator on 2026-09-20. This plan supersedes four decisions in
> `line-follow-u11-layout-plan.md` (see **Supersedes**) and moves U11 behind it in
> `unified_amr_service_plan.md`; both carry amendment notes pointing here.
>
> Increment order is 0 → 1 → 2 → 3 → 4 → 5. Each ends somewhere testable and safe.
>
> **Updated 2026-09-20 for the `agv_core` restructure (`dc9f608`).** The vehicle library
> moved out of the repo root into a package: `config.py` → `agv_core/config.py`,
> `core/*.py` → `agv_core/*.py`, `drivers/**` → `agv_core/drivers/**`, and
> `amr_base/agv_repo.py` is **deleted** — ROS nodes now say `from agv_core import config`.
> Paths below are the post-restructure ones. `git show gy-demo:core/X.py` source paths are
> unchanged (that branch still has the old layout); only the destination moved.

### Porting into `agv_core` after the restructure

A file ported from `gy-demo` arrives with **bare** imports (`import config`, `import branch`)
because that branch used a flat `sys.path`. `tests/test_layout.py` now pins the package
boundary — "no module inside `agv_core` imports a sibling by bare name" — so every ported
module must have its imports rewritten to package form. That is the *only* edit the
second port commit makes (see **Port procedure**); it does not change the engine's logic.

Note the engine modules that land in `amr_line` are handed a namespace rather than importing
`config` at all, so for those the rewrite is the single `from amr_line import runtime as config`
line the plan already specifies.

## Context

The product decision changed on 2026-09-20. The vehicle ships as a **magnetic-tape
line-following AGV with RFID stations** — functionally the `gy-demo` branch — and the
trackless SLAM AMR we have been building for weeks becomes a **paid upgrade**. A
customer who bought the tape AGV must not see the SLAM pages at all.

The tape engine was deleted from `slam-roadmap` in `8403dcb` (2026-09-06) and lives only
on `gy-demo` (tip `534e58d`). It cannot simply be run alongside the ROS stack:
`canworker.py` and `amr_base/drive_node.py` both claim exclusive ownership of `can0`,
behind a three-deep interlock (cooperative `ownerlock` in `/run/lock/amr`,
`amr_base/legacy_guard.py:refuse_if_legacy_running()`, and systemd `Conflicts=`). So the
engine must be **ported into the ROS stack**, not resurrected beside it.

The good news, established by exploration: the architecture already has the right seam.
`supervisor_node` runs an always-on **base layer** plus **exactly one swappable layer**
(`mapping_layer` | `navigation_layer`), and the 8-step swap transaction in `mode_fsm.py`
is already mode-agnostic. Tape following becomes a **third layer**. And the SICK
nanoScan3 is one device serving both the hardwired safety chain (`/output_paths`→STO)
and `/scan` for SLAM — it sits in the base layer, so **the hardware BOM is identical for
both products and the upgrade is software-only**.

**Outcome:** one repo, one safety argument, one test suite, one web app; a per-robot
`product` flag decides which layers may start and which pages exist.

## Decisions taken (2026-09-20)

| Topic | Decision |
|---|---|
| Architecture | LINE is a third supervisor mode with its own swappable layer |
| Vehicle gains | Restored to `profiles/agv-01.json` verbatim (35 flat scalars) |
| Site mission | New versioned ROS store, modelled on `map_bundle.py`; validators **copied** from `gy-demo`, not re-derived |
| Engine config access | Engine is **handed** a namespace, never imports `config` |
| Protective field | One field set, sized for full speed → speed zones are ordinary application logic. Confirm and record the number at commissioning |
| Interim product | `gy-demo` ships as-is on its own image. **U11 moves behind this port** |
| Obstacles | Safety chain only, in both modes (already true since 2026-09-20) |

## Supersedes

`manuals/slam-generalized-plan/line-follow-u11-layout-plan.md` (approved 2026-09-19) keeps
Parts 2–3 and its authority rules, but four decisions are superseded. Add an amendment
note at its top:

| Plan-doc decision | Status |
|---|---|
| §1.1 sensor reading (`read_mls.py`, `mls_track.py`, `LineTrack.msg`, `tests/test_mls.py`) | **Stands. Already shipped.** |
| §1.2 "a new cut-down PID" | **Superseded** — port `gy-demo:core/autopilot.py`, which carries two field-debugged fixes a rewrite would lose |
| §1.2 "out of scope: branch, RFID, stops" | **Superseded** — deferred to Increments 2–3, not dropped |
| §1.3 node in BASE, armed by CLI | **Superseded** — node moves into the LINE layer; arming becomes mode entry + a layer-local service |
| §1.3 `LEASE_LINE = 8`, `LINE = 7`, and the authority rules | **Kept verbatim** |
| §1.3 "not shown in the UI" | **Superseded** — the tape UI *is* product A's UI |

---

## Architecture

```
BASE layer (always on, both products)     LAYER slot (exactly one)
─────────────────────────────────────     ────────────────────────
drive_node      — owns can0               ├─ mapping_layer     (SLAM survey)
panel_node      — panel + horn            ├─ navigation_layer  (AMCL + Nav2 + executor)
cmd_mux         — authority               └─ line_layer        ← NEW
odometry, EKF, scan_gate
nanoScan3 (safety field + /scan)
```

Why a layer and not the approved doc's IDLE substate: a substate puts the tape engine,
an open RFID socket and mission config in the base layer of **both** products. The layer
slot already means "exactly one product's brain at a time". The engine is also stateful
(route index, branch seal-in latches, RFID cursor, lap count) and the transaction's
generation rotation + `CLEAR_CACHES` + `readiness.reset_layer_state()` is exactly
"forget what the old run believed".

**One layer slot, not two.** Optional sub-nodes (station reader, route engine) compose
*inside* `line_layer.launch.py` on launch args, the way `navigation_layer` handles
`blank_dynamic`. Increment gating costs a launch arg, not a slot.

### Config: split along the vehicle/site seam `gy-demo` already drew

Verified: `gy-demo:profiles/agv-01.json` has a flat `"autopilot"` section of **35 scalars
with zero nested values**, plus a separate top-level `"mission": "gy-demo"` string. The
two halves have very different costs.

- **Vehicle gains → `agv_core/config.py` `_SCHEMA` + `profiles/agv-01.json`, verbatim.** Flat
  scalars need no bespoke reader; `describe()`, `/params` and the schema-coverage tests
  work for free. These are commissioning constants of *this vehicle* and belong where
  `TRACK_M` and `MOTOR_MAX_RPM` already are.
- **Site mission → `amr_mission/mission_store.py`**, modelled on `map_bundle.py`:
  `~/amr_missions/<id>/rev<N>/mission.json` + `manifest.json` with sha256, atomic
  revision-bumping, `verify()`. A mission change is then a ~20 s layer swap, and product
  A gets the same "you are running exactly this, verified by sha" story product B has for
  maps.
- **The 260 lines of mission validation are copied, not re-derived** into
  `amr_mission/tape_mission.py` (`_read_branch_latch` 91, `_read_stop_tags` 16,
  `_read_route` 42, `_read_route_guard` 37, `_read_u_turn` 13, `_parse_mission` 36,
  `_routeless` 24), signature changed from `(doc, ns)` to `parse(doc, vehicle) -> dict`.
  It already enforces tag-namespace disjointness and hex-string-not-number.
- **The engine is handed a namespace.** `autopilot.py` reads `config.*` at **call** time
  by design (its docstring says so, and `test_control.py` exploits it). So
  `amr_line/runtime.py` exposes the flat CONST names with `apply(mapping)` /
  `snapshot()`, and a module-level `__getattr__` that raises `LineConfigError` on a
  missing key — preserving `agv_core/config.py`'s strict-both-ways property in its new home.
  **Each ported module changes exactly one line:** `import config` →
  `from amr_line import runtime as config`. The ported tests change the same one line.

---

## Increment 0 — identifiers and dispatch. No new behaviour, no motion.

Ship the "things that resist a third mode" alone, so the diff that adds motion is small.

**Modify only:**

| File | Change |
|---|---|
| `amr_interfaces/msg/ModeState.msg` | **append** `uint8 LINE=7`, `bool line_available`, `string product` |
| `amr_interfaces/msg/MuxState.msg` | **append** `uint8 LINE=7` |
| `amr_interfaces/msg/ControlLease.msg` | **append** `uint8 LINE=8` |
| `amr_interfaces/msg/MotionPermit.msg` | **append** `uint8 LINE=4` (reserved) |
| `amr_bringup/amr_bringup/mode_fsm.py` | `:16` `range(7)`→`range(8)`, `LINE` **appended last**; `MODE_NAMES[LINE]`; `REQ_LINE` admitted at `:113`; `Conditions.line_active` |
| `amr_bringup/amr_bringup/readiness.py` | `line_gen`/`line_state`, `on_line()`, `line_state_for()`, **and both added to `reset_layer_state()` `:145`** |
| `amr_bringup/amr_bringup/supervisor_node.py` | `:70` `LEASE_LINE = 8`; `:280` `layer_ready` → `_LAYER_MODES` membership; `:305` accept `LINE`; `:311`/`:338` → dicts; **`:546` `START_NEW` and `:591` `WAIT_READY` → dispatch tables that raise on an unknown target** |
| `amr_base/amr_base/gating.py` | `:31` append `LINE = 7` + `NAMES`; `:43` `LEASE_LINE = 8` |
| `amr_base/amr_base/cmd_mux_kinematics_node.py` | `:297` `gating.NAMES[…]` → `.get(…, str(…))` |
| `amr_web/amr_web/adapter.py` | `MODE_NAMES[7]` `:66`, `MUX_NAMES[7]` `:76` (display only) |

> **`supervisor_node.py:559` is the highest-risk line in the whole change.** `START_NEW`
> is `if target == MAPPING: … else: <navigation>` — a bare `else`. A LINE target would
> silently launch the **navigation** layer. `:593` has the same shape.

> **`mode_fsm.py:16` must be APPENDED to, never inserted into.** It is positional tuple
> unpacking against constants frozen in the `.msg`; inserting `LINE` mid-list silently
> renumbers `TRANSITIONING`/`FAULT`/`STOPPING` with no compile-time error anywhere.

**Tests (laptop):** `test_mode_fsm.py` gains `REQ_LINE` admission cases. New
`test_names_cover_the_enum` in `test_gating.py` **and** `test_mode_fsm.py` — assert every
constant has a `NAMES`/`MODE_NAMES` entry. That is the real fix for the three
hand-maintained int→name dicts whose unguarded subscripts sit in the 50 Hz mux tick and
the supervisor's serial loop; `.get()` is only the seatbelt.

**Vehicle:** none. The regression that matters is that IDLE↔NAVIGATION still swaps.

---

## Increment 1 — follows tape. Nothing else.

**Do this first, before booking any vehicle time:** port `gy-demo:tests/helpers.py`
`simulate()`/`_integrate()` verbatim and **re-run it with `delay_s=0.040`**. The gains
were tuned at 50 Hz with 20 ms of transport delay; the ROS path adds a hop
(line node → `/amr/line_cmd` → mux 50 Hz tick → `/amr/wheel_velocities` → drive_node → CAN).
If overshoot and settle still pass, the ported gains are usable. If not, the gains move —
not the architecture.

**Create — new package `amr_line`:**

| File | Provenance |
|---|---|
| `amr_line/runtime.py` | new, ~60 lines, strict `__getattr__` |
| `amr_line/autopilot.py` | **byte-for-byte** `gy-demo:core/autopilot.py`, one import line changed |
| `amr_line/branch.py` | **byte-for-byte** `gy-demo:core/branch.py` (imports nothing). Ported now, fed only `STRAIGHT` until Inc 3 — cheaper than stubbing it out of the follower |
| `amr_line/track.py` | new, ~40 lines: `LineTrack` msg → the engine's sensor dict (the `sensor()` helper in `gy-demo:tests/helpers.py`), plus freshness and a rate estimator |
| `amr_line/line_follow_node.py` | new. Arm/disarm + physical Start edge: copy `amr_base/commissioning_node.py`. Hold set and safety causes: copy `route_executor_node.py:387-423` (`_torque_off`, `_field_tripped`, `_safety_cause`, `_hold`) so **both products share one auto-resume story** |
| `amr_interfaces/msg/LineState.msg` | new, **generation-tagged** so `adapter.layer()` and `readiness.line_state_for()` work |
| `amr_bringup/launch/line_layer.launch.py` | new, ~70 lines; template is `mapping_layer.launch.py` |
| `amr_sim/tape_synth_node.py` | new. Re-hosts the `simulate()` plant so a LINE layer swap is testable in sim |

**Modify:** `agv_core/config.py` (restore the flat `autopilot` block to `_SCHEMA` + its
docstring paragraphs, which keeps the tuning-notes count test safe); `profiles/agv-01.json`
(still at the repo root; `config.PROFILE_DIR` walks up one level from the package);
`tests/test_config.py` (flip **only** the `autopilot` refusal at `:85-87` — `:88-92`
pin `branch_latch`/`stop_until_start_button` at *top level*, which was never their home
and stays refused); `tests/run_all.py` `EXPECTED_CHECKS` (**898** over 93 test functions
after the `agv_core` restructure).

`gating.py`: new LINE branch on the **AUTO** side, **above `:303`**, mirroring the
COMMISSIONING branch at `:276-283` — requires `lease.allowed & LEASE_LINE`, requires
`line.generation == lease.generation`, and returns an explicit `Selection(NONE, …, reason)`
rather than falling through.

`cmd_mux_kinematics_node.py`: subscribe `gating.nav_topic("/amr/line_cmd", gen)` in
`_subscribe_nav` `:265`; **clear the cache in `_apply_generation` `:257`**; do **not** add
LINE to the manual tuple at `:314`, so it lands on the `else` at `:330` and gets
`slew_asym` for free.

`LineTrack.msg`: **append** `float32 sample_age_s` and `uint32 seq`; set them in
`drive_node._publish_track` `:783` (today `stamp` is set at publish time and
`TrackSample.t_mono` is dropped, so in SDO mode a fresh-looking message can carry ~0.5 s
of sensor latency invisibly).

`supervisor_node.py`: `_start_line`; `_ready_line` copied from `_ready_mapping` `:609`
(wait for the generation-tagged state topic, no RPC); `_allowed()` `:235` gains
`if self.mode == fsm.LINE: return LEASE_LINE` — **exclusive**, so a stray browser jog
cannot fight the tape.

**Command shape:** publish a body `Twist` on `/amr/line_cmd` (the follower already
computes `v_mps`/`omega_out`), so LINE shares FOLLOW's accel path. Add a
`test_inverse_forward_round_trip`. Per-wheel with `sel.wheels=True` like commissioning is
a legitimate alternative if the double conversion proves lossy.

**In:** straight tape, speed ramp, PID, line-loss-by-distance stop, Start/Stop/Reset
authority, `v` capped at 0.30 m/s in **both** the node and the mux branch.
**Out:** RFID, stations, branches (choice pinned STRAIGHT), u-turn, speed zones, mission
files, web pages. CLI only.

**Port procedure (applies to every engine module).** First commit is a pure
`git show gy-demo:core/X.py` with **zero edits** (that branch still has the pre-`agv_core`
layout, so the source path is unchanged). Second commit contains **only** the import
rewrite — bare `import config` / `import branch` become the package or injected-namespace
form, which `tests/test_layout.py` now requires. Then `git diff` between the two commits is
the entire review surface. Do not
reformat, re-order or tidy — `branch.py` rung order is semantics, `select_track`
continuity is the run-0023 merge fix, the lateral **rate** clamp (not a position clamp)
is the run-0020 fix where a 120 mm merge jump became 6.66 rad/s of commanded yaw, and
`begin_measured_stop` solving `a = v²/2d` once rather than per tick is deliberate.

**Tests (laptop):** `simulate(delay_s=0.040)` first; port `gy-demo:tests/test_control.py`
(428 lines, 14 tests **including the `K_RATIO=100` negative control** — without it the
passes mean nothing); new `test_line_authority.py`, `test_line_mux.py`,
`test_line_sim.py` (opt-in `AMR_SIM_TESTS=1`, full IDLE→LINE→IDLE).

**Tests (vehicle), in this order:**
1. **Before anything moves:** read MLS `1800h:02` (transmission type) and `1800h:05`
   (event timer). Two minutes. See R1.
2. Jacked up, tape under the sensor: arm + Start → wheels turn the right way; Reset stops.
3. Floor, 0.20 m/s, 5 m of straight tape, person at the E-stop.
4. Selector→MANUAL mid-run kills it; Reset mid-run kills it.
5. Lift the sensor off the tape → DONE "track lost" within `LINE_LOSS_GRACE_M` (0.075 m).
6. Record `track_hz` and `sample_age_s` at speed from `/amr/line_state`.

---

## Increment 2 — mission store, RFID stations, stop-at-tag, speed zones

### Feature map: what `missions/gy-demo.json` carries, and where each lands (2026-09-21)

Every feature below is **tag-driven**. Increment 2's real deliverable is the RFID node and
the encounter stream; the rest is glue around engines that are pure Python and mostly
already in the tree.

| gy-demo feature | What it does there | ROS home | Inc | Notes |
|---|---|---|---|---|
| `route` (ordered stations, direction, `high_speed_to_next`) | tags are **not** locations; the stage index resolves repeated tag values | `amr_line/route.py`, byte-for-byte `core/route.py` (130 lines, stdlib) + `station_node` | 2 | `agv_core.drivers.rfid` exists; nothing in ROS reads it yet |
| `stop_until_start_button` (tag, direction, `stop_distance_m`) | decelerate over a distance after the tag, park as a **pause in the same run**; Start + `auto_start_delay_s` resumes | `FollowJob` gains hold cause `"station"`: `auto_resume=False`, cleared by a Start edge — the E-stop hold's mechanism, already tested | 2 | distance from the `followed_m` job.py already integrates |
| `high_speed_mode` (entry/exit tag pairs, direction-qualified) | latches `auto_rpm_high` (2000 vs 1000), 3 s ramp; cleared by stations, holds, disarm, RFID loss | a second cap in `FollowJob._cap()` selected by the latch; the mux keeps the absolute ceiling | 2 | Increment 1 caps at 0.30 m/s: high speed is meaningless until line-layer test §4 unlocks the profile value |
| `route_guard` (leg min/max distance) | rejects early tags, faults on overrun | inside the `route.py` port; distance source `followed_m` | 2 | disabled in gy-demo's own file; low priority |
| `branch_latch` / `branch_default` | entry tag latches left/right/straight, the fork consumes it, the exit tag ends the slow zone | `branch.py` is **already ported** verbatim; needs `nlcp` (in `LineTrack`) and the tag feed | 3 | wiring only |
| `u_turn` (tag, cw/ccw) | stop over 0.4 m, encoder-gated pivot at 200 r/min, tape-closed re-centring | port `core/uturn.py` as a sub-state of RUNNING | 3 | counts already on `/wheel_states` |

**Create:** `amr_mission/tape_mission.py` (the validator port); `amr_mission/mission_store.py`
(shape from `map_bundle.py`); `amr_line/route.py` (**byte-for-byte** `gy-demo:core/route.py`,
stdlib only); `amr_line/station_node.py` (owns `agv_core/drivers/rfid.py` over TCP — not
can0, so no ownership conflict); an `empty.json` equivalent as the store's legal zero state.

**Modify:**
- `agv_core/drivers/rfid.py` — apply the `gy-demo` **+25/−1 encounter diff**. Purely additive
  (`snapshot(encounters=False)` preserves every existing caller). The current copy is the
  pre-encounter version and **cannot drive `route.py`**: the engine consumes a frozen
  ordered batch exactly once per tick, detects buffer overrun (`seq != cursor+1`) and
  continuity loss (generation change) and faults under `route_guard`.
- `agv_core/config.py` / profile — add `rfid.tag_clear_s` → `RFID_TAG_CLEAR_S`.
- `StationDetection.msg` — **append** `uint64 encounter_seq`, `uint32 generation`,
  `bool comms_ok`, `float32 rx_age_s`. As specified today it is a per-read event with no
  ordering or continuity evidence, and nothing publishes or subscribes it. Publish one
  message per encounter on `/amr/stations`, RELIABLE depth 50.
- `amr_localization/package.xml:6` — delete the advertisement of a `station_id_node` that
  does not exist. The node lives in `amr_line`; there is no `stations.yaml`.
- `mode_fsm.Transaction` + `supervisor_node._srv_mode` — resolve and verify the mission
  bundle exactly as the NAVIGATION branch resolves a map (`:322-336`); `COMMIT` records it;
  `ModeState` gains `active_mission_id/_revision/_sha256`.

Route distance comes from `/wheel_states` continuous `left_pos_rad`/`right_pos_rad`.
Stop-at-tag uses `autopilot.begin_measured_stop(d)`. **No `drive_node` change.**

Speed zones are ordinary application logic (one field set sized for full speed). Record
the commissioned field number and its assumed stopping distance in the doc at this point.

**Out:** branches, u-turn, web pages.

**Tests (laptop):** pure cases from `gy-demo:tests/test_route.py`; `test_mission_store.py`
copying `test_map_bundle.py`'s atomicity/rollback/sha shape; `test_stations.py` (cursor
advance, overrun, generation loss); `test_tape_mission.py` (tag disjointness,
`0 < stop_distance_m <= 5`, hex-string-not-number).

**Tests (vehicle):** two-tag loop; **measured vs commanded stop distance** (see R3 — two
ramps in series); speed-zone entry/exit; deliberate RFID unplug mid-run → guard fault,
not a runaway.

---

## Increment 2b — planned stops in the trackless route: the `wait` step

"Stop until Start" is not a tape feature; it is a mission feature, and the trackless route
has no planned stop at all today. `run_fsm` knows `PAUSED` (an operator click) and
`BLOCKED` (the safety chain). A route that must halt at a station and wait for a human
cannot be written.

**Decision: a step type, not a per-route list.** A route stops at different points on
different passes (gy-demo's own route has four stations on two tag values), and a step is
ordered by construction. A side list would need the direction/stage disambiguation
`core/route.py` had to invent. The same primitive serves LINE mode once tags arrive:
`stop_until_start_button[tag]` becomes "inject a `wait` when the tag is read" — one hold
cause, two triggers.

**Modify:**
- `amr_navigation/route.py` — `WAIT = "wait"`; `{"id": ..., "type": "wait", "until":
  "start"}`. `until` is an enum with one value now so that `"seconds"` / `"io"` slot in
  later without touching the FSM. A `wait` has no geometry: the compiler emits no motion
  primitive and no swept envelope for it; two adjacent `wait`s are a validation error.
- `amr_mission/run_fsm.py` — reuse `PAUSED` with `reason="station: <step_id>"`. `start()`
  already leaves `PAUSED` on a Start edge (`run_fsm.py:97`); `auto_resume()` already
  refuses from `PAUSED`, which is exactly the semantics wanted.
- `route_executor_node.py` — on reaching a `wait` step: publish zero, `resume_prepared =
  true`, no permit requested, no goal sent; on the physical Start edge, `step_done()`.
  MANUAL during the wait aborts the run as it does during any pause.
- `RunState.step_type` comment: `straight | rotate | reverse | arc | wait | ""`.
- Editor (`amr_web/static/editor.js`): one more palette entry, placed between steps,
  drawn as a marker on the preceding step's end point.

**Out:** dwell timers, I/O-triggered release, and any coupling to LINE mode (that is
Increment 2's `"station"` hold, which uses the same Start-edge release).

**Tests (laptop):** schema round-trip with a `wait`; compiler emits N-1 primitives for N
steps with one `wait`; FSM: `wait` -> `PAUSED` -> Start -> next step; `auto_resume()` from
a station pause is refused; two adjacent `wait`s refused; `repeat_count` > 1 re-waits on
each pass. **Vehicle:** straight -> wait -> straight; Start with the selector in MANUAL is
ignored; E-stop during the wait is a `BLOCKED/estop` hold, and Start after it resumes into
the wait, not past it.

---

## Increment 3 — branches at diverters, u-turn

- Feed `BranchEngine` from the mission's `branch_latch`; the `choice` argument goes live.
- Port `gy-demo:core/uturn.py` → `amr_line/uturn.py`. Its inputs (`counts_per_wheel_rev`,
  `left_counts`/`right_counts`, `counts_valid`) are **already published** on
  `/wheel_states` in driver terms, and `counts_delta` already handles the INT32 `6064h`
  wrap. No `drive_node` change.
- **Horn: no work.** `panel_node.py:122` already drives `HORN_DO_CHANNEL` via
  `panel_io.horn_wanted(...)`, renewed every tick with a deadline — identical semantics to
  the legacy auto loop. Verify on the vehicle that it follows in LINE mode.

**Tests:** the ~75 % pure cases from `gy-demo:tests/test_branch.py`; a u-turn geometry test
including a counts wrap. Vehicle: diverter latch both ways, simultaneous-set rung order
(right wins), u-turn at a tag, **field coverage during a spin and in reverse**.

---

## Increment 4 — the product flag, the UI, the upgrade path

Deliberately last: building the flag early means every prior increment must keep it working.

**Where it lives.** A top-level scalar `"product"` (`"tape"` | `"slam"` | `"both"`) in the
profile via `_TOP_LEVEL_SCALARS` — one schema line, free `/params` display, free strict
validation (absent → refuses to boot). Not an env var: invisible on the UI, trivially
edited, and silently-unset is a proven failure mode here.

**Fix the `AGV_PROFILE` gotcha in the same increment.** `AGV_PROFILE` is set **nowhere** in
`amr_ws/deploy/`; every unit falls through to `DEFAULT_PROFILE = "agv-01"`, so two robots
would silently share one profile — and one licence. Add it to `amr.env`, require it in
`amr-supervisor.sh`, assert it in `validate.sh`, and log the resolved profile name,
product and profile sha256 at boot.

**How it reaches the supervisor — and why launch files never need to know.** The supervisor
*is* a node, so it may `from agv_core import config` directly. Enforce in exactly two
places: the allowed-target set in `_srv_mode` `:305`, and `_srv_survey` START `:369`
(refused under `tape`). A target the product disallows never reaches `START_NEW`, so the
layer launch files stay unconditional and `test_launch_layers.py`'s exact-set assertions
stay meaningful. Belt and braces: re-check inside `_start_navigation`/`_start_line` so a
bug cannot start an unlicensed layer. Publish it as `ModeState.product`.

**How it reaches the web.** New `GET /api/product` — deliberately **not**
`/api/capabilities`, because `/api/commissioning/capabilities` (`server.py:549`) already
means "what the blind-run form may offer". Pages must **404**, not be CSS-hidden: register
the slam-only closures (`/maps`, `/editor`, `/review`, `/run` and their APIs) only under
`slam`/`both`; add a `/line` page under `tape`/`both`. Name new APIs `/api/line/arm`,
`/api/line/mission`, … — **no URL may contain `cmd_vel`, `drive` or `jog`**
(`test_server.py:243`). Replace the flat 10-anchor nav in `base.html:12-23` with a loop over
a list injected by one Flask context processor.

**The two page-enumerating tests must be repaired properly, not patched.**
`test_pages_fonts_and_style_guards` `:421` and
`test_every_element_id_a_page_script_uses_exists_on_that_page` `:454` each iterate a literal
tuple of 11 paths, and the second concatenates `amr.js` into **every** page's id check — so
touching a rail tile id breaks all eleven. Derive the page list from
`client.application.url_map` and parametrise the fixture by product. For the rail itself,
split the updater into `rail-tape.js` / `rail-slam.js` so `amr.js` stays honest about what
it touches.

**The upgrade.** Edit `product`, restart. `config.load()` at module scope makes a restart
mandatory anyway. Ship `deploy/upgrade-product.sh`: rewrite the key → `validate.sh` →
`systemctl restart amr`.

**Security, stated honestly.** This is a **licensing** flag on a machine the customer owns
and can SSH into. It is not a security boundary and must not be documented as one. It buys:
no accidental or URL-guessed access to product B, no unlicensed layer even on a forged
request, visible and auditable licensing on `/params`, and a boot-logged profile sha256 so
tampering is *detectable*. It does not buy protection from a determined customer editing
one JSON key. **If you want a real boundary, ship product A with a slimmer install image**
— omit `amr_navigation`, `amr_localization`, `slam_toolbox`, `nav2_*`. Do both. One wrinkle:
`supervisor_node.py:32` has a module-level `from nav2_msgs.srv import ManageLifecycleNodes`
that would fail on a slim image — move it to a lazy import inside `_ready_navigation`.

---

## Increment 5 — U11, now unblocked

`gy-demo` ships as the interim tape product on its own image, so **U11 must wait until the
ported LINE mode is accepted on the vehicle**. Reorder it behind Increment 4 in
`unified_amr_service_plan.md`. Only then delete `canworker.py`, `main.py`, `app/`,
`amr_nav.service`, `amr_mapping.service` and `amr_legacy.env`.

`gy-demo:core/manualturn.py` / `/blind` stays deprioritised — it drags in `blindrun.py`
(already present as `agv_core/blindrun.py`) and is orthogonal to line following.

---

## Risks

**R1 — The MLS is Pre-operational. This is the schedule risk, not a detail.**
Nothing registers `0x18A` today, and the measured SDO fallback was **9.8 Hz** on
2026-09-19. That is not enough for a 50 Hz PID tuned with 20 ms of transport delay:
at ~102 ms intervals the D term (`TAU_D_S = 0.05`) differentiates data older than its own
filter. Mitigations in order: **(a)** NMT-start node 10 — `canopen.Link.nmt(cmd, node)`
already exists at `canopen.py:463`, so this is one line in `drive_node`'s bus-arm path
behind a new `mls_nmt_start` parameter (default false), and `mls_track.py:167` already
handles the tpdo↔sdo transition. **(b)** A **rate** gate: refuse to arm and HOLD below
`line_min_track_hz` (default 40) — `SENSOR_TIMEOUT_S` is a staleness gate and will not catch
a slow-but-fresh SDO stream. **(c)** If TPDO cannot be made to flow, Increment 1 stops at a
0.2 m/s bench demo and the product is not shippable at speed — say so now, not in
Increment 3. **(d)** Do **not** "fix" it by reading more SDO objects per bus tick;
`mls_track.poll()` deliberately does one read per tick so the drive setpoint is never
delayed.

> **Unresolvable from the repo:** the MLS TPDO1 transmission type and event timer. If TPDO1
> is event-driven-on-change, rate follows the tape and you win; if it is a 100 ms timer,
> TPDO buys latency but not rate. **Read `1800h:02` and `1800h:05` on the vehicle first.**

**R2 — `LineTrack` staleness and sequence gaps.** Fixed in Increment 1 by appending
`sample_age_s` and `seq`. Also: `SENSOR_DATA` is BEST_EFFORT depth 5 — measure the drop
rate at 50 Hz before trusting it. If nonzero, increase depth plus the R1(b) rate gate;
do **not** switch to RELIABLE, which adds retransmit latency to a control path.

**R3 — Two ramps in series.** The mux's `slew_asym` stacks on the engine's own ramp, so
measured stop distance will exceed what `a = v²/2d` predicts. Either set the mux's
autonomous limits above the engine's for LINE, or measure the real stop distance in
Increment 2 and put the margin in `stop_distance_m`. Do not leave both ramps tight and
then tune `stop_distance_m` blind. Related: pass **measured** `dt` to `update()`, never
`0.02`; and `autopilot.py:70` says it is not thread-safe, so run the line node on a single
-threaded executor or one `MutuallyExclusiveCallbackGroup`.

**R4 — Where the two products' safety arguments differ.** Mostly they do not: one
nanoScan3 in the base layer, obstacles are the safety chain's job in both modes, identical
BOM. Genuine differences: **(i)** speed — one field set sized for full speed is the stated
answer; record the number and its assumed stopping distance at Increment 2. **(ii)** spins
and reverse — `uturn.py` spins in place and the field may have asymmetric coverage;
confirm before Increment 3 goes on the floor.

**R6 — nanoScan3 output rate: `skip: 1` (34 -> 17 Hz) is safe for SLAM and AMCL, not for
the scan-age watchdog.** Studied 2026-09-21 against the tree. Where the scan goes:
`scan_gate_node` already thins `/scan_gated` to 10 Hz (`min_period_s 0.1`); slam_toolbox
thins again to 5 Hz (`minimum_time_interval 0.2`); AMCL updates on motion (`update_min_d
0.05 m / 0.03 rad`), not on scan arrival; the local costmap updates at 5 Hz. None of them
would notice. The one consumer that would is `localization_monitor_node`, which ages raw
`/scan` against `scan_age_limit_s = 0.15` (`readiness.py` `age_limits["scan"]`): at 34 Hz
one dropped UDP datagram is 58 ms (5x margin); at 17 Hz one drop is 118 ms and two are
176 ms -> LOST -> the vehicle stops. `/output_paths` rides the same datagram, so the
`field`/`estop` hold *classification* also slows to 59 ms - not a safety matter, the
OSSD -> FX3 -> STO path is hardwired and `skip` is data-output configuration, not the
verified safety configuration. Obstacle-layer smear at 0.60 m/s grows from 17 mm to 35 mm
per scan, inside the 0.05 m costmap cell. **Rule:** do not set `skip: 1` without measured
CPU pressure on the N97 (none recorded); if it is set, raise `scan_age_limit_s` and
`age_limits["scan"]` to 0.30 in the **same commit**, and re-run the localisation-loss test.

**R5 — The test-count pins will fight you, by design.** `tests/run_all.py` pins its module
list and `EXPECTED_CHECKS = 898` (93 test functions, post-`agv_core`);
`test_launch_layers.py:118` asserts exact executable sets and handler counts; and
`tests/test_layout.py` now pins the package boundary — no module inside `agv_core` may
import a sibling by bare name, which is exactly what a raw `gy-demo` port violates. Bump or
satisfy each in the **same commit** as the change, never a follow-up.

---

## Verification

```bash
# laptop / vehicle, every increment
cd ~/agv_can/amr_ws && env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test
env AMR_SIM_TESTS=1 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test   # layer-swap sims
cd ~/agv_can && python3 tests/run_all.py                              # Increments 1 and 4 only
cd ~/agv_can/amr_ws && ruff check --config ruff.toml src/             # from INSIDE amr_ws
```

> Run ruff from inside `amr_ws`: after the `agv_core` restructure, `ruff check amr_ws/src`
> from the repo root reports ~30 phantom `I001`s, and the root `pyproject.toml` excludes
> `amr_ws` for that reason.

| Inc | Laptop gate | Vehicle gate |
|---|---|---|
| 0 | name-coverage tests; `ros2 interface show` on the four appended msgs | one IDLE↔NAVIGATION swap still works |
| 1 | **`simulate(delay_s=0.040)` before any vehicle booking**; 14 ported control tests incl. the negative control; LINE mux precedence; full IDLE→LINE→IDLE sim | **read `1800h:02`/`:05` first**; jacked-up direction check; 5 m at 0.20 m/s, person at the E-stop; MANUAL/Reset kill; track-lost stop within 0.075 m; record `track_hz` |
| 2 | mission-store atomicity/rollback; station cursor overrun + generation loss; tag disjointness | two-tag loop; measured vs commanded stop distance; speed-zone entry/exit; RFID unplug → guard fault |
| 2b | `wait` round-trip; N-1 primitives; FSM wait -> PAUSED -> Start; auto_resume refused from a station pause; re-waits per pass | straight -> wait -> straight; Start in MANUAL ignored; E-stop during the wait resumes into the wait |
| 3 | ported branch tests; u-turn counts-wrap | diverter latch both ways; rung order; u-turn at a tag; field coverage in spin and reverse; horn follows |
| 4 | product-filtered page discovery from `url_map`; `/api/product`; unlicensed-target refusal | boot both images; B's pages 404 under `tape`; supervisor refuses NAVIGATION |

## Open items to resolve outside the code

1. MLS TPDO1 transmission type and event timer (`1800h:02`, `1800h:05`) — gates R1.
2. The commissioned nanoScan3 field set and its assumed stopping distance — record it.
3. Whether `/amr/line_track` drops at 50 Hz on BEST_EFFORT depth 5 — the `seq` field
   added in Increment 1 answers this on the first vehicle run.
4. Whether the N97 ever needs `skip: 1` on the nanoScan3 — measure CPU with all layers up
   before deciding; see R6 for what must change with it.
