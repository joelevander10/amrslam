# Agent prompt: verify dynamic areas, then build the line-follow / layout plan on the AGV PC

Paste everything below the line into the coding agent on the AGV PC (Ubuntu 22.04, ROS 2 Humble, repo at `~/agv_can`), once the laptop's working tree has been synced there.

---

You are on the vehicle PC of a 150 kg differential-drive AMR: two Oriental Motor BLV-R drives on `can0` (nodes 1 and 2) and a SICK MLS on node 10. The code was written on a Windows laptop that has no ROS, so parts of it have never run.

You have two jobs, in order:
- **Phase A**: verify and build the dynamic-areas change. This moves nothing.
- **Phase B**: implement the plan in `manuals/slam-generalized-plan/line-follow-u11-layout-plan.md`, step by step, with its tests.

Stop and report at each gate.

## Hard rules (all phases)

**Motion**
- **Do not move the vehicle.**
  - Do not press or simulate Start.
  - Do not call `/drives/arm`, `/amr/commissioning/plan` or `/amr/line_follow/arm`.
  - Do not publish on `/cmd_vel*`, `/amr/manual_command`, `/amr/line_cmd` or `/amr/commissioning_*`.
- Line following is **implemented and unit-tested only**. Its first drive on tape is check K6, done by the operator with someone at the E-stop.

**Service**
- Before any `sudo systemctl restart amr.service`, **ask the operator and wait for a yes**. A restart re-arms the drives: someone must be at the vehicle with the E-stop in reach and the selector on MANUAL.
- `agv_controller.service`, `amr_nav.service` and `amr_mapping.service` must not run beside `amr.service`. Check them; do not start them.

**Drives and profile**
- Do not write any drive or sensor object (SDO, `cansend`, MEXE02).
- Do not touch the `pp` section of `profiles/agv-01.json` or `guard.py`'s allow-list.

**Scope**
- **U11 deletions are forbidden in this session.** Deletion waits for the witnessed U10 acceptance. Prepare only (audit lists, test-migration notes).
- Do not commit or push unless the operator asks. Say exactly what you changed and why.

**On failure**
- Stop and report.
- You may fix a test-environment problem (a missing package, a stale overlay, CRLF endings) without asking.
- A fix to existing production code needs the operator's OK first. That means `amr_base`, `amr_mission`, `amr_navigation`, `amr_localization`, `amr_bringup`, `drivers/` and `config.py`.
- New files from the Phase B plan are yours to write.

## Where the tests are

| Location | What | How to run |
|---|---|---|
| `tests/` (repo root) | Legacy offline suite for the shared root modules (`config`, CAN helpers, DIO, RFID, panel, blind run, the legacy `canworker`/`app`). `tests/run_all.py` pins the module list and `EXPECTED_CHECKS` (869 today). | `python3 tests/run_all.py` |
| `amr_ws/src/<package>/test/` | ROS workspace tests, one folder per package: `amr_base`, `amr_bringup`, `amr_description`, `amr_localization`, `amr_maps`, `amr_mission`, `amr_navigation`, `amr_sim`, `amr_web` | `cd ~/agv_can/amr_ws && env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test` |
| `amr_ws/src/amr_web/test/*_harness.js` | The real page scripts run in Node against a fake DOM (editor, jog pad, blind run, review). Driven by `test_*_js.py`; they SKIP if `node` is missing. | Run as part of the pytest command above. |

## Phase A — verify the dynamic-areas change (Increment 1 of `dynamic-mapping.md`)

**What changed (uncommitted on branch `slam-roadmap`):**
- Operators mark "dynamic areas" (trolleys, parked forklifts) on a saved map. Maps → **Edit areas** opens the new `/review` page, where they drag rectangles; saving derives a new revision.
- Routes through mapped clutter inside such an area validate as `provisional` (an `info` issue).
- During a run, any scan return inside the area counts as an obstacle.
- The localisation monitor ignores beams into dynamic areas.
- AMCL can optionally localise against a blanked copy of the map (`AMR_LOC_BLANK_DYNAMIC`, default false), served on `/map_loc` by a second map server.

**New files:**
- `amr_maps/amr_maps/edit.py`
- `amr_mission/amr_mission/map_edit.py` (console script `map_edit`)
- `amr_web/amr_web/templates/review.html`, `amr_web/amr_web/static/review.js`
- tests: `amr_maps/test/test_edit.py`, `amr_web/test/review_harness.js`, `amr_web/test/test_review_js.py`

**Changed files:**
- `amr_maps/grid.py` (`rasterize_polygon`, `mask_misalignment` moved here)
- `amr_navigation/footprint.py`, `validate.py` (`Issue.severity`, `provisional`, `load_dynamic`)
- `amr_mission/map_bundle.py` (`derive_edit`, `load_mask`, `load_edits`, `dynamic.*`/`edits.json` in `verify`)
- `route_executor_node.py` (`explained_by_map`)
- `amr_localization/scan_consistency.py`, `localization_monitor_node.py` (`dynamic_yaml` parameter)
- `amr_bringup/launch/navigation_layer.launch.py` (`blank_dynamic`, `map_server_loc`)
- `amr_web/server.py`, `editor.js`, `maps.js`, `mapview.js`, `amr.css`
- `amr_mission/setup.py`, `deploy/amr.env`, `RUNBOOK.md` §A2

**What the laptop could and could not run:**
- **Never executed anywhere:**
  - `navigation_layer.launch.py` with a dynamic mask, and its three new tests at the end of `amr_bringup/test/test_launch_layers.py`;
  - `localization_monitor_node.py`.
- **Run only against fake ROS modules:** the two new executor tests at the end of `amr_mission/test/test_route_executor_logic.py`.
- **Failed on Windows for known platform reasons, and must pass here:**
  - route/mission store tests (file locks);
  - the symlink test in `test_map_bundle.py`;
  - the `SIGALRM` tests in `test_grid.py`;
  - `test_edit_endpoint_derives_a_revision_and_validation_turns_provisional` in `amr_web/test/test_server.py`.

**Steps:**
1. **Inspect the tree.**
   ```bash
   cd ~/agv_can && git status --short && git log --oneline -3
   git ls-files --eol | grep -E 'w/(crlf|mixed)' || echo "no tracked CRLF"
   git ls-files --others --exclude-standard | xargs -r file | grep CRLF || echo "no untracked CRLF"
   ```
   Fix tracked CRLF with `git add --renormalize .` (no commit) and untracked files with `sed -i 's/\r$//' <file>`. The new files show as untracked (`??`): check that the seven listed above are present.
2. **Legacy suite:** `python3 tests/run_all.py`. Expected: `869 check(s)`, all passed. The root tests are unchanged by Phase A.
3. **Build:**
   ```bash
   cd ~/agv_can/amr_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash
   ros2 pkg executables amr_mission | grep map_edit
   ```
4. **Workspace tests and lint:**
   ```bash
   which node || echo "node missing: JS tests will SKIP (report it)"
   env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test
   ruff check src && ruff format --check src && ./deploy/validate.sh
   ```
   Pay attention to:
   - `test_launch_layers.py`: `test_dynamic_mask_reaches_the_monitor…`, `test_blank_dynamic_serves_amcl…`, `test_blank_dynamic_refuses…`. The walker reads `c["params"]["localization_monitor_node"]`; if launch_ros stores parameters in another shape, fix the **test**, not the launch file.
   - `test_route_executor_logic.py`: `test_explained_by_map…`, `test_obstruction_blocks_on_a_mapped_trolley…`.
5. **Offline CLI check** against a **copy** of the real maps, never `~/amr_maps` itself:
   ```bash
   rm -rf /tmp/maps_copy && cp -r ~/amr_maps /tmp/maps_copy && ls /tmp/maps_copy
   ros2 run amr_mission map_edit --maps-dir /tmp/maps_copy --map <an id> --rev <its latest> --dynamic-rect <x0 y0 x1 y1 inside the map> --note test
   ```
   Expected: a new `rev N+1` with `dynamic.pgm`, `dynamic.yaml` and `edits.json`, and the parent unchanged.
6. **Gate A. Stop for the operator.** Report the step table. Then run `systemctl is-active agv_controller amr_nav amr_mapping amr` and ask: "OK to restart amr.service? Someone at the vehicle, E-stop in reach, selector MANUAL?"
7. **After a yes: read-only checks.**
   ```bash
   sudo systemctl restart amr.service && journalctl -u amr.service -n 80 --no-pager
   curl -s -o /dev/null -w '%{http_code}\n' localhost:5001/review     # 200
   curl -s localhost:5001/api/maps | python3 -m json.tool | grep -E 'dynamic_cells|edits' | head
   ```
   Ask the operator to open Maps → **Edit areas** in a browser. Check that rectangles draw and that **Save as new revision** creates a revision; save only if the operator agrees, since it adds a revision to the real maps directory. Report what they saw.

## Phase B — implement `line-follow-u11-layout-plan.md`

Read the whole plan first. Follow its **Order of work**. After each step:
- run the tests the plan names for that step, plus the affected existing tests;
- run lint;
- report.

**Step 1 — Part 1.1: MLS track reading, no motion.**
- Restore `drivers/canbus/read_mls.py` from `git show 8403dcb^:drivers/canbus/read_mls.py`, trimmed as the plan says.
- Add `amr_base/mls_track.py`, the `LineTrack.msg` message, the `/amr/line_track` topic and the `mls/track` diagnostics status in `drive_node.py`.
- Add `tests/test_mls.py` to `tests/run_all.py`. Raise `EXPECTED_CHECKS` deliberately, with a comment.
- Add `amr_base/test/test_mls_track.py`.
- **Gate B1:**
  - with the operator's OK to restart (same question as Gate A), check that `candump -n 20 can0,18A:7FF` shows frames and that Monitor shows `mls/track`;
  - if there are no frames because the MLS is in Pre-operational, **do not send NMT**. Report it: the operator decides between an NMT start for node 10 only and SDO polling, and the plan's §1.1 records the answer.

**Step 2 — Part 3: plant-layout background in the route editor.**
- `amr_navigation/layout.py`, the layout endpoints in `server.py`, and the Layout block in `editor.html`, `editor.js` and `mapview.js`.
- Tests: `test_layout.py`, the `test_server.py` extensions, and `layout_harness.js` with `test_layout_js.py`.
- Layout files live in `~/amr_maps/<map>/layouts/rev<N>/`. Confirm that `mb.verify()` of every existing revision is unchanged afterwards.

**Step 3 — Parts 1.2 and 1.3: controller and authority.**
- `line_follow.py`, `line_follow_node.py` and `LineFollowState.msg`.
- `LEASE_LINE = 8` in `supervisor_node.py`, `mode_fsm.py` and `gating.py`.
- The `LINE` source in `cmd_mux_kinematics_node.py`, and the node in `base.launch.py`.
- Tests:
  - `test_line_follow.py` and `test_line_follow_node.py`;
  - the `test_gating.py` extensions;
  - the supervisor and mode-FSM extensions;
  - the `test_launch_layers.py` extension.
- **Gate B3: no arming, no Start, no K6.** Report test results. The operator runs K6.

**Step 4 — Part 2: U11 preparation only.**
- Run the import audit from §2.1 and §2.2: for each delete candidate, give the `grep` evidence of no importer.
- Draft the `tests/run_all.py` migration note from §2.3.
- Write both into a new section at the end of the plan doc.
- **Delete nothing.**

**Docs** (as each part lands):
- `amr_ws/RUNBOOK.md`: engineering line-follow section, CLI only, with the safety note from §1.3; Routes → Layout.
- `unified_amr_service_plan.md` §12: P9, K6 and K7 as written in plan §4.2.

## Report back

**After each gate:**
- a table with one row per step: step, pass/fail, key output line;
- every failure, with the full traceback;
- every file you changed or created, and why;
- whether `node` was present.

**After Gate B1:**
- the `candump` result;
- the MLS NMT state;
- a Monitor screenshot or the `mls/track` keys.

**At the end:**
- the final `EXPECTED_CHECKS` value, and how it was derived;
- any place where the code differed from what the plan or this prompt assumed. Fix the code or the plan and say which.
