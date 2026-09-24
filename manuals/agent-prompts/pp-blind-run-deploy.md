# Agent prompt: verify and deploy the PP blind-run change on the AGV PC

Paste everything below the line into the coding agent on the AGV PC (Ubuntu, ROS 2 Humble, repo at `~/agv_can`) after pulling the branch.

---

You are on the vehicle PC of a 150 kg differential-drive AMR (two Oriental Motor BLV-R drives on `can0`, nodes 1 and 2). The code was written and unit-tested on a Windows laptop that has no ROS. This is the first time it runs on Ubuntu with ROS. Your job is to **verify, build and deploy it without moving the vehicle**, and report back.

## What changed (branch `slam-roadmap`)

1. **Blind-run test page** (`/commissioning` on port 5001):
   - A form for one move per plan: straight, rotate or arc, with speed up to 0.8 m/s and a backend PV or PP.
   - A tape-measurement entry and a commanded-vs-measured history.
   - Files: `amr_ws/src/amr_web/amr_web/{templates/commissioning.html, static/blindrun.js, commissioning_log.py, server.py, adapter.py, web_node.py}`.
2. **Profile position (PP) backend**:
   - Files: `amr_ws/src/amr_base/amr_base/pp.py` (new), `canopen.py` (DriveLink `pp_enter`, `pp_exit`, `send_controlwords`, `in_pp`), `drive_node.py`, `commissioning.py`, `commissioning_node.py`.
   - New messages `PpMove.msg` and `PpStatus.msg`, plus 2 new fields in `CommissioningState.msg`.
   - **PP is LOCKED** in `profiles/agv-01.json` (`pp.enabled: false`, all `pp.expect` values `null`). It must stay locked during this step: the motor (BLMR6400SKM-GFV-B, 400 W, 1:30 gearhead) is unlocked separately, by `pp-profile-fill.md`, after the drive settings are in place.
3. **Stop-path fixes for pp mode** (in `canopen.py`):
   - Stop frames carry Halt while in pp.
   - Standstill in pp is judged on 606Ch = 0, not on statusword bit 12 (which means set-point acknowledged in pp).
4. **Motion extension**: `drivers/canbus/drive_forward.py` `CW_ENABLE` is now `0x000F` (it was `0x200F`, PV normal mode), as the manual requires for a 400 W motor with a gear. Both `drive_node` (ROS) and `canworker` (legacy) arm with it.
5. **Write guard and config**:
   - `guard.py` now allows writes to 607Ah and 6081h only. The pp safety objects stay non-writable.
   - `config.py` adds `blind_run.max_speed_mps` and the `pp` section.
   - `tests/run_all.py` `EXPECTED_CHECKS` is now 869.

## Hard rules

- **Do not move the vehicle.** Do not press Start, do not call `/drives/arm` or `/amr/commissioning/plan`, and do not publish on `/cmd_vel*`, `/amr/manual_command` or `/amr/commissioning_pp`.
- **Do not unlock PP.** Do not edit the `pp` section of the profile or `guard.py`'s allow-list, and do not write any drive object (SDO, `cansend` or MEXE02).
- Before `sudo systemctl restart amr.service`, **ask the operator** and wait for a yes. A restart re-arms the drives (energised, holding zero); someone must be at the vehicle with the E-stop in reach.
- `agv_controller.service` must not run beside `amr.service`: both would own `can0`. Check it; do not start it.
- If a step fails, **stop and report**. You may fix a test-environment problem (a missing Python package, a stale overlay), but not production code in `amr_base`, `drivers/` or `config.py` without asking. Say exactly what you changed.

## Steps

1. **Pull and inspect.**
   ```bash
   cd ~/agv_can && git status && git log --oneline -3
   git ls-files --eol | grep -E 'w/(crlf|mixed)' || echo "no CRLF in working tree"
   ```
   The code came from Windows. Any `w/crlf` or `w/mixed` file is a problem: report it and fix it with `git add --renormalize .`, without committing unless asked.
2. **Legacy offline suite.**
   ```bash
   python3 tests/run_all.py
   ```
   Expected: `87 test function(s), 869 check(s)` and `all checks passed`. The owner-lock checks that fail on Windows must pass here.
3. **Build the overlay.** New messages, so build and re-source:
   ```bash
   cd ~/agv_can/amr_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 interface show amr_interfaces/msg/PpMove | head -5
   ros2 interface show amr_interfaces/msg/PpStatus | head -5
   ros2 interface show amr_interfaces/msg/CommissioningState | tail -3   # backend, run_id
   ```
4. **ROS workspace tests, lint, deploy check.**
   ```bash
   which node || echo "node missing: the blindrun/editor/jogpad JS tests will SKIP (report it)"
   env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/*/test
   ruff check src && ruff format --check src
   ./deploy/validate.sh
   ```
   Expected: no failures. The new test files are:
   - `amr_base/test/test_pp.py`, `test_pp_drivelink.py`, `test_commissioning_pp.py`;
   - `amr_web/test/test_blindrun_page.py`.

   `drive_node.py` and `commissioning_node.py` have NOT been executed anywhere yet (Windows has no rclpy). An import error, a wrong message field name or an attribute error in them shows up here or in step 6. Report it with the traceback.
5. **Stop for the operator.** Run the checks below and report their output, then ask: "OK to restart amr.service? Someone at the vehicle, E-stop in reach, selector MANUAL?"
   ```bash
   systemctl is-active agv_controller amr_nav amr_mapping amr
   ```
6. **After a yes: restart and read-only checks.**
   ```bash
   sudo systemctl restart amr.service
   journalctl -u amr.service -n 80 --no-pager            # look for tracebacks in drive_node / commissioning_node / web
   source ~/agv_can/amr_ws/env/vehicle.sh
   ros2 topic echo --once /drives/pp_status              # available: false, reason: "pp locked in the profile (pp.enabled false)"
   ros2 topic echo --once /amr/commissioning_state       # backend: pv, phase 0
   ros2 topic echo --once /diagnostics | grep -A1 -E 'pp_state|pp_in_mode|pp_last'   # idle / False / ""
   curl -s localhost:5001/api/commissioning/capabilities | python3 -m json.tool   # pp_available false, max_speed_mps 0.8
   curl -s localhost:5001/api/commissioning/history
   curl -s -o /dev/null -w '%{http_code}\n' localhost:5001/commissioning   # 200
   candump -n 6 can0,201:7FF,202:7FF     # passive listen: RPDO1 bytes 0-1 must be "0F 00" (motion extension), not "0F 20"
   ```
   Also check `~/.amr/logs/base.log` and `~/.amr/logs/web.log` for tracebacks.

## Report back

Give a table with one row per step: step, pass/fail, and the key output line. Then list separately:
- any failure, with the full traceback;
- anything you changed, and why;
- whether node was present (JS tests ran or skipped);
- the `candump` bytes and the `/drives/pp_status` reason.

Do not start the floor test. The PV baseline runs are done by the operator from the page.
