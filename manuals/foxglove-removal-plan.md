# Foxglove removal plan

Status: IMPLEMENTED in the working tree, not yet committed or deployed (2026-11-03). Branch
`kyb-trackless` only; `gvievo-main` is a separate project and keeps its own copy. Sections
5-6 (tests on Ubuntu, vehicle deployment) are still to do.

## 1. Summary

Remove `foxglove_bridge` from the vehicle software: the supervisor stops starting it, the
standalone launch files lose their `foxglove:=` argument, the package drops the
dependency, and the docs stop pointing to it. Operators lose nothing they use today; the
web UI already covers the initial pose, the live scan and the map. Engineers lose the
remote 3D view, most notably of the local costmap.

| Where | References | What changes |
| --- | --- | --- |
| Supervisor and deploy | `supervisor_node.py` (9), `amr-supervisor.sh`, `amr.env` | No fourth process group; `AMR_FOXGLOVE` and `-p foxglove:=` go |
| Standalone launch files | `launch_helpers.foxglove()`; `drivers`, `lidar`, `mapping`, `nav`, `sim` `.launch.py` | Helper and every `foxglove` argument go |
| Package and tests | `amr_bringup/package.xml`; `test_unified_sim.py`, `test_fault_injection_sim.py` | `exec_depend` and the `foxglove:=false` arguments go |
| Text only | `agv_core/alarms.py` (BOOT_ERROR), `domains.py`, `cyclonedds-local.xml`, `base.launch.py`, `navigation_layer.launch.py` | Wording |
| Docs | `amr_ws/README.md` (15), `amr_ws/RUNBOOK.md` (6), `alpha-setup.md` (5), `manuals/gls6-qr/` (2) | Rewritten to the web UI and bags |
| Left as history | `manuals/obsolete/`, `field-logs/`, `trackless-only-removal-plan.md` | Unchanged: they record what was true then |

## 2. Why

- **An open, unauthenticated port.** The bridge listens on `0.0.0.0:8765` on every
  interface, in every session (`field-logs/2026-11-03/amr-logs/foxglove.log*`: version
  3.6.0, started with each of the seven sessions from 11:52 on 3 November; the earlier
  logs had rotated out). It exposes every ROS topic to anyone on the network.
  By default `foxglove_bridge` also lets clients publish and call services; verify which
  capabilities 3.6.0 enables, but the removal makes the question moot.
- **CPU and memory on the N97** in every session, for a process group the supervisor must
  spawn, respawn and tear down.
- **No operating procedure depends on it.** The RUNBOOK mentions it only as part of the
  boot topology and the shutdown check.

The logs show no client connections, but 3.6.0 may not log them, so they cannot prove
nobody used it (see Decisions).

## 3. What is lost, and what replaces it

| Used Foxglove for | Replacement |
| --- | --- |
| Live map, scan and odometry view (sim and vehicle) | Web Run page: `mapview.js` and `liveview.js` already draw the map and the live scan |
| "Publish pose estimate" on `/initialpose` in the sim walkthrough | Web Run page sets the initial pose (`adapter.py`, `run.js`) |
| Scanner commissioning view (`lidar.launch.py foxglove:=true`) | Start the normal service and use the web live view; field geometry stays in SICK Safety Designer |
| Local costmap view (what showed phantom obstacles) | None in the web today. Record `/local_costmap/costmap` in a bag and inspect it offline; follow-up: a costmap overlay in the web live view |
| Topic inspection, TF tree | SSH: `ros2 topic echo`, `ros2 run tf2_tools view_frames` (tf2-tools is in the alpha-setup apt list) |
| Viewing recorded bags | Unchanged: Foxglove Studio on a laptop opens `.mcap`/`.db3` bags offline without the bridge |

DDS stays confined to the vehicle (`cyclonedds-local.xml`), so after the removal no ROS
data leaves the vehicle except through the web UI on :5001.

## 4. Changes

One commit per phase, so each can be reviewed and reverted on its own.

**P1. Supervisor and deploy**

1. `amr_bringup/amr_bringup/supervisor_node.py`: drop the `foxglove` parameter, the spawn
   block (`self._respawnable["foxglove"]`, `self.groups["foxglove"]`, `foxglove.log`),
   `"foxglove"` from the teardown order `("layer", "base", "foxglove", "web")`, and the
   Foxglove wording in the module docstring ("four process groups" becomes three) and in
   the respawn comment.
2. `amr_ws/deploy/amr-supervisor.sh`: drop `-p foxglove:="${AMR_FOXGLOVE:-true}"`.
3. `amr_ws/deploy/amr.env`: drop `AMR_FOXGLOVE=true`. Change it together with step 2:
   `validate.sh` checks that every variable the unit uses is defined.

**P2. Standalone launch files**

1. `amr_bringup/amr_bringup/launch_helpers.py`: delete `foxglove(condition)`.
2. `drivers.launch.py`, `lidar.launch.py`, `mapping.launch.py`, `nav.launch.py`,
   `sim.launch.py`: remove the `foxglove` import, the `DeclareLaunchArgument("foxglove", ...)`
   and the `foxglove(IfCondition(...))` include, and "(+ Foxglove)" from each docstring.

**P3. Package and tests**

1. `amr_bringup/package.xml`: remove `<exec_depend>foxglove_bridge</exec_depend>`.
2. `test_unified_sim.py` and `test_fault_injection_sim.py`: remove `foxglove:=false` from
   the supervisor arguments.
3. Add a guard, `amr_bringup/test/test_no_foxglove.py`: no code or deploy file under
   `amr_ws/src`, `amr_ws/deploy` or `agv_core` mentions `foxglove`, so it cannot creep back
   unnoticed. It imports no ROS, so it runs on any machine (not in `test_launch_layers.py`,
   which needs ROS to collect).

**P4. Text**

1. `agv_core/alarms.py`, `BOOT_ERROR` detail: "spawning the base/web/foxglove groups"
   becomes "spawning the base/web groups". Regenerate RUNBOOK section 4 with
   `python3 -m agv_core.alarms --md`; never edit that table by hand.
2. Comments: `domains.py` (L15), `cyclonedds-local.xml` (L4-5), and the "No Foxglove"
   phrases in the `base.launch.py` and `navigation_layer.launch.py` docstrings.

**P5. Docs**

1. `amr_ws/README.md`: the launch table (remove `[foxglove]` and "(+ Foxglove)"), the sim
   walkthroughs (replace the Foxglove 3D-panel and "Publish pose estimate" steps with the
   web Run page), the scanner-commissioning line, and the DDS note (L388-389).
2. `amr_ws/RUNBOOK.md`: L4 boot topology, L67 process groups, L74 and L734 commands
   (`pgrep -af 'amr_|nav2|slam_toolbox'`), and L846 (the bags/layouts note stays as
   history, reworded).
3. `alpha-setup.md`: drop `ros-humble-foxglove-bridge` from the apt list (L580) and
   `AMR_FOXGLOVE` from the env table (L727), and the `:8765` mentions (L792, L1232, L1270).
4. `manuals/gls6-qr/gls6-system.md` L239 and `gls-wait-coding-plan.md` L524: "visible in
   the web and in Foxglove" becomes "visible in the web and in TF".

## 5. Tests before the vehicle

| Check | Runs on | Expectation |
| --- | --- | --- |
| `git grep -i foxglove -- amr_ws/src amr_ws/deploy agv_core` | any machine | no hits |
| `ruff check agv_core tests && (cd amr_ws && ruff check src)` | any machine | clean |
| `bash amr_ws/deploy/validate.sh` | Ubuntu | passes (env and unit agree) |
| `colcon build --packages-select amr_bringup` | Ubuntu with ROS | builds without the dependency |
| `pytest src/amr_bringup/test/test_no_foxglove.py` | any machine | passes |
| `pytest src/amr_bringup/test` | Ubuntu with ROS | passes |
| `AMR_SIM_TESTS=1 pytest src/amr_bringup/test/test_unified_sim.py` | Ubuntu with ROS | the sim boots to IDLE with three groups |
| `python3 tests/run_all.py` | Ubuntu | pinned count unchanged (text-only change in `alarms.py`) |

## 6. Vehicle deployment

1. With `amr.service` stopped: `git pull` on `kyb-trackless`, then
   `colcon build --symlink-install --packages-select amr_bringup`. `amr.env` is read from
   the repo (`EnvironmentFile=` in `amr.service`), so the pull updates it.
2. `sudo systemctl start amr.service`, then check:
   - [ ] `journalctl -u amr.service` shows `spawned base`, `web`, no `foxglove`, and reaches IDLE
   - [ ] `ss -ltnp | grep 8765` prints nothing
   - [ ] `pgrep -af foxglove` prints nothing
   - [ ] `systemctl stop amr.service` logs `teardown: done` with no foxglove group
   - [ ] one navigation run and one mapping session work as before
3. Old `~/.amr/logs/foxglove.log*` stop growing; delete them, or leave them to age out.
4. Keep the `ros-humble-foxglove-bridge` apt package installed for a month as a manual
   fallback (`ros2 launch foxglove_bridge foxglove_bridge_launch.xml` by hand, over SSH).
   Then `sudo apt remove ros-humble-foxglove-bridge`.

## 7. Decisions and risks

| Item | Recommendation |
| --- | --- |
| Is any engineer still using Foxglove on the vehicle? The logs cannot tell | Ask before P1; if yes, agree the SSH fallback in 6.4 |
| Costmap visibility is gone until the web shows it | Record `/local_costmap/costmap` in field bags; plan the web overlay as a follow-up |
| When to uninstall the apt package | One month after deployment, if the fallback was not needed |
| Someone types `foxglove:=true` out of habit | Launch ignores an undeclared argument; the docs no longer show it |
