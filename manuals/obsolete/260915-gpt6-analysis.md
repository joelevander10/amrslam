# Development review: ROS 2 readiness and controller risks

**Date:** 2026-09-15  
**Scope:** Device inspection and code review, starting from [install-list.md](manuals/slam-generalized-plan/install-list.md).  
**Context:** This is an unfinished product under active development. Missing planned features are backlog, not automatically defects.

The investigation used read-only commands, existing logs/test results, and a read-only controller API request. No hardware commands, package changes, builds, service restarts, or controller tests were initiated. This report is the only requested file change. Other development sessions were active, so runtime observations and test artifacts are snapshots rather than guarantees about the latest working tree.

## 1. Assessment

**The device already has a usable ROS 2 Humble development foundation.** The immediate problems are integration consistency and controller failure/recovery behavior, rather than missing ROS installation or an obviously inadequate computer.

Priorities:

1. Correct the TF tree and Python environment conflict so development results are trustworthy.
2. Resolve partial-arm cleanup, disarm cancellation, and fault gating before reusing the controller in autonomous motion.
3. Validate CAN transaction matching, worst-case timing, and the actual adapter transport before increasing traffic.
4. Complete sensor timestamps, measured geometry, hardware wrappers, and navigation integration incrementally.

Evidence labels used below:

- **Observed:** Read directly from this device, its logs, or existing artifacts.
- **Static finding:** Supported by code paths; not reproduced on physical hardware.
- **Potential risk:** A failure mechanism or missing measurement that needs validation.
- **Backlog:** Expected unfinished implementation.

## 2. Device and software baseline

| Area | Evidence from inspection | Assessment |
|---|---|---|
| OS | Ubuntu 22.04.5, kernel 5.15.0-187-generic, Python 3.10.12 | Matches the reconciled Humble plan. |
| CPU/RAM | Intel N97, 4 cores/threads; approximately 7.5 GiB usable RAM | Suitable for continued development; full navigation capacity remains unmeasured. |
| Storage | Approximately 75 GiB available on the root filesystem | Enough for initial work; establish rosbag retention limits before prolonged recording. |
| Temperature | Approximately 62°C; exposed throttle counters were zero | No throttling demonstrated. This was not a full navigation soak. |
| CPU policy | `intel_pstate`, `powersave`, boost enabled | `powersave` does not mean a fixed low clock. Measure before changing policy. |
| ROS packages | Humble base, CycloneDDS, Nav2, SLAM Toolbox, robot_localization, SICK driver, Foxglove, robot_state_publisher, MCAP and development tools installed | The installation document's “nothing installed” statement is outdated. |
| Dependency resolution | `rosdep check --from-paths amr_ws/src --ignore-src --rosdistro humble` reported all system dependencies satisfied | Declared dependencies resolve; this does not establish runtime compatibility. |
| Workspace | Five source/build/install packages: interfaces, base, simulation, description, bringup | Implementation is already beyond a package-installation stage. |
| Python/ROS basics | `rclpy`, message tooling and custom messages imported; custom message serialization round trip worked | Core bindings and generated interfaces are usable. |
| Clock | Chrony running and synchronized; approximately 3 ms system offset in the sample | Host synchronization works. Sensor clock alignment is a separate task. |
| Existing controller | Armed in MANUAL with zero targets and measured zero speed; sampled loop average 20.25 ms, maximum 21.57 ms | Useful evidence of nominal operation, not a failure-response timing bound. |
| Existing lidar | Approximately 34 Hz, 1,652 beams, no gaps/errors in the sampled state | Legacy reception works; this is not proof of ROS scan/fusion readiness. |

## 3. Confirmed integration issues

### R01 — Simulation launch gives `base_link` competing TF parents

**Static finding — fix during current ROS development.**

- [amr.urdf.xacro](amr_ws/src/amr_description/urdf/amr.urdf.xacro) defines the fixed transform `base_footprint → base_link`.
- [diff_drive_odom_node.py](amr_ws/src/amr_base/amr_base/diff_drive_odom_node.py) defaults to `odom` and `base_link`, with TF publishing disabled.
- [sim.launch.py](amr_ws/src/amr_bringup/launch/sim.launch.py) enables TF publishing without changing that child frame.
- The composed launch therefore requests both `base_footprint → base_link` and `odom → base_link`. A TF child needs one parent; navigation and visualization cannot rely on this tree.

**Next action:** Choose one consistent tree and authority for each transform. With the existing URDF arrangement, use `odom → base_footprint → base_link`, checking odometry frame IDs and height conventions together. Later, ensure the EKF replaces rather than duplicates the odometry TF publisher.

**Acceptance:** Launch the composed system and verify one parent and one intended publisher per transform. Individual URDF and odometry unit tests do not establish this.

### R02 — User Python packages conflict with ROS image bindings

**Observed — isolate before using affected image-processing paths.**

- The default Python environment selected user-installed NumPy 2.2.6 and `opencv-python-headless` 5.0.0.93.
- Importing `cv_bridge` emitted the NumPy ABI incompatibility diagnostic and `AttributeError: _ARRAY_API not found`. The top-level import nevertheless returned, so an import-only smoke test can miss the problem.
- With `PYTHONNOUSERSITE=1`, system NumPy 1.21.5, OpenCV 4.5.4, and `cv_bridge` imported cleanly.
- The installed headless OpenCV package requires NumPy 2 on this Python version. Downgrading NumPy alone would create another dependency conflict.

**Impact:** This does not establish that the current CAN controller or lidar path is broken. It does establish that the shared Python environment is unsuitable for blindly combining all installed ROS and image packages.

**Next action:** Isolate optional image/map tooling, or select a mutually compatible NumPy/OpenCV/ROS package set. Do not globally disable user packages without accounting for the controller's user-installed CAN, Modbus and Flask dependencies. Test actual image conversions if that functionality is used.

### R03 — The observed Foxglove process did not have the workspace overlay

**Observed — runtime environment issue.**

- The bridge process inspected during the audit had only `/opt/ros/humble` in `AMENT_PREFIX_PATH`.
- Its launch log repeatedly reported that `amr_interfaces` could not be found when loading custom schemas.
- A newly sourced shell could resolve the workspace packages. Editing `.bashrc` does not update a process that is already running.

**Next action:** At the next authorized launch, start Foxglove from the same explicit underlay/overlay environment as the application. Apply the same rule to future systemd services.

**Acceptance:** Open a custom-message topic in Foxglove and confirm schema loading and message decoding, not merely WebSocket connectivity.

## 4. Controller findings to address before autonomous reuse

These are static code findings. No physical fault injection was performed.

### C01 — Partial arming can bypass cleanup

**High priority.** In [canworker.py](canworker.py), `_do_arm()` enables the drives sequentially but sets `_armed = True` only after completing both drives and reading encoder scale. `_do_disarm()` returns immediately when `_armed` is false.

**Trigger:** One drive reaches Operation enabled, then another step fails. The explicit statusword-failure cleanup can return without disabling the already enabled drive. Exceptions during writes also lack unconditional rollback covering the whole transition.

**Consequence:** Software can report unarmed while a drive remains enabled. This does not by itself prove unintended movement: arming writes a zero target, but state ownership and recovery are inconsistent.

**Next action:** Track the transition or attempt bounded cleanup independently of the final `_armed` flag. Preserve cleanup failures in diagnostics.

**Acceptance:** Inject failure at each arm step and verify cleanup attempts for both drives, final software state, and retry behavior.

### C02 — Disarm does not cancel an active or delayed blind run

**High priority.** `_do_disarm()` does not clear `_blind` or `_blind_start_at`. The worker handles queued actions before the panel scan; MANUAL level-based auto-arm can re-arm before the later blind-run eligibility check.

**Trigger:** A web disarm occurs during a blind run while MANUAL auto-arm remains active.

**Consequence:** Depending on timing and drive status, the same run can survive disarm and resume without a new physical Start. This is a reachable sequencing concern, not a hardware-reproduced event. `halt()` already requests blind-run cancellation, so the stop paths differ.

**Next action:** Centralize cancellation for active and pending autonomous actions during disarm. Keep automatic energizing, if intended, separate from permission to resume a run.

**Acceptance:** Test active and delayed runs through web disarm, panel transitions, and automatic re-arm. Require a fresh Start for a canceled run.

### C03 — Manual commands can overwrite a latched health stop

**High priority.** `drive()` checks mode, `_armed`, and blind-run activity, but does not reject `_fault` or `_health.system_error`. `_apply_health()` zeroes the target on a health transition without clearing `_armed`; it is not a continuous output gate.

**Trigger:** A health fault zeroes the target, then continued browser jog requests arrive while software remains armed.

**Consequence:** A subsequent request can replace the zero target despite the latched fault. Whether a physical drive responds depends on the underlying fault and recovery, but the software acceptance gap is present.

**Next action:** Define one motion-permission rule and enforce it at command acceptance and final output. Preserve the deliberate policy that a manual keepalive timeout is nonlatching; it is a different condition from a latched device fault.

**Acceptance:** Test repeated jog requests during device loss, communication recovery without acknowledgement, and the intended acknowledgement/re-arm sequence.

### C04 — SDO replies are not matched to the requested object

**Priority before increasing CAN traffic.** [verify_drivers.py](drivers/canbus/verify_drivers.py) and [drive_forward.py](drivers/canbus/drive_forward.py) accept replies based on the node response ID and command byte without validating the echoed index/subindex and complete frame shape.

**Trigger:** A delayed reply from an earlier request arrives after the receive queue was drained and while a different object request is pending.

**Consequence:** The earlier reply can be treated as the current transaction's result. Malformed short replies can also reach indexing/unpacking paths. Draining the queue before sending does not eliminate late replies.

**Next action:** Validate response ID, frame length, supported command specifier, index and subindex. Use a monotonic timeout consistently; the write helper currently uses wall-clock time.

**Acceptance:** Fake-bus tests with delayed replies for other objects, malformed frames, aborts, and valid responses. Test fixtures must include realistic echoed object addresses.

### C05 — The 0.6-second watchdog is not a demonstrated worst-case stop bound

**Potential risk.** The same worker performs command handling, blocking SDO transactions, telemetry and watchdog evaluation. SDO reads/writes can wait approximately 0.4/0.5 seconds each, and `_drain_queue()` is unbounded. Slow diagnostics can therefore delay the next watchdog evaluation substantially beyond the nominal 20 ms loop period.

**Next action:** Establish a failure-path timing budget. Restrict slow diagnostics while moving, bound command processing, and document the drive's independently configured communication-loss behavior. Review the health-history reset after slow work so “previously missing” does not unintentionally become an acceptable “never seen” state.

**Acceptance:** Measure command-loss and device-loss response under transaction timeouts and queued work. Nominal averages and rolling maximums from healthy operation are insufficient.

## 5. Hardware integration risks and assumptions

| Finding | Evidence and potential issue | Next development step |
|---|---|---|
| CAN transport differs from a native USB-CAN assumption | `can0` is provided by `slcand` through `/dev/canable0 → ttyACM0`; service option `-s4` selects 125 kbit/s. SocketCAN access does not imply `gs_usb` firmware. | Make bitrate/PDO migration instructions match the actual transport. Changing a profile value does not reconfigure an existing interface. Validate throughput and error reporting before the planned higher-rate setup. |
| CAN recovery remains unproven | The service has `Restart=no`, but udev requests the instance at device appearance. A disabled template alone is not evidence of broken boot startup. RX dropped count was 423 and remained stable during inspection. | Test process failure and adapter disconnect/reconnect separately during a controlled bench session. Do not label a stable cumulative count as an active fault. |
| Lidar ownership changes during ROS bringup | Legacy reception uses `192.168.3.2:6060`; ROS configuration uses the same endpoint. The ROS driver opens CoLa2 and writes data-output settings, as documented in [nanoscan3.yaml](amr_ws/src/amr_bringup/config/nanoscan3.yaml). | Define one owner and an explicit handover. Reconcile blanket “no scanner configuration changes” wording with intentional data-output setup; distinguish it from safety-field configuration. |
| ROS lidar initialization is not end-to-end validation | Existing logs showed successful driver initialization and communication. During one graph observation `/scan` had zero publishers despite an existing subscription. | Validate sustained scan rate, frame ID, timestamps and content in a dedicated ROS session. A topic name alone does not prove a live source. |
| IMU rate and timestamps need a new acquisition contract | Existing polling cycles through eight fields at roughly 40 ms per field, approximately 3.125 Hz per field. Internal sensor sampling rate is not the delivered estimator rate. | Define required published rate, coherent samples, acquisition timestamps and stale-data behavior before EKF tuning. |
| Geometry is partly provisional | URDF contains assumed/unverified sensor offsets and placeholder chassis dimensions. Track width was being updated during this audit; no current numeric mismatch is asserted. | Measure extrinsics and actual collision footprint, then maintain a clear source of truth shared with kinematics. Do not tune around placeholder geometry. |
| Simulation and future hardware can share a ROS domain | Domain was unset, meaning the default domain; multiple development publishers were observed. Physical ROS drive output is not implemented yet. | Separate simulation and hardware launch environments before introducing physical actuation. There is no evidence that simulated commands moved this vehicle. |
| Multi-interface DDS behavior is implicit | Ethernet sensor networks, Wi-Fi and USB tethering were active; no explicit CycloneDDS interface configuration was found in the inspected process environment. | Choose the intended DDS scope and test discovery with tether/Wi-Fi changes. Keep remote visualization through the intended Foxglove connection. |
| Chrony is currently a client | Synchronization worked, but no local-server configuration or UDP 123 listener was observed. | Implement local time serving only if another device needs it. Host NTP does not automatically translate sensor timestamps. |

## 6. Expected unfinished work, not defects by itself

- Physical ROS wrappers for the drives, IMU, RFID and DIO are not yet present in the inspected workspace.
- EKF configuration, navigation configuration, mission execution and deployment services remain implementation work.
- High-rate PDO operation and the proposed CAN bitrate change have not been validated on hardware.
- A complete mapping/navigation resource benchmark, thermal soak and recovery campaign have not been performed.
- Optional joystick packages are absent; they are only necessary if that input method is selected.

The architectural concern is how these additions inherit ownership, timestamps, fault gates and recovery behavior. Their current absence is consistent with a product still being coded.

## 7. Tests and reproducibility

Existing test XML reported:

| Package | Reported tests | Failures/errors |
|---|---:|---:|
| `amr_base` | 9 | 0 / 0 |
| `amr_description` | 6 | 0 / 0 |
| `amr_sim` | 7 | 0 / 0 |
| `amr_bringup` | 0 | — |

These were existing artifacts from other development activity, not tests run by this review. They do not establish current full-tree correctness, physical behavior, or the composed TF tree. The short observed build was incremental, not a clean-build performance benchmark.

Useful next coverage is failure injection and composed launches: partial arming, repeated commands after a fault, disarm/re-arm ordering, delayed CAN replies, and transform ownership. Avoid relying on test counts or source-text assertions as evidence that these behaviors work.

Python reproducibility also needs attention:

- Installed `pymodbus` 3.15.0 matches the inspected driver's `device_id=` calls, but a broad `>=3` requirement does not guarantee that API across older releases.
- `pip check` reported missing `pycairo` for `PyGObject`; no failure of the current CAN/lidar path was demonstrated from that finding.
- The expected pre-install package snapshot files were not found in their documented home-directory locations. This limits rollback evidence; it does not prove that no backup exists elsewhere.

## 8. Documentation corrections

Update the installation record to distinguish **planned**, **installed**, **verified**, and **pending hardware validation**:

- Replace “nothing installed” with the actual package/workspace status.
- Describe CAN as SocketCAN over the present SLCAN transport, including the current bitrate and udev ownership.
- Record the user-site Python conflict and the chosen environment strategy.
- Keep manual/blind-run descriptions and test counts aligned with the current code.
- Clarify that Python 3.10 supports `match`; avoiding it can be a project policy, but is not a 3.10 syntax limitation.
- Record a lifecycle decision date: Humble support ends in May 2027. This is a planning constraint, not a reason to interrupt development with an immediate OS migration.

## 9. Suggested implementation order

1. **Development consistency:** Correct TF ownership, establish the Python environment strategy, and launch the bridge with the overlay.
2. **Controller invariants:** Fix C01–C04 with targeted failure tests; establish the timing budget in C05.
3. **Physical ROS integration:** Establish exclusive hardware ownership, separate simulation, validate CAN transport, then implement timestamped hardware interfaces.
4. **Estimation and navigation:** Measure geometry; validate scan/IMU/odometry data before tuning EKF, SLAM and Nav2.
5. **Deployment:** Add explicit service environments and recovery policy; measure CPU, memory, recording growth, temperatures and latency under the actual workload.

## 10. Remaining privileged read-only checks

No sudo commands were run. If you provide these outputs, they can close the remaining host-configuration gaps:

```bash
sudo turbostat --quiet --interval 2 --num_iterations 5
sudo netplan get network.ethernets
sudo ufw status verbose
```

- `turbostat`: package/power/frequency evidence; unprivileged access could not read the required MSR data.
- `netplan`: persistent Ethernet configuration; configuration files were root-readable only. The query deliberately excludes Wi-Fi credentials.
- `ufw`: host firewall state relevant to the intended remote bridge connection.

These checks do not replace the later workload and hardware recovery tests.

## References

- [ROS platform support and distribution lifetimes, REP 2000](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst)
- [NumPy troubleshooting: downstream binary compatibility](https://numpy.org/doc/stable/user/troubleshooting-importerror.html)
- [ROS TF tutorial: frames have one parent](https://docs.ros.org/en/ros2_documentation/lyrical/Tutorials/Intermediate/Tf2/Adding-A-Frame-Cpp.html)
- [Linux CAN utilities: slcand implementation and options](https://raw.githubusercontent.com/linux-can/can-utils/master/slcand.c)
- [SICK ROS 2 safety scanner driver](https://raw.githubusercontent.com/SICKAG/sick_safetyscanners2/master/README.md)
- [CycloneDDS 0.10 configuration behavior](https://cyclonedds.io/docs/cyclonedds/0.10.2/config/cyclonedds_specifics.html)
