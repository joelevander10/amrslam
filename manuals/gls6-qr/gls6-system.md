# GLS621 home fix — design brief for gvievo-01 (trackless)

Status: **design, not implemented.** Bench-tested sensor, no vehicle code yet.
Date: 2026-10-20. §5 device state updated 2026-10-30.

**Superseded in part by [gls-wait-coding-plan.md](gls-wait-coding-plan.md) (2026-10-30).** That plan covers HOME
plus three destination labels at route wait steps, label poses stored as a separate label set instead of in the
map revision, and `label_fix_node` in place of `home_fix_node`. Where the two differ, the coding plan wins.

This brief is for a coding agent with the full gvievo-01 repository. It records
what was established on the bench and what the integration should look like.
Node, topic and file names below come from the repo's `README.md` and its
architecture pages; confirm each against the code before relying on it.

Reference documents supplied with this brief:

| File | What it is | Use it for |
|---|---|---|
| `gls621_op-ins.pdf` | GLS621 operating instructions (8024289, 2026-07) | Mounting, mounting angles (§5.2.2, Fig. 9), dimensions (Fig. 17, p.43), electrical, SOPAS setup |
| `gls621_tel-list.pdf` | CoLa telegram listing, FW 4.14 (device runs 4.16.0.7) | Every variable's communication name, type, range; output template syntax |
| `gls621_special-info.pdf` | 2007 SICK guide to direct-part-mark reading | Background only, not needed |
| `Label_Generator_GUI_manual.html` | SICK GLS label generator manual | How the home label was generated |
| `home_qr/*.pdf` | The home label, `x0000y0000` | Printing (see §6) |

---

## 1. Goal and scope

Give the SLAM/AMCL/Nav2 product an
**absolute, millimetre-level position and heading fix at the home position**,
from a SICK GLS621 downward camera reading a floor label.

The fix is used to:

1. **Localise at boot without an operator.** Today every run needs an operator-given
   initial pose, plus an operator confirmation that the live scan lines up with the map.
   With the vehicle parked on the home label, the fix provides both the pose and the
   evidence that it is right.
2. **Re-zero at home after runs.** AMCL can carry a few cm of error after loops through
   feature-poor areas. Each return home re-seeds it from the label.
3. **Anchor surveys (later).** Starting and ending a survey on the label gives the
   map a known absolute reference and a closure residual for revision approval.

Out of scope:
- Continuous GLS localisation along routes: there is one label, at home only.
- Anything in the safety path. The GLS is not a safety device. Nothing here may decide
  whether it is safe to move.

---

## 2. Existing system: the facts this design depends on

From the README and the localisation architecture page:

- **TF:** `map → odom` (AMCL in run mode, slam_toolbox in survey mode) → `odom → base_footprint`
  (robot_localization EKF, 50 Hz, fusing only vx and yaw rate) → sensors (URDF, fixed).
  The nanoScan3 is 0.964 m ahead of the drive axle.
- **Localisation monitor** (`amr_localization`, `localization_monitor_node`, `readiness.py`,
  `scan_consistency.py`), publishing `/amr/localization_state`:
  - States: UNLOCALIZED → CHECKING → READY → LOST.
  - A new `/initialpose` restarts CHECKING.
  - **READY requires every automatic check to hold for 2 s *and* an operator confirmation.**
  - Checks:
    - Covariance xx, yy ≤ 0.05 m², yaw ≤ 0.03 rad².
    - A pose jump (map→odom moving > 0.15 m or > 5°) means LOST. This check is ignored for 1.5 s after an initial pose.
    - Long beams > 15 % for 1 s means LOST.
    - Scan match ≥ 60 % gates the operator confirm.
    - Freshness limits on scan, wheels, IMU, TF and AMCL.
- **AMCL never re-scatters** (`recovery_alpha_* = 0`). LOST stops the robot until an operator gives a new initial pose.
- **Routes:** route compiler (straight segments plus signed spins), Nav2 RPP, spin behaviour,
  route executor in `amr_mission`. Trackless speed 0.60 m/s.
- **Map revisions are immutable bundles** (`amr_maps`, `mapping_session_node`, `map_bundle`).

Repo invariants that constrain this work:
- `agv_core` imports neither ROS nor Flask (`tests/test_layout.py`).
- **One owner per device.** Each device has exactly one owning process.
- **Profiles are strict:** unknown *and* missing keys are fatal. Only primitives are stored; anything derivable is derived. `profile_name` must match the filename.
- `tests/run_all.py` pins its module list and `EXPECTED_CHECKS`. Raise the count deliberately, with a note.
- Every autonomous motion goes through leases, permits and `cmd_mux`.

---

## 3. Target architecture

```
                      Ethernet (LAN3, enp3s0), CoLa A over TCP
 GLS621 ──────────────────────────────────────────────┐
 192.168.4.1  host 2112: result stream                │
              aux  2111: diagnostics + commands       ▼
                                          ┌──────────────────────────┐
                                          │ amr_base/gls_node        │  sole owner of the GLS socket
                                          │  uses agv_core.drivers.  │  (reconnect, staleness,
                                          │  gls (parser, no ROS)    │   per-frame aggregation)
                                          └────────────┬─────────────┘
                                         /gls/reading  │ amr_interfaces/GlsReading (every frame, ~33 Hz)
                                                       ▼
 map bundle: home label pose ───────────► ┌──────────────────────────┐
 profile: camera extrinsic ─────────────► │ amr_localization/        │  pure math in agv_core.glspose
 /amcl_pose, /amr/localization_state ───► │ home_fix_node            │
 wheel speeds (stationary check) ───────► │                          │
                                          └──┬───────────────┬───────┘
                               /initialpose  │               │ /amr/home_fix (evidence + status)
                                             ▼               ▼
                                          AMCL      localization_monitor_node
                                                    (GLS fix accepted as the
                                                     "operator confirm" input)
                                                             │
                                          route executor ◄───┘  "re-zero at home" step after a route ending at home
```

### 3.1 `agv_core/drivers/gls.py` (new, no ROS)

- A TCP client helper plus a telegram parser. Pure functions where possible, so `tests/` can cover them.
- **Framing:** CoLa A. Each telegram is `0x02 … 0x03`. Bytes may arrive split or several telegrams per read, so buffer and split on ETX.
- **Result telegram,** with the template in §5: `<label code>;<XMCL>;<YMCL>;<angle>;<timestamp>`.
  Example captured on the vehicle on 2026-10-30, label at home (one frame, four of its codes):

  ```
  A2x0000y0000;11;9;0.593822;946687420249
  D2x0000y0000;4;9;0.292423;946687420249
  A1x0000y0000;11;11;0.298404;946687420249
  D1x0000y0000;4;11;0.584598;946687420249
  ```

- **Label code:** two characters for the code's position in the label (`A1`…`D4`: row letter, column digit), followed by the label ID (`x0000y0000`). Split them. Only the label ID identifies the label.
- **`$FAMS`** behaves as a millisecond timestamp. The device clock is unset, so it counts from 2000-01-01. Use it only to group telegrams into frames: all telegrams with the same timestamp come from one image.
- **Output mode is "Multiple":** one telegram per decoded code per frame, typically 3–16 per frame.
- **Aggregation per frame:**
  - Take the median of x, y and angle across the frame's codes.
  - Use a circular mean or median for the angle; values sit near 0/360 when the label is rotated.
  - Report `n_codes` and the spread.
  - Reject outliers beyond a configurable tolerance.
- **Units and signs are not yet confirmed (see §7).** Do not hard-code them. Put a scale factor and sign convention behind profile keys, set from the bench test.
- Other telegrams that may appear on the socket, which the parser must ignore or route:
  - CoLa replies `sRA`, `sWA`, `sAN`, `sFA <err>`.
  - A heartbeat, if it is ever enabled.

### 3.2 `amr_base/gls_node` (new)

- **Sole owner of the GLS connection,** following the one-owner rule.
  - Decide whether it needs an owner lock like `ownerlock.py` (can/dio). A TCP device has no OS-level exclusivity.
  - Bench tools should refuse to connect to the result port while the node runs, the same way the CAN tools do.
- Connects to the host port and reconnects with backoff.
- Publishes `/gls/reading` every frame. Suggested message `amr_interfaces/GlsReading`:
  - `header`: ROS receive time
  - `label_id`: string
  - `valid`: bool
  - `x_m`, `y_m`, `yaw_rad`: the camera pose in the label frame after units and sign conversion
  - `n_codes`
  - `spread_xy_m`, `spread_yaw_rad`
  - `device_ts_ms`
- Publishes "no label" frames (`valid=false`) when nothing is read for longer than a timeout, so consumers can tell no label from a stale node.
  - The device sends nothing on a no-read. Absence of telegrams is the only no-read signal.
- **Read-only at runtime.** The node does not write device parameters. Configuration lives in the device (SOPAS, saved permanently) and is version-controlled as an exported parameter file.
  - Optionally, at start, read back a few critical variables over CoLa and refuse to run, with a diagnostic, if they differ: `GlsCodeSel`, `D2MCGridModSize`, `OFtplstr1`. Those three are the ones that broke on the bench.
- Diagnostics: connected, frame rate, codes per frame, last label ID. These feed the web UI's status.

### 3.3 `agv_core/glspose.py` (new, pure math)

SE(2) only: the camera is perpendicular to the floor (β = γ = 0, op-ins §5.2.2).

Frames:
- **`L` (label):** origin at the printed centre cross. +X and +Y as the printed arrows. z up, out of the floor.
- **`C` (camera):** the point where the optical axis meets the floor, with yaw fixed to the housing.
- **`B` (`base_footprint`):** as in the URDF.
- **`M` (`map`).**

Measurement after conversion: **`T_LC`**, the camera pose in the label frame. "XMCL/YMCL from the AGV's point of view" should give the camera/vehicle relative to the label; confirm the direction in the bench test, §7.

```
T_MB = T_ML ⊕ T_LC ⊕ (T_BC)⁻¹
```

- `T_ML`: the home label's pose in the map. Per map revision (§3.5).
- `T_BC`: the camera extrinsic. Per vehicle (profile, §3.6).

Covariance for `/initialpose`:
- Use the measured spread plus the extrinsic uncertainty plus the label survey uncertainty.
- Floor it, for example σ_xy ≥ 5 mm and σ_yaw ≥ 0.5°, so AMCL is not over-confident.

### 3.4 `amr_localization/home_fix_node` and monitor changes

**Fix acceptance.** All of the following must hold for N consecutive frames (≈ 0.5 s at 33 Hz):
- `label_id` equals the current map revision's home label ID.
- `n_codes ≥ 2` and the spread is within tolerance (start with 2 mm / 0.5° and tune from bench data).
- The vehicle is stationary. Use wheel speeds or odom twist near zero; the EKF already knows.
- Localisation-state preconditions:
  - **UNLOCALIZED** (boot): no AMCL prior is needed. The unique label ID is the evidence.
  - **READY** (re-zero after a route): `|AMCL − GLS| ≤ 0.30 m and ≤ 10°`.
    - Larger disagreement means **do not snap**. Raise an operator-visible fault, because either the label is wrong or moved, or AMCL is badly off.
    - Below a small threshold (for example 5 mm / 0.3°), do nothing, to avoid needless CHECKING cycles.

**Action.**
1. Publish `/initialpose` with `T_MB` and its covariance. AMCL re-initialises its particles there, and the monitor restarts CHECKING; its pose-jump check is already ignored for 1.5 s after an initial pose.
2. Publish `/amr/home_fix` (accepted, the pose, the residual against AMCL before the snap, the spread). This is the audit trail; log every fix with its residual, because that residual is the drift measurement.

**Monitor change: replace the operator confirm, keep the automatic checks.**
- Today: READY requires the automatic checks for 2 s, plus an operator confirm gated by scan match ≥ 60 %.
- New: a fresh, accepted GLS home fix (same label, still valid during the 2 s window) **satisfies the confirm input**.
  - Scan match ≥ 60 % and all the other checks still apply.
  - If no GLS fix is available (no label in view, camera down, a vehicle without GLS), the existing operator-confirm path is unchanged.
- Keep the logic in `readiness.py` (pure, testable). Add the fix as an input; don't special-case it in the node.

### 3.5 Map revision: home label pose

`T_ML` belongs to the map, not the vehicle, so store it in the map bundle or revision metadata (`amr_maps`). A label that moves invalidates only the maps that reference it.

Fields:
- `home_label_id` (e.g. `"x0000y0000"`)
- `x`, `y`, `yaw` in the map frame
- the survey method and date

How to obtain it:
- **Preferred: start every survey parked on the label.** At survey start, read `T_LC` and the SLAM pose `T_MB(start)`, then compute `T_ML = T_MB ⊕ T_BC ⊕ (T_LC)⁻¹`.
  - Average over a stationary window.
  - Store it with the revision when the revision is saved.
- **Survey closure check (later):**
  - End the survey back on the label.
  - The difference between the SLAM pose and the GLS-derived pose is a closure residual. Show it in the revision approval UI.
  - Do not feed it into slam_toolbox. It has no clean API for absolute priors; that's out of scope.
- **Alternative for existing maps:** park on the label, localise manually (operator confirm), then record `T_ML` from the AMCL pose averaged over a stationary window. This is less accurate (it's AMCL-limited); flag it in the metadata.

### 3.6 Profile keys (strict schema)

Add a `gls` section, validated in `agv_core/config.py`. It must be added to **every** profile; missing keys are fatal, so use `"enabled": false` on vehicles without the camera.

Suggested keys, all primitives:
- `enabled`, `host`, `port`
- `connect_timeout_s`, `stale_s`
- `mount_x_m`, `mount_y_m`, `mount_yaw_rad`: `T_BC`
- `xy_scale_m_per_count`
- `x_sign`, `y_sign`, `angle_sign`, `angle_offset_rad`: from the bench test
- `min_codes`
- `fix_frames`, `fix_spread_xy_m`, `fix_spread_yaw_rad`
- `fix_gate_xy_m` (0.30), `fix_gate_yaw_rad` (≈0.17)
- `fix_deadband_xy_m`, `fix_deadband_yaw_rad`
- `cov_floor_xy_m`, `cov_floor_yaw_rad`

Also add the camera to the URDF as a fixed `gls_frame`, so it shows in TF and the web app. Generate it from the same profile values; don't keep a second copy.

### 3.7 Route executor: re-zero step

When a route ends at the home station and the vehicle has stopped:
1. Wait up to ~2 s for an accepted fix.
2. If one arrives, apply it (§3.4) and log the residual.
3. If none arrives, finish the route normally with a warning (label not seen). Don't fail the run.
4. If the fix is rejected by the 0.30 m / 10° gate, stop and alert. Don't continue autonomously on a disputed pose.

### 3.8 Phase 2 (optional): final creep onto the label

- Closed-loop on GLS x/y/yaw at creep speed (≤ 0.05 m/s) under the autonomous lease, through `cmd_mux`.
- Limit: a differential drive cannot correct lateral error by creeping. The useful range is along-track distance and heading within the camera's view (about ±30 mm at the 203 mm mount).
- Larger lateral error needs a re-approach.
- **Phase 1 is localisation, not placement.** Do Phase 2 only if docking tolerance requires it.

### 3.9 Simulation and tests

- `tests/` (offline, `agv_core`):
  - Parser cases: split and joined telegrams, CoLa replies mixed in, malformed fields, angle wrap at 0/360, frame grouping by timestamp.
  - `glspose` composition and inversion round-trips.
  - Readiness with and without a fix.
  - Update `EXPECTED_CHECKS` with a note.
- `amr_sim`: a fake GLS that emits telegrams from the sim's ground-truth pose when within ~30 mm of a configured label pose, so the home-fix path and monitor run in sim.
- Per-package pytest for `gls_node` reconnection and staleness, and for `home_fix_node` gating.

---

## 4. Hardware

| Item | Value / note |
|---|---|
| Device | GLS621, type V2D621G-2MSXBB5x, S/N 26261075, FW V4.16.0.7 |
| Network | On the vehicle: static IP **192.168.4.1** /24, gateway 0.0.0.0, on its own port (PC enp3s0 at 192.168.4.2, `/etc/netplan/80-lan3-gls621.yaml`, resolved 2026-10-21), so there is no clash with the nanoScan3 subnet 192.168.3.0/24. Bench setup used 192.168.3.1 (laptop 192.168.3.4). Factory default is 192.168.0.1. |
| Protocol | **CoLa A (ASCII) only.** CoLa B gets no reply, so SOPAS must use CoLa A. Host port 2112 (results), aux port 2111 (diagnostics + commands). Commands are also answered on 2112. |
| Power | 12–24 V DC ±20 % (28.8 V max), 4 W typical / 22 W max, 2 A fuse (op-ins) |
| Mount height | **Fixed by the chassis at ~203 mm.** Focus is set to 203 mm. At this height a 0.7 mm module images at ~7 px; the decoder minimum is 6 px (`DCDMMinModSize100` = 600), so **don't mount higher**. |
| Orientation | Perpendicular to the floor. A 180° yaw flip relative to op-ins Fig. 6 is fine (same field-of-view orientation); it only changes `mount_yaw_rad`. Mounting angle α stays 0 in the device; the yaw is handled in `T_BC`. |
| Lens position | Ø10 lens, 21.5 mm from each long side, 26 mm from the end *opposite* the connectors (op-ins Fig. 17, p.43). It is not the housing centre. Place the lens over the drive-axle midpoint if possible (then the spin-calibration circle is small), but calibration measures it anyway. |
| Environment | Own LEDs, so a dark chassis cavity is fine. Ambient on the label ≤ 2,000 lx; keep direct sun off it. Optional polariser 2088228 reduces glare from glossy labels. |
| Safety | Not a safety device. Nothing derived from it may gate motion safety. |

---

## 5. Device configuration: state as of 2026-10-30

Template, ISO15415, contrast and code selection were read back from the device
on the vehicle (192.168.4.1) on 2026-10-30 with CoLa A `sRN`. The other rows are
from the 2026-10-20 bench session and were not re-read.

Configured in SOPAS ET (CoLa A, Authorized Client; the default password is `client`, CoLa `SetAccessMode 3 F4724744`). CoLa communication names are in brackets. Numeric values in CoLa A replies are **hex**.

| Setting | Value | Note |
|---|---|---|
| GLS application | on, **Multicode label** (`GlsCodeSel` 0) | |
| Distance code → label centre | **9 mm** (`D2MCGridModSize` 9) | Correct for this label: inner codes at ±9 mm, outer at ±27 mm |
| XMCL/YMCL values | **from the AGV's point of view** (`D2MCMAACompAct` 0) | Keep |
| Data Matrix | on (`DCDMAct` 1); QR off (`DCQRAct` 0) | DM had been off |
| Min code contrast | **30 %** (`DC2DContrast` 0x1E) | **Was 70 %, which caused 100 % no-read** on the paper label |
| Module size window | 0.5–0.9 mm (`DC2DMinModSize`/`MaxModSize`) | Label is 0.7 mm. Consider widening to 0.4–1.2 |
| ISO15415 verification | **off** (`DCDMVerISO15415` 0) | Turned off after the bench session (read back 2026-10-30). Print grading, slowed each cycle |
| Trigger | continuous; output "for each image" ticked; transmission ASAP; output mode Multiple | ~33 Hz result stream |
| Good-read condition | min 1 / max 1 codes (`EVmincodes`/`EVmaxcodes`) | Hidden in GLS mode. Output works anyway; left at 1 (a test write to 20 was reverted, not saved) |
| Host output template | `[iec _GoodReadCond][f][$BC];[$XMCL];[$YMCL];[$AN2P];[$FAMS][e][e]` (`OFtplstr1`) | Changed from `[$X];[$Y]` (read back 2026-10-30). As found on the bench, `$X/$Y` gave each code's own position, so codes disagreed by ±27 mm. With XMCL/YMCL the codes in one frame still differ by up to ~7 counts in x and y; see §7 item 1. |
| Aux output | Reading Diagnostics | Human-readable per-code dump; not for runtime use |
| Device name | "not defined" | Optional: set something like `gvievo-01-gls` |
| Saved permanently? | **Yes** (2026-10-30) | TODO: export the parameter file and commit it under `manuals/` or `profiles/` |

**Pitfall:** in SOPAS **Live / Auto-Setup (setup mode) the host output is muted** (op-ins: "In Setup mode, data output via the host interface is deactivated"). The aux diagnostics keep flowing. If 2112 goes silent on the vehicle, check that nobody left SOPAS in Live mode.

**Pitfall:** the Auto-Setup "Code Settings: Failed!" result is expected with a multicode label (16 distinct codes). Focus and image steps are what matter.

---

## 6. The home label

- Generated by the SICK GLS Label Generator: GLS6, Multi-Code, X start 0 / amount 1, Y start 0 / amount 1, giving label ID **`x0000y0000`**.
- **75.0 × 75.0 mm**, 16 Data Matrix codes (14×14 modules, 0.7 mm module, ~9.8 mm per code).
- Code centres at ±9 / ±27 mm from the centre cross. Printed +X/+Y arrows. Grey contrast wedge. SICK P/N 4105695.
- Files:
  - `home_qr/GLS6_Multicode_1270dpi_x0000y0000_to_x0000y0000.pdf`: the original. One label on A4, embedded at 1270 ppi, so it prints 75.0 mm at 100 % / actual size.
  - `home_qr/GLS6_Multicode_x0000y0000_PRESCALED_104.6pct.pdf`: a copy scaled ×1.046 **only** to compensate one office printer whose "fit to page" printed the original at 71.7 mm. Never use it at actual size; it prints 78.45 mm.
- **Acceptance:** frame 75.0 mm, and the −27↔+27 tick span **54.0 mm ±0.2**. A wrongly scaled label scales the reported offsets.
- Floor label: print on matte, durable, laminated stock at 508/762/1016/1270 dpi, or ask SICK for a pre-printed label. A glossy surface causes glare under the LEDs.
- When laying it, align **+X with a known map direction** (for example the vehicle's forward direction when parked at home) and record it. That becomes `T_ML.yaw`; the survey procedure in §3.5 measures it anyway.
- Use one ID per physical location. Spares are reprints of the same ID. A second dock gets a new ID (e.g. `x0001y0000`).

---

## 7. Open items: bench tests before code depends on them

The template is now XMCL/YMCL (§5). Log port 2112 raw to a file.

1. **Units:** slide the label exactly +10 mm along its printed +X, then +Y, using a ruler or jig.
   - With the as-found `$X/$Y`, codes 18 mm apart differed by ~20 counts, so **it is not plain mm.** Determine `xy_scale_m_per_count` from XMCL/YMCL.
   - With XMCL/YMCL the codes in one frame should agree, but on 2026-10-30 they ranged 4–11 counts in both x and y (the A codes read x=11, the D codes x=4). Find out why before you set the outlier tolerance in §3.1. It could be per-code geometry, a lens offset, or the scale.
   - Check it at 203 mm. It may depend on the focus or working-distance setting.
2. **Signs:** from the same test, determine whether XMCL/YMCL is camera-in-label or label-in-camera, and set `x_sign`/`y_sign`.
3. **Angle:** rotate the label about +90° and confirm the direction and zero.
   - As found, the label sitting ~90° rotated read ≈ 271.8°, with per-code variation of ±0.5°.
   - Set `angle_sign` and `angle_offset_rad`.
4. **Repeatability:** 60 s static, per frame (after aggregation). Report σ for x, y and yaw. SICK publishes no repeatability figure, so this number sets `fix_spread_*` and `cov_floor_*`.
5. **Rate:** frame rate and codes per frame now that ISO15415 is off. ~33 Hz was observed with it on.
6. **On the vehicle:** extrinsic calibration (`T_BC`).
   - Spin in place over the label.
   - The label-centre trace is a circle around the rotation centre. Its centre and radius give `mount_x/y`.
   - Comparing the GLS angle with the EKF/gyro yaw gives `mount_yaw`.
   - Repeat in both directions. Check the wheel-track sign convention first (README invariant).
7. **Static vs. moving:** confirm that fixes are only taken when stationary. Characterise the read range: how far off-centre (about ±30 mm expected) and at what creep speed reads still succeed.
8. **Ambient:** check that the label area at home is ≤ 2,000 lx with a phone lux meter, including sunlight at different times of day.

---

## 8. Acceptance (suggested)

- Boot on the label goes to READY with no operator input, in sim and on the vehicle. The fallback operator path still works with the label covered.
- A covered or wrong label never produces a fix. A label ID mismatch is reported.
- After N loops returning home:
  - The logged residual (AMCL − GLS) before each snap shows the drift.
  - The snap brings the home pose repeatability to the bench σ (target: a few mm, < 0.5°).
- A GLS disagreement > 0.30 m / 10° stops the run with an alert and never snaps.
- `tests/run_all.py` is green with a deliberately raised `EXPECTED_CHECKS`. Ruff is clean. The layout test is green (no ROS in `agv_core`).
