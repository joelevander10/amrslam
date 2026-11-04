# GLS621 bench results + wait-step acceptance A (MCQ4-4), 2026-10-30

Plan: `gls-wait-coding-plan.md` §2 (GLS621, read live), §9, §11 A. Requested by agv-can-f9.

**Conventions.** Raw device units, not converted. Vehicle axes: x forward, y left, yaw CCW seen
from above. Each position is the mean of every valid `/gls/reading` message over the window
given (about 14.5 msg/s), from a read-only subscriber. The raw CSVs and the two read-only tools
(`glsavg.py`, `glslog.py`) are in `bench-data-2026-10-30/`, next to this file; they're untracked.
The camera was never written to. `amr.service` was not restarted.

## Baseline at rest (label x0000y0000 under the camera, 13:37, the HOME park before any driving)

| n valid | x_counts (σ) | y_counts (σ) | angle_deg (σ) | spread_counts mean [min..max] | n_codes mean [min..max] |
|---|---|---|---|---|---|
| 72/72 over 5 s | 5.10 (1.28) | −26.18 (0.42) | 1.699 (0.133) | 5.16 [4.1..7.3] | 5.24 [4..9] |

## 1. Print check

The operator measured the taped HOME label (x0000y0000) in place with a caliper.

| span | nominal | measured | ratio |
|---|---|---|---|
| tick span, label x axis | 54.0 mm | 54.5 mm | 1.0093 (+0.9 %) |
| tick span, label y axis | 54.0 mm | 54.5 mm | 1.0093 (+0.9 %) |
| code-to-code pitch, both axes | 18 mm | 18 mm ("correct", operator; the span measured isn't recorded) | 1.00 within reading |

- The print is about 0.9 % oversize and isotropic (the same on both axes).
- 0.9 % of an 18 mm row is 0.17 mm, about 0.2 counts. That's far too small to explain the
  ~2.3 counts per 18 mm row per-code bias, or the 4–11 count spread between codes. The
  print scale isn't the cause.
- The resolution of a caliper reading on printed ticks is probably about ±0.1–0.2 mm, so the
  ratio is good to about ±0.3 %.

## 2. Scale + signs, vehicle x

**Method changed (operator).** The HOME label stays taped down, and the vehicle drives over it
instead of the label being slid. Pendant jog, MANUAL, drives armed.

**First drive-over (no odometry logged).** Four crossings, operator overshooting back and forth.
The operator confirms pass 1 was reversing.

| pass | direction | y_counts | x_counts | angle_deg | n_codes |
|---|---|---|---|---|---|
| 1 | reverse (−x) | +74 → −40 | 9.5–15 | 1.8–2.4 | 2 → 5 |
| 2 | forward (+x) | −78 → +47 | 10–17 | 1.2–1.95 | 2 → 7 → 3 |
| 3 | reverse | +63.5 → −65 | 16–23 | 1.9–2.4 | 2 → 6 → 2 |
| 4 | forward | −73 → +51 | 18–22 | 1.8–2.1 | 2 → 7 → 3 |

- **Vehicle x (fore-aft) moves the device's y_counts.** x_counts barely moves, so the device's
  X axis is lateral.
- **Sign:** reversing makes y_counts go down, so device +Y points vehicle-forward (as camera
  position in the label frame).
- Each crossing took about 0.4–1 s at jog speed, roughly 150–300 counts/s. The scale comes from
  the odometry run below.

**Second drive-over (no odometry logged).** One reverse crossing, y +74 → −40, x 7.5–13,
angle about 1.2–1.5°. Then one forward crossing, y −79 → +67, x 24.5–28, angle about 359.7–359.85°.
The sign is the same as above.

**Tap run with odometry (13:47–13:48).** Pendant taps of about 0.2 s, reversing across the
whole matrix and then forward across it. `/odom_raw` (wheel odometry, diff_drive_odom, 45 Hz)
was logged with every frame. Each row below is a stationary pause between taps (odometry
unchanged): the mean of n frames. s is the odometry distance along the mean heading
(−6.3° odometry yaw), measured from the first pause.

| t (s) | n | odom x, y (m) | odom yaw (°) | x_counts | y_counts | angle_deg (wrapped) |
|---|---|---|---|---|---|---|
| 159.4 | 13 | 0.0692, 0.0125 | −6.02 | 19.9 | 50.0 | −2.69 |
| 160.6 | 9 | 0.0426, 0.0154 | −6.13 | 19.0 | 21.6 | −2.76 |
| 161.7 | 10 | 0.0063, 0.0193 | −6.25 | 15.6 | −16.7 | −2.93 |
| 163.0 | 11 | −0.0298, 0.0232 | −6.34 | 12.4 | −56.5 | −2.81 |
| 164.0 | 4 | −0.0493, 0.0254 | −6.37 | 13.2 | −74.9 | −2.83 |
| 173.1 | 11 | −0.0362, 0.0236 | −6.52 | 14.2 | −66.5 | −3.61 |
| 174.1 | 10 | −0.0033, 0.0198 | −6.55 | 16.7 | −32.2 | −4.11 |
| 175.2 | 7 | 0.0236, 0.0167 | −6.54 | 20.1 | −3.8 | −3.90 |
| 176.4 | 8 | 0.0747, 0.0109 | −6.54 | 22.5 | 50.9 | −3.87 |

The first five rows are the reverse taps. The last four are forward.

- **A first fit, for a cross-check only:** y_counts = 1049.5 · s_m + 47.3, residual rms 2.6
  counts. That's about 0.95 mm/count, but it depends on the wheel odometry's own scale, which
  hasn't been checked against a tape. Please derive the scale yourself.
- **x_counts drifts with the travel:** a slope of 0.067 counts of x per count of y, which is
  3.85°. So the vehicle's path is about 3.9° off the device's Y axis. In the same rows
  angle_deg reads −2.7 to −4.1°.
- **The angle stepped about −1.1° between the two passes.** Odometry yaw moved only about −0.3°
  in that time. Noise or wheel scrub; not resolved.
- Raw CSV of every frame (valid or not), with odometry:
  `bench-data-2026-10-30/approach3.csv`. The earlier, odometry-less passes are in
  `approach.csv` and `approach2.csv` in the same folder.

## 3. Scale + signs, vehicle y

**Not done.** A diff drive can't translate sideways, and the label is taped down. The lateral
sign and scale are left to the T_BC fit from a full turn (§7), or to a hand-slid spare label.

## 4. Angle

From the turn in place (§7). The label wasn't rotated by hand.

- **Sign:** angle_deg increases when the vehicle turns CCW seen from above (pendant LEFT, odometry
  yaw increasing). Operator sequence: LEFT, then RIGHT, then LEFT again. Every leg agrees.
- **Pure-rotation stationary pauses, odometry yaw against angle_deg:**
  angle_deg = 0.966 · odom_yaw + 19.52°, rms 0.38°, over 64° of yaw (−48.6° to +15.7°).
  Per leg: +38.1° odometry gave +36.9° angle; −64.2° gave −62.2°; +30.0° gave +29.0°.
  - The angle consistently reads about 3.4 % less than odometry yaw. Either the odometry
    over-reads spins (an effective track about 3 % wide, or wheel scrub), or the angle scale
    is off. A protractor or tape check on a spin would settle it.
- **Angle offset:** not measured squared to the vehicle axes. The vehicle was never squared to
  the label. The 19.52° intercept is against odometry yaw, whose zero is arbitrary, so it means
  nothing.
- During the fore-aft drive-overs the angle read −2.7..−4.1°, and x drifted at 3.85° against the
  travel (§2). So angle_deg ≈ the heading of the vehicle's x axis relative to the label, to
  within about 1°.

## 5. Capture window

**Fore-aft (vehicle x = device y_counts), from all six drive-over crossings:**
- valid first and last at y_counts about +74..+76 and −75..−79, with 2 codes at the edges.
- n_codes ≥ 4 between about y −62 and +56. Peaks of 6–8 codes near −30..+20.
- At the rough 0.95 mm/count, that's about ±72 mm valid and about ±55 mm with ≥ 4 codes. That's
  much wider than the sim's ±20 mm fore-aft.

**Sideways:** not measured. A diff drive can't translate sideways, and the label is taped down.
During the crossings x_counts ranged from 7.5 to 28.

## 6. Code-to-code spread at rest

| where | window | spread_counts mean [min..max] | n_codes mean [min..max] |
|---|---|---|---|
| HOME park, 13:37 (baseline) | 72 frames, 5 s | 5.16 [4.1..7.3] | 5.24 [4..9] |
| final park after the spin, 13:55 | 51 frames, 4 s | 6.85 [5.7..8.6] | 7.02 [6..8] |
| per pause during the spin (§7) | 5–641 frames | 4.7–7.8 (means) | 4.0–9.1 (means) |

- In this session's data the spread stayed within 3–9 counts. No 11-count frames were seen at rest.
- At the final park x_counts showed σ 1.85, with the median hopping between about −2 and +3
  as the set of codes changed. y_counts was steady (σ 0.25).

## 7. Mount calibration (T_BC), spin in place

**Partial, not a full turn.** The operator tapped the pendant: LEFT, RIGHT, LEFT. agv-can-f9
relayed the user's stop request at 13:55, so the test was stopped there, with the vehicle
already stationary. **Heading coverage was 64° (odometry yaw −48.6..+15.7), not 360°.**
The label stayed valid at every heading, with 6–9 codes.

The raw data, every frame from 13:47:02 to 13:56:02, with odometry:
`bench-data-2026-10-30/spin.csv`. Valid from t_wall 1793343249.4 onwards. Before that, the vehicle
drives in, about 1.8 m forward and back. The columns are the same as approach3.csv.

Stationary pauses (odometry yaw unchanged), pure rotation only. Odometry x, y stayed at
(0.0438, 0.0923) ±0.5 mm throughout:

| t_wall − 1793342786 (s) | n | odom yaw (°) | x_counts (σ) | y_counts (σ) | angle_deg (wrapped) | n_codes | spread |
|---|---|---|---|---|---|---|---|
| 558.0 | 116 | −22.46 | 5.23 (1.35) | −0.23 (1.97) | −1.85 | 6.8 | 5.5 |
| 568.5 | 11 | −13.06 | 3.09 (2.47) | −1.91 (1.24) | 6.21 | 7.5 | 7.8 |
| 570.5 | 10 | −0.04 | 3.95 (1.80) | −6.10 (1.59) | 19.64 | 7.5 | 5.9 |
| 572.5 | 30 | 15.66 | 2.72 (1.27) | −6.80 (0.91) | 35.06 | 8.1 | 6.1 |
| 576.3 | 11 | 2.41 | 3.36 (1.23) | −7.32 (0.78) | 21.74 | 7.4 | 5.9 |
| 578.3 | 19 | −11.28 | 2.79 (1.82) | −1.66 (0.71) | 8.21 | 7.3 | 6.8 |
| 580.8 | 9 | −25.48 | 2.72 (1.65) | 3.00 (0.85) | −5.00 | 7.4 | 6.3 |
| 582.8 | 8 | −40.85 | 2.75 (1.58) | 6.50 (0.75) | −19.81 | 7.6 | 5.9 |
| 584.3 | 17 | −48.58 | 4.18 (0.98) | 7.71 (0.94) | −27.11 | 9.1 | 5.7 |
| 586.9 | 7 | −41.88 | 4.00 (1.07) | 8.29 (0.88) | −20.82 | 8.4 | 6.0 |
| 588.5 | 9 | −32.07 | 3.33 (0.94) | 7.78 (1.31) | −12.15 | 8.1 | 6.5 |
| 590.4 | 641 | −18.58 | 1.97 (1.87) | 5.88 (0.42) | 1.92 | 7.1 | 6.9 |

The first table row is the pause right after the approach. Two earlier pauses, with y_counts
38–45, were still translating and are left out.

- **Over 64° of rotation, x/y move by only about 15 counts.** The camera is close to the
  rotation centre, roughly 1.5–2 cm off it.
- **My quick fit, for a cross-check only:** a circle c + R(±angle)·m. Either rotation sense
  fits with |m| ≈ 17 counts and rms about 1.7 counts. With 64° of arc and a 17-count radius,
  the fit can't tell the direction of the offset or the sense. **A full turn is needed for a
  usable T_BC.** At this offset the label should stay in view all the way round.
- **The vehicle is parked on the label, stationary.** Final reading: x 1.75 (σ1.85),
  y 5.95 (σ0.25), angle 1.914° (σ0.140), 7 codes. Odometry is (0.0438, 0.0923, −18.58°).
  - It is NOT at the original HOME park, which read x 5.1, y −26.2, angle 1.70. It's about
    32 counts (about 3 cm) further forward.

## 8. Acceptance A (§11 A)

**Not started.** The session was stopped at the user's request (13:55) before the route
HOME → wait "UBC hitch" → HOME was made.

**State at stop:** drives 'Ready to switch on', link standby, power_reason "waiting for Reset".
Panel: estop_ok true, emergency 0, MANUAL, manual_arm on. `amr.service` was not restarted, and
no settings or camera parameters were changed.
