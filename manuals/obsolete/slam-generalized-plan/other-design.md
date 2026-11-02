# IUV AMR — Software Architecture & Controller Topology (Slide Transcription)

## 0. Context for the reading agent

| Item | Detail |
|---|---|
| Source | 2 presentation slides, photographed from a projector screen (a person's head partially blocks the bottom-left of both) |
| Presenter | "EDC — Engineering Development Center" (logo top-left) |
| Product | "IUV" — a small 4-wheel skid-steer AMR |
| Ownership | **Third-party / competitor AMR**, not the user's product |
| User's assessment | Components are **below industrial grade** |
| Transcription rule | Sections 1–3 = what the slides literally show. Section 4 = derived values. Section 5 = gaps/observations (inferred, NOT on slides). |
| Illegible parts | Hardware layer label (Slide 1) hidden by head; footer first word partially hidden ("…ustrial PC" → "Industrial PC") |

---

## 1. Slide 1 — "IUV SOFTWARE ARCHITECTURE"

Five horizontal layers, stacked top to bottom. Arrows between layers carry the labels shown.

### 1.1 Layer stack

| # | Layer | Modules (name — subtitle) |
|---|---|---|
| 1 | Application / Fleet Layer | Mission Manager · Fleet Client · HMI / Web UI · Diagnostics · WMS / MES API |
| ↓ | *interface label: "mission"* | |
| 2 | Autonomy & Navigation Layer | Mapping / SLAM — LiDAR + odometry · Localization — AMCL / SLAM localization · Path Planner — Global route / waypoint · Local Planner — Obstacle avoidance · Mode State Manager — Straight / turn / pivot / dock |
| ↓ | *interface label: "pose / path"* | |
| 3 | Motion Control Layer | 4WID Kinematics — Skid steer / differential drive · Velocity Allocation — Left/right wheel speed command · Straight Line Control — Encoder + IMU correction · Odometry Fusion — Encoder + IMU · Safety Stop Logic — Software-level stop |
| ↓ | *interface label: "ROS 2 msgs"* | |
| 4 | ROS 2 Middleware & System Services | ROS 2 DDS — Topics / services · TF2 — Frames / transforms · Lifecycle — Node state control · rosbag2 — Data logging · Parameters — Calibration / tuning · OTA — Update manager |
| ↑ | *interface label: "HW feedback"* | |
| 5 | Hardware / Driver Layer *(label obscured; inferred from contents)* | LiDAR Driver — USB / Ethernet · IMU Driver — I²C / UART · Motor Driver Node — CAN / RS485 · Encoder Interface — Pulse / CAN feedback · BMS Interface — CAN / RS485 · Safety I/O — E-stop / bumper |

### 1.2 Footer banner (highlighted yellow)

- **Industrial PC:** autonomy + ROS 2
- **MCU:** deterministic motion control
- **Independent safety circuit:** E-stop / contactor / safety sensors

### 1.3 Hardware interface summary (from Layer 5)

| Device | Bus options stated |
|---|---|
| LiDAR | USB or Ethernet |
| IMU | I²C or UART |
| Motor drivers | CAN or RS485 |
| Wheel encoders | Pulse or CAN |
| BMS | CAN or RS485 |
| Safety I/O | Discrete (E-stop, bumper) |

---

## 2. Slide 2 — "IUV CONTROLLER & SOFTWARE TOPOLOGY"

### 2.1 Vehicle headline spec (under product render)

| Parameter | Value |
|---|---|
| Name | IUV |
| Battery | 24 V / 50 Ah |
| Drive | 4WID (4-wheel independent drive) |
| Payload | 100 kg |
| Visual | Yellow, low-profile rectangular chassis; red mushroom E-stop on top; small cylindrical sensor at a front corner (likely 2D LiDAR); black trim; "IUV" and EDC branding on side |

### 2.2 Blocks

| Block | Content as shown |
|---|---|
| **Main Controller** | Industrial PC · Ubuntu 24.04 + ROS 2 Jazzy · Software chips: SLAM / Localization, Motion Planner, Diagnostics, Fleet Client · Role: "High-level autonomy, mission execution, navigation, map handling and system diagnostics" |
| **Fleet Management / Factory IT** | Mission Dispatcher · Traffic Management · Map & Robot Status · API to WMS/MES · IoT · Link badge: Wi-Fi / Ethernet |
| **Real-Time Motion Controller** | MCU / Dedicated Motion Controller · 4WID skid steer kinematics · wheel-speed control · straight line correction · odometry · **100–500 Hz loop** |
| **Sensors & Safety Inputs** | 2D LiDAR — Obstacle detection / mapping · IMU — Yaw rate / acceleration · Wheel Encoders ×4 — Wheel speed / odometry · E-Stop + Bumper — Hardwired safety chain · Safety Sensors — Front/rear obstacle zone |
| **Actuation: Drive Drivers ×2** | 24 V motor power stage · CAN / RS485 / PWM |
| **Actuation: Hub Motors ×4** | 24 V · 25 A · 15 Nm · 370 RPM |
| **Power & Safety Distribution** | 24 V / 50 Ah Battery → BMS → Fuse / Contactor → 24 V Motor Bus · DC/DC 24 → 12 V / 5 V for controller & sensors · E-Stop interrupts traction power |

### 2.3 Connections (edges)

| From | To | Label |
|---|---|---|
| Main Controller | Fleet Management / Factory IT | Wi-Fi / Ethernet |
| Main Controller | Real-Time Motion Controller | ROS2 / UART / CAN |
| Sensors & Safety Inputs (all) | Real-Time Motion Controller | Sensor data |
| Real-Time Motion Controller | Drive Drivers ×2 | CAN / RS485 |
| Real-Time Motion Controller | Drive Drivers ×2 | PWM / CAN |
| Real-Time Motion Controller | Power & Safety Distribution | 24 V / 5 V *(arrow drawn pointing down to power block; physical power flow is presumably the reverse)* |
| Drive Drivers ×2 | Hub Motors ×4 | Power + command |
| Power & Safety Distribution | Hub Motors ×4 | 24 V bus |

### 2.4 Power path (linearised)

- 24 V / 50 Ah battery
  - → BMS
    - → Fuse / Contactor
      - → 24 V motor bus → Drive drivers → Hub motors ×4
      - → DC/DC → 12 V / 5 V → Industrial PC, MCU, sensors
- E-stop chain → opens contactor → traction power cut

---

## 3. Consolidated system model (both slides merged)

| Tier | Hardware | OS / Runtime | Responsibilities |
|---|---|---|---|
| Fleet / Factory IT | Server (unspecified) | Unspecified | Dispatch, traffic mgmt, map & status, WMS/MES API, IoT |
| High-level compute | Industrial PC | Ubuntu 24.04, ROS 2 Jazzy | SLAM, AMCL localization, global/local planning, mode state machine, mission mgmt, HMI/Web UI, diagnostics, OTA, logging |
| Real-time control | MCU / dedicated motion controller | Bare-metal/RTOS (not stated) | Skid-steer kinematics, L/R velocity allocation, wheel-speed loop, encoder+IMU straight-line correction, odometry, 100–500 Hz |
| Power stage | 2× motor drivers (presumably dual-channel) | — | 24 V drive of 4 hub motors |
| Safety | Hardwired E-stop + bumper chain, contactor, "safety sensors" | — | Traction power cut; plus a separate software-level stop |

### 3.1 Inconsistencies between the two slides (flag, do not resolve)

| Topic | Slide 1 | Slide 2 |
|---|---|---|
| Where sensor drivers live | LiDAR/IMU/encoder drivers shown in the ROS 2 stack (implies Industrial PC) | All sensors feed "Sensor data" into the MCU |
| Motion control location | Motion Control Layer sits above ROS 2 middleware (implies ROS 2 nodes) | Kinematics/odometry/straight-line correction in the MCU |
| Drive type wording | "4WID" and "Skid steer / differential drive" | "4WID skid steer" — no independent steering; 4WID here means 4 independently driven fixed wheels |
| Drivers vs motors | "Motor Driver Node" (single) | 2 drivers for 4 motors |

---

## 4. Derived values (computed from slide numbers — not stated on slides)

| Quantity | Value | Note |
|---|---|---|
| Battery energy | ~1.2 kWh | 24 V × 50 Ah, nominal |
| Motor electrical input at 25 A | ~600 W each, ~2.4 kW total | If 25 A is rated/continuous |
| Total motor current at 25 A each | ~100 A | ~2C on a 50 Ah pack |
| Mechanical power if 15 Nm and 370 RPM simultaneous | ~580 W each | ≈97% of 600 W input → unrealistic; 15 Nm is likely peak/stall and 370 RPM likely no-load |
| Runtime at full 2.4 kW draw | ~30 min | Worst case, ignores BMS cutoff and depth-of-discharge |
| Top speed | Not computable | Wheel diameter not given |

---

## 5. Observations relevant to "below industrial grade" (inferred / absent from slides)

| Area | What slides show or omit | Why it matters for industrial deployment |
|---|---|---|
| Safety standard | No ISO 3691-4, ISO 13849 PL, IEC 61508 SIL, or EN 1175 reference | No evidence of a certified safety function |
| Safety scanner | "2D LiDAR" and generic "Safety sensors"; no safety-rated scanner model or protective/warning field switching | Speed-dependent protective fields typically require a safety-rated laser scanner |
| Safety logic | "Safety Stop Logic — software-level stop"; hardware side is E-stop + bumper + contactor only | No safety PLC/relay, no STO on drivers, no safe speed monitoring |
| IMU bus | I²C option | Board-level bus, short range, EMI-susceptible in a vehicle harness |
| LiDAR bus | USB option | Connector retention and EMI robustness poor vs Ethernet/M12 |
| Motor command | PWM option to drivers | Analog-style command, no feedback/diagnostics vs fieldbus |
| Motors | 24 V hub motors, 15 Nm, 370 RPM | Consumer/light-duty class; no gearbox, no brake mentioned |
| Brakes | Not shown | No holding/parking brake on slopes or power loss |
| Payload | 100 kg | Light for automotive intralogistics |
| Voltage | 24 V system, ~100 A peak bus | High current, large cabling/contactor; 48 V is common for industrial AMRs |
| Charging | Not shown | No auto-docking charger or charging contacts described (though "dock" mode exists) |
| Environmental | No IP rating, temperature range, or EMC compliance stated | — |
| Localization robustness | 2D LiDAR + AMCL/SLAM only | No secondary reference (QR/reflector/UWB) for dynamic or feature-poor floors |
| Fleet comms | Wi-Fi / Ethernet; no roaming or loss-of-comms behaviour described | — |
| Brand/part numbers | None given for IPC, MCU, LiDAR, IMU, drivers, motors, BMS | Cannot verify grade of any component |

---

## 6. Keywords for retrieval

`AMR` `4WID` `skid steer` `ROS 2 Jazzy` `Ubuntu 24.04` `Industrial PC` `MCU motion controller` `100–500 Hz` `2D LiDAR` `AMCL` `SLAM` `IMU` `wheel encoders` `CAN` `RS485` `UART` `I2C` `PWM` `BMS` `24V 50Ah` `hub motor 15Nm 370RPM` `E-stop contactor` `fleet management` `WMS/MES API` `OTA` `rosbag2` `TF2` `lifecycle nodes`
