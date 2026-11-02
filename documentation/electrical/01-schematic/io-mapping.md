# IO mapping: motor drivers, safety PLC, digital IO module

Source: `proj01-sheet-01..09.html` only. PNG, PDF and anything in the root codebase can be stale and were not used.
Terminal-block numbers are left out on purpose. This is just what goes where.

Net names used below: `1P24` / `1N24` = switched 24 V / 0 V (after MCB-04). `1N48` = 48 V return.
Driver 01 = LEFT motor, Driver 02 = RIGHT motor.

---

## 1. Motor drivers (2 × BLVD-KRD, Oriental Motor)

Sheets 02, 04, 06, 07.

### Power (driver power connector, cable LC03D06A)

| Driver | 48 V + | 48 V − | Earth |
|---|---|---|---|
| Driver 01 (left) | 2P48 (battery → MCB-01 → MCB-02, 10 A) | 1N48 | PE |
| Driver 02 (right) | 3P48 (battery → MCB-01 → MCB-03, 10 A) | 1N48 | PE |

The motor itself (UVW and encoder) connects with the Oriental Motor cable CCM010B1AAF. The 28-pin connector below is for control only.

### 28-pin control connector: wired pins

Same on both drivers. Only the pins below are connected.

| Pin | Signal | Goes to | What it is |
|---|---|---|---|
| 5 | CAN_L | CAN L bus (sheet 06) | Drive command and feedback from the IPC |
| 6 | CAN_H | CAN H bus (sheet 06) | same |
| 7 | CAN_GND | CAN GND bus (sheet 06) | same |
| 20 | NET-VIN | 1P24 | 24 V supply for the driver's control and network side |
| 21 | NET-GND | 1N24 | its 0 V |
| 11 | HWTO1+ | Safety PLC **Q1** | Hardwired torque-off channel 1 |
| 12 | HWTO1− | 1N24 | HWTO1 return |
| 26 | HWTO2+ | Safety PLC **Q2** | Hardwired torque-off channel 2 |
| 27 | HWTO2− | 1N24 | HWTO2 return |
| 14 | EDM− | Safety PLC **I3** (driver 01) or **I4** (driver 02) | Feedback so the PLC can check that torque-off really happened |
| 28 | EDM+ | 1P24 | EDM supply |

- Q1 and Q2 each feed both drivers in parallel. Driver 02's HWTO+ is jumpered on the terminal block from Driver 01's.
- The factory jumpers 11–25, 12–26 and 13–27 are **removed** on both drivers.
- Not connected: pins 1–4 (OUT0/OUT1), 8–10 and 22–24 (485 / TR), 13 (0V), 15–19 (IN-COM, IN0–IN3), 25 (+V). The drivers take no commands on their own digital inputs. Motion comes only over CAN.
- All safety wiring is AWG22 orange.

### CAN bus (sheet 06)

One CAN-to-USB adapter plugs into the **IPC USB port** and feeds a single CAN H / L / GND bus. On that bus:
Driver 01, Driver 02, the **MLSE line sensor**, and the **battery BMS**.

---

## 2. Safety PLC (SICK Flexi Soft)

Sheets 03, 07, 08.

- **CPU module**: only 24 V power (A1 = 1P24, A2 = 1N24, in parallel with the XTIO). No field IO is drawn on it.
- **FX3-XTIO module**: all the field safety IO, below.

### Supply

| Terminal | Goes to |
|---|---|
| A1 | 1P24 |
| A2 | 1N24 |

### Inputs

| Terminal | Goes to | What is connected |
|---|---|---|
| X1 → I1 | E-stop push-button contact 1 (NC) | **Emergency stop** (turn-to-release, 2 NC). Test output X1 goes through the first NC pole and returns on I1. |
| X2 → I2 | E-stop push-button contact 2 (NC) | Same E-stop, second pole. X2 goes through it and returns on I2. |
| I3 | Driver 01 EDM− (pin 14) | Left motor driver feedback |
| I4 | Driver 02 EDM− (pin 14) | Right motor driver feedback |
| I5 | Safety lidar OSSD 1 | Lidar safety output, via SICK connection cable |
| I6 | Safety lidar OSSD 2 | Lidar safety output, via SICK connection cable |
| I7 | **MANUAL/AUTO selector switch**, pole 1 (NO) | Fed from 1P24. The sheet labels this line "MANUAL". |
| I8 | **RESET push-button**, pole 1 (NO) | Fed from 1P24 |

The selector and the reset button each have two NO poles. Pole 1 goes to the safety PLC (above). Pole 2 goes to the digital IO module (section 3), wired to 1N24 on the other side.

### Outputs

| Terminal | Goes to | What it does |
|---|---|---|
| Q1 | HWTO1+ on Driver 01 **and** Driver 02 | Safe torque-off channel 1 |
| Q2 | HWTO2+ on Driver 01 **and** Driver 02 | Safe torque-off channel 2 |
| Q3 | Digital IO module **DI08** (sheet 09) | Non-safety status signal. Test pulses must be off for Q3 in the Flexi Soft config. AWG22 orange. |
| Q4 | not connected | spare |

### Safety lidars (sheet 03)

Two SICK nanoScan3 safety lidars (01, 02) are powered from 1P24 / 1N24 through their SICK connection cables. Their OSSD outputs are the signals going to I5 / I6.

---

## 3. Digital IO module (CK-EC5163, 16 DI + 12 relay DO, Ethernet)

Sheets 03, 08, 09. Field wiring is AWG22 blue.

The audio horn (sheet 03, PATLITE) is powered from 1P24 / 1N24. Its COM is jumpered to 1N24 at the horn; a sound plays while its sound input is connected to COM, which DO0 / DO1 do.

### Supply and Ethernet

| Port | Goes to |
|---|---|
| 24V | 1P24 |
| 0V | 1N24 |
| PE | not connected |
| Ethernet IN | **IPC** |
| Ethernet OUT | **RFID reader** |

### Digital inputs

COM0 is tied to 1P24. Each input is a normally-open contact that pulls the input to 1N24 when pressed.

| Input | Device | Notes |
|---|---|---|
| DI00 | **PB START** | Other side to 1N24 |
| DI01 | **PB RESET**, pole 2 (NO) | Same push-button as safety PLC I8 |
| DI02 | **MANUAL/AUTO selector**, pole 2 (NO) | Same switch as safety PLC I7 |
| DI03 | Pendant **FORWARD** (NO) | Pendant = 4 × 1NO buttons, all sharing one 1N24 return |
| DI04 | Pendant **REVERSE** (NO) | |
| DI05 | Pendant **LEFT** (NO) | |
| DI06 | Pendant **RIGHT** (NO) | |
| DI07 | not used | |
| DI08 | Safety PLC **Q3** | COM1 group: Q3 switches 24 V onto the input. AWG22 orange. |
| DI09–DI0F | not used | |
| COM0 | 1P24 | Common for DI00–DI07 |
| COM1 | 1N24 | Common for DI08–DI0F |

### Relay outputs

| Output | Status |
|---|---|
| DO0 | **Audio horn 01, sound 1** (closes 1N24 onto the horn's sound 1 input) |
| DO1 | **Audio horn 01, sound 2** (closes 1N24 onto the horn's sound 2 input) |
| DO2–DO7 | **Spare, nothing connected.** Only the commons are wired (below). |
| DO8–DOB and their commons | Not used |

| Relay common | Goes to |
|---|---|
| C of DO0 / DO1 | 1N24 |
| C of DO2 / DO3 | 1P24 |
| C of DO4 / DO5 | 1N24 |
| C of DO6 / DO7 | 1N24 |

---

## Things the sheets do not settle

- **Lidar OSSD mapping.** Sheet 03 has two lidars, but sheet 08 shows only two lidar inputs (I5 = "OSSD 1", I6 = "OSSD 2") and does not say which lidar each belongs to. It does not show how the second lidar's OSSD outputs are wired.
- **Selector polarity.** Sheet 08 labels the I7 line "MANUAL". It does not say which switch position closes the contact, or what the module's DI02 reads in AUTO.
