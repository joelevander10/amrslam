# NEXCOM Neu-X102-N97 — RS-485 Interface Reference

**Scope:** COM1 (external DB9) RS-485 operation on Ubuntu/Linux.
**Sources:** Neu-X102/104/1025/1045 Series User Manual (NEXCOM, published June 2026); Neu-X102-N97 datasheet (last update 03/30/2026); Fintek F81439 product documentation; Linux `drivers/tty/serial/8250/8250_fintek.c` (v6.19-rc8).
**Status:** Desk research. Items marked **[VERIFY]** are inferences or documentation conflicts that must be confirmed on hardware before design freeze.

---

## 1. Summary

| Item | Value |
|---|---|
| RS-485 capable ports | **COM1 only** (1 port) |
| Physical connector | DB9 **[VERIFY gender]** |
| Modes | RS-232 / RS-422 / RS-485, mutually exclusive |
| Mode selection | **BIOS only** — no jumper, no DIP switch, no runtime control |
| Direction control | Hardware auto-direction via Super I/O, or `TIOCSRS485` |
| Termination | **Not fitted / not exposed** — external 120 Ω required |
| Biasing | Internal transceiver failsafe — no external bias needed |
| Isolation | **None** |
| Linux device | `/dev/ttyS0` (legacy 8250, ACPI PNP0501) |
| Linux driver | `8250_fintek` — binds, `TIOCSRS485` supported |

---

## 2. Hardware chain

| Stage | Part | Notes |
|---|---|---|
| UART | Fintek **F81804U-I** Super I/O, eSPI-attached | 2 × 16550-compatible UARTs. Legacy I/O-mapped, not USB, not PCIe. |
| Transceiver | Fintek **F81439** multi-protocol transceiver | Programmable monolithic device containing RS-232 and RS-485/RS-422 drivers and receivers. Reconfigurable into eight operating modes via the logic state of three mode-selection pins. |
| External port | 1 × DB9 | RS-232/422/485 |
| Internal tap | `COM1` header — WtoB, 1 × 9-pin, **1.0 mm pitch** | Feeds the DB9. Use this if bringing RS-485 to a terminal block instead. |

COM2 is a separate 1 × 9-pin 1.0 mm header, **RS-232 only**, no multi-protocol transceiver, with `JP1` selecting RI / +5 V / +12 V on pin 9. COM2 cannot be used for RS-485.

### 2.1 Documentation conflict — port location

| Document | COM port listed under |
|---|---|
| Neu-X102-N97 datasheet | **I/O Interface Front** |
| Neu-X102/104/1025/1045 User Manual | **I/O Interface Rear** (callout 8, with remote power switch jack and storage LED) |

**[VERIFY]** The manual's panel photographs are the more reliable of the two. Confirm on the physical unit before cutting a cabinet panel or specifying cable lengths. Note that the manual places DC-in, power button, USB, LAN and HDMI on the *front*, which is the opposite of the datasheet — the two documents appear to have swapped the face labels wholesale, not just for the COM port.

---

## 3. DB9 pinout

| DB9 pin | RS-232 | RS-422 (4-wire) | RS-485 (2-wire half-duplex) |
|---|---|---|---|
| **1** | DCD | TX− | **TR− (Data−, commonly "A")** |
| **2** | RXD | TX+ | **TR+ (Data+, commonly "B")** |
| 3 | TXD | RX+ | — |
| 4 | DTR | RX− | — |
| **5** | GND | GND | **GND (signal reference)** |
| 6 | DSR | — | — |
| 7 | RTS | — | — |
| 8 | CTS | — | — |
| 9 | RI | — | — |

### 3.1 [VERIFY] — the manual's pin table is internally inconsistent

In the Neu-X102 manual, the `COM1` table prints three columns (RS232 / RS422 / RS485) against a single pin column, but the columns do not share a numbering origin:

- The **RS-232 column** is in *header* order — pin 1 = RI, 2 = CTS, 3 = RTS, 4 = DSR, 5 = GND, 6 = DTR, 7 = TXD, 8 = RXD, 9 = DCD. This is the exact reverse of standard DB9 order, consistent with a flat-cable transition from the internal header to the connector.
- The **RS-422/485 columns** follow NEXCOM's standard *DB9* convention — pin 1 = TX−, 2 = TX+, 3 = RX+, 4 = RX−. This matches what NEXCOM publishes for multi-protocol COM ports on other platforms (e.g. NEX 609 COM2).

The table above assumes NEXCOM reused their standard DB9 differential block while the RS-232 column reflects the actual internal header. That is the most probable reading and matches industry convention (Advantech, Digi, NEXCOM all put the 2-wire pair on DB9 pins 1/2 or 3/9). **It is an inference, not a documented statement.**

### 3.2 Bench verification procedure

1. Set BIOS `Onboard Serial Port 1 Mode` = **RS485**. Boot.
2. Transmit a continuous stream: `while true; do echo -n U > /dev/ttyS0; done` at 9600 8N1.
3. Scope or DMM each DB9 pin against pin 5.
   - Active differential pair → that is TR+/TR−.
   - Idle failsafe should read **≥ +200 mV** differential.
4. If activity appears on pins 8/9 rather than 1/2, the differential columns are in header order and the entire mapping is reversed. Wire accordingly.
5. Confirm polarity by connecting to a known-good node; if frames are garbage but the bus is electrically active, swap TR+/TR−.

Do this **before** building any cable harness in quantity.

---

## 4. Mode selection — BIOS only

There is **no RS-485 jumper or DIP switch on this board.** The complete jumper/switch inventory is:

| Designator | Function |
|---|---|
| `CLRCMOS` | Clear CMOS (1×3, 2.0 mm) |
| `LCD_PWR` | LVDS panel voltage 3.3 V / 5 V |
| `JP1` | COM2 RI / +5 V / +12 V select |
| `SW1` | eDP panel resolution (4-pos DIP) |

None of these affect RS-485. The F81439's three mode pins are driven from the Super I/O under BIOS control.

### 4.1 BIOS path

`Del` at POST → **Advanced** → **F81804 Super IO Configuration** → **Serial Port 1 Configuration**

| Setting | Options | Effect |
|---|---|---|
| Serial Port | Enabled / Disabled | Enables UART1 |
| Device Settings | read-only | Displays I/O base address and IRQ — record these for driver verification |
| **Onboard Serial Port 1 Mode** | **RS232 / RS422 / RS485** | Sets the F81439 mode pins. The only RS-485 selector on the platform. |
| **RTS# Auto Flow Control** | Enabled / Disabled | Delegates half-duplex direction control to Super I/O hardware |

### 4.2 Operational consequences

- **Not persistent across a CMOS clear.** A depleted RTC coin cell drops COM1 back to RS-232 and the bus goes silent with no other symptom. On a fielded AGV this is a realistic failure mode — include RTC battery replacement and post-service BIOS verification in the maintenance SLA.
- **Not settable from userspace.** RS-232 ↔ RS-485 switching cannot be scripted. If a deployment needs both, specify two units or an external USB-RS485 adapter.
- **`RTS# Auto Flow Control` should be Enabled.** With it on, the Super I/O asserts the transceiver driver-enable on TX-FIFO-not-empty and releases it after the stop bit. This is sub-bit-time and invisible to the application stack. With it off, direction must be handled via `TIOCSRS485` or manual RTS toggling, the latter being unreliable at Modbus-RTU inter-frame timings.

---

## 5. Termination, biasing, isolation

| Item | Status | Action required |
|---|---|---|
| **Failsafe biasing** | Handled internally. All receivers have advanced failsafe protection preventing oscillation when inputs are unconnected, defaulting to logic-high output. Datasheet feature list states no external biasing resistors are required for RS-422/485. | None. Do not add external bias unless a third-party node demands it. |
| **Termination** | The F81439 *supports* modes with a built-in termination resistor, but the BIOS exposes only RS232/RS422/RS485 with no termination sub-option, and there is no jumper. **Assume not enabled.** | Fit external 120 Ω. |
| **Isolation** | **None.** No galvanic isolation in the chain. DB9 shell is chassis-bonded; chassis is DC-ground referenced through the 12 V input. | Add isolated repeater — see 5.2. |
| **ESD** | ±15 kV IEC 61000-4-2 air gap, ±8 kV contact, at the transceiver. | Protects handling events only, not steady-state ground offsets. |
| **Node count** | Receivers are high-impedance, allowing up to 256 transceivers on a shared bus. | IPC is not the unit-load bottleneck. |

### 5.1 Termination rules for this port

- IPC at a **physical end** of the trunk → fit 120 Ω across TR+/TR−. Easiest inside the DB9 backshell.
- IPC as a **mid-drop** → no termination, and keep the stub as short as mechanically possible.
- Measure idle differential before deciding on bias. Below ~200 mV with the bus idle means another node is loading the line and bias may genuinely be needed.

### 5.2 Isolation recommendation

Insert a DIN-rail isolated RS-485 repeater between the IPC and any bus segment that:

- leaves the cabinet,
- crosses to a different ground reference,
- runs alongside VFD, servo, or contactor wiring,
- is on a moving vehicle with switching drives.

A non-isolated, chassis-referenced port in that environment is the classic source of intermittent CRC errors, progressive transceiver degradation, and eventual silent node death. This is the single highest-value hardening step for an AGV application.

### 5.3 Slew rate

The F81439 has an adjustable slew-rate control pin (slew-controlled outputs minimise reflection and ringing on long or unterminated cables). On this platform the pin is a fixed board-level strap — **not** a BIOS option. You cannot trade EMI performance against data rate in the field.

---

## 6. Linux integration

### 6.1 Device node

The UART is Super I/O / eSPI, enumerated via ACPI PNP (`PNP0501`), so it presents as a **legacy `/dev/ttyS*`** — almost certainly `ttyS0` for COM1. Not `ttyUSB*`.

```
dmesg | grep -i ttyS
setserial -g /dev/ttyS*
```

Cross-check the reported I/O base against the value shown in BIOS under Device Settings.

Default ownership is `root:dialout`. Add the service account to `dialout` or install a udev rule; do not run the control stack as root just for serial access.

### 6.2 Driver binding

The mainline `8250_fintek` driver covers this chip, though not under an obvious name:

- The F81804's Super I/O chip ID reads back as **`0x0215`** in the byte order `8250_fintek` uses — identical to `CHIP_ID_F81966`, which the driver already handles. The hwmon subsystem documents the same ID collision from the opposite byte order (`SIO_F81804_ID 0x1502`, annotated "same for Fintek F81966").
- `fintek_8250_set_rs485_handler()` registers `rs485_config` for `CHIP_ID_F81966`. **`TIOCSRS485` therefore works on this port.**
- Probe sequence: config ports `0x4e` then `0x2e`, entry keys `{0x77, 0xa0, 0x87, 0x67}`, then LDN scan `0x10`–`0x16` matching the UART I/O base.

Verify:

```
ls /sys/bus/pnp/drivers/ | grep fintek
dmesg | grep -i fintek
```

If `CONFIG_SERIAL_8250_FINTEK` is not set, `TIOCSRS485` returns `ENOTTY` and you are limited to BIOS-only control. Stock Ubuntu 22.04 and 24.04 kernels ship it.

### 6.3 Driver behaviour — three gotchas

| # | Behaviour | Impact |
|---|---|---|
| 1 | **No RTS delays on this chip.** The driver advertises `delay_rts_before_send` / `delay_rts_after_send` only when `pdata->index == 0`, but for the F81866/F81966/F81804 family the first UART's LDN is `0x10` — never 0. Delay fields are silently zeroed. | If slaves need a guaranteed inter-frame gap, enforce it in application code, or rely on the transceiver's native auto-direction, which is tighter than software anyway. |
| 2 | **`SER_RS485_RTS_ON_SEND` and `SER_RS485_RTS_AFTER_SEND` must differ.** The driver returns `-EINVAL` if both are set or both cleared — the hardware cannot hold the same RTS level on send and receive. | Set exactly one flag. |
| 3 | **`TIOCSRS485` does not change the electrical mode.** It sets only `RS485_URA` (bit 4) and `RTS_INVERT` (bit 5) in Super I/O register `0xF0` for that LDN, i.e. direction control. The RS-232/422/485 transceiver mode still comes from BIOS. | Both BIOS and (optionally) ioctl must be configured. |

### 6.4 Baud rates

The driver installs a chip-specific `set_termios` that switches the UART clock between 1.8432 / 14.769 / 18.432 / 24 MHz, giving exact divisors at 115200, 921600, 1152000 and 1500000. Standard Modbus rates (9600 / 19200 / 38400 / 57600 / 115200) are unproblematic.

### 6.5 Recommended configuration

**Preferred — hardware direction control:**

1. BIOS: `Onboard Serial Port 1 Mode` = **RS485**
2. BIOS: `RTS# Auto Flow Control` = **Enabled**
3. Application: treat `/dev/ttyS0` as an ordinary serial port. No `TIOCSRS485`, no `rs485_mode` in pyserial, no RTS management. `pyserial` / `pymodbus` / `minimalmodbus` work unmodified.

**Fallback — software direction control:** use `TIOCSRS485` (or pyserial's `rs485_mode`) only if bench testing shows the BIOS auto-flow-control setting is not actually driving the F81439 driver-enable on your board revision. Test this first; it determines the whole software design.

---

## 7. Verification checklist

| # | Check | Rationale |
|---|---|---|
| 1 | Scope the DB9 in RS-485 mode; identify the differential pair | Resolves the pinout inconsistency in §3.1 |
| 2 | Confirm DB9 gender and panel face on the physical unit | Datasheet and manual disagree (§2.1) |
| 3 | `dmesg \| grep 8250_fintek` after boot | Confirms `TIOCSRS485` fallback is available |
| 4 | Confirm BIOS `RTS# Auto Flow Control` actually drives the transceiver DE | Determines software architecture (§6.5) |
| 5 | Loopback against a known-good USB-RS485 adapter at 9600 and 115200 | End-to-end direction-control timing validation |
| 6 | Measure idle differential voltage, bus unterminated | Determines whether external bias is genuinely needed |
| 7 | Long soak at 50 °C ambient with bus active | Rated −5 °C to 50 °C ambient **with 0.7 m/s air flow**; fanless in a sealed AGV enclosure with no forced air is outside spec |
| 8 | Deliberately clear CMOS, re-check port mode, document recovery | Establishes the RTC-battery failure mode and SLA runbook entry |

---

## 8. Certification note

NEXCOM lists the following for this unit:

- CE (EMC: EN 55035 + EN 55032)
- FCC Class A (EMI Part 15B)
- LVD (EN 62368-1)
- UL/CUL (62368-1)

This is an **ITE/AV EMC and safety basis**, not EN 61000-6-2 / 6-4 industrial immunity and emissions, and not EN IEC 60204-1. It provides component-level evidence for a technical file but does not substitute for machine-level testing.

Specific risk: a Class A EMI classification combined with an **unshielded, non-isolated RS-485 port** is a common source of conducted-immunity (EN 61000-4-6) and fast-transient (EN 61000-4-4) failures at machine level. Mitigate by isolating the port (§5.2) and using shielded twisted pair with the shield bonded at one end only.

Request the full Declaration of Conformity and underlying test reports from NEXCOM early — lead times are long and the reports are needed for the technical file, not just the DoC.

---

## 9. Open items

| Item | Owner | Blocking |
|---|---|---|
| DB9 differential pin assignment confirmation | Bench | Cable harness design |
| DB9 gender and panel face | Bench | Cabinet/panel mechanical design |
| BIOS auto-flow-control efficacy | Bench | Software architecture decision |
| Isolated repeater part selection | Electrical | BOM |
| NEXCOM DoC + test reports | Procurement / compliance | CE technical file |
