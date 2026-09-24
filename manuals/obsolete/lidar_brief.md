# Brief — Reading the nanoScan3 from the AGV controller PC

**For:** the agent working on the Linux vehicle-controller PC
**Device:** SICK nanoScan3 Core I/O, `NANS3-AAAZ30AN1`, with Ethernet system plug `NANSX-AAABAEZZ1`
**Status:** scanner is configured, verified and streaming. Network link is up and UDP has been confirmed arriving.

---

## 0. Rules that are not negotiable

Read these before writing any code.

1. **This data is not a safety function and must never stop the vehicle.** The stop comes from the scanner's OSSD pair, hardwired into an FX3 safety controller, which drops the drives' STO. The manual states it plainly: *"This data must not be used for safety-related applications."* Everything below is for navigation, speed setpoints, HMI and logging.
2. **Do not change the scanner's configuration.** Not the IP, not the fields, not the data output settings. The device carries a verified safety configuration and altering it invalidates the verification and the CE evidence. You have read-only business here. If something needs changing on the device, report it — a human does it in Safety Designer on Windows and re-verifies.
3. **Do not touch the FX3 safety controller.** It is not on this network and is not yours.
4. **Absence of data is never "clear".** See §4. This is the single most likely way to write a dangerous bug against a non-safety data source.

---

## 1. What is already set up

| Item | Value |
|---|---|
| Scanner IP | `192.168.3.10` / `255.255.255.0` |
| This PC | `192.168.3.2` / `255.255.255.0` |
| UDP stream target | `192.168.3.2:6060` |
| Send mode | On request **and** continuously to target computer |
| Rate | Every measurement — scan cycle 30 ms, scan rate 33 Hz |
| Angular range | −47.5° … +227.5° (full 275°) |
| Angular resolution | 0.17°, **1651 measured values per scan** |
| TCP CoLa 2 (config/on-request) | port `2122` — leave alone |

Enabled telegram blocks:

| Block | On? | Carries |
|---|---|---|
| Device Status | **yes** | cut-off path states, errors, contamination, active monitoring case |
| Configuration of Data Output | **yes** | the angular range actually used |
| Measurement Data | **yes** | 1651 distances + RSSI + reflector flag |
| Object detection | no | which beams inside fields saw an object |
| Application Data | no | I/O states used in the monitoring case table |
| Local I/Os | **yes** | physical state of the scanner's local inputs and outputs |

---

## 2. Testing, in order

Do not skip to step 5. Each step isolates one failure.

### 2.1 Prove the stream exists

```
sudo tcpdump -i <iface> -n udp port 6060
```

Expect a steady flow from `192.168.3.10`. If it is silent:

- Ping `192.168.3.10` — if ping works but no UDP, the problem is scanner-side, not network.
- The most likely cause is that the scanner's **safety application is stopped**. That is a Windows/Safety Designer state; report it, do not chase it in network config.
- Check the firewall allows inbound UDP 6060.

### 2.2 Measure the rate

```
sudo tcpdump -i <iface> -n udp port 6060 -c 200 -tt
```

Compute inter-arrival times. Expect roughly **30 ms**, i.e. ~33 telegrams/s. Note that a single scan's telegram exceeds the MTU and arrives **fragmented across several datagrams** — count reassembled telegrams, not packets. If you see a multiple of 30 ms, the send-mode divisor has been changed or datagrams are being dropped.

### 2.3 Do not hand-roll the parser

Use SICK's own decoder:

- **ROS 2:** `sick_safetyscanners2`
- **Plain C++:** `sick_safetyscanners_base`

Both speak this exact protocol — a TCP CoLa 2 session on port 2122 for setup, then the UDP stream. Configure with `sensor_ip = 192.168.3.10`, `host_ip = 192.168.3.2`, `host_udp_port = 6060`.

The authoritative wire format is SICK technical information **"microScan3, outdoorScan3, nanoScan3: Data output via UDP and TCP/IP", part number 8022706**. It is not in the repo — fetch it before interpreting any field you are unsure about. Do not guess at byte offsets.

### 2.4 Sanity-check the geometry

Before trusting anything:

- Point count per scan is **1651**.
- Angular span matches the *Configuration of Data Output* block, not what you assumed. The manual warns data may be output from a slightly larger angle range than configured — take the range from the telegram, never hardcode it.
- Zero degrees is the scanner's forward axis; the 275° sector is centred such that the mechanical dead zone sits behind. Verify empirically with a target at a known bearing before you trust any transform.

### 2.5 Validate the three zone flags by hand

Cut-off path indices come from the device's verification report, §5.9:

| Index | Path | Field type | Meaning |
|---|---|---|---|
| 1 | `stop` | protective, 2.15 m max | inner — this is also what drops the OSSD |
| 2 | `slow` | warning | middle |
| 3 | `warn` | warning | outer |

Walk an object inward through all three rings and confirm each index changes state in the order you expect, one at a time. Log the raw values. **Do not assume the ordering** — confirm it.

### 2.6 Validate the watchdog

Unplug the Ethernet cable while the reader is running. Confirm the controller falls to its safe default (§4) rather than latching the last-known-good state. This test is more important than any of the above.

---

## 3. What a proper AGV controller should read

The three zone flags are the minimum. A real controller uses more.

### 3.1 Measurement data — 1651 points, 33 Hz

The main reason a safety scanner has an Ethernet port at all.

- **Localisation / scan matching** against a map — AMCL, Cartographer, KISS-ICP or similar. This is normally the primary pose source on an indoor AGV.
- **Obstacle handling beyond the fields.** The protective and warning fields are fixed polygons. The point cloud sees to 40 m and lets the planner slow or re-route for something at 6 m that no field would ever catch.
- **Reflector detection.** Each point carries RSSI and a reflector flag, which supports reflector-marker localisation. Note the datasheet caveat: *distance* accuracy degrades on retroreflective surfaces because the range measurement is tuned for low remission. Use the flag for identification, treat the range on those points with suspicion.

### 3.2 Contamination status

In Device Status. Two levels: contamination **warning** (still running, clean it soon) and contamination **error** (outputs off, clean it now). Surface the warning to fleet management so the vehicle schedules itself for cleaning between shifts instead of failing mid-route. This is free availability.

### 3.3 Device error vs application error

They mean different things and need different operator responses:

- **Device error** — serious; all safety outputs off, device enters locking state, requires a **complete restart** after the cause is fixed.
- **Application error** — safety outputs off, but only the **safety function** needs restarting.

Do not collapse them into a single "lidar fault" flag.

### 3.4 Active monitoring case

Currently there is only one, so this looks pointless. It will not stay that way — speed-dependent field switching is planned, driven by the FX3 from safe encoders.

Read it for two reasons:

1. **Interpretation.** Cut-off path 2 means "whatever field is assigned to path 2 in the *currently active* monitoring case." Once cases differ, the same index means different geometry. Structure the decoder to key on (monitoring case, path index) from the start rather than retrofitting.
2. **Cross-check.** If the controller believes it requested slow-speed operation but the scanner reports the fast-speed case active, something is wrong in the FX3 chain. That mismatch is worth alarming on.

### 3.5 Scan timestamp and scan counter

- **Sequence gaps** tell you datagrams are being dropped — a network or CPU-load problem you otherwise will not notice.
- **Timestamps** let you time-align the scan with wheel odometry, which is required for correct scan matching. Without it, fusion at speed produces a smeared map.
- The scanner supports **SNTP**, so its clock can be disciplined to the same source as this PC. Worth doing before any serious mapping work.

### 3.6 Configuration checksum and device identity

The telegram carries device identification and the configuration checksum. At startup, compare against the expected values and **refuse to enter automatic mode on mismatch**.

This catches a swapped scanner, a scanner running an unverified or altered configuration, and a system plug moved between units. It is cheap, and it directly supports the project's configuration-management gap (finding F-06).

### 3.7 Consider enabling "Object detection"

Currently off. It reports *which beams inside the fields* detected an object. That gives you bearing-to-obstacle within a field, so the planner can distinguish "obstacle to the left, steer right" from "obstacle dead ahead, stop" instead of treating the whole warning ring as one boolean.

Enabling it is a scanner-side configuration change — request it, do not attempt it.

---

## 4. The watchdog rule

Write this before you write anything else that consumes the data.

```
STALE_TIMEOUT = 0.15   # ~5 scans at 30 ms; tune against observed jitter

if (now - last_valid_telegram) > STALE_TIMEOUT:
    zone_state       = INNERMOST_OCCUPIED
    target_speed     = 0
    localisation_ok  = False
```

Missing data means the scanner, cable, switch or this process has failed. It does **not** mean the path is clear. Latching the last-known-good state is the bug that kills someone.

Apply the same rule to individual fields within the telegram: if a block is absent or fails its own validity check, treat that block's information as worst-case, not as unchanged.

---

## 5. Known numbers

| Parameter | Value |
|---|---|
| Scan cycle time | 30 ms |
| Scan rate | 33 Hz |
| Measured values per scan | 1651 |
| Angular resolution | 0.17° |
| Scanning angle | 275° |
| Protective field range | **2.15 m** (capped by the 50 mm object resolution, not 3 m) |
| Warning field range | 10 m |
| Distance measurement range | 40 m |
| Safety response time | 130 ms (4× multiple sampling × 30 ms + 10 ms OSSD) |
| Systematic error | ±10 mm |
| Total error at 3σ | ±19 mm |
| Detectable remission | 1.8% to several 1000% |

The protective field figure matters: do not design planner behaviour around a 3 m safety stop. The scanner will stop the vehicle at **2.15 m** at most, and the vehicle then needs its own braking distance on top of the 130 ms.

---

## 6. Report back

- Which decoder library you used and its version
- Observed telegram rate and any sequence gaps over a 10-minute soak
- The raw values you logged for each cut-off path during the §2.5 walk-through
- Result of the §2.6 cable-pull test
- Anything you believe needs changing **on the scanner** — as a request, not an action
