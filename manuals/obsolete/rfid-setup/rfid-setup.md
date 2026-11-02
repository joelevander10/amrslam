# RFID Reader Link (LAN1) — Setup & Operation Guide

Bring-up, verification and soak-testing of the ethernet link between the AGV
controller PC and the RFID reader.

**On this vehicle (agv-01) the link is already set up and reboot-safe.** This
guide exists to re-commission it after a reader swap, a cable change, an OS
reinstall, or when provisioning a second vehicle.

Design rationale — why each setting is what it is — is in
[`../rfid-link-notes.txt`](../rfid-link-notes.txt). This file is the *how*; that
file is the *why*. Read that one before changing any value here.

---

## The four files

| File | Lives at | Run as | When |
|---|---|---|---|
| `rfid-test.sh` | `manuals/rfid-setup/` | **root** | First. Non-destructive probe. |
| `60-rfid-lan1.yaml` | `manuals/rfid-setup/` | *(not run)* | Reference copy of the netplan config. |
| `rfid-persist.sh` | `manuals/rfid-setup/` | **root** | Second. Installs the config permanently. |
| `rfid-link-watch.sh` | `logs/` | normal user | Ongoing. Soak monitor. |

`rfid-link-watch.sh` sits in `logs/` rather than here on purpose: it follows the
existing `logs/pwrbtn-watch.sh` precedent and writes its log beside itself.

`60-rfid-lan1.yaml` is a **reference copy**. The live file is
`/etc/netplan/60-rfid-lan1.yaml`, installed by `rfid-persist.sh`. Editing the
copy in this directory changes nothing until you re-run that script.

> `sudo` on this host requires a password — there is no NOPASSWD rule. Run the
> two root scripts from a terminal you can type into.

---

## Step 1 — Probe the link (non-destructive)

Nothing here survives a reboot and nothing is written to disk. Safe to run any
time, including on a vehicle already in service.

```bash
sudo bash ~/agv_can/manuals/rfid-setup/rfid-test.sh
```

It brings `enp2s0` up, waits up to 15 s for carrier, assigns `192.168.1.10/24`
temporarily, then pings the reader, ARPs it, and scans the common RFID ports.

**Expected output on a healthy link:**

```
carrier: 1
Speed: 100Mb/s   Duplex: Full   Link detected: yes
3 packets transmitted, 3 received, 0% packet loss
192.168.1.200 dev enp2s0 lladdr 8c:4b:41:3b:cc:a4 REACHABLE
  OPEN: 2022
```

**If it reports `NO LINK`:** the script stops there deliberately — everything
after it would fail confusingly. Check, in this order: reader powered, cable
seated at both ends, and then try the other port. `enp2s0` vs `enp3s0` was
determined here by carrier and ARP, **not** by the case labelling — do not
assume the silkscreen "LAN1" matches. To test the other port, edit `IF=enp2s0`
at the top of the script.

**If carrier is up but the ping fails:** the reader is on a different subnet or
a different IP. Nothing downstream will work until that is resolved.

---

## Step 2 — Make it permanent

```bash
sudo bash ~/agv_can/manuals/rfid-setup/rfid-persist.sh
```

The script is ordered so that nothing destructive happens before it is safe:

1. Tars `/etc/netplan` to `/root/netplan-backup-<timestamp>.tar.gz`
2. Prints the existing `50-cloud-init.yaml` (so a global `renderer:` is visible)
3. Installs `60-rfid-lan1.yaml` to `/etc/netplan/`, mode `0600`
4. Validates with `netplan generate` — **stops on invalid config, before applying**
5. Shows the rendered `systemd-networkd` unit
6. Applies
7. Verifies addresses, routes, reader reachability, and the internet path

It reads `60-rfid-lan1.yaml` from **its own directory**, so the script and the
yaml must stay together.

**The three things to check in the output:**

```
=== 3. validating ===        config valid
=== 7. VERIFY: routes ===    both original defaults still present,
                             and NO "default via ... enp2s0"
=== 9. VERIFY: internet ===  0% packet loss
```

Route check is the one that matters. The config deliberately sets **no gateway
and no DNS** — this box already carries two default routes (`usb0`, the primary
uplink, and `wlp1s0`). A third would blackhole traffic.

`WARNING:root:Cannot call Open vSwitch: ovsdb-server.service is not running` is
**benign**. There is no OVS on this host; netplan mentions it unconditionally.

### If it goes wrong

```bash
sudo tar xzf /root/netplan-backup-<timestamp>.tar.gz -C /etc
sudo netplan apply
```

If you lose the network entirely, note that `usb0` may be your remote access
path — **run step 2 from a local console**, not over the tether.

---

## Step 3 — Soak test (the step people skip)

```bash
nohup ~/agv_can/logs/rfid-link-watch.sh >/dev/null 2>&1 &
```

Samples once a second: carrier, link speed, ICMP RTT, TCP reachability of
`:2022`, and summed NIC error counters. Needs no root. Appends to
`logs/rfid-link-watch.log` and survives logout.

Check for faults — transitions are marked `***` so they are greppable:

```bash
grep '\*\*\*' ~/agv_can/logs/rfid-link-watch.log     # flaps and error-counter ticks
tail -f ~/agv_can/logs/rfid-link-watch.log           # live
```

Stop it:

```bash
pkill -f rfid-link-watch.sh
```

**Run this while the vehicle is actually driving.** The commissioning
measurements were all taken parked, which proves the cable is good — not that
the *installation* is. Vibration-induced connector faults and EMI from the drive
cabling do not exist at standstill. Several hours of moving soak with zero `***`
lines is what justifies trusting this link.

A healthy sample:

```
2026-08-31T18:49:55+07:00 up=14652 carrier=1 speed=100 rtt_ms=0.068 tcp=ok errs=0
```

| Field | Meaning if it goes bad |
|---|---|
| `carrier=0` | Cable/connector or reader power. Physical, not software. |
| `rtt_ms=LOSS` | Carrier up but no ICMP reply — reader wedged or IP changed. |
| `tcp=REFUSED` | Ping works but port 2022 is shut — reader application crashed. |
| `errs=` rising | Cable damage or EMI. Reroute away from motor power. |

---

## Quick reference

```bash
# Where things are
/etc/netplan/60-rfid-lan1.yaml          # live config
~/agv_can/manuals/rfid-setup/           # scripts + reference copy of the yaml
~/agv_can/logs/rfid-link-watch.log      # soak log
/root/netplan-backup-*.tar.gz           # pre-change backups

# Current state, no changes made
ip -br addr show enp2s0
ip route                                # must show NO default via enp2s0
cat /sys/class/net/enp2s0/carrier       # 1 = cable good
ping -c 3 -I enp2s0 192.168.1.200
ethtool -S enp2s0 | grep -viE " 0$"     # any non-zero error counter

# Reapply live config after hand-editing /etc/netplan
sudo netplan generate && sudo netplan apply
```

**`nmcli` will not work on this interface.** NetworkManager is restricted to
wifi by `/usr/lib/NetworkManager/conf.d/10-globally-managed-devices.conf`
(`unmanaged-devices=*,except:type:wifi,...`). Every ethernet port belongs to
netplan → systemd-networkd. `nmcli` commands against `enp2s0` appear to succeed
and silently do nothing. This is the single most time-wasting trap on this host.

---

## Reference values (agv-01, commissioned 2026-08-31)

| | |
|---|---|
| Interface | `enp2s0` (LAN1), Intel i225/i226, driver `igc` |
| Host IP | `192.168.1.10/24`, static, no gateway, no DNS |
| Reader | `192.168.1.200`, MAC `8c:4b:41:3b:cc:a4`, TCP **2022** |
| Topology | direct point-to-point, **no switch** |
| Link | 100BASE-TX full duplex, autonegotiated |
| Baseline RTT | 0.096 ms avg, 0.010 ms jitter, 0% loss over 50 packets |

Keep the link **direct**. On a point-to-point run, carrier is a true proxy for
"the reader is physically there" — and carrier loss is detected in milliseconds,
far faster than any TCP mechanism. Insert a switch and a yanked reader-side
cable leaves our carrier up and lying to us.

---

## What is not yet built

The link is commissioned; **nothing reads tags yet.** Still open:

- Reader make/model unknown (OUI `8c:4b:41` did not resolve). The protocol
  framing on port 2022 blocks the client implementation.
- The reader is **command/response** — it sends nothing unsolicited, so
  something must poll it.
- Application-layer integration: TCP keepalive / `TCP_USER_TIMEOUT` hardening, a
  dedicated socket-owning thread, and the two distinct fault paths
  (`rfid_comms_lost` vs `rfid_tag_overdue`). See sections 5 and 6 of
  [`../rfid-link-notes.txt`](../rfid-link-notes.txt) — a stock TCP socket would
  let the vehicle drive 3.6 km past a branch point still believing it was
  connected.
