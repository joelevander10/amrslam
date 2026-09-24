#!/bin/bash
set -e
SP="$(dirname "$(readlink -f "$0")")"

echo "=== 0. backup of /etc/netplan ==="
tar czf /root/netplan-backup-$(date +%Y%m%d-%H%M%S).tar.gz -C /etc netplan
ls -1 /root/netplan-backup-*.tar.gz | tail -1

echo
echo "=== 1. existing 50-cloud-init.yaml (checking for a global renderer) ==="
cat /etc/netplan/50-cloud-init.yaml

echo
echo "=== 2. installing 60-rfid-lan1.yaml ==="
install -m 600 -o root -g root "$SP/60-rfid-lan1.yaml" /etc/netplan/60-rfid-lan1.yaml
ls -l /etc/netplan/

echo
echo "=== 3. validating (netplan generate) ==="
netplan generate && echo "  config valid"

echo
echo "=== 4. rendered networkd units ==="
ls -1 /run/systemd/network/ 2>/dev/null
grep -H . /run/systemd/network/*enp2s0* 2>/dev/null || true

echo
echo "=== 5. applying ==="
netplan apply
sleep 3

echo
echo "=== 6. VERIFY: addresses ==="
ip -br addr show
echo
echo "=== 7. VERIFY: routes (must still show BOTH original defaults, ==="
echo "===    and NO default via enp2s0) ==="
ip route
echo
echo "=== 8. VERIFY: reader reachable ==="
ping -c 2 -W 1 -I enp2s0 192.168.1.200 | tail -3
echo
echo "=== 9. VERIFY: internet path still alive ==="
ping -c 2 -W 2 1.1.1.1 | tail -3 || echo "  !! check uplink"
echo
echo "=== done — reboot-safe ==="
