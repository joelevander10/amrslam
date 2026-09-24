#!/bin/bash
# Non-persistent test of enp2s0 <-> RFID reader 192.168.1.200
# Nothing here survives a reboot.
IF=enp2s0
HOSTIP=192.168.1.10/24
READER=192.168.1.200

echo "=== 1. bringing $IF up ==="
ip link set "$IF" up
for i in $(seq 1 15); do
    [ "$(cat /sys/class/net/$IF/carrier 2>/dev/null)" = "1" ] && break
    sleep 1
done
echo -n "carrier: "; cat /sys/class/net/$IF/carrier 2>/dev/null || echo "(none)"
ethtool "$IF" | grep -E "Speed|Duplex|Link detected"

if [ "$(cat /sys/class/net/$IF/carrier 2>/dev/null)" != "1" ]; then
    echo
    echo ">>> NO LINK on $IF. Cable unplugged, wrong port, or reader powered off."
    echo ">>> Try the other port (enp3s0) before going further."
    exit 1
fi

echo
echo "=== 2. assigning $HOSTIP (temporary) ==="
ip addr flush dev "$IF"
ip addr add "$HOSTIP" dev "$IF"
ip -br addr show "$IF"

echo
echo "=== 3. ping $READER ==="
ping -c 3 -W 1 -I "$IF" "$READER"

echo
echo "=== 4. ARP (works even if reader ignores ICMP) ==="
arping -c 3 -I "$IF" "$READER" 2>/dev/null || echo "(arping not installed)"
ip neigh show dev "$IF"

echo
echo "=== 5. common RFID reader TCP ports ==="
for p in 23 80 100 502 2022 4001 5084 8080 10001; do
    timeout 1 bash -c "echo >/dev/tcp/$READER/$p" 2>/dev/null && echo "  OPEN: $p"
done
echo "=== done ==="
