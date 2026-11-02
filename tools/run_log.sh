#!/usr/bin/env bash
# Record one AUTO run for analysis: pre-flight checks, a bag of every control-relevant
# topic, a snapshot of the logs/code/profile, then tools/bag_report.py on the result.
#
#   tools/run_log.sh [mission] [destination]      e.g. tools/run_log.sh line-a MRU4
#
# Start it BEFORE pressing Start on the panel; Ctrl-C once when the run is over.
# Read-only towards the vehicle: it never starts/stops amr.service and sends nothing
# to the vehicle. Output: ~/.amr/runs/<stamp>/
set -u
MISSION="${1:-}"
DEST="${2:-}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$HOME/.amr/runs/$STAMP"
mkdir -p "$OUT"

# The service's ROS environment (amr.env), whatever this shell had.
# shellcheck disable=SC1091
set +u  # the ROS setup scripts read unset variables
source "$REPO/amr_ws/install/setup.bash" >/dev/null 2>&1
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_RUN:-10}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

warn() { echo "  WARN  $*" | tee -a "$OUT/preflight.txt"; }
ok()   { echo "  ok    $*" | tee -a "$OUT/preflight.txt"; }
field() { grep -m1 "^$2:" <<<"$1" | sed "s/^$2: *//; s/^'//; s/'$//"; }

echo "pre-flight ($OUT)" | tee "$OUT/preflight.txt"

# 1. The service is up and runs the code on disk (Python is symlinked: a restart loads it).
if systemctl is-active --quiet amr.service; then
    since=$(systemctl show amr.service -p ActiveEnterTimestamp --value)
    since_s=$(date -d "$since" +%s 2>/dev/null || echo 0)
    newest=$(find "$REPO/agv_core" "$REPO/amr_ws/src" "$REPO/profiles" "$REPO/missions" \
             -name '*.py' -o -name '*.json' -o -name '*.yaml' 2>/dev/null \
             | grep -v -e /test/ -e __pycache__ | xargs stat -c '%Y %n' 2>/dev/null | sort -n | tail -1)
    newest_s=${newest%% *}
    if [ "${newest_s:-0}" -gt "$since_s" ]; then
        warn "amr.service started $since but ${newest#* } changed later: RESTART it to run the current code"
    else
        ok "amr.service up since $since, no newer code on disk"
    fi
else
    warn "amr.service is not active"
fi

# 2. What the line layer says right now.
ls=$(timeout 6 ros2 topic echo --once /amr/line_state 2>/dev/null)
if [ -z "$ls" ]; then
    warn "no /amr/line_state (not in LINE mode?)"
else
    m=$(field "$ls" mission); d=$(field "$ls" destination); st=$(field "$ls" state)
    h=$(field "$ls" at_home); hd=$(field "$ls" home_detail); msg=$(field "$ls" message)
    ok "line: state=$st mission='${m:-plain}' destination='${d}' at_home=$h ${hd:+($hd)}"
    ok "line message: $msg"
    [ -n "$MISSION" ] && [ "$m" != "$MISSION" ] && warn "mission is '${m:-plain}', expected '$MISSION': Set job on Run tracked"
    [ -n "$DEST" ] && [ "$d" != "$DEST" ] && warn "destination is '$d', expected '$DEST'"
    # Only a mission with an RFID Home says why not (home_detail); a marker run has none.
    [ -n "$MISSION" ] && [ "$h" != "true" ] && [ -n "$hd" ] && warn "not at Home ($hd)"
fi

# 3. RFID link, mode, drives.
rf=$(timeout 6 ros2 topic echo --once /amr/rfid 2>/dev/null)
[ "$(field "$rf" comms_ok)" = "true" ] && ok "RFID link up" || warn "RFID link down or silent"
md=$(timeout 6 ros2 topic echo --once /amr/mode_state 2>/dev/null)
ok "mode: $(field "$md" mode) (7 = LINE) phase=$(field "$md" phase)"
dr=$(timeout 6 ros2 topic echo --once /drives/status 2>/dev/null)
ok "drives: operational=$(field "$dr" operational) link=$(field "$dr" link_state) power_reason='$(field "$dr" power_reason)'"

# 4. Disk.
free_g=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc 0-9)
[ "${free_g:-0}" -lt 5 ] && warn "only ${free_g} GB free" || ok "${free_g} GB free"

# 5. Snapshot of what is being tested.
{
    echo "stamp $STAMP"; echo "mission ${MISSION:-?} destination ${DEST:-?}"
    git -C "$REPO" rev-parse HEAD; git -C "$REPO" status --short
} > "$OUT/code_state.txt" 2>&1
git -C "$REPO" diff > "$OUT/code.diff" 2>/dev/null
cp "$REPO/profiles/${AGV_PROFILE:-agv-01}.json" "$OUT/profile.json" 2>/dev/null
[ -n "$MISSION" ] && cp "$REPO/missions/$MISSION.json" "$OUT/mission.json" 2>/dev/null
cp "$REPO/missions/empty.json" "$OUT/empty.json" 2>/dev/null

# Every control-relevant topic. Not recorded: scans/raw scanner data, map, particle cloud
# (CPU and size); /output_paths carries the field states. line_cmd is generation-private.
# Localization (tracked plan 3, P0 tool T2): /amcl_pose, /tf (map->odom; odom->base is small
# beside it) and the readiness state, whenever AMCL runs (p0_localization or tape_loc).
TOPICS='^/amr/(line_state|line_track|line_marker|rfid|mux_state|panel_state|io|events|mode_state|control_lease|run_state|localization_state)$'
TOPICS+='|^/amr/layers/g[0-9]+/amr/line_cmd$'
TOPICS+='|^/(cmd_wheel_vel|wheel_states|drives/status|imu/data|odom_raw|output_paths|rosout|diagnostics)$'
TOPICS+='|^/(amcl_pose|tf)$'

# CPU and temperature beside the bag (P0 M5): pidstat per process every 5 s, the package
# temperature every 5 s. Both stop with the recorder.
if command -v pidstat >/dev/null; then
    pidstat -u -h 5 > "$OUT/pidstat.txt" 2>&1 &
    PIDSTAT=$!
else
    warn "pidstat not installed (apt install sysstat): no per-process CPU for P0 M5"
    PIDSTAT=
fi
(
    zone=$(grep -l -m1 -e x86_pkg_temp -e coretemp /sys/class/thermal/thermal_zone*/type 2>/dev/null | head -1)
    zone=${zone%/type}
    echo "t,cpu_busy_pct,temp_c" > "$OUT/cpu_temp.csv"
    read -r _ a b c d e f g _ < /proc/stat
    while sleep 5; do
        read -r _ a2 b2 c2 d2 e2 f2 g2 _ < /proc/stat
        busy=$(( (a2-a)+(b2-b)+(c2-c)+(f2-f)+(g2-g) )); idle=$(( (d2-d)+(e2-e) ))
        temp=$([ -n "$zone" ] && awk '{printf "%.1f", $1/1000}' "$zone/temp" || echo "")
        echo "$(date +%s),$(( 100*busy/(busy+idle) )),$temp" >> "$OUT/cpu_temp.csv"
        a=$a2 b=$b2 c=$c2 d=$d2 e=$e2 f=$f2 g=$g2
    done
) &
CPUMON=$!

echo
echo "recording to $OUT/bag  -  press Start on the panel when ready, Ctrl-C ONCE when done"
T0=$(date +%s.%N)
ros2 bag record -s mcap -o "$OUT/bag" -e "$TOPICS" --include-hidden-topics \
    > "$OUT/record.log" 2>&1 &
REC=$!
trap 'echo; echo "stopping the recorder..."; kill -INT $REC 2>/dev/null' INT
wait $REC
trap - INT
wait $REC 2>/dev/null
T1=$(date +%s.%N)
kill "$CPUMON" ${PIDSTAT:+"$PIDSTAT"} 2>/dev/null

# Logs for the run window (events.jsonl lines by their own timestamp).
python3 - "$T0" "$T1" "$OUT" <<'EOF'
import json, os, sys
t0, t1, out = float(sys.argv[1]) - 5, float(sys.argv[2]) + 5, sys.argv[3]
src = os.path.expanduser("~/.amr/logs/events.jsonl")
n = 0
with open(src) as f, open(os.path.join(out, "events.jsonl"), "w") as g:
    for line in f:
        try:
            if t0 <= json.loads(line)["t"] <= t1:
                g.write(line); n += 1
        except (ValueError, KeyError):
            pass
print(f"{n} events copied")
EOF
for f in layer.log base.log web.log; do
    tail -n 400 "$HOME/.amr/logs/$f" > "$OUT/$f" 2>/dev/null
done

echo "report..."
python3 "$REPO/tools/bag_report.py" "$OUT" | tee "$OUT/report.txt"
echo
echo "done: $OUT"
