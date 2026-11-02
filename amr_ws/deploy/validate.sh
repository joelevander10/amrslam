#!/bin/bash
# P8: cheap deployment validation. No service state or hardware is changed.
set -euo pipefail

DEPLOY_DIR=$(cd "$(dirname "$0")" && pwd)
WORKSPACE=$(cd "$DEPLOY_DIR/.." && pwd)
UNITS=(amr.service)

for path in \
  "$DEPLOY_DIR/amr-supervisor.sh" \
  "$DEPLOY_DIR/amr-launch.sh" \
  "$DEPLOY_DIR/install.sh" \
  "$WORKSPACE/install/setup.bash" \
  "$WORKSPACE/env/vehicle.sh"; do
  [[ -f "$path" ]] || { echo "missing required path: $path" >&2; exit 1; }
done

bash -n "$DEPLOY_DIR/amr-supervisor.sh" "$DEPLOY_DIR/amr-launch.sh" "$DEPLOY_DIR/install.sh"
for unit in "${UNITS[@]}"; do
  systemd-analyze verify "$DEPLOY_DIR/$unit"
done

# Every ${VAR} a unit's Exec lines use must be defined by that unit's EnvironmentFile
# (or Environment=). Found 2026-09-16: amr.env lost AMR_MAP_ID while the (since
# retired) amr_nav.service still used it, and the installed unit restart-looped.
for unit in "${UNITS[@]}"; do
  envfile=$(sed -n 's/^EnvironmentFile=-\{0,1\}//p' "$DEPLOY_DIR/$unit" | head -1)
  envfile_local="$DEPLOY_DIR/$(basename "${envfile:-/nonexistent}")"
  defined=$( { [[ -f "$envfile_local" ]] && sed -n 's/^\([A-Za-z_][A-Za-z0-9_]*\)=.*/\1/p' "$envfile_local"; \
               sed -n 's/^Environment=\([A-Za-z_][A-Za-z0-9_]*\)=.*/\1/p' "$DEPLOY_DIR/$unit"; } | sort -u)
  for var in $(grep -E '^Exec' "$DEPLOY_DIR/$unit" | grep -oE '\$\{[A-Za-z_][A-Za-z0-9_]*\}' | tr -d '${}' | sort -u); do
    if ! grep -qx "$var" <<<"$defined"; then
      echo "$unit uses \${$var} but $(basename "${envfile:-<no EnvironmentFile>}") does not define it" >&2
      exit 1
    fi
  done
done

if grep -Eq '^[[:space:]]*systemctl[[:space:]]+(enable|disable|start|stop|restart)' \
  "$DEPLOY_DIR/install.sh"; then
  echo "installer must not change service enable/run state" >&2
  exit 1
fi

# The vehicle library must be where amr-launch.sh will put it on PYTHONPATH.
# Without agv_core every amr_base node dies on its first import, and the unit
# restart-loops with a traceback that names no unit file - so check it here,
# where the message can say what is actually wrong.
REPO_ROOT=$(cd "$DEPLOY_DIR/../.." && pwd)
for required in "agv_core/__init__.py" "agv_core/config.py" "profiles"; do
  if [[ ! -e "$REPO_ROOT/$required" ]]; then
    echo "vehicle library incomplete: $REPO_ROOT/$required is missing" >&2
    echo "amr-launch.sh puts $REPO_ROOT on PYTHONPATH; agv_core must live there" >&2
    exit 1
  fi
done

# The watchdog is only real if the unit asks for it AND the process is told the
# timeout: Type=notify without WatchdogSec is a no-op, and sdnotify.py then pings
# nothing (power-loss plan W5).
for unit in "${UNITS[@]}"; do
  if grep -q '^Type=notify' "$DEPLOY_DIR/$unit"; then
    grep -q '^WatchdogSec=' "$DEPLOY_DIR/$unit" || {
      echo "$unit is Type=notify but sets no WatchdogSec: the watchdog does nothing" >&2; exit 1; }
    grep -q '^NotifyAccess=' "$DEPLOY_DIR/$unit" || {
      echo "$unit is Type=notify but sets no NotifyAccess" >&2; exit 1; }
  fi
done
if [[ -f /etc/systemd/system/amr.service ]]; then
  live=$(systemctl show -p WatchdogUSec --value amr.service 2>/dev/null || echo 0)
  case "$live" in
    0|""|0s) echo "note: installed amr.service has no watchdog yet (run install.sh, then restart)";;
    *) echo "watchdog armed on the installed unit: $live";;
  esac
fi

# Journal size (W7): persistent and uncapped is how a root filesystem fills up.
if command -v journalctl >/dev/null; then
  usage=$(journalctl --disk-usage 2>/dev/null | grep -oE '[0-9.]+[KMG]' | head -1 || true)
  if [[ -n "$usage" ]]; then
    mb=$(awk -v u="$usage" 'BEGIN{n=u+0; s=substr(u,length(u));
         print (s=="G")? n*1024 : (s=="K")? n/1024 : n}')
    if (( $(awk -v m="$mb" 'BEGIN{print (m>400)}') )); then
      echo "warning: journal is $usage (cap is 300M); install journald-amr.conf and" >&2
      echo "         run: sudo journalctl --vacuum-size=300M" >&2
    else
      echo "journal on disk: $usage"
    fi
  fi
fi

# Where the state lives (W8). Not a failure on the bench - the data volume is
# optional - but a vehicle should not keep its maps on the root filesystem.
STATE_DIR=$(sed -n 's/^AMR_STATE_DIR=//p' "$DEPLOY_DIR/amr.env" | head -1)
STATE_DIR=${STATE_DIR/#\~/$HOME}
if [[ -n "$STATE_DIR" ]] && command -v findmnt >/dev/null; then
  src=$(findmnt -no TARGET --target "$STATE_DIR" 2>/dev/null || echo /)
  opts=$(findmnt -no OPTIONS --target "$STATE_DIR" 2>/dev/null || echo "")
  if [[ "$src" == "/" ]]; then
    echo "note: AMR_STATE_DIR ($STATE_DIR) is on the root filesystem; see RUNBOOK section 5 (data volume)"
  elif [[ "$opts" != *data=journal* ]]; then
    echo "note: $src is a separate mount but not data=journal"
  else
    echo "state on $src with data=journal"
  fi
fi

echo "deployment validation passed"
