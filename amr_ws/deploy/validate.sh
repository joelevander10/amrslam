#!/bin/bash
# P8: cheap deployment validation. No service state or hardware is changed.
set -euo pipefail

DEPLOY_DIR=$(cd "$(dirname "$0")" && pwd)
WORKSPACE=$(cd "$DEPLOY_DIR/.." && pwd)
UNITS=(amr.service amr_nav.service amr_mapping.service)

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
# (or Environment=). Found 2026-09-16: amr.env lost AMR_MAP_ID while amr_nav.service
# still used it, and the installed unit restart-looped on "malformed launch argument".
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

echo "deployment validation passed"
