#!/bin/bash
# Install or refresh deployment files only. This script deliberately does not
# enable, start, stop or restart any service; cutover is a separate witnessed
# step in RUNBOOK.md after vehicle acceptance.
set -euo pipefail

DEPLOY_DIR=$(cd "$(dirname "$0")" && pwd)
BACKUP_DIR="/var/backups/amr-units/$(date +%Y%m%d-%H%M%S)"
UNITS=(amr.service amr_nav.service amr_mapping.service)

if [[ ${EUID} -ne 0 ]]; then
  echo "run with sudo: sudo $0" >&2
  exit 2
fi

"$DEPLOY_DIR/validate.sh"
install -d -m 0755 "$BACKUP_DIR"
for unit in "${UNITS[@]}"; do
  if [[ -f "/etc/systemd/system/$unit" ]]; then
    cp -a "/etc/systemd/system/$unit" "$BACKUP_DIR/$unit"
  fi
  install -m 0644 "$DEPLOY_DIR/$unit" "/etc/systemd/system/$unit"
done
systemctl list-unit-files 'agv_controller.service' 'amr*.service' --no-legend \
  >"$BACKUP_DIR/enabled-state.txt" || true
systemctl daemon-reload

echo "Installed unit files without changing enabled or running services."
echo "Backup: $BACKUP_DIR"
echo "Follow the witnessed cutover or rollback procedure in $DEPLOY_DIR/../RUNBOOK.md."
