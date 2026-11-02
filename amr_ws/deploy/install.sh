#!/bin/bash
# Install or refresh deployment files only. This script deliberately does not
# enable, start, stop or restart any service; cutover is a separate witnessed
# step in RUNBOOK.md after vehicle acceptance.
set -euo pipefail

DEPLOY_DIR=$(cd "$(dirname "$0")" && pwd)
BACKUP_DIR="/var/backups/amr-units/$(date +%Y%m%d-%H%M%S)"
UNITS=(amr.service)

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

# Power-loss hardening (plan W6/W7): a shorter dirty-page window and a bounded
# journal. Both are drop-ins that only take effect where they are installed, and
# neither changes service state. The old copies are backed up like the units.
install_dropin() {  # <source> <destination>
  local src="$DEPLOY_DIR/$1" dst="$2"
  [[ -f "$src" ]] || return 0
  if [[ -f "$dst" ]]; then
    cp -a "$dst" "$BACKUP_DIR/$(basename "$dst")"
  fi
  install -m 0644 -D "$src" "$dst"
  echo "installed $dst"
}
install_dropin sysctl-amr.conf /etc/sysctl.d/60-amr.conf
install_dropin journald-amr.conf /etc/systemd/journald.conf.d/amr.conf
sysctl --quiet --system || echo "warning: sysctl --system failed; reboot applies it anyway" >&2

systemctl list-unit-files 'amr*.service' --no-legend \
  >"$BACKUP_DIR/enabled-state.txt" || true
systemctl daemon-reload

echo "Installed unit files without changing enabled or running services."
echo "The journal cap needs: systemctl restart systemd-journald"
echo "Backup: $BACKUP_DIR"
echo "Follow the witnessed cutover or rollback procedure in $DEPLOY_DIR/../RUNBOOK.md."
