#!/usr/bin/env bash
# Install the Wi-Fi watchdog as a system service (run this on the car).
#
# It runs as root because `nmcli con up` needs it, and it is deliberately a
# separate service from the autonomy: it must keep the car reachable whether or
# not the race stack is running, and it must survive the stack being stopped.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APEX_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
UNIT_SRC="${APEX_ROOT}/systemd/apex-net-watchdog.service"
UNIT_DST=/etc/systemd/system/apex-net-watchdog.service

sed "s|__APEX_ROOT__|${APEX_ROOT}|g" "${UNIT_SRC}" | sudo tee "${UNIT_DST}" >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now apex-net-watchdog
sleep 2
systemctl --no-pager --lines=5 status apex-net-watchdog || true
echo
echo "log: ${APEX_ROOT}/data/net_watchdog.csv"
