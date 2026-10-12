#!/usr/bin/env bash
# Keep the car reachable, and record every time it was not.
#
# The car is on Wi-Fi (wlan0, NetworkManager profile `Voiture_bleu_ensta`) to a
# laptop hotspot, and the laptop is how every run is started, watched and
# fetched. When that link drops there is nobody on the car to notice: the
# Docker stack keeps running -- it is detached and its artifacts go to disk --
# but it cannot be reached until someone walks to the car.
#
# This reconnects it. It pings the default gateway; after `FAILS` consecutive
# misses it asks NetworkManager to bring the Wi-Fi profile back up, and it
# writes one CSV line for every check that failed and every reconnect it tried.
# That log is the evidence for the next question: a link that drops and comes
# back leaves rows here, and a car that went down entirely leaves none, because
# nothing was running to write them.
#
#   ./tools/core/apex_net_watchdog.sh                  run in the foreground
#   ./tools/core/install_apex_net_watchdog.sh          install it as a service
#   sudo systemctl status apex-net-watchdog
#   column -ts, ~/AiAtonomousRc/APEX/data/net_watchdog.csv
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APEX_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

INTERVAL_S="${APEX_NET_INTERVAL_S:-10}"
FAILS="${APEX_NET_FAILS:-3}"          # consecutive misses before acting
COOLDOWN_S="${APEX_NET_COOLDOWN_S:-30}"
IFACE="${APEX_NET_IFACE:-wlan0}"
PROFILE="${APEX_NET_PROFILE:-}"       # empty: whatever NetworkManager has for IFACE
LOG="${APEX_NET_LOG:-${APEX_ROOT}/data/net_watchdog.csv}"
MAX_LOG_BYTES="${APEX_NET_MAX_LOG_BYTES:-1048576}"

mkdir -p "$(dirname "${LOG}")"
[[ -s "${LOG}" ]] || echo "utc_iso,event,detail" > "${LOG}"

log_row() {
  if [[ -f "${LOG}" && "$(stat -c %s "${LOG}" 2>/dev/null || echo 0)" -gt "${MAX_LOG_BYTES}" ]]; then
    mv -f "${LOG}" "${LOG}.1" 2>/dev/null || true
    echo "utc_iso,event,detail" > "${LOG}"
  fi
  echo "$(date -u +%Y-%m-%dT%H:%M:%SZ),$1,$2" >> "${LOG}"
}

gateway() { ip route | awk '/^default/ {print $3; exit}'; }

link_is_up() {
  # Deliberately NOT a ping of the gateway: the laptop hotspot this car
  # associates with does not answer ICMP, so pinging it reports a dead link
  # over a perfectly good one -- measured, and it made the first version of
  # this watchdog bounce the Wi-Fi every 30 s. What actually says "the link is
  # gone" is NetworkManager's own state plus an address and a route.
  local state
  state="$(nmcli -t -f DEVICE,STATE device status 2>/dev/null | awk -F: -v d="${IFACE}" '$1 == d {print $2; exit}')"
  [[ "${state}" == "connected" ]] || return 1
  ip -4 addr show dev "${IFACE}" 2>/dev/null | grep -q 'inet ' || return 1
  [[ -n "$(gateway)" ]] || return 1
  return 0
}

wifi_profile() {
  [[ -n "${PROFILE}" ]] && { echo "${PROFILE}"; return; }
  nmcli -t -f DEVICE,CONNECTION device status 2>/dev/null | awk -F: -v d="${IFACE}" '$1 == d {print $2; exit}'
}

reconnect() {
  local profile
  profile="$(wifi_profile)"
  if ! command -v nmcli >/dev/null 2>&1; then
    log_row reconnect_skipped "no nmcli"
    return 1
  fi
  if [[ -n "${profile}" && "${profile}" != "--" ]]; then
    log_row reconnect "nmcli con up ${profile}"
    nmcli con up "${profile}" >/dev/null 2>&1 && return 0
  fi
  log_row reconnect "nmcli device reconnect ${IFACE}"
  nmcli device reconnect "${IFACE}" >/dev/null 2>&1
}

log_row started "iface=${IFACE} interval=${INTERVAL_S}s fails=${FAILS}"
misses=0
container_was_up=-1
while true; do
  if link_is_up; then
    if (( misses > 0 )); then
      log_row recovered "${IFACE} is connected again after ${misses} misses"
      misses=0
    fi
  else
    misses=$((misses + 1))
    state="$(nmcli -t -f DEVICE,STATE device status 2>/dev/null | awk -F: -v d="${IFACE}" '$1 == d {print $2; exit}')"
    log_row miss "${IFACE} state=${state:-unknown} route=$(gateway) (miss ${misses}/${FAILS})"
  fi

  # The autonomy itself: whether it is still there is the other half of "is
  # the car alive". The container is detached and keeps its artifacts on disk,
  # so a dropped link does not stop a run -- but a run that died while nobody
  # could see it should leave a line here.
  if command -v docker >/dev/null 2>&1; then
    running="$(docker ps --filter name=apex_pipeline --format '{{.Names}}' 2>/dev/null)"
    if [[ -z "${running}" && "${container_was_up}" == "1" ]]; then
      log_row container_gone "apex_pipeline is no longer running"
      container_was_up=0
    elif [[ -n "${running}" && "${container_was_up}" != "1" ]]; then
      log_row container_up "apex_pipeline is running"
      container_was_up=1
    fi
  fi

  if (( misses >= FAILS )); then
    # Record the power state next to the reconnect: on this car an
    # unreachable Pi has so far meant the 5 V rail, not the radio, and the two
    # are told apart by whether this line exists at all.
    alarm="$(cat /sys/class/hwmon/hwmon2/in0_lcrit_alarm 2>/dev/null || echo '?')"
    throttled="$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2 || echo '?')"
    log_row power "in0_lcrit_alarm=${alarm} get_throttled=${throttled}"
    reconnect
    misses=0
    sleep "${COOLDOWN_S}"
  fi
  sleep "${INTERVAL_S}"
done
