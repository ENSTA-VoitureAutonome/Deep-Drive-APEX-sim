#!/usr/bin/env bash
# RPLIDAR alone, in the APEX image, detached: no IMU node, no actuators.
#
#   ./tools/lidar/apex_lidar_only.sh up [-p name:=value ...]   start (soft motor start, after APEX_LIDAR_START_DELAY_S)
#   ./tools/lidar/apex_lidar_only.sh logs                      follow the node's log (Ctrl+C leaves it running)
#   ./tools/lidar/apex_lidar_only.sh check [args]              lidar_bringup_check.py inside the container
#   ./tools/lidar/apex_lidar_only.sh record NAME [SECONDS]     record_real2sim_sensors.py -> data/real2sim_capture/NAME
#   ./tools/lidar/apex_lidar_only.sh power                     the Pi's under-voltage flags
#   ./tools/lidar/apex_lidar_only.sh boot on|off|status        on: Docker restarts the LiDAR after every Pi reboot
#                                                              (power cut) and apex-real-ready is disabled meanwhile;
#                                                              off: back to manual start and the service as it was
#   ./tools/lidar/apex_lidar_only.sh down                      stop (the node stops the motor); also 'boot off'
#
# Start-loop guard: each container start counts; a start that runs
# APEX_LIDAR_STABLE_S (120 s) clears the count. After APEX_LIDAR_MAX_STARTS (3)
# starts in a row that died sooner (the Pi lost power when the motor started),
# the container no longer starts the motor until the next 'up'.
#
# Environment: APEX_LIDAR_PORT (/dev/ttyUSB0), APEX_LIDAR_BAUDRATE (115200),
# APEX_LIDAR_BACKEND (rplidar_express), APEX_LIDAR_START_DELAY_S (5),
# APEX_LIDAR_MAX_STARTS (3), APEX_LIDAR_STABLE_S (120), ROS_DOMAIN_ID (30).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APEX_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONTAINER="${APEX_LIDAR_CONTAINER:-apex_lidar}"
PIPELINE_CONTAINER="${APEX_CONTAINER_NAME:-apex_pipeline}"
SERVICE="apex-real-ready.service"
PORT="${APEX_LIDAR_PORT:-/dev/ttyUSB0}"
BAUD="${APEX_LIDAR_BAUDRATE:-115200}"
BACKEND="${APEX_LIDAR_BACKEND:-rplidar_express}"
DELAY_S="${APEX_LIDAR_START_DELAY_S:-5}"
MAX_STARTS="${APEX_LIDAR_MAX_STARTS:-3}"
STABLE_S="${APEX_LIDAR_STABLE_S:-120}"
STARTS_FILE="${APEX_ROOT}/data/.apex_lidar_starts"  # /work/data/.apex_lidar_starts in the container
BOOT_FILE="${APEX_ROOT}/data/.apex_lidar_boot"      # boot mode on; holds whether the service was enabled
PARAMS="/work/ros2_ws/src/apex_telemetry/config/apex_params.yaml"
ROS_ENV='source /opt/ros/jazzy/setup.bash && export PYTHONPATH=/work/ros2_ws/src/apex_telemetry:${PYTHONPATH:-}'

throttled_hex() {
  local f=/sys/devices/platform/soc/soc:firmware/get_throttled
  if command -v vcgencmd >/dev/null 2>&1; then
    vcgencmd get_throttled 2>/dev/null | sed -n 's/^throttled=//p'
  elif [[ -r "${f}" ]]; then
    printf '0x%s\n' "$(cat "${f}")"
  fi
}

power_report() {
  local raw value
  raw="$(throttled_hex)"
  if [[ -z "${raw}" ]]; then
    echo "[LIDAR] Pi power flags unavailable (no vcgencmd, no get_throttled)"
    return 0
  fi
  value=$((raw))
  echo "[LIDAR] Pi throttled=${raw}: under-voltage now $(( value & 0x1 ? 1 : 0 )), since boot $(( value & 0x10000 ? 1 : 0 ))"
  return $(( value & 0x1 ? 1 : 0 ))
}

container_running() {
  docker ps --format '{{.Names}}' | grep -qx "$1"
}

apex_image() {
  local image
  image="$(docker compose -f "${APEX_ROOT}/docker/docker-compose.yml" config --images 2>/dev/null | head -n 1 || true)"
  if [[ -n "${image}" ]] && docker image inspect "${image}" >/dev/null 2>&1; then
    echo "${image}"
    return
  fi
  docker images --format '{{.Repository}}:{{.Tag}}' | grep -m 1 'apex_pipeline' || true
}

require_running() {
  if ! container_running "${CONTAINER}"; then
    echo "[LIDAR][ERROR] ${CONTAINER} is not running: ./tools/lidar/apex_lidar_only.sh up" >&2
    exit 1
  fi
}

cmd_up() {
  if container_running "${PIPELINE_CONTAINER}"; then
    echo "[LIDAR][ERROR] ${PIPELINE_CONTAINER} is running and holds the LiDAR port: ./tools/core/apex_core_down.sh" >&2
    exit 1
  fi
  if systemctl is-active --quiet "${SERVICE}" 2>/dev/null; then
    echo "[LIDAR][ERROR] ${SERVICE} is active and restarts the full stack: sudo systemctl stop ${SERVICE}" >&2
    exit 1
  fi
  if [[ ! -e "${PORT}" ]]; then
    echo "[LIDAR][ERROR] ${PORT} not found. Serial devices:" >&2
    ls -l /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM* 2>/dev/null >&2 || true
    exit 1
  fi
  local dev image
  dev="$(readlink -f "${PORT}")"
  if command -v fuser >/dev/null 2>&1 && fuser "${dev}" >/dev/null 2>&1; then
    echo "[LIDAR][ERROR] ${dev} is open by another process: $(fuser "${dev}" 2>&1)" >&2
    exit 1
  fi
  if ! power_report; then
    echo "[LIDAR][WARN] the Pi is in under-voltage now; the node waits for a clear moment before the motor start"
  fi
  image="$(apex_image)"
  if [[ -z "${image}" ]]; then
    echo "[LIDAR][ERROR] APEX image not found: docker compose -f docker/docker-compose.yml build" >&2
    exit 1
  fi
  mkdir -p "${APEX_ROOT}/data"
  rm -f "${STARTS_FILE}"
  local restart=no
  [[ -f "${BOOT_FILE}" ]] && restart=unless-stopped
  docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
  docker run -d --init --name "${CONTAINER}" --restart "${restart}" \
    --network host --device "${dev}:${dev}" \
    -v "${APEX_ROOT}/ros2_ws:/work/ros2_ws" -v "${APEX_ROOT}/data:/work/data" -w /work/ros2_ws \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}" \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    -e PYTHONUNBUFFERED=1 -e MAX_STARTS="${MAX_STARTS}" -e STABLE_S="${STABLE_S}" \
    "${image}" /bin/bash -c "${ROS_ENV}"' && \
      f=/work/data/.apex_lidar_starts; n=$(cat "$f" 2>/dev/null || echo 0); \
      if [ "$n" -ge "$MAX_STARTS" ]; then \
        echo "[LIDAR][ERROR] $n starts in a row died within $STABLE_S s (power cut at the motor start?): motor NOT started. Fix the supply, then ./tools/lidar/apex_lidar_only.sh up"; \
        exec sleep infinity; fi; \
      echo $((n + 1)) > "$f"; sync; echo "[LIDAR] start $((n + 1)) of at most $MAX_STARTS without $STABLE_S s of stable running"; \
      (sleep "$STABLE_S" && echo 0 > "$f" && echo "[LIDAR] $STABLE_S s stable: start count cleared") & \
      for p in /opt/apex_venv/lib/python*/site-packages; do [ -d "$p" ] && export PYTHONPATH="$p:$PYTHONPATH"; done; \
      echo "[LIDAR] starting in $0 s"; sleep "$0"; \
      exec nice -n 5 python3 -m apex_telemetry.perception.rplidar_publisher_node --ros-args "$@"' \
    "${DELAY_S}" --params-file "${PARAMS}" \
    -p "port:=${dev}" -p "baudrate:=${BAUD}" -p "source_backend:=${BACKEND}" "$@" >/dev/null
  echo "[LIDAR] ${CONTAINER} started on ${dev} (${BACKEND}, ${BAUD} baud, restart ${restart}); the motor starts in ${DELAY_S} s with a soft ramp"
  echo "[LIDAR] logs: ./tools/lidar/apex_lidar_only.sh logs    stop: ./tools/lidar/apex_lidar_only.sh down"
}

cmd_down() {
  docker stop -t 10 "${CONTAINER}" >/dev/null 2>&1 || true
  docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
  echo "[LIDAR] ${CONTAINER} stopped"
  [[ -f "${BOOT_FILE}" ]] && cmd_boot off
  return 0
}

cmd_boot() {
  case "${1:-status}" in
    on)
      if [[ ! -f "${BOOT_FILE}" ]]; then
        local was=disabled
        systemctl is-enabled --quiet "${SERVICE}" 2>/dev/null && was=enabled
        echo "service_was=${was}" > "${BOOT_FILE}"
        if [[ "${was}" == enabled ]]; then
          sudo systemctl disable "${SERVICE}" >/dev/null 2>&1
          echo "[LIDAR] ${SERVICE} disabled while the LiDAR starts at boot (it would take the port); 'boot off' re-enables it"
        fi
      fi
      if docker inspect "${CONTAINER}" >/dev/null 2>&1; then
        docker update --restart unless-stopped "${CONTAINER}" >/dev/null
        echo "[LIDAR] boot on: ${CONTAINER} restarts after every Pi reboot"
      else
        echo "[LIDAR] boot on: the next 'up' starts ${CONTAINER} with restart after reboot"
      fi
      ;;
    off)
      docker update --restart no "${CONTAINER}" >/dev/null 2>&1 || true
      if [[ -f "${BOOT_FILE}" ]] && grep -qx 'service_was=enabled' "${BOOT_FILE}"; then
        sudo systemctl enable "${SERVICE}" >/dev/null 2>&1
        echo "[LIDAR] ${SERVICE} enabled again (starts at the next boot)"
      fi
      rm -f "${BOOT_FILE}"
      echo "[LIDAR] boot off: the LiDAR starts only with 'up'"
      ;;
    status)
      echo "[LIDAR] boot $([[ -f "${BOOT_FILE}" ]] && echo on || echo off);" \
        "restart policy $(docker inspect -f '{{.HostConfig.RestartPolicy.Name}}' "${CONTAINER}" 2>/dev/null || echo none);" \
        "${SERVICE} $(systemctl is-enabled "${SERVICE}" 2>/dev/null || true);" \
        "failed starts in a row $(cat "${STARTS_FILE}" 2>/dev/null || echo 0)/${MAX_STARTS}"
      ;;
    *) echo "usage: boot on|off|status" >&2; exit 1 ;;
  esac
}

cmd_check() {
  require_running
  local tty=()
  [[ -t 0 ]] && tty=(-it)
  docker exec "${tty[@]}" "${CONTAINER}" /bin/bash -c "${ROS_ENV} && python3 /work/ros2_ws/scripts/lidar_bringup_check.py \"\$@\"" _ "$@"
}

cmd_record() {
  require_running
  local name="${1:?usage: record NAME [SECONDS]}" duration="${2:-60}"
  docker exec "${CONTAINER}" /bin/bash -c "${ROS_ENV} && python3 /work/ros2_ws/scripts/capture/record_real2sim_sensors.py \
    --out /work/data/real2sim_capture/${name} --duration ${duration}"
  echo "[LIDAR] recorded into ${APEX_ROOT}/data/real2sim_capture/${name}"
}

case "${1:-}" in
  up) shift; cmd_up "$@" ;;
  down) cmd_down ;;
  logs) docker logs -f --tail 100 "${CONTAINER}" ;;
  check) shift; cmd_check "$@" ;;
  record) shift; cmd_record "$@" ;;
  power) power_report || true ;;
  boot) shift; cmd_boot "$@" ;;
  *) sed -n '2,22p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac
