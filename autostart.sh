#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main"
LOG_DIR="${PROJECT_DIR}/logs"
PID_FILE="${LOG_DIR}/safe_debug.pid"
BOOT_LOG="${LOG_DIR}/$(date "+%Y-%m-%d_%H-%M-%S")-safe-debug.bootstrap.log"

sleep 5

cd "${PROJECT_DIR}"
mkdir -p "${LOG_DIR}"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}")"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "[autostart] safe debug is already running, pid=${old_pid}"
    exit 0
  fi
fi

echo "[autostart] starting safe debug..." | tee -a "${BOOT_LOG}"
echo "[autostart] project: ${PROJECT_DIR}" | tee -a "${BOOT_LOG}"
echo "[autostart] bootstrap log: ${BOOT_LOG}" | tee -a "${BOOT_LOG}"

nohup bash -lc "cd '${PROJECT_DIR}' && exec './safe_debug_start.sh'" >> "${BOOT_LOG}" 2>&1 &
pid="$!"
echo "${pid}" > "${PID_FILE}"

echo "[autostart] started safe debug, pid=${pid}" | tee -a "${BOOT_LOG}"
echo "[autostart] pid file: ${PID_FILE}" | tee -a "${BOOT_LOG}"
echo "[autostart] use 'tail -f logs/*-safe-debug.log' to watch logs." | tee -a "${BOOT_LOG}"
