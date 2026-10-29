#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main"
CONFIG_PATH="${1:-configs/infantry_daheng.yaml}"
MODE="${SAFE_DEBUG_MODE:-yolo}"
HEADLESS="${SAFE_DEBUG_HEADLESS:-false}"
LOG_DIR="${PROJECT_DIR}/logs"
LOG_FILE="${LOG_DIR}/$(date "+%Y-%m-%d_%H-%M-%S")-safe-debug.log"

cd "${PROJECT_DIR}"
mkdir -p "${LOG_DIR}" patterns

exec > >(tee -a "${LOG_FILE}") 2>&1

echo "[safe-debug] project: ${PROJECT_DIR}"
echo "[safe-debug] config: ${CONFIG_PATH}"
echo "[safe-debug] mode: ${MODE}"
echo "[safe-debug] headless: ${HEADLESS}"
echo "[safe-debug] log: ${LOG_FILE}"
echo "[safe-debug] this script only runs camera detection; it does not start gimbal control or fire."

if [[ -f /opt/intel/openvino_2024.6.0/setupvars.sh ]]; then
  # Keep OpenVINO available when the script is launched by autostart instead of an interactive shell.
  set +u
  source /opt/intel/openvino_2024.6.0/setupvars.sh
  set -u
elif compgen -G "/opt/intel/openvino_*/setupvars.sh" > /dev/null; then
  set +u
  source "$(find /opt/intel -maxdepth 2 -path "/opt/intel/openvino_*/setupvars.sh" | sort -V | tail -n 1)"
  set -u
fi

export DISPLAY="${DISPLAY:-:0}"
export QT_X11_NO_MITSHM=1

if [[ ! -x build/camera_detect_test ]]; then
  echo "[safe-debug] build/camera_detect_test not found or not executable. Please build the project first."
  exit 1
fi

if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "[safe-debug] config not found: ${CONFIG_PATH}"
  exit 1
fi

cmd=(./build/camera_detect_test "${CONFIG_PATH}")
if [[ "${MODE}" == "tradition" ]]; then
  cmd+=("-t")
fi
if [[ "${HEADLESS}" == "true" || "${HEADLESS}" == "1" ]]; then
  cmd+=("--headless")
fi

echo "[safe-debug] start command: ${cmd[*]}"
exec "${cmd[@]}"
