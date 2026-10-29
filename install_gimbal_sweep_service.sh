#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main"
CAMERA_SERVICE="sp_vision_safe_debug.service"
GIMBAL_SERVICE="sp_vision_gimbal_sweep.service"
UDEV_RULE="99-sp-vision-gimbal.rules"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run this installer with sudo."
  exit 1
fi

if [[ ! -x "${PROJECT_DIR}/build/gimbal1_autorun" ]]; then
  echo "Missing build/gimbal1_autorun. Build the project first."
  exit 1
fi

install -m 0644 "${PROJECT_DIR}/${CAMERA_SERVICE}" "/etc/systemd/system/${CAMERA_SERVICE}"
install -m 0644 "${PROJECT_DIR}/${GIMBAL_SERVICE}" "/etc/systemd/system/${GIMBAL_SERVICE}"
install -m 0644 "${PROJECT_DIR}/${UDEV_RULE}" "/etc/udev/rules.d/${UDEV_RULE}"

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --action=change
systemctl daemon-reload
systemctl enable "${CAMERA_SERVICE}" "${GIMBAL_SERVICE}"

echo "Installed stable /dev/gimbal mapping and fixed the systemd ordering cycle."
echo "${CAMERA_SERVICE} and ${GIMBAL_SERVICE} are enabled for the next boot."
echo "It was not started now, so this installation does not move the gimbal."
