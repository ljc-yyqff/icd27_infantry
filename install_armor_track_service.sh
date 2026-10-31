#!/usr/bin/env bash
set -euo pipefail
if [[ ${EUID} -ne 0 ]]; then
  echo "Run: sudo /home/ros/infantry/ict27_infantry/install_armor_track_service.sh" >&2
  exit 1
fi
project=/home/ros/infantry/ict27_infantry
service=ict27_armor_track.service
[[ -x "$project/build/ascii_tracking/gimbal1_armor_track" ]]
[[ -r "$project/configs/infantry_daheng.yaml" ]]
/usr/bin/systemd-analyze verify "$project/$service"
/usr/bin/install -m 0644 "$project/$service" "/etc/systemd/system/$service"
# Previous boot tasks use the same camera/serial hardware. Replace their boot role.
for old in sp_vision_safe_debug.service sp_vision_gimbal_sweep.service; do
  if /usr/bin/systemctl cat "$old" >/dev/null 2>&1; then
    /usr/bin/systemctl disable --now "$old"
  fi
done
/usr/bin/systemctl daemon-reload
/usr/bin/systemctl enable "$service"
/usr/bin/systemctl is-enabled "$service"
echo "Installed and enabled for next boot. The tracking service was NOT started now."
echo "Start now: sudo systemctl start $service"
echo "Follow logs: journalctl -u $service -f"
