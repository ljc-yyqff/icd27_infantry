# 开机自动识别并跟踪装甲板

当前服务为 `ict27_armor_track.service`，运行用户ros，工作目录 `/home/ros/infantry/ict27_infantry`。不依赖桌面登录，不弹相机窗口。

## 安装

程序已构建在 `build/ascii_tracking/gimbal1_armor_track`。运行：

```bash
sudo /home/ros/infantry/ict27_infantry/install_armor_track_service.sh
```

安装脚本验证服务配置、安装到 `/etc/systemd/system/`、停止并禁用旧的 `sp_vision_safe_debug.service` 和 `sp_vision_gimbal_sweep.service`，然后启用新服务。它不立即启动新服务，下次开机运行。重复安装不会创建第二份服务。

## 启动行为

完整执行参数：

```bash
./build/ascii_tracking/gimbal1_armor_track configs/infantry_daheng.yaml \
  --control=true --control_rate=50 --speed=30 --max_step=2.0 \
  --yaw_gain=4.0 --pitch_gain=2.5 --deadband=10 --deadband_exit=16 \
  --headless=true --auto_start=true
```

`--headless=true` 禁用imshow/waitKey，不依赖DISPLAY。`--auto_start=true` 必须配合control，在收到第一张可用于控制的新鲜图像后发送一次ge，开始视觉指令输出。目标关联仍需确认；电机反馈仅记录，不用作到位/保持/保护判断。未加auto_start的手动模式仍需按e。

串口设备尚未出现或没有读写权限时，每3秒重新尝试启动。相机打开失败、取帧超时或无窗口模式下串口写入失败，程序报错退出，由systemd重启。不会自动重发单条相对指令，普通运行错误不会触发gd。无装甲板时继续检测，不发送运动增量。

## 使用

```bash
# 查看服务是否已启用、是否在运行
systemctl is-enabled ict27_armor_track.service
systemctl status ict27_armor_track.service --no-pager

# 立即启动（自动使能并跟踪）
sudo systemctl start ict27_armor_track.service

# 停止视觉服务
sudo systemctl stop ict27_armor_track.service

# 取消开机启动并停止
sudo systemctl disable --now ict27_armor_track.service

# 查看本次开机日志 / 实时日志
journalctl -u ict27_armor_track.service -b --no-pager
journalctl -u ict27_armor_track.service -f
```

SIGTERM/SIGINT会请求程序退出。服务模式（control/headless/auto_start同时为true）在关闭串口前发送一次gd，随后关闭相机和串口；因此 `systemctl stop` 会请求电控失能。这属于用户显式停止命令，不是运行中的保护或到位逻辑。目标丢失、反馈异常、正常EOF、普通运行错误都不触发这个停止命令。SIGKILL或断电无法执行进程清理，gd是否生效仍以A板反馈为准。手动窗口模式的d/q行为不变。开机启动设置不受stop影响。

不要同时运行手动相机程序。手动调试前先停止此服务，避免相机和串口占用。新旧开机任务互斥，安装脚本也会禁用旧任务。

串口及逐帧日志在 `logs/gimbal1/`，算法参数和含义参见 `gimbal1_tracking.md`。

目前已完成编译、无硬件参数检查、systemd配置验证和离线测试；安装需要本机sudo密码，实际冷启动跟踪需要上电验证。
