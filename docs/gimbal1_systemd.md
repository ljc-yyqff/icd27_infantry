# gimbal1 开机扫描任务

开机后由两个独立的 systemd 服务并行工作：

- `sp_vision_safe_debug.service`：持续运行无界面相机识别。
- `sp_vision_gimbal_sweep.service`：每次开机只执行一次云台扫描，完成后保持 `active (exited)`，不会因相机服务重启而重复运动。

## 运动序列

```text
ge
yaw:   +90 -> -180 -> +90
pitch: +20 -> -40  -> +20
等待10秒
gd
```

云台从 yaw 中心到右侧 `+90°`，再到左侧 `-90°`，最后回到中心。pitch 假定启动时位于 `25°`，扫描 `45° -> 5° -> 25°`。

所有运动都拆成最大 `5°` 的 `yr/pr` 相对指令，速度为 `10°/s`。由于协议没有串口反馈，程序通过 `角度/速度 + 0.1秒` 估算每步完成时间。

## 保护逻辑

- 串口最多等待 `30秒`；仍无法打开时任务失败，不发送 `ge`。
- 服务停止、`SIGTERM`/`SIGINT`、命令失败或异常退出时，只要串口已使能，析构阶段都会尝试发送 `gd`.
- 不包含摩擦轮、拨弹或发射命令。
- A 板必须独立执行 pitch `5°–45°` 限位和机械保护。

## 无硬件模拟

```bash
./build/gimbal1_autorun configs/gimbal1.yaml --dry-run
```

`--dry-run` 只打印命令顺序，不打开串口、不等待、不驱动云台。

## 安装开机服务

```bash
sudo ./install_gimbal_sweep_service.sh
```

该安装脚本只复制、重载并启用服务，不会立即启动云台任务。下次开机才会执行。

安装脚本还会安装 `99-sp-vision-gimbal.rules`，将已经通过热插拔确认的 A 板 CH340（`1a86:7523`、`bcdDevice=0264`）固定映射为 `/dev/gimbal`。另一个 `bcdDevice=8134` 的 CH340 不会被匹配。

同时，相机服务的启动顺序已从错误的 `After=multi-user.target` 改为 `After=local-fs.target`，避免 `multi-user.target -> gimbal -> camera -> multi-user.target` 的 systemd 顺序环。
