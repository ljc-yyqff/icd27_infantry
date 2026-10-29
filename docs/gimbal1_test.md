# gimbal1 串口云台运动测试

`gimbal1_test` 是一个独立的 NUC 到 A 板串口测试程序。它不初始化 CAN，也不使用原来的二进制 `Gimbal` 协议。

| 功能 | 实际串口内容 |
|---|---|
| 使能 yaw/pitch | `ge\r\n` |
| 停止 yaw/pitch | `gd\r\n` |
| yaw 相对运动 | `yr 增量角度 最大速度\r\n` |
| pitch 相对运动 | `pr 增量角度 最大速度\r\n` |

角度单位为度，速度单位为度/秒。每条指令都以真实的 CRLF 字节 `0D 0A` 结束。

## 编译

```bash
cmake -S . -B build
cmake --build build --target gimbal1_test gimbal1_feedback gimbal1_protocol_test gimbal1_feedback_monitor -j2
./build/gimbal1_protocol_test
```

## 三个程序的职责

- `gimbal1_test`：只发送 `ge/gd/yr/pr`，不显示反馈。
- `gimbal1_feedback`：可发送指令，只显示手眼标定需要的 `yaw_actual` 和 `pitch_actual`。
- `gimbal1_feedback_monitor`：只监听，显示完整 26 项 CSV 和分组解析，不发送运动指令。

## 反馈监视

可发送指令并只看实际 yaw/pitch：

```bash
./build/gimbal1_feedback configs/gimbal1.yaml
```

A 板每 100 ms 发送的 26 项 CSV 可通过以下程序查看：

```bash
./build/gimbal1_feedback_monitor configs/gimbal1.yaml
```

该程序只读取串口，不会发送 `ge` 或任何运动指令。按 `Ctrl+C` 退出。运行前应停止其他可能读取 `/dev/gimbal` 的程序，否则两个进程会拆分同一串口数据流。

## 上电测试

1. 确保机器人悬空或云台无碰撞风险，摩擦轮和拨弹电机保持断电/禁用。
2. 停止 `standard`/`standard_mpc`/`gimbal_test` 等其他可能占用串口或发送 CAN 命令的视觉程序。
3. 确认 A 板串口是 `115200 8N1`；如果不同，先修改 `configs/gimbal1.yaml`。
4. 确认 `/dev/gimbal` 指向正确的 USB 串口。
5. 运行 `./build/gimbal1_test configs/gimbal1.yaml`。
6. 按下面的顺序逐条输入，每条后观察云台：

```text
enable
yaw 1 10
yaw -1 10
pitch 1 10
pitch -1 10
disable
quit
```

程序启动时默认为禁用状态，未输入 `enable` 时会拒绝运动指令。正常退出或输入结束时会尝试再发一次 `gd\r\n`。

## 安全说明

- `yr/pr` 是增量指令，每重复发送一次都会继续累加。本测试不会自动以 20–50 Hz 重复发送。
- NUC 端默认限制单次增量为 `±5°`、速度为 `30°/s`，可在配置文件中收紧。
- `gimbal1_feedback_monitor` 保留全部 26 个值，并显示 yaw/pitch 的目标、实际角度、速度、ACTIVE、故障码和电机状态。
- A 板 LED 闪烁三次只表示运动指令被接受；CSV 反馈是独立的状态数据。
