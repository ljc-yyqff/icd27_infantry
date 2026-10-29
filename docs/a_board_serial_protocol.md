# NUC 与 RoboMaster A 板串口通信

本项目的步兵串口链路为：相机 → NUC 识别/跟踪/弹道解算 → A 板控制云台和发射机构。运行 `standard_mpc` 时不依赖 CAN。

## 1. 物理接口

- A 板通过 USB CDC 虚拟串口连接 NUC，建议用 udev 规则固定为 `/dev/gimbal`。
- 默认参数：115200 baud、8 数据位、无校验、1 停止位、无流控。
- NUC 配置项位于 `configs/infantry_daheng.yaml`。
- 建议双向发送频率均为 100 Hz。当前协议每周期共 72 字节，在 115200 baud 下有足够余量。
- NUC 连续 100 ms 未收到 A 板合法反馈时，会把所有下发命令强制改为“不控制、不开火”。

## 2. 共享协议定义

线格式定义和无查表 CRC16 实现在 `io/gimbal/gimbal_protocol.h`。该文件为 C/C++ 兼容，可复制到 STM32 工程。

协议要求双方均使用：

- 小端字节序；
- IEEE-754 32 位 `float`；
- packed 结构体，不允许编译器插入对齐字节；
- 帧头固定为 ASCII `SP`；
- CRC16 覆盖帧头至 CRC 字段之前的全部字节，CRC 自身按小端发送。

### A 板发送给 NUC：43 字节

| 偏移 | 字段 | 类型 | 单位/含义 |
|---:|---|---|---|
| 0 | head | uint8[2] | `'S','P'` |
| 2 | mode | uint8 | 0 空闲、1 自瞄、2 小符、3 大符 |
| 3 | q | float[4] | IMU 四元数，wxyz |
| 19 | yaw | float | rad |
| 23 | yaw_vel | float | rad/s |
| 27 | pitch | float | rad |
| 31 | pitch_vel | float | rad/s |
| 35 | bullet_speed | float | m/s |
| 39 | bullet_count | uint16 | 累计发弹数，允许回绕 |
| 41 | crc16 | uint16 | 小端 |

A 板必须持续发送合法四元数，即使当前处于空闲模式；NUC 启动时会等待第一帧姿态数据。

### NUC 发送给 A 板：29 字节

| 偏移 | 字段 | 类型 | 单位/含义 |
|---:|---|---|---|
| 0 | head | uint8[2] | `'S','P'` |
| 2 | mode | uint8 | 0 不控制、1 控制但不开火、2 控制且请求开火 |
| 3 | yaw | float | rad |
| 7 | yaw_vel | float | rad/s |
| 11 | yaw_acc | float | rad/s² |
| 15 | pitch | float | rad |
| 19 | pitch_vel | float | rad/s |
| 23 | pitch_acc | float | rad/s² |
| 27 | crc16 | uint16 | 小端 |

## 3. A 板安全逻辑

A 板必须把 NUC 的 `mode=2` 当作“开火请求”，不能绕过遥控器急停、裁判系统热量/弹速限制和机构状态检查。

建议板端实现以下状态机：

1. 每次收到完整、帧头正确且 CRC 正确的数据后更新时间戳。
2. 超过 100 ms 没收到合法 NUC 数据，立即关闭视觉控制和发射，回到手动/安全状态。
3. `mode=0`：忽略所有目标量并禁止发射。
4. `mode=1`：允许云台跟随，禁止发射。
5. `mode=2`：允许云台跟随；仅在遥控器允许、裁判系统允许且机构正常时发射。
6. 对 yaw/pitch、速度和加速度做机械限位与数值范围检查；遇到 NaN/Inf 或越界值按 `mode=0` 处理。

NUC 与 A 板两端都实施 100 ms 看门狗：任意单向链路中断，视觉控制和开火都会被撤销。

## 4. 推荐联调顺序

1. 保持 `auto_fire: false`，运行 `./build/gimbal_protocol_test` 验证协议布局和 CRC。
2. A 板只发送状态，运行 `./build/gimbal_test configs/infantry_daheng.yaml`，确认四元数、角度、弹速和模式。
3. 架空机器人并关闭摩擦轮，使 A 板接受 `mode=1`，检查 yaw/pitch 方向、零点和限位。
4. 运行 `./build/standard_mpc configs/infantry_daheng.yaml`，完成无弹跟踪测试。
5. 验证急停、100 ms 通信看门狗、裁判系统限制后，才将 `auto_fire` 改为 `true`。

## 5. A 板接收伪代码

```c
VisionToGimbalPacket command;

if (uart_frame_ready() && frame_length == VISION_TO_GIMBAL_PACKET_SIZE) {
  memcpy(&command, uart_rx_buffer, sizeof(command));
  if (command.head[0] == GIMBAL_PROTOCOL_SOF_0 &&
      command.head[1] == GIMBAL_PROTOCOL_SOF_1 &&
      gimbal_protocol_check_crc16((const uint8_t *)&command, sizeof(command))) {
    last_valid_nuc_tick = HAL_GetTick();
    apply_vision_command_with_limits(&command);
  }
}

if (HAL_GetTick() - last_valid_nuc_tick > 100U) {
  disable_vision_control_and_fire();
}
```

推荐使用 DMA + UART IDLE/USB CDC 收包，再按 `SP` 帧头和固定长度进行分帧，不要假设一次 USB/UART 回调必然包含完整数据包。
