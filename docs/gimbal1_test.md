# Gimbal1 新版串口测试

完整说明见 [新版协议与相对跟随](gimbal1_tracking.md)。适配 9 列 CSV、`#GM` 全局状态和 `#GC` 命令回执，旧 26 列接口已移除。

- `gimbal1_test`：交互发送相对点动指令；使能时等待双轴 ACTIVE，提示符期间持续接收反馈。
- `gimbal1_feedback`：上述功能加实际角度观测。
- `gimbal1_feedback_monitor`：只读 9 列遥测，不主动查询状态。
- `gimbal1_protocol_test`：无硬件协议和到位判断测试。

构建、运行路径和操作示例均以完整说明为准。发送仍使用 ge/gd/yr/pr；交互别名 y/p 实际发送 yr/pr。gd 表示撤力，相对零增量也不能取消未完成目标。程序必须收到对应 `#GC,ok=1` 才确认命令被电控接受。
