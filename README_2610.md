# 同济大学 SuperPower 25 赛季自瞄项目—当前进度与操作指南

> 整理日期：2026-10-28  
> 主项目目录：`sp_vision_25-main/`  
> 目前用途：步兵机器人 NUC 自瞄，大恒相机采集图像，NUC 识别装甲板并通过串口向 A 板发送云台控制指令。

## 1. 当前总体进度

| 功能 | 状态 | 说明 |
|---|---|---|
| 大恒相机接入 | 已完成 | MER2-630-60U3C，可稳定获取图像 |
| 相机内参标定 | 已完成 | 3088×2064，重投影误差约 0.2375 px |
| 相机倒装处理 | 已完成 | 采集层统一旋转 180°，内参已换算 |
| YOLO 装甲板识别 | 已完成 | 真实装甲板可稳定识别，约 40～50 FPS |
| NUC ↔ A 板串口 | 已完成 | 115200 8N1，ASCII + CRLF |
| 手动云台控制 | 已完成 | Yaw/Pitch 相对角度指令可驱动云台 |
| 26 字段反馈解析 | 已完成 | 可读取 actual/active/fault，但 A 板反馈曾出现跳变和滞后 |
| 无角度反馈视觉跟随 | 原型已跑通 | 可将装甲板向画面中心跟随，响应速度仍在调参 |
| 当前机器人手眼标定 | 未完成 | 配置中旧 R/t 不能作为当前机器人最终标定结果 |
| 可靠三维解算/弹道解算 | 未完成 | 需在反馈和手眼标定可靠后继续 |
| 自动发射 | 禁用 | `auto_fire: false`，联调阶段不允许开启 |

## 2. 硬件与通信方式

### 2.1 系统链路

```text
大恒相机
  ↓ USB3
NUC（图像采集、装甲板识别、跟随计算）
  ↓ USB 串口 /dev/gimbal
A 板（云台和发射机构控制）
  ↓
Yaw/Pitch 电机、摩擦轮、拨弹轮
```

CAN 是 A 板内部与电机等设备的通信方式；NUC 与 A 板之间当前使用串口，暂不使用原项目的 CAN/CBoard 通信路径。

### 2.2 串口配置

```text
设备：/dev/gimbal
波特率：115200
数据位：8
校验位：None
停止位：1
流控：None
简称：115200 8N1
结尾：\r\n
```

### 2.3 A 板命令

```text
ge\r\n                       使能 Yaw/Pitch
gd\r\n                       失能 Yaw/Pitch
yr <相对角度> <最大速度>\r\n   Yaw 相对运动
pr <相对角度> <最大速度>\r\n   Pitch 相对运动
```

示例：

```text
ge\r\n
yr -2.000 5.000\r\n
pr 1.000 5.000\r\n
gd\r\n
```

## 3. 相机与识别

### 3.1 当前相机参数

```text
分辨率：3088 × 2064
fx：2493.8093629470959
fy：2493.4350742530842
cx：1528.1888096222829
cy：951.1380332621368
rotate_180：true
```

主配置：[`configs/infantry_daheng.yaml`](sp_vision_25-main/configs/infantry_daheng.yaml)

### 3.2 只运行装甲板识别

```bash
cd "/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main"

./build/camera_detect_test configs/infantry_daheng.yaml
```

传统方法：

```bash
./build/camera_detect_test configs/infantry_daheng.yaml --tradition=true
```

按 `q` 退出。

### 3.3 采集标定图像

```bash
./build/capture_camera \
  configs/calibration_daheng.yaml \
  --output-folder=assets/handeye_preview_rotated
```

按 `s` 保存，按 `q` 退出。

## 4. Gimbal1 手动串口测试

### 4.1 发送命令

```bash
./build/gimbal1_test configs/gimbal1.yaml
```

程序内命令：

```text
e          发送 ge，使能
d          发送 gd，失能
y 2 5      Yaw 相对移动 +2°，限速 5°/s
y -2 5     Yaw 相对移动 -2°，限速 5°/s
p 1 5      Pitch 相对移动 +1°，限速 5°/s
h          显示帮助
q          发送 gd 并退出
```

### 4.2 发送命令并显示实际角度

```bash
./build/gimbal1_feedback configs/gimbal1.yaml
```

输出示例：

```text
[feedback] yaw_actual=-10.732 pitch_actual=34.949
```

### 4.3 查看完整 26 字段反馈

```bash
./build/gimbal1_feedback_monitor configs/gimbal1.yaml
```

重要字段：

```text
2   Yaw actual
5   Yaw active
6   Yaw fault
9   Pitch actual
12  Pitch active
13  Pitch fault
```

> 注意：一次只能运行一个打开 `/dev/gimbal` 的程序。

## 5. 视觉跟随程序

源码：[`tests/gimbal1_armor_track.cpp`](sp_vision_25-main/tests/gimbal1_armor_track.cpp)

### 5.1 跟随原理

```text
识别装甲板四角
  ↓
计算装甲板中心
  ↓
与相机光轴中心比较
  ↓
使用相机内参换算 Yaw/Pitch 角度误差
  ↓
按比例、限幅和死区生成 yr/pr 相对命令
```

经实验确认的方向映射：

```text
yaw_command   = -yaw_error
pitch_command = -pitch_error
```

显示元素：

- 绿色四边形：装甲板检测框；
- 红点：装甲板中心；
- 青色十字：相机光轴中心；
- 紫色连线：两个中心的偏差；
- 跟随目标：让红点向青色十字靠拢并进入死区。

### 5.2 Dry-run：只计算，不控制云台

```bash
./build/gimbal1_armor_track configs/infantry_daheng.yaml
```

窗口将显示像素误差和估算角度，不打开串口。

### 5.3 保守跟随参数

```bash
./build/gimbal1_armor_track configs/infantry_daheng.yaml \
  --control=true \
  --gain=0.35 \
  --min_step=1.0 \
  --max_step=10.0 \
  --control_rate=2 \
  --speed=10 \
  --deadband=80 \
  --max_active_seconds=0
```

### 5.4 较快跟随参数（待进一步验证）

```bash
./build/gimbal1_armor_track configs/infantry_daheng.yaml \
  --control=true \
  --gain=0.35 \
  --min_step=1.0 \
  --max_step=10.0 \
  --control_rate=4 \
  --speed=15 \
  --deadband=80 \
  --max_active_seconds=0
```

参数说明：

```text
gain                 角度误差到相对命令的比例
min_step             超出死区后的最小非零角度
max_step             单条命令最大相对角度，不是每次固定转过的角度
control_rate          每秒最多发送的控制指令数
speed                 A 板命令中的最大速度（°/s）
deadband              画面中心死区（px）
max_active_seconds    0 表示不自动暂停，由人工按 s 暂停
```

### 5.5 控制按键

**先用鼠标点击图像窗口，再按键，不要在终端输入这些字母。**

```text
e   使能云台并开始/继续跟随
s   暂停跟随，保持云台使能和当前位置
d   发送 gd，云台失能
q   发送 gd 并退出
```

Pitch 失能后可能因重力下垂，异常时优先按 `s`；按 `d/q` 前扶住云台。

## 6. 编译

### 6.1 完整配置和编译

```bash
cd "/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main"

cmake -S . -B build
cmake --build build -j2
```

### 6.2 只编译视觉跟随

```bash
cmake --build build --target gimbal1_armor_track -j2
```

### 6.3 查看跟随参数

```bash
./build/gimbal1_armor_track --help
```

## 7. Systemd 开机服务

项目中有两个服务文件：

```text
sp_vision_safe_debug.service      开机启动无窗口相机识别，Restart=always
sp_vision_gimbal_sweep.service    开机执行一次云台扫描任务
```

查看是否已启用：

```bash
systemctl is-enabled sp_vision_safe_debug.service
systemctl is-enabled sp_vision_gimbal_sweep.service

systemctl status sp_vision_safe_debug.service
systemctl status sp_vision_gimbal_sweep.service
```

手动调试前停止服务，避免占用相机或串口：

```bash
sudo systemctl stop sp_vision_safe_debug.service
sudo systemctl stop sp_vision_gimbal_sweep.service
```

如果暂时不希望每次开机自动执行：

```bash
sudo systemctl disable --now sp_vision_safe_debug.service
sudo systemctl disable --now sp_vision_gimbal_sweep.service
```

重新启用：

```bash
sudo systemctl enable sp_vision_safe_debug.service
sudo systemctl enable sp_vision_gimbal_sweep.service
```

安装/更新云台扫描服务：

```bash
sudo ./install_gimbal_sweep_service.sh
```

## 8. 设备占用与故障排查

### 8.1 查看大恒相机

```bash
lsusb -d 2ba2:
```

已测设备示例：

```text
ID 2ba2:4d55 Daheng Imaging MER2-630-60U3C
```

### 8.2 查看相关程序

```bash
pgrep -af 'gimbal1|camera_detect|capture_camera|sp_vision'
```

### 8.3 查看串口占用

```bash
ls -l /dev/gimbal
sudo fuser -v /dev/gimbal
```

### 8.4 主要原则

- 同一时间只允许一个程序打开大恒相机；
- 同一时间只允许一个程序控制 `/dev/gimbal`；
- 不要同时运行 `gimbal1_test`、`gimbal1_feedback` 和 `gimbal1_armor_track`；
- `GXOpenDeviceByIndex failed` 通常表示相机已被另一个程序占用；
- 终端显示 `[control] sent ...` 只证明 NUC 已写串口，最终是否执行需结合画面误差和云台实际动作判断。

## 9. 当前已知问题

1. **A 板反馈可靠性**  
   曾出现 `actual` 值滞后、跳变，以及失能后云台已下垂但角度仍保持旧值。需电控继续检查编码器零点、连续角度、ACTIVE 和故障码。

2. **Pitch 失能下垂**  
   `gd` 后 Pitch 可能自由下垂到机械下限，测试时需扶住云台。

3. **跟随响应速度与振荡折中**  
   提高 `control_rate`、`gain`、`speed` 可加快响应，但会增加过冲和左右摆动风险。快速参数需继续用真实装甲板测试。

4. **当前跟随是二维画面居中**  
   它不依赖云台角度反馈，也未包含完整手眼标定、目标三维坐标、弹道补偿和运动预测。

5. **旧手眼参数不能直接作为当前机器人最终结果**  
   当前 `R_camera2gimbal` / `t_camera2gimbal` 来自旧配置，未完成当前相机和云台的正式手眼标定。

## 10. 建议的后续顺序

1. 用真实装甲板测试较快跟随参数，记录响应时间、过冲和丢目标情况；
2. 在跟随程序中同时显示 A 板 `target/actual/active/fault`，但不依赖错误反馈发控制命令；
3. 修复 Pitch 失能下垂与反馈跳变问题；
4. 完成当前机器人的正式手眼标定；
5. 接入三维解算、云台角度反馈、弹速和弹道补偿；
6. 进行静态靶、慢速移动靶和快速移动靶测试；
7. 在限位、急停、方向、反馈和弹道全部验证前，保持 `auto_fire: false`。

## 11. 主要新增/修改文件

- [`io/gimbal1/gimbal1.hpp`](sp_vision_25-main/io/gimbal1/gimbal1.hpp)
- [`io/gimbal1/gimbal1.cpp`](sp_vision_25-main/io/gimbal1/gimbal1.cpp)
- [`tests/gimbal1_test.cpp`](sp_vision_25-main/tests/gimbal1_test.cpp)
- [`tests/gimbal1_feedback.cpp`](sp_vision_25-main/tests/gimbal1_feedback.cpp)
- [`tests/gimbal1_feedback_monitor.cpp`](sp_vision_25-main/tests/gimbal1_feedback_monitor.cpp)
- [`tests/gimbal1_armor_track.cpp`](sp_vision_25-main/tests/gimbal1_armor_track.cpp)
- [`src/gimbal1_autorun.cpp`](sp_vision_25-main/src/gimbal1_autorun.cpp)
- [`configs/gimbal1.yaml`](sp_vision_25-main/configs/gimbal1.yaml)
- [`configs/infantry_daheng.yaml`](sp_vision_25-main/configs/infantry_daheng.yaml)
- [`configs/calibration_daheng.yaml`](sp_vision_25-main/configs/calibration_daheng.yaml)
- [`sp_vision_safe_debug.service`](sp_vision_25-main/sp_vision_safe_debug.service)
- [`sp_vision_gimbal_sweep.service`](sp_vision_25-main/sp_vision_gimbal_sweep.service)

---

## 安全提醒

- 调试前确保云台运动范围内无人员、线束和障碍物；
- 视觉跟随异常时优先按 `s` 暂停；
- `d/q` 会发送 `gd`，Pitch 可能因重力下垂；
- 当前仅用于识别与云台联调，不得开启自动发射。
