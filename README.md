# icd27_infantry
目前项目已经完成“相机识别 → 串口控制 → 基于画面误差的云台跟随”基本链路，但反馈、手眼标定和发射尚未完成。
一、目前完成的功能
1. 大恒相机接入
- 相机：Daheng MER2-630-60U3C。
- 分辨率：3088×2064。
- 相机倒装问题已处理，配置中启用：
rotate_180: true
- 已下载并接入大恒 SDK。
- 相机内参标定已完成，重投影误差约 0.2375 px。
- 旋转后的主要内参：
fx = 2493.809
fy = 2493.435
cx = 1528.189
cy = 951.138
配置文件：
- [infantry_daheng.yaml](/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main/configs/infantry_daheng.yaml)
- [calibration_daheng.yaml](/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main/configs/calibration_daheng.yaml)
2. 装甲板识别
已经确认：
- 相机读取正常；
- YOLO 推理正常；
- 能稳定识别真实装甲板；
- 可以输出颜色、数字、大小和置信度；
- 画面速度约 40～50 FPS；
- 例如：
0.96 blue three small
运行识别：
cd "/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main"

./build/camera_detect_test configs/infantry_daheng.yaml
传统识别模式：
./build/camera_detect_test configs/infantry_daheng.yaml --tradition=true
按 q 退出。
3. NUC与A板串口通信
当前采用：
115200 baud
8N1
无流控
CRLF结尾：\r\n
设备：/dev/gimbal
协议：
ge\r\n                       使能
gd\r\n                       失能
yr 相对角度 最大速度\r\n      Yaw相对运动
pr 相对角度 最大速度\r\n      Pitch相对运动
例如：
ge\r\n
yr -2.000 5.000\r\n
pr 1.000 5.000\r\n
gd\r\n
已确认手动命令能驱动云台上下、左右运动。
手动测试：
./build/gimbal1_test configs/gimbal1.yaml
程序内输入：
e
y 2 5
y -2 5
p 1 5
d
q
4. A板反馈解析
A板每100 ms发送一行26字段CSV。
已实现：
./build/gimbal1_feedback configs/gimbal1.yaml
它显示实际角度：
[feedback] yaw_actual=...
[feedback] pitch_actual=...
查看完整26字段：
./build/gimbal1_feedback_monitor configs/gimbal1.yaml
关键字段：
第2项：Yaw actual
第5项：Yaw active
第6项：Yaw fault
第9项：Pitch actual
第12项：Pitch active
第13项：Pitch fault
目前A板反馈曾出现滞后、跳变或失能后仍保持旧值，因此视觉追踪暂时没有依赖反馈角度。
5. 无反馈视觉跟随
新增程序：
[gimbal1_armor_track.cpp](/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main/tests/gimbal1_armor_track.cpp)
控制过程：
识别装甲板
→ 计算装甲板中心
→ 与相机光轴中心比较
→ 根据内参换算角度
→ 发送yr/pr相对角度
已经确认方向映射：
yaw_command   = -yaw_error
pitch_command = -pitch_error
画面元素：
- 绿色四边形：装甲板检测框；
- 红点：装甲板中心；
- 青色十字：相机光轴中心；
- 紫色连线：目标中心与光轴中心的偏差；
- 控制目标：让红点和青色十字重合。
已经观察到云台能够跟随，但早期参数响应约需20秒。之后提高到固定2°命令，响应缩短到约4～5秒。目前正在继续提高响应速度。
配置中的单条相对角度安全上限已经提高到：
gimbal1_max_relative_angle_deg: 10.0
gimbal1_max_speed_deg_s: 30.0
注意：10° 是最大值，不是每次固定转10°。
二、视觉跟随相关指令
只预览、不控制云台
./build/gimbal1_armor_track configs/infantry_daheng.yaml
它不会打开串口。
当前较快的跟随参数
./build/gimbal1_armor_track configs/infantry_daheng.yaml \
  --control=true \
  --gain=0.35 \
  --min_step=1.0 \
  --max_step=10.0 \
  --control_rate=4 \
  --speed=15 \
  --deadband=80 \
  --max_active_seconds=0
含义：
gain=0.35              误差到相对角度的比例
min_step=1°            非零命令最小1°
max_step=10°           单次最多10°
control_rate=4         每秒最多发送4条命令
speed=15°/s            A板运动限速
deadband=80 px         中心±80 px内停止该轴
max_active_seconds=0   不自动暂停
程序启动后默认不会运动。点击图像窗口，然后按：
e  使能并开始跟随
s  暂停发送命令，但云台保持使能
d  发送gd，云台失能
q  发送gd并退出
由于Pitch失能后可能下垂，测试时优先使用 s；按 d/q 前扶住云台。
若出现过冲或来回振荡，降低为：
--control_rate=2
--speed=10
三、编译指令
修改代码后重新编译：
cd "/home/ros/步兵/同济大学SuperPower战队25赛季自瞄算法开源/sp_vision_25-main"

cmake -S . -B build
cmake --build build -j2
只编译追踪程序：
cmake --build build --target gimbal1_armor_track -j2
查看参数：
./build/gimbal1_armor_track --help
四、Systemd开机服务
存在两个服务：
相机识别服务
sp_vision_safe_debug.service
- 开机运行识别；
- Restart=always；
- 运行时会占用大恒相机。
云台扫描服务
sp_vision_gimbal_sweep.service
- 开机后执行一次云台扫描；
- 完成后等待10秒并发送 gd；
- 如果已启用，则每次开机都会运行。
查看状态：
systemctl status sp_vision_safe_debug.service
systemctl status sp_vision_gimbal_sweep.service
手动测试前停止：
sudo systemctl stop sp_vision_safe_debug.service
sudo systemctl stop sp_vision_gimbal_sweep.service
如果暂时不希望开机自动运行：
sudo systemctl disable --now sp_vision_safe_debug.service
sudo systemctl disable --now sp_vision_gimbal_sweep.service
重新启用：
sudo systemctl enable sp_vision_safe_debug.service
sudo systemctl enable sp_vision_gimbal_sweep.service
五、重要注意事项
同一时间：
- 只能有一个程序占用大恒相机；
- 只能有一个程序控制 /dev/gimbal；
- 不要同时运行 gimbal1_test、gimbal1_feedback 和 gimbal1_armor_track。
检查串口占用：
sudo fuser -v /dev/gimbal
检查相关程序：
pgrep -af 'gimbal1|camera_detect|sp_vision'
当前发射功能保持关闭：
auto_fire: false
不要在反馈、限位、手眼标定和弹道解算完成前开启。
六、仍未完成的问题
1. 响应速度继续调节
当前能跟随，但需要验证最新的 4 Hz、15°/s、最大10° 参数是否稳定，有无过冲。
2. Pitch电控和角度反馈
反馈曾出现跳变、旧值和失能后角度不更新，需要电控检查零点、连续角度、ACTIVE和故障状态。
3. 手眼标定
当前配置中的 R_camera2gimbal、t_camera2gimbal 不是这台机器人最终可靠标定结果。视觉居中可以暂时绕过，但三维瞄准和弹道解算不能绕过。
4. 真实运动装甲板测试
当前应继续用真实装甲板测试不同距离、速度、照明和曝光，而不只是手机/电脑屏幕。
5. 弹道解算与发射
尚未接入可靠的距离解算、弹速、提前量和发射控制，auto_fire 仍为关闭状态。
当前整体进度可以概括为：
相机标定 ✅
倒装修正 ✅
装甲板识别 ✅
串口手动控制 ✅
26字段反馈解析 ✅（但A板反馈可靠性待修）
无反馈视觉跟随 ✅（参数仍在优化）
手眼标定 ❌
可靠三维瞄准 ❌
弹道解算与自动发射 ❌
