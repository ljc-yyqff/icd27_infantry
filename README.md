# icd27_infantry · 20261031

基于同济大学 SuperPower 战队 25 赛季开源项目的步兵视觉适配，当前硬件链路为 **大恒相机 → NUC → A 板 ASCII 串口 → 云台**。本分支整理截至 2026-10-31 的相机、串口、视觉跟随、单装甲板预测和运行维护工具。

项目目录与 GitHub 仓库统一命名为 `icd27_infantry`。原项目介绍及理论见 [readme.md](readme.md)，历史适配记录见 [README_2610.md](README_2610.md)。历史文档中的旧路径、26 列反馈、`--gain` 和非零 `--min_step` 不适用于当前跟随入口。

## 相比“10 月前步兵”增加或调整了什么

对比基线为本仓库 `202610_beford` 分支的提交 `78fbe77`（提交说明：上传10月前自瞄）。以下按代码差异整理；基线中已经有大恒相机、ASCII 串口、像素跟随和旧开机脚本，不能把它们全部视为本次新增。

| 模块 | 基线已有 | 本次新增或调整 |
| --- | --- | --- |
| 串口协议 | 手动 `ge/gd/yr/pr` 与旧反馈解析 | 适配 9 列遥测、`#GM` 状态、`#GC` 回执；增加逐行姿态样本、有界缓存及协议测试 |
| 视觉跟随 | 装甲板中心误差转相对角度 | 按每轴实际发送间隔积分，Yaw/Pitch 独立增益、小角度速度匹配、死区滞回、目标连续关联与轻量滤波 |
| 图像时效 | 大恒图像采集与缓存 | 队列满时替换旧帧；使用主机取帧时间检查图像年龄，避免旧帧继续生成增量 |
| 相机配置 | 180° 旋转画面及对应内参 | 当前 `rotate_180: false`，内参及切向畸变同步还原到原始正向画面 |
| 单装甲板模型 | 当前 ASCII 跟随入口未接入此链路 | 新增 PnP 位置解算、原项目 EKF 位置/速度估计、两轴 IMU 补偿、短时预测及速度前馈 |
| 模式对照 | 像素跟随 | 新增 `pixel / shadow / predictive`；默认仍为 pixel，模型不可用时保留有效图像的像素基础跟随 |
| 日志与回放 | 基础运行与串口日志 | 增加逐帧 `.vision.csv`、模型原因、角点/姿态/速度/发送参数，以及无相机、无串口的离线回放 |
| 开机运行 | 旧识别服务、旧扫描服务 | 新增 `icd27_armor_track.service` 及安装脚本；安装时替换旧服务的开机职责 |
| 验证 | 原有协议测试 | 扩展协议测试，新增伪串口集成、姿态样本、视觉跟随、单板估计及信号停止测试 |

当前 ASCII 跟随入口不调用弹道、自动开火或旧二进制电控接口；单装甲板模型也不是整车多装甲板旋转模型。预测目前按底座固定、roll 近似不变的两轴姿态补偿实现，相机外参、IMU 符号和时延仍需实体核对，离线通过不能证明实体跟随效果。

## 目录与配置

| 路径 | 用途 |
| --- | --- |
| `configs/infantry_daheng.yaml` | 大恒相机、内参、模型、串口及单板预测配置 |
| `configs/gimbal1.yaml` | 独立串口工具配置 |
| `configs/calibration_daheng.yaml` | 相机标定采集配置 |
| `io/gimbal1/` | 独立 ASCII 串口实现，不使用旧 CBoard 二进制协议 |
| `tests/gimbal1_armor_track.cpp` | 当前识别与跟随程序入口 |
| `tools/visual_tracking.hpp` | 图像时效、目标关联、视觉相对指令生成 |
| `tasks/auto_aim/single_armor_tracking.*` | 单装甲板估计、姿态匹配与预测 |
| `tests/single_armor_replay.cpp` | 历史跟随日志回放 |
| `docs/` | 当前串口、跟随与服务说明 |

## 构建

使用 C++17、CMake ≥ 3.16.3，依赖 OpenCV、fmt、Eigen3、spdlog、yaml-cpp、nlohmann_json、libusb、OpenVINO 及相机 SDK；其余原项目依赖见 [原项目编译说明](readme.md#32-编译方式)。顶层 CMake 当前把 OpenVINO 路径写为 `/opt/intel/openvino_2024.6.0/runtime/cmake/`，换机器时需按实际安装位置调整。未找到 ROS2 时跳过相关入口。

大恒头文件默认从 `third_party/daheng_sdk/sdk_payload/Galaxy_camera` 查找，也可通过 `-DDAHENG_SDK_ROOT=/实际路径/Galaxy_camera` 指定；链接及运行还需能找到 `gxiapi` 动态库。项目的 `io` 库还链接海康、迈德威视 SDK，并非只安装大恒 SDK 就能完成所有依赖。

```bash
cd /home/ros/infantry/icd27_infantry
cmake -S . -B build/ascii_tracking -DBUILD_TESTING=ON
cmake --build build/ascii_tracking --target \
  gimbal1_armor_track gimbal1_test gimbal1_feedback gimbal1_feedback_monitor \
  single_armor_replay gimbal1_protocol_test gimbal1_pose_samples_test \
  visual_tracking_test single_armor_tracking_test gimbal1_signal_stop_test -j2
```

这里的 `ascii_tracking` 是构建目录名，不是独立构建开关。

## 常用程序和操作

先停止占用相机/串口的服务，再进行手动调试：

```bash
sudo systemctl stop icd27_armor_track.service
cd /home/ros/infantry/icd27_infantry

# 只识别和显示，不打开串口
./build/ascii_tracking/gimbal1_armor_track configs/infantry_daheng.yaml

# 查看完整参数
./build/ascii_tracking/gimbal1_armor_track --help

# 手动串口控制 / 带反馈交互 / 只监视反馈（选一个运行）
./build/ascii_tracking/gimbal1_test configs/gimbal1.yaml
./build/ascii_tracking/gimbal1_feedback configs/gimbal1.yaml
./build/ascii_tracking/gimbal1_feedback_monitor configs/gimbal1.yaml
```

串口使用 `/dev/gimbal`、115200 8N1、ASCII、CRLF。手动工具中 `e` 发送 `ge`，`d` 发送 `gd`，`y 2 5` / `p 1 5` 分别请求 Yaw / Pitch 相对运动，`q` 撤力并退出。相机跟随窗口中 `e` 开始输出、`s` 暂停视觉输出、`d/q` 显式撤力；Pitch 撤力后可能下垂。

跟随默认 `--control=false`、`--tracking=pixel`；加 `--control=true` 才打开串口。`shadow` 计算并记录模型但仍输出像素控制，`predictive` 在同一像素基础上叠加可用的预测与前馈。详细运行示例、参数、日志字段和限制见 [当前跟随说明](docs/gimbal1_tracking.md)。旧 `--gain` 已停用，`--min_step` 必须为 0。

所有 ASCII 行的总发送频率不超过 50Hz；当前 shadow/predictive 还受配置中的 25Hz 上限约束，以适配当前固件遥测供给。串口写成功仅代表主机完成写入，不代表电机已执行；电机执行、到位和保护由电控负责。

## 脚本与服务

| 文件 | 作用与当前适用性 |
| --- | --- |
| `install_armor_track_service.sh` | 安装并启用新服务；停止、禁用两个旧服务；不会立即启动新服务 |
| `icd27_armor_track.service` | 新跟随服务，默认 pixel，`--control=true --auto_start=true`，失败后每 3 秒重启 |
| `safe_debug_start.sh` / `autostart.sh` | 历史相机识别启动脚本，仍引用旧项目绝对路径和 `build/`，迁移前须修改 |
| `install_gimbal_sweep_service.sh` | 历史扫描服务安装脚本，也保留旧绝对路径，不作为当前跟随安装入口 |
| `sp_vision_safe_debug.service` | 旧相机识别开机任务 |
| `sp_vision_gimbal_sweep.service` | 旧云台扫描开机任务 |
| `99-sp-vision-gimbal.rules` | 串口设备命名规则，部署时核对设备匹配条件 |

**当前服务文件与旧服务说明有差异：仓库中的 ExecStart 实际为 `--headless=false`。** 它会使用图像窗口，不能直接宣称“不依赖桌面登录”。若需无桌面运行，先把服务文件改为 `--headless=true`，再安装。服务和安装脚本固定使用用户 `ros`、目录 `/home/ros/infantry/icd27_infantry`；换机器时同步修改。

```bash
# 安装/更新：只启用下次开机，不立即运行
sudo ./install_armor_track_service.sh

systemctl is-enabled icd27_armor_track.service
systemctl status icd27_armor_track.service --no-pager
journalctl -u icd27_armor_track.service -b --no-pager
journalctl -u icd27_armor_track.service -f

# 手动停止 / 取消开机启动并停止
sudo systemctl stop icd27_armor_track.service
sudo systemctl disable --now icd27_armor_track.service
```

安装完成后，如需立即运行，可执行 `sudo systemctl start icd27_armor_track.service`；该服务会在首张新鲜图像后请求使能。只有 `control/headless/auto_start` 同时为 true 时，程序收到 SIGTERM/SIGINT 才在关闭串口前发送一次 `gd`；因此当前 `headless=false` 的仓库服务不满足此条件。更多背景见 [服务说明](docs/gimbal1_systemd.md)，其中无窗口描述须以上述实际配置为准。

## 日志、离线测试与排查

运行日志在工作目录 `logs/gimbal1/`：`tracking_*.log` 为原始串口收发，`tracking_*.vision.csv` 为逐帧视觉与预测数据。离线回放接受配置、CSV、原始串口日志三个位置参数：

```text
single_armor_replay <配置.yaml> <tracking_*.vision.csv> <tracking_*.log>
```

```bash
# 无实体相机/串口测试；集成测试仅打开伪终端
ctest --test-dir build/ascii_tracking --output-on-failure

# 设备与占用
ls -l /dev/gimbal
sudo fuser -v /dev/gimbal
pgrep -af 'gimbal1|camera_detect|capture_camera|sp_vision'
```

同一时间只运行一个相机采集程序和一个串口控制程序。相机打开失败先检查服务是否仍在运行。模型不可用原因和回放示例见 [跟随说明](docs/gimbal1_tracking.md)。

## 上传到 GitHub 的 20261031 分支

目标仓库：`https://github.com/ljc-yyqff/icd27_infantry.git`。本目录已经有 `.git` 和 origin，不必重新 `git init`，也不必把整个文件夹嵌套上传到仓库子目录。

以下命令适用于本地仍在 `202610_beford`、尚未创建本地 `20261031` 的状态：

```bash
cd /home/ros/infantry/icd27_infantry
# 先确认远端，再取回分支信息；连接失败时先解决网络/登录
git remote -v
git fetch origin

# 从当前旧基线创建分支，现有未提交修改会保留
git switch -c 20261031
git status --short
git add -A
git diff --cached --stat
# 检查暂存内容后再提交
git commit -m "更新10月31日步兵视觉适配、跟随工具与使用说明"

# 合入远端分支已有提交，再正常推送；不使用强制覆盖
git merge origin/20261031 --no-edit
git push -u origin 20261031
```

若本地已存在 `20261031`，使用 `git switch 20261031`；若已经在该分支则无需切换。合并若出现冲突，应先解决、`git add` 并完成提交，再 push；不要在冲突未解决时继续。HTTPS 登录按本机 Git 凭据管理器提示授权；需要令牌时使用 GitHub Personal Access Token，不使用账户密码。

`.gitignore` 已排除 `build/`、`logs/`、`records/`、`imgs/` 等本地产物；`git add -A` 会包含其余修改、新文件及删除，请先审阅暂存列表。仓库已有较大的 SDK 和演示资源，首次推送可能较慢。若文件已被 Git 跟踪，后来添加忽略规则也不会自动移除它。

上传后在 GitHub 选择 `20261031`，确认根目录代码、README 与最新提交。上述流程保留旧 `202610_beford` 分支，并将新内容提交到目标分支。

## 许可证

沿用仓库 [LICENSE](LICENSE)，保留原项目及第三方组件的署名与许可文件。

## 目录更名后的本机迁移

若之前安装了旧名称服务，先执行 `sudo systemctl disable --now ict27_armor_track.service`，再安装新名称服务。仓库更名不会自动修改 `/etc/systemd/system/` 中已安装的服务。旧构建缓存含有原目录绝对路径，需重新生成；可先将 `build/ascii_tracking` 移到项目外备份，再按上文重新配置和编译。
