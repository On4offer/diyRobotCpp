# diyRobotCpp

SO-ARM101 双臂项目的 C++17 重构版。工程按照原项目 `learning_roadmap.md` 的 D1–D8 组织：自研部分使用 C++，ACT/Diffusion 训练继续调用 LeRobot（路线图明确把它作为算法库），ROS 2 集成在 WSL Ubuntu 24.04 + Jazzy 下构建。

## 工程结构

```text
include/diyrobot/       对外稳定头文件
src/                    D1/D2/D4/D5/D6/D8 核心库实现
apps/                   CLI、可选 Qt 与 OpenCV 程序
tests/                  无实机离线测试
config/                 SO-101 参数与安全默认值
ros2_ws/src/            D7 ROS 2 Jazzy 包、URDF、launch
docs/                   架构、路线图完成边界、运行手册
scripts/                Windows/WSL 构建入口
deploy/                 systemd 服务示例
.github/workflows/      Windows + Ubuntu CI
.agents/skills/         项目专用的 C++/ROS 2 工程质量 skill
```

`build/`、`build-wsl/`、ROS 的 `build/install/log` 都是生成目录，不应提交。

## Windows 构建与测试

```powershell
pwsh -File scripts/build-windows.ps1
```

手动方式：

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Qt6/OpenCV C++ 不存在时，对应 D3/D4 GUI 程序会自动跳过，核心库、CLI 和测试仍可构建。D1/D2/D3 的真机命令默认仅打印 dry-run；只有显式增加 `--run` 才访问串口。

```powershell
build/diyrobot_cli.exe d1-scan COM24
build/diyrobot_cli.exe d1-scan COM24 --run
build/diyrobot_cli.exe d2-home COM24
# 真机回中还必须同时给出 --run --confirm-upright
build/diyrobot_cli.exe d3-status COM24 --run
build/diyrobot_cli.exe d4-sim
```

## WSL / Linux / ROS 2 Jazzy

从 PowerShell 执行：

```powershell
wsl -e bash -lc 'cd /mnt/d/Users/Lenovo/Desktop/CppLearning/Projects/diyRobotCpp && bash scripts/build-wsl.sh'
wsl -e bash -lc 'cd /mnt/d/Users/Lenovo/Desktop/CppLearning/Projects/diyRobotCpp && bash scripts/build-ros2.sh'
```

运行 ROS 2 离线 launch：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch diyrobot_ros diyrobot.launch.py
```

若当前 WSL 没有 `colcon`，脚本会自动采用单包 ament/CMake 构建并安装到
`ros2_ws/install-direct`；此时把上面的 source 路径替换为
`ros2_ws/install-direct/share/diyrobot_ros/local_setup.bash`。标准多包工作流可另行安装
`python3-colcon-common-extensions`，本项目不会自动执行 `sudo apt`。

另一个终端发布目标像素并触发服务：

```bash
ros2 topic pub --once /target_pixel geometry_msgs/msg/Point '{x: 400.0, y: 240.0, z: 0.0}'
ros2 service call /start_grasp std_srvs/srv/Trigger '{}'
```

ROS 节点默认 `dry_run: true`，只发布规划关节角，不连接舵机。

## D5/D6 LeRobot 边界

C++ CLI 负责校验实验参数、生成可审计命令和检查本地数据集质量；LeRobot 负责数据格式、ACT/Diffusion 训练及模型推理。

```powershell
build/diyrobot_cli.exe d5-record On4offer/so101_pick "Pick the block"
build/diyrobot_cli.exe d6-train On4offer/so101_pick act
build/diyrobot_cli.exe d6-rollout outputs/train/act_so101/checkpoints/last/pretrained_model "Pick the block"
build/diyrobot_cli.exe dataset-check D:\datasets\so101_pick
```

生成命令不会被自动执行，避免误启动双臂或长时间训练。完整状态与未实机验收边界见 [docs/ROADMAP_STATUS.md](docs/ROADMAP_STATUS.md)。

## 工程质量

代码使用统一的 100 列 C++17 格式，严格警告覆盖库、应用与测试；CI 包含 Windows、Ubuntu、格式检查和 ASan/UBSan。设计依据、优秀项目参考和本地质量门禁见 [docs/ENGINEERING_GUIDE.md](docs/ENGINEERING_GUIDE.md)。

## 安全约束

- 从臂默认 `COM24`，主臂默认 `COM22`；不要仅凭默认值接真机。
- 广播 Ping 会导致 HX 多舵机应答碰撞，D1 只做顺序 Ping。
- 使能力矩或回中前先把大臂摆到近竖直，避免重力过载 `0x20`。
- 所有真机动作要求明确 `--run`；ROS 默认只离线发布命令。
- 无实机测试只能证明协议数学、规划、视觉、状态机和 ROS 接口，不能证明抓取成功率、标定精度或硬件安全。

## License

MIT，见 [LICENSE](LICENSE)。
