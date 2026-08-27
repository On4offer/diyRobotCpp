# learning_roadmap 对照状态

更新时间：2026-08-27。这里区分“C++ 软件完成”“当前电脑已验证”和“必须实机/数据验证”。

## 已完成并可离线验证

- D1：Feetech Protocol 1.0 帧构造、状态解析、顺序 Ping、寄存器读写、Windows/POSIX 串口 RAII。
- D2：梯形/三角形、线性、正弦缓动轨迹，逐帧限幅，多关节控制抽象，FK/垂直抓取 IK，路径防撞。
- D3：Qt6 可选调试面板源码；CLI 只读状态命令；控制模式与急停交互。
- D4：HSV 环绕阈值、连通域、面积/主方向、平面单应最小二乘、抓取/重试/恢复状态机、合成图像闭环。
- D5：LeRobot 录制命令契约、30 条默认下限、本地 v3 元数据读取和质量门禁。
- D6：ACT/Diffusion 训练命令与有界 rollout 命令；保持 LeRobot 算法库边界。
- D7：ROS 2 Jazzy 包、Topic/Service、URDF、robot_state_publisher、launch，默认离线发布关节命令。
- D8：核心状态机、异常边界、日志友好 CLI、CMake install、Docker、systemd、Windows/Linux CI。

## 当前电脑验证证据

- Windows MinGW：严格警告视为错误，核心库、CLI 和测试构建成功，42/42 离线检查通过。
- WSL Ubuntu 24.04：GCC 13 严格构建和 42/42 离线检查通过。
- WSL ASan/UBSan：Debug 构建及全部离线检查通过。
- ROS 2 Jazzy：`diyrobot_ros` 通过 `colcon build`，严格警告视为错误。
- Qt6/OpenCV 可选应用在缺少对应开发包的平台会跳过，因此本轮结果不代表 GUI 或真实相机验收。

## 无实机条件下不能判定完成

- D1/D2：真实 COM24 舵机发现、平滑运动、过载与急停响应。
- D3：Qt 真机轮询、滑块写控和物理急停。
- D4：相机内参、真实桌面单应、50 次随机摆放 ≥80% 成功率。
- D5：主从臂遥操作和 30–50 条真实演示数据。
- D6：GPU 训练曲线、策略真机 rollout、成功率和视频。
- D7：真机 ROS 驱动、TF 实测、RViz/MoveIt/Gazebo 碰撞模型验收。

这些条目不会因为 mock、合成数据或 dry-run 通过而标记为物理交付。
