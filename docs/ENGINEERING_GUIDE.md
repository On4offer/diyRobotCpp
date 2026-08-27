# 工程质量基线

本项目借鉴成熟机器人项目的做法，但不会照搬它们的框架规模：

- [MoveIt 2](https://github.com/moveit/moveit2)：统一 `clang-format`、多 ROS 发行版 CI、测试与覆盖率门禁。
- [ros2_control](https://github.com/ros-controls/ros2_control)：稳定接口与硬件后端解耦、插件边界、严格格式和跨发行版构建。
- [Navigation2](https://github.com/ros-navigation/navigation2)：按职责拆包、参数与插件文档、迁移说明、CI、覆盖率和容器化工作流。

对应到 diyRobotCpp：

- `include/diyrobot` 只放稳定接口，平台和设备细节留在实现层或应用层。
- 串口、舵机、抓取硬件通过接口注入，因此无设备时仍能覆盖协议、运动和恢复逻辑。
- 所有自有 C++ target 使用同一组严格警告；CI 把警告视为错误。
- `.clang-format` 统一为 C++17、100 列，禁止把控制流和多个语句压成一行。
- Windows、Ubuntu、ASan/UBSan 分开验证；ROS 2 包单独构建。
- 真机命令默认 dry-run；未实现的 ROS 硬件后端会拒绝启动，而不是静默模拟成功。
- 每项路线图状态区分“已实现”“离线验证”“实机验收待完成”。

项目专用 Agent Skill 位于 `.agents/skills/diyrobot-cpp-roadmap/`。它把上述约束变成修改代码时的检查流程，并提供 Windows/WSL 验证入口。

## 本地质量检查

Windows：

```powershell
pwsh -File .agents/skills/diyrobot-cpp-roadmap/scripts/verify.ps1
```

WSL：

```bash
bash .agents/skills/diyrobot-cpp-roadmap/scripts/verify.sh
bash scripts/build-ros2.sh
```

无实机验证不包含真实舵机响应、相机标定误差、抓取成功率、GPU 训练或策略真机 rollout。
