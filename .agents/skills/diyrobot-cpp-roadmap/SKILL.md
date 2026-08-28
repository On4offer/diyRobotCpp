---
name: diyrobot-cpp-roadmap
description: Implement, refactor, review, or validate diyRobotCpp and similar C++17/ROS 2 SO-ARM101 robotics code against the D1-D8 learning roadmap. Use for production structure, readable C++, safety boundaries, offline tests, CMake/CI, ROS 2 integration, or roadmap completion claims; do not use for unrelated C++ exercises or pure conceptual Q&A.
---

# diyRobot C++ 路线图工程

把路线图当作验收契约，而不是单纯的文件清单。

1. 修改某个阶段之前，先阅读 `docs/ROADMAP_STATUS.md` 和相关源码。
2. 任何代码、CMake、测试或 CI 改动，先阅读 [references/quality-gates.md](references/quality-gates.md)。
3. 更新路线图状态或声明完成之前，先阅读 [references/roadmap-acceptance.md](references/roadmap-acceptance.md)。
4. 硬件访问默认保持 dry-run（只打印、不操作）。必须显式传入运行标志，并满足该阶段对应的实物确认条件，才允许访问硬件。
5. 把平台细节放在接口后面；串口、舵机、相机、抓取硬件都通过接口注入，让离线测试不依赖真实设备。
6. 优先使用小而命名清晰的函数和类型化配置，避免压缩成一行、重复的字面量、隐藏的全局状态，以及只是复述语法的注释。
7. 为本次改动涉及的路径补充或更新测试：正常路径、边界、非法输入、超时/失败、恢复。
8. 报告完成前，先运行仓库的验证入口：
   - Windows：`pwsh -File .agents/skills/diyrobot-cpp-roadmap/scripts/verify.ps1`
   - WSL/Linux：`bash .agents/skills/diyrobot-cpp-roadmap/scripts/verify.sh`
9. 软件证据要与相机、舵机、训练和实物世界验收分开报告。

把 MoveIt 2、ros2_control、Navigation2 当作结构参考，而不是照抄的代码。保持本项目面向初学者的可读范围；除非框架真的能消除重复或强制必要的边界，否则不要引入框架。
