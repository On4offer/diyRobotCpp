# 系统架构

```text
Camera / synthetic image
          |
          v
 ColorTargetDetector ----> TableCalibration ----> SO101Kinematics
          |                                          |
          +--------------> GraspController <---------+
                                  |
                                  v
                         ArmController / FeetechBus
                                  |
                         COM24 or /dev/ttyUSB*

LeRobot dataset <---- C++ quality gate / command builder ----> ACT or Diffusion

ROS 2 target_pixel topic --> grasp_node --> joint_states topic
                                |               |
                          start_grasp service   +--> robot_state_publisher / RViz
```

## 分层原则

- `protocol` 只处理字节帧、寄存器和串口，不理解关节语义。
- `motion` 依赖抽象 `ServoIO`，测试使用假总线，真机使用 Feetech 适配器。
- `vision` 和 `calibration` 不依赖 GUI；OpenCV 只位于可选相机入口。
- `kinematics` 负责 FK/IK、关节限位和整条插值路径的桌面防撞检查。
- `grasp` 用显式状态机编排检测、接近、下降、夹取、校验、放置和恢复。
- `lerobot_pipeline` 不重新实现 ML，只验证数据契约并生成边界明确的命令。
- `diyrobot_ros` 复用核心 C++ 库，ROS 消息只是系统适配层。

## 线程与所有权

核心库不创建后台线程。Qt/ROS 事件循环拥有调度权，串口对象使用 RAII，接口以引用表达非拥有关系。这样测试可确定性执行，也避免退出时遗留硬件写线程。
