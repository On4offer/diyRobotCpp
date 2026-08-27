# D1-D8 acceptance boundaries

Use three labels: **implemented**, **offline verified**, and **physical acceptance pending**.

- D1 protocol: offline verification covers packet bytes, parsing, corrupt input, stream recovery, and fake transport. Physical acceptance requires real discovery, read/write, timeout, wrong-port, and bus-contention evidence.
- D2 motion: offline verification covers profiles, limits, kinematics, unreachable poses, path checks, and emergency-stop commands. Physical acceptance requires upright setup, bounded speed/load, smooth motion, and measured stop behavior.
- D3 tooling: offline verification covers GUI/CLI build and mocked state transitions. Physical acceptance requires live polling, controlled writes, disconnect behavior, and an independently reachable stop path.
- D4 vision/grasp: offline verification covers image processing, calibration math, state transitions, retries, and recovery injection. Physical acceptance requires camera calibration, workspace error measurements, collision checks, and the roadmap's repeated randomized grasp trial.
- D5 data: command generation and metadata checks do not equal data collection. Acceptance requires real synchronized episodes, video/state/action review, and dataset quality evidence.
- D6 learning: command generation does not equal training. Acceptance requires GPU training logs, selected checkpoints, bounded physical rollout, videos, failures, and measured success rate.
- D7 ROS 2: package/launch/URDF build and dry-run messages are offline evidence. Physical acceptance requires the real driver, TF/joint-state validation, collision geometry, and requested RViz/MoveIt/Gazebo behavior.
- D8 delivery: require reproducible builds, CI, tests, install/deploy docs, configuration ownership, logs, failure behavior, and a clean repository. Deployment files alone are not production validation.

Never infer a later-stage acceptance from an earlier-stage mock. Record exact commands and results, and keep missing hardware/data evidence visible in `docs/ROADMAP_STATUS.md`.
