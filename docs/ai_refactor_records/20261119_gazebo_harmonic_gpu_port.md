# Gazebo Harmonic GPU 加速仿真移植（路线 B）改造记录

## User Intent

将本仓库 `/home/xyt/map/src/MA_2027_Simulation` 中的 `pb_rm_simulation` 与 `livox_laser_simulation_RO2`（`ros2_livox_simulation`）原地从 Gazebo Classic 11 移植到宿主机原生 Gazebo Harmonic（`gz sim 8.15.0` + OGRE2 GPU 渲染 + `ros-humble-ros-gzharmonic`），参考 `3rd/` 下的开源实现，按阶段增量功能开发、合理测试并分阶段本地 `git commit`（不推送远端）。

关键约束与目标：
1. 原地替换旧包，宿主机直接运行 Gazebo Harmonic + OGRE2 GPU 加速。
2. 保持与下游 `ma_nav` 和 `Super-LIO` 已验证一致的 ROS 2 接口契约（topic、frame、频率、量程、噪声、时间偏移、IMU 单位与外参、无头模式 `gui:=false`、RTF 接近 1.0）。
3. 本次任务获得用户明确授权执行 `colcon build`、无头仿真测试与本地分阶段 `git add`/`git commit`（禁止 `git push`）。
4. 本次修改涉及比赛验证逻辑（IMU 外参/单位、Mid-360 点云距离与时间模型、斜坡静止防滑），采用最小改动与接口保真策略。

## Scope

本次允许修改的范围：
- `src/pb_rm_simulation/`：世界文件（SDF 1.9）、全向底盘 Gazebo Harmonic 插件（`OmniChassisSystem`）、动态障碍物插件（`DynamicObstacleSystem`）、机器人 xacro、launch、`ros_gz_bridge` 配置、环境变量 hook、无头自动化验证脚本。
- `src/livox_laser_simulation_RO2/`：替换为 Harmonic OGRE2 `gpu_lidar` + 独立 C++ `gz-transport13` 桥接节点 `mid360_gz_bridge`、edge-aware 重采样器 `Mid360Resampler` 与 GTest 单元测试。
- `.gitignore`（忽略 `3rd/` 外部参考仓库与本地 `livox_ros_driver2`）。
- `docs/ai_refactor_records/20261119_gazebo_harmonic_gpu_port.md`（本记录）。

## Out of Scope

本次明确不处理的内容：
1. 不修改 `3rd/` 目录下的开源参考仓库源码（`livox_gz_sim`、`rbot`、`Mid360_simulation_plugin`、`RMUL2027_map_transfer_20261016_STEP_R1`）。
2. 不改动 `src/pb_rm_simulation/world/old/`、`src/pb_rm_simulation/world/obstacles/` 与 `src/pb_rm_simulation/meshes/obstacles/` 中的历史归档文件。
3. 不改动 `src/livox_laser_simulation_RO2/scan_mode/mid360.csv`（经检验与 `3rd/livox_gz_sim/scan_patterns/mid360.csv` 字节一致）。
4. 不改动裁判系统业务节点或下游 `ma_nav`/`Super-LIO` 代码，不改变已有对外 ROS 2 topic/frame 名称与 IMU `m/s^2` 单位约定。
5. 不向远端执行 `git push`。

## Explorer Findings

### Files inspected
- `src/pb_rm_simulation/{CMakeLists.txt,package.xml,launch/rm_simulation.launch.py,urdf/simulation_waking_robot.xacro,world/*/*.world}`
- `src/livox_laser_simulation_RO2/{CMakeLists.txt,package.xml,urdf/mid360.xacro,scan_mode/mid360.csv,src/*,include/*}`
- `3rd/livox_gz_sim/{src/livox_lidar.cpp,urdf/livox_lidar.xacro,scan_patterns/mid360.csv,worlds/demo.sdf}`
- `3rd/rbot/src/robot/rlai_description/urdf/gazebo/gazebo_sensors.urdf.xacro`、`3rd/rbot/src/simulation/rlai_gazebo/config/ros_gz_bridge.yaml`
- `docs/ai_refactor_records/{20261020_mid360_imu_mount.md,20261020_mid360_pointcloud_fidelity.md,20261020_fix_robot_static_drift.md}`

### Active logic path
```text
ros2 launch pb_rm_simulation rm_simulation.launch.py world:=<WORLD> gui:=false
  ├─ gz sim (Harmonic 8.15.0, DART physics @ 1000 Hz, OGRE2 GPU rendering)
  │    ├─ pb_rm_omni_chassis_system (/cmd_vel_chassis -> AddWorldWrench + static slope hold)
  │    ├─ pb_rm_dynamic_obstacle_system (dynamic_obstacles:=true -> SetWorldPoseCmd)
  │    ├─ gz-sim-sensors-system (OGRE2 gpu_lidar: 901x151 @ 50 Hz, [-7.4°, +52.4°] -> /livox/lidar/gz_scan)
  │    ├─ gz-sim-imu-system (200 Hz, m/s^2 -> /livox/imu)
  │    ├─ gz-sim-joint-state-publisher-system (50 Hz -> /joint_states)
  │    └─ gz-sim-odometry-publisher-system (3D, 50 Hz -> /sim/ground_truth/odom, 不桥接 /tf)
  ├─ ros_gz_bridge (parameter_bridge): /clock, /cmd_vel_chassis, /livox/imu, /joint_states, /sim/ground_truth/odom
  ├─ mid360_gz_bridge (ros2_livox_simulation):
  │    5 个 50 Hz LaserScan 子帧 (0/20/40/60/80 ms)
  │      --Mid360Resampler (edge-aware bilinear + <laser_retro>)-->
  │    10 Hz /livox/lidar (CustomMsg, 20000 pts) + /livox/lidar/pointcloud (26B LivoxPointXyzrtlt PointCloud2)
  └─ robot_state_publisher (/tf_static + 轮关节 /tf)
```

### Data flow
- **坐标与外参**：`sdformat14` 将固定关节（`mesh` -> `base_link` -> `livox_frame` -> `imu_link`）合并到 canonical link `mesh`，同时精确保留传感器 `<pose>`（`livox_frame` 在 `(0, 0, 0.15)`，`mid360_imu` 在 `(0.011, 0.02329, 0.10588)`）与 `<gz_frame_id>`。
- **激光雷达重采样**：GPU 渲染 `0.4°` 均匀球面网格（`901×151 @ 50 Hz`），`mid360_gz_bridge` 按 `mid360.csv`（800k 射线非重复扫描模式）每帧取 5 个物理时刻子帧（每子帧 4000 点），对平滑表面使用双线性插值、对深度跳变边缘（`edge_jump_min_m=0.20` / `edge_jump_rel=0.10`）回退最近像素，反射率读取 `<laser_retro>`（缺失时回退 100）。
- **真值里程计与 TF 隔离**：`OdometryPublisher`（`<dimensions>3</dimensions>`）仅通过 `gz_bridge.yaml` 桥接 `/sim/ground_truth/odom`，不桥接 `/tf`，避免与 Super-LIO/Point-LIO 的 `odom/world -> base_link` 变换冲突。
- **动态障碍物**：`DynamicObstacleSystem` 支持 `ellipse` 与 `waypoints` 模式，通过 `Model::SetWorldPoseCmd` 驱动 `<kinematic>true</kinematic>` 圆柱体，同时对 DART 碰撞和 OGRE2 `gpu_lidar` 可见，并通过 `<laser_retro>120</laser_retro>` 区分反射率。

### Risk notes
1. `gz-sim-sensors-system` 在无订阅者时惰性跳过渲染；使用 `gz topic -e` 会触发 ASCII protobuf 序列化瓶颈，基准测试必须通过二进制 C++ / ROS 2 订阅端测算 RTF。
2. `mid360.csv` 单帧 100 ms 内仰角周期性上下扫动（0 ms 与 80 ms 子帧偏高仰角 `+23°~+52°`），无屋顶场地中这两个子帧可能全部打向天空返回 `(0,0,0)`，属实物 Risley 棱镜扫描特性。
3. 未改用 `3rd/livox_gz_sim` 的 `LivoxImu`（单位 `g`）和进程内 `rclcpp` 插件，而是保留 `ros_gz_bridge` 的 `m/s^2` IMU 与独立 `mid360_gz_bridge` 进程，避免与 `Super-LIO` 约定冲突及 Gazebo 内部 `rclcpp` 符号/关闭冲突。

### Recommended modification boundary
分 3 个阶段独立验证并本地提交：
- Stage 1（`7f29b9f`）：世界文件 SDF 1.9、全向底盘插件 `OmniChassisSystem`、机器人 xacro、launch 与基础桥接。
- Stage 2（`f990d77`）：`ros2_livox_simulation` Harmonic GPU Mid-360 重采样与桥接节点、`mid360.xacro` 与单元测试。
- Stage 3（本阶段）：3D 真值里程计 `/sim/ground_truth/odom`、`DynamicObstacleSystem` + RMUL2024/RMUL2026 动态障碍物世界、可复用无头测试脚本 `verify_harmonic_sim.py` 与本改造记录。

## Modifier Changes

### Files changed

**Stage 1（已提交 `7f29b9f`）**：
- `.gitignore`
- `src/pb_rm_simulation/CMakeLists.txt`
- `src/pb_rm_simulation/package.xml`
- `src/pb_rm_simulation/env-hooks/pb_rm_simulation.dsv.in`
- `src/pb_rm_simulation/config/gz_bridge.yaml`
- `src/pb_rm_simulation/src/omni_chassis_system.cpp`
- `src/pb_rm_simulation/urdf/simulation_waking_robot.xacro`
- `src/pb_rm_simulation/launch/rm_simulation.launch.py`
- `src/pb_rm_simulation/world/{RMUC2024_world/RMUC2024_world.world,RMUC2027_world/RMUC2027_world.world,RMUL2024_world/RMUL2024_world.world,RMUL2026_world/RMUL2026_world.world,RMUL2026H_world/RMUL2026H_world.world}`

**Stage 2（已提交 `f990d77`）**：
- `src/livox_laser_simulation_RO2/CMakeLists.txt`
- `src/livox_laser_simulation_RO2/package.xml`
- `src/livox_laser_simulation_RO2/include/ros2_livox/mid360_resampler.hpp`（新增）
- `src/livox_laser_simulation_RO2/src/mid360_gz_bridge.cpp`（新增）
- `src/livox_laser_simulation_RO2/test/test_mid360_resampler.cpp`（新增）
- `src/livox_laser_simulation_RO2/urdf/mid360.xacro`
- 删除旧 Gazebo Classic 插件源码：`livox_points_plugin.{h,cpp}`、`livox_ode_multiray_shape.{h,cpp}`、`csv_main.cpp`
- `src/pb_rm_simulation/{launch/rm_simulation.launch.py,package.xml,urdf/simulation_waking_robot.xacro}`

**Stage 3（本次提交）**：
- `src/pb_rm_simulation/src/dynamic_obstacle_system.cpp`（新增）
- `src/pb_rm_simulation/CMakeLists.txt`
- `src/pb_rm_simulation/urdf/simulation_waking_robot.xacro`
- `src/pb_rm_simulation/config/gz_bridge.yaml`
- `src/pb_rm_simulation/launch/rm_simulation.launch.py`
- `src/pb_rm_simulation/world/RMUL2024_world/RMUL2024_world_dynamic_obstacles.world`
- `src/pb_rm_simulation/world/RMUL2026_world/RMUL2026_world_dynamic_obstacles.world`
- `src/pb_rm_simulation/scripts/verify_harmonic_sim.py`（新增）
- `docs/ai_refactor_records/20261119_gazebo_harmonic_gpu_port.md`（新增）

### Key changes
1. **Stage 1**：将 5 个静态场地世界迁移至 SDF 1.9 + OGRE2；实现基于 `Link::AddWorldWrench` 的 `pb_rm_simulation::OmniChassisSystem`（含斜坡静止锁定）；重构 `rm_simulation.launch.py` 使用 `ros_gz_sim` + `ros_gz_bridge`，支持 `gui:=false` 真正 GPU 无头渲染。
2. **Stage 2**：将 Mid-360 仿真从 Gazebo Classic ODE CPU 射线迁移为 Harmonic OGRE2 `gpu_lidar`（`901×151 @ 50 Hz`）+ 独立 C++ 节点 `mid360_gz_bridge`；实现 `Mid360Resampler` 边缘感知双线性插值与 `<laser_retro>` 反射率读取；输出 10 Hz `CustomMsg`（5 子帧 `0/20/40/60/80 ms` 偏移、20000 点）与 26 字节 `LivoxPointXyzrtlt` `PointCloud2`；补齐世界重置时间戳保护与可复现噪声种子；默认开启 `use_livox:=true` 并暴露 `lidar_noise_stddev`。
3. **Stage 3**：接入 `gz-sim-odometry-publisher-system`（3D，50 Hz，仅发布 `/sim/ground_truth/odom`，不发 `/tf`）；实现 `pb_rm_simulation::DynamicObstacleSystem` 并移植 `RMUL2024_world_dynamic_obstacles.world`（同时修复原文件两个同名 `turtlebot3_drl_obstacle4` 冲突）和 `RMUL2026_world_dynamic_obstacles.world`（移除硬编码本机路径 `/home/ros2/Desktop/...`）；在 `rm_simulation.launch.py` 新增 `dynamic_obstacles` 参数（默认 `false`）；提供可复用无头回归验证脚本 `scripts/verify_harmonic_sim.py`。

### Behavior preserved
- `/livox/lidar`（`CustomMsg`，10 Hz，20000 点/帧，有效量程 `[0.1, 40.0)` m，默认高斯测距噪声 `σ = 0.02 m`，无效点 `(0,0,0)` 且 `reflectivity = 0`，`frame_id = livox_frame`）。
- `/livox/imu`（200 Hz，`frame_id = imu_link`，加速度单位 `m/s^2`，与 `livox_frame` 外参偏移 `(0.011, 0.02329, -0.04412)` 严格匹配 Super-LIO）。
- `/cmd_vel_chassis` 全向速度控制与 `/joint_states` 50 Hz 发布；RMUC2027 坡面静止防滑（3 s 漂移约 `1.0 mm`）。
- 默认启动行为保持兼容（`world:=RMUC`、`dynamic_obstacles:=false`、`use_livox:=true`、`lidar_noise_stddev:=0.02`）。
- 仅 `robot_state_publisher` 发布机器人内部 TF，无 `world/odom -> base_link` TF 冲突。

### Behavior intentionally adjusted
- 仿真引擎从 Gazebo Classic 11（ODE CPU 射线）切换为 Gazebo Harmonic 8.15.0（DART 物理 + OGRE2 GPU 渲染），开启 20000 点 Mid-360 后无头与全向运动场景均可稳定维持 `RTF ≈ 1.00`。
- `/livox/lidar/pointcloud` 从旧版 16 字节 `xyz + rgb(0)` 升级为与实车 `livox_ros_driver2` 一致的 26 字节 `LivoxPointXyzrtlt`（`x, y, z, intensity, tag, line, timestamp`），且反射率支持 `<laser_retro>`。
- 新增 3D 真值里程计话题 `/sim/ground_truth/odom`（50 Hz，`world -> base_link`）。
- 新增 `dynamic_obstacles` launch 参数支持 `RMUL` 与 `RMUL2026H` 动态障碍物场景，并修复旧动态世界重复 model 名与硬编码绝对插件路径问题。

### Notes
- 原仓库 `src/livox_laser_simulation_RO2/include/ros2_livox/csv_reader.hpp` 与 `src/pb_rm_simulation/world/{old,obstacles}` 等历史文件未删除，保留以供追溯。

## Auditor Review

### Checks performed

- [x] 关键路径检查（`rm_simulation.launch.py`、`gz_bridge.yaml`、`simulation_waking_robot.xacro`、`mid360.xacro`、`mid360_gz_bridge.cpp`、`omni_chassis_system.cpp`、`dynamic_obstacle_system.cpp`）
- [x] diff 检查（逐阶段审查 `git diff` 与 `git status`）
- [x] grep 检查（确认有效源码与世界文件中无残留 `gazebo_ros` / `libgazebo` / `RayPlugin` / `libobstacle` 引用）
- [x] XML / launch / yaml 检查（`xml.etree.ElementTree` 与 `yaml.safe_load` 全量解析通过）
- [x] 用户允许范围内的测试与静态检查
- [x] 构建已取得用户明确许可

### Build & Test Evidence
- **构建命令**（已获用户明确授权）：
  - `source /opt/ros/humble/setup.zsh && colcon build --packages-select ros2_livox_simulation pb_rm_simulation --cmake-args -DCMAKE_BUILD_TYPE=Release` -> **通过**（零编译错误、零告警）。
- **单元测试**：
  - `colcon test --packages-select ros2_livox_simulation pb_rm_simulation` -> **通过**（`test_mid360_resampler`: 3/3 passed, 0 failures）。
- **无头端到端 GPU 仿真验证**（`python3 src/pb_rm_simulation/scripts/verify_harmonic_sim.py --world ALL` 及静态 `RMUL`/`RMUL2026H` 补充测试）：
  - `RMUC2027 (noise=0.0)`：`RTF=1.010`，`lidar=10.00 Hz`，`imu=203.3 Hz`，`odom=50.6 Hz`，`joints=50.6 Hz`，3s 静止漂移 `1.04 mm`，真值里程计 vs Gazebo 位姿误差 `0.01 mm`，STL 平滑内表面测距中位绝对误差 `1.22 mm`（`bias=+0.19 mm`，`std=3.80 mm`），自遮挡点数 `0`。
  - `RMUC2027 (noise=0.02 + 运动与去畸变)`：`RTF=0.996`，`lidar=10.00 Hz`，`imu=200.0 Hz`，`odom=50.0 Hz`，注入噪声实测 `std=19.42 mm`（`bias=-1.09 mm`），5 子帧旋转去畸变对齐 `best=+3.5°`（IMU 期望 `+3.4°`，3D 最近邻误差 `84.3 mm -> 50.8 mm`），全向运动 `dx_body=+0.499 m`、`dy_body=+0.573 m`。
  - `RMUC (noise=0.02)`：`RTF=1.000`，`valid=20000/20000`，`static_drift_3s=0.00 mm`，`vs_gz_pose=0.01 mm`。
  - `RMUL (static & dynamic_obstacles=true)`：`RTF=1.013 / 1.017`，动态障碍物 3s 位移 `0.599 m` / `0.346 m`，GPU LiDAR 检测到 `1071` 个 `<laser_retro=120>` 动态障碍物命中点。
  - `RMUL2026H (static & dynamic_obstacles=true)`：`RTF=1.011 / 1.017`，动态障碍物 3s 位移 `0.595 m` / `0.345 m`，GPU LiDAR 检测到 `35` 个 `<laser_retro=120>` 动态障碍物命中点。
  - 所有测试场景均确认：无 `world/odom -> base_link` `/tf` 冲突，Gazebo/ROS 启动日志零 `[Err]`/`FATAL`/`ERROR`。

### Issues found
无阻塞问题。剩余非阻塞说明：
1. 物体轮廓/台阶边缘（占少量射线）存在约 `0.4°` GPU 深度图离散化带来的边缘跳变误差，属于 GPU 球面深度重采样的固有物理边界（平滑表面误差仅 `1.22 mm`）。
2. 当前所有本地提交尚未推送至远端（按规则禁止代理执行 `git push`）。

### Final result

PASS
