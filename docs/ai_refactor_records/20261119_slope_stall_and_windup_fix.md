# 斜坡卡死与底盘速度控制积分饱和修复 改造记录

## User Intent

修复在 Gazebo Harmonic 仿真中向 `/cmd_vel_chassis` 发布速度指令后，机器人在小斜坡上卡死不动的问题（场景 A），保持已有 ROS 2 话题、坐标系、传感器链路及底盘力控 IMU 保真特性不变。

## Scope

- `src/pb_rm_simulation/src/omni_chassis_system.cpp`：底盘速度环抗饱和、反向/零轴积分释放、斜坡重力切向前馈、坡面接地判定。
- `src/pb_rm_simulation/urdf/simulation_waking_robot.xacro`：四轮碰撞体改用同轴球体（保留圆柱外观）并为 `base_link` 增加低摩擦配置。
- 7 个现役 Harmonic `.world` 文件（`RMUC2027`、`RMUC2024`、`RMUL2024`、`RMUL2024_dynamic`、`RMUL2026`、`RMUL2026_dynamic`、`RMUL2026H`）：将 DART 碰撞检测器由默认 `ode` 显式切换为 `bullet`。

## Out of Scope

- 不修改任何 STL 场地网格模型。
- 不修改 `world/old/` 历史备份文件。
- 不修改 ROS 2 topic、frame_id、QoS、launch 参数或 `OmniChassisSystem` SDF 默认参数值。
- 不修改 LiDAR (`mid360_gz_bridge`) 与 IMU 配置。
- 未获用户明确许可前不执行 `colcon build` 或任何 Git 提交操作。

## Explorer Findings

### Files inspected

- `src/pb_rm_simulation/src/omni_chassis_system.cpp`
- `src/pb_rm_simulation/urdf/simulation_waking_robot.xacro`
- `src/pb_rm_simulation/world/RMUC2027_world/RMUC2027_world.world`
- `src/pb_rm_simulation/world/RMUC2024_world/RMUC2024_world.world`
- `src/pb_rm_simulation/world/RMUL2024_world/RMUL2024_world.world`
- `src/pb_rm_simulation/world/RMUL2024_world/RMUL2024_world_dynamic_obstacles.world`
- `src/pb_rm_simulation/world/RMUL2026_world/RMUL2026_world.world`
- `src/pb_rm_simulation/world/RMUL2026_world/RMUL2026_world_dynamic_obstacles.world`
- `src/pb_rm_simulation/world/RMUL2026H_world/RMUL2026H_world.world`
- `src/pb_rm_simulation/meshes/RMUC2027_world/meshes/rmuc_2027.stl`
- `src/pb_rm_simulation/launch/rm_simulation.launch.py`

### Active logic path

1. ROS 2 `/cmd_vel_chassis` (`geometry_msgs/msg/Twist`) 经 `ros_gz_bridge` 转发到 Gazebo 话题 `/cmd_vel_chassis` (`gz.msgs.Twist`)。
2. `pb_rm_simulation::OmniChassisSystem::PreUpdate` 在每个 1 ms 物理步内，根据 `base_link` 姿态与速度计算底盘切向驱动力 `(fx, fy)` 及偏航力矩 `tau_z`，通过 `AddWorldWrench` 驱动底盘。
3. 场地碰撞体由 7 个 `.world` 文件中的 STL 三角网格提供，默认 `<physics name="1ms" type="ignored">` 由 DART 加载默认 `ode` 碰撞检测器求解轮地接触。

### Data flow

`/cmd_vel_chassis` (ROS 2) → `ros_gz_bridge` → `/cmd_vel_chassis` (gz-transport) → `OmniChassisSystem` (`AddWorldWrench`) → DART 物理引擎（轮/底盘碰撞体 vs STL 场地网格） → `/sim/ground_truth/odom` & `/livox/imu`。

### Risk notes

在无修改实测中复现并定位到 4 个导致斜坡卡死/脱困失败的根因：

1. **DART 默认 `ode` 碰撞检测器的三角网格接缝法向伪影（主因）**：在 `RMUC2027` 12° 南坡坡底 `y=2.655` 处，坡面 (`z=0.375`) 与平地 (`z=0.378`) 存在 2.9 mm 接缝与竖直裙边 `tri 128265` (`n=(0,+1.00,+0.03)`)。接触传感器实测显示：在 `DART + ode` 下，即使球形车轮滚过该接缝也会拿到纯水平挡墙法向 `n=(0,+0.9997,+0.026)`，单轮产生 `+76.38 N` 水平阻挡力（双前轮 `152.7 N > max_force_=150 N`），直接锁死为 `v=0.000`。切换为 `DART + bullet` 后接触法向正常，畅通通过。
2. **轮子圆柱碰撞体侧面卡缝 & `base_link` 默认高摩擦**：即使切换为 `DART + bullet`，圆柱车轮横移跨接缝仍会在 `y=2.79` 卡死（`d=0.31 m`）；改用同轴球体碰撞体（`wheel_1/2` 沿轮轴内移 `-0.025 m`，`wheel_3/4` 内移 `+0.025 m`，接地高度维持 `-0.06 m`，外廓宽度 `0.33 m` 接近原圆柱 `0.31 m`）后横移/正向跨坡均顺利通过。`base_link` 缺少 `<mu1>/<mu2>` 配置（默认 `1.0`），补齐 `0.02` 防止坡顶/边沿剐蹭锁死。
3. **`OmniChassisSystem` 速度环 PI 积分严重饱和**：`vel_int_x_`/`vel_int_y_`/`vel_int_w_` 上限为 `±2.0`（对应 `±510 N`，为 `max_force_=150 N` 的 3.4 倍），且无抗饱和与反向清零；一旦顶墙/卡缝 1~2 s，反向指令 5 s 内仅移动 `-0.003 m`，垂直方向指令时旧轴积分不衰减并长期抢占合力圆。
4. **陡坡缺少重力切向前馈与坡面接地判定偏差**：在 20° 坡上 `g*sin(20°)=3.36 m/s²`，低速起步（如 `0.15 m/s`）切换出 `pos_hold` 时丢弃积分导致先下滑再爬升；停车制动阶段仅靠 `-kp*v` 与重力在 `v=3.36/22=0.153 m/s > 0.08 m/s` 处平衡，无法触发 `pos_hold_active_`；且原 `grounded_hint` 用世界 `|v_world.Z()| < 0.15`，在斜坡行驶时易误判。

### Recommended modification boundary

仅修改 `omni_chassis_system.cpp`、`simulation_waking_robot.xacro` 和 7 个现役 `.world` 的 `<physics>` 配置块，不改任何 topic、frame、launch、默认控制器增益及 STL 网格。

## Modifier Changes

### Files changed

- `src/pb_rm_simulation/src/omni_chassis_system.cpp`
- `src/pb_rm_simulation/urdf/simulation_waking_robot.xacro`
- `src/pb_rm_simulation/world/RMUC2027_world/RMUC2027_world.world`
- `src/pb_rm_simulation/world/RMUC2024_world/RMUC2024_world.world`
- `src/pb_rm_simulation/world/RMUL2024_world/RMUL2024_world.world`
- `src/pb_rm_simulation/world/RMUL2024_world/RMUL2024_world_dynamic_obstacles.world`
- `src/pb_rm_simulation/world/RMUL2026_world/RMUL2026_world.world`
- `src/pb_rm_simulation/world/RMUL2026_world/RMUL2026_world_dynamic_obstacles.world`
- `src/pb_rm_simulation/world/RMUL2026H_world/RMUL2026H_world.world`

### Key changes

1. `omni_chassis_system.cpp`：
   - 翻车保护分支（`z_b.Z() < 0.5`）额外清空 `vel_int_*` 与 `pos_hold_active_`/`yaw_hold_active_` 状态。
   - `grounded_hint` 改为底盘法向速度 `std::abs(v_world.Dot(z_b)) < 0.25`，在斜坡行驶时保持有效。
   - 增加底盘切向重力前馈 `g_ff_x = 9.81 * x_b.Z()`、`g_ff_y = 9.81 * y_b.Z()`，作用于行进、制动与静态驻坡三个分支（平地为 0）。
   - 为线速度与角速度 PI 积分增加条件积分抗饱和（合力圆/力矩饱和时不继续同向积分）、合理积分上限（`min(0.5, 0.5*max_force/(m*ki))` 与对应角速度上限）、反向指令立即清零，以及零指令轴 `1 - 5*dt` 衰减。
2. `simulation_waking_robot.xacro`：
   - `wheel_1..4` 的 `<collision>` 由 `<cylinder>` 改为 `<sphere radius="${radius_wheel}"/>`，并沿各自轮轴内移 `±0.025 m` 保持轮地接触高度 `-0.06 m` 且维持车身外廓宽度（保留圆柱 `<visual>` 不变）。
   - 新增 `<gazebo reference="base_link"><mu1>0.02</mu1><mu2>0.02</mu2></gazebo>`。
3. 7 个现役 `.world` 文件：
   - `<physics name="1ms" type="ignored">` 改为 `<physics name="1ms" type="dart">` 并显式配置 `<dart><collision_detector>bullet</collision_detector></dart>`。

### Behavior preserved

- 保留全部 ROS 2 话题（`/cmd_vel_chassis`、`/sim/ground_truth/odom`、`/livox/imu`、`/livox/lidar`、`/joint_states`）、frame_id、QoS 及 `OmniChassisSystem` SDF 参数名与默认值。
- 保留底盘纯力控（`AddWorldWrench`）与 `wheel_1..4` 圆柱外观，不破坏 IMU 动力学保真与平地全向跟踪特性。
- 保留 `world/old/` 历史文件不动。

### Behavior intentionally adjusted

- 跨 STL 网格接缝与斜坡时消除 ODE 鬼影水平法向与圆柱侧边卡缝。
- 顶墙或短时阻滞后反向/侧向脱困不再受大额积分饱和拖累。
- 陡坡低速起步与坡面驻车不再下滑或无法锁入静态驻坡。

### Notes

- 已在未编译 C++ 插件的前提下，使用当前编辑后的 `RMUC2027_world.world` 与 `xacro` 展开 URDF 验证了 12° 南坡 `y=2.655` 接缝低速（`-0.25 m/s`）横移通过（`y: 3.25 -> 2.44`，`x` 偏移 `0.00 m`），并在此前对比测试中验证了 `RMUC2024`、`RMUL2024_dyn`、`RMUL2026_dyn` 与 `RMUC2027` 在 `DART + bullet` 下 `RTF ≈ 1.000`。
- `omni_chassis_system.cpp` 的实际编译产物 `libpb_rm_omni_chassis_system.so` 尚未重建，需待用户明确许可后执行 `colcon build` 与完整回归测试。

## Auditor Review

### Checks performed

- [x] 关键路径检查（`OmniChassisSystem::PreUpdate`、URDF 固定关节合并与轮轴方向、`world` 物理插件配置）
- [x] diff 检查（无越界修改、无新增成员变量/SDF 参数、无未授权文件变动）
- [x] grep 检查（确认全部 7 个现役 `.world` 均已同步，未触动 `world/old/`）
- [x] XML / launch / yaml 检查（8 个 XML/xacro/world 文件 `ElementTree` 解析全部通过，7 个 `.world` `gz sdf -k` 全部 `Valid.`，`xacro` + `gz sdf -p` 在 `use_livox:=true/false` 下均验证通过）
- [x] 用户允许范围内的测试或静态检查（编辑后 URDF + `RMUC2027_world.world` 12° 接缝横移实测通过）
- [ ] 如需构建，已取得用户明确许可（待用户确认后执行 `colcon build` 及完整回归）

### Issues found

无静态/逻辑缺陷；`omni_chassis_system.cpp` 改动需编译后方可在仿真运行时生效并完成最终验证。

### Final result

PASS（静态检查与 URDF/world 实测通过；C++ 构建与完整回归待用户授权）
