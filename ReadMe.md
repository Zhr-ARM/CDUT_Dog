# Leg Control Project

单腿机器人仿真模型、控制器与硬件驱动，基于 ROS 2 + Gazebo。

## 项目组成

- `leg_model`：URDF、Mesh、Gazebo/RViz 启动与控制器配置。
- `leg_mit_controller`：MIT 5 参数控制器（p,v,t_ff,kp,kd）。
- `position_control`：步态/位控节点（发布 `/motor_cmd`）。
- `deep_motor_ros`：DeepRobotics J60 电机驱动（SocketCAN）。

---

## 环境与依赖

- 推荐：Ubuntu 22.04 + ROS 2 Humble。
- 依赖包见各包 `package.xml`，Gazebo 需要 `gazebo_ros` 与 `gazebo_ros2_control`。

---

## 编译

```bash
cd ~/leg_control
colcon build --symlink-install
source install/setup.bash
```

仅编译某个包：

```bash
colcon build --packages-select leg_model
```

---

## 仿真使用（Gazebo）

### 1) 启动模型与控制器

```bash
ros2 launch leg_model gazebo.launch.py
```

`gazebo.launch.py` 会自动：
- 启动 `gzserver/gzclient`。
- 载入 URDF，并启动 `controller_manager`。
- 自动加载 `joint_state_broadcaster` 与 `mit_controller`。

可选参数：
- `gui`：是否启动 Gazebo GUI（默认 `true`）。

```bash
ros2 launch leg_model gazebo.launch.py gui:=false
```

### 2) 启动步态控制器（可选）

```bash
ros2 launch position_control gait_sim.launch.py
```

可选参数：
- `param_file`：步态参数文件路径。

```bash
ros2 launch position_control gait_sim.launch.py \
  param_file:=/home/llq/leg_control/src/APP/src/position_control/config/gait_controller.yaml
```

### 3) RViz 可视化（可选）

```bash
ros2 launch leg_model display.launch.py
```

可选参数：
- `rviz_config_file`、`use_rviz`、`use_robot_state_pub`、`use_joint_state_pub`、`urdf_file`。

---

## 话题接口与协议

### `/motor_cmd`（控制指令）

- 类型：`std_msgs/msg/Float64MultiArray`
- 协议：每个关节 5 个数，按关节顺序平铺。

```
[p, v, t_ff, kp, kd,  p, v, t_ff, kp, kd, ...]
```

当前仿真关节顺序（来自 `leg_model/config/ros2_controllers.yaml`）：

```
hip_joint, thigh_joint, knee_crank_joint
```

示例（3 关节）：

```bash
ros2 topic pub -r 10 /motor_cmd std_msgs/msg/Float64MultiArray \
"{data: [3.21, 0.0, 0.0, 5.0, 0.5,  0.0, 0.0, 0.0, 12.0, 0.6,  0.0, 0.0, 0.0, 10.0, 0.5]}"
```

### `/motor_feedback`（反馈）

- 类型：`std_msgs/msg/Float64MultiArray`
- 结构：每关节 5 个数（位置/速度/力矩/预留/预留）。

---

## MIT 控制器（`leg_mit_controller`）

控制器参数位于 `leg_model/config/ros2_controllers.yaml`：

- `joints`：关节名称列表（必须非空）。
- `mode`：`sim_effort_pd` 或 `passthrough`。
- `command_topic`：命令话题名（默认 `/motor_cmd`）。
- `feedback_topic`：反馈话题名（默认 `/motor_feedback`）。
- `publish_feedback`：是否发布反馈。
- `command_timeout_ms`：超时保护（ms），超时输出归零。
- `feedback_rate_hz`：反馈发布频率。

模式说明：
- `sim_effort_pd`：控制器内部计算 $\tau = K_p(p_{des}-p_{cur}) + K_d(v_{des}-v_{cur}) + t_{ff}$，输出 `effort`。
- `passthrough`：直接把 5 个参数写入硬件接口（需要硬件接口支持 `position/velocity/effort/kp/kd`）。

---

## 步态控制器（`position_control/gait_controller`）

参数文件：`src/APP/src/position_control/config/gait_controller.yaml`

- `use_sim_time`：是否用仿真时间。
- `step_length`：步长 (m)。
- `step_height`：抬腿高度 (m)。
- `gait_period`：步态周期 (s)。
- `stance_depth`：站立时足端深度 (m)。
- `control_rate`：控制频率 (Hz)。
- `swing_ratio`：摆动相比例 [0,1]。
- `hip_angle`：hip 固定角 (rad)。
- `hip_kp/hip_kd`、`thigh_kp/thigh_kd`、`knee_crank_kp/knee_crank_kd`：PD 增益。
- `filter_alpha`：低通滤波系数 (0,1]。
- `auto_zero_offsets`：是否自动对齐零位。
- `thigh_offset`、`knee_offset`：手动零位偏置 (rad)。

说明：
- 该节点向 `/motor_cmd` 发布 15 个元素的数组，对应 3 个关节。
- 启用 `auto_zero_offsets` 时会根据当前步态起点自动计算零位偏置。

---

## 真实硬件驱动（`deep_motor_ros`）

启动：

```bash
source install/setup.bash
ros2 run deep_motor_ros motor_node
```

运行时参数：
- `motor_ids`：电机 ID 数组（默认 `[1, 2, 3]`）。
- `can_interface`：CAN 口名（默认 `can0`）。

示例：

```bash
ros2 run deep_motor_ros motor_node --ros-args \
  -p motor_ids:="[11, 13, 15]" -p can_interface:=can0
```

**重要注意：**
- `motor_node.cpp` 中包含自动配置 CAN 的 `sudo` 命令，请将 `sudo_password` 改为当前机器真实密码。
- 若密码错误，CAN 口无法自动拉起。

---

## 模型与 URDF 注意事项

- `leg_model/urdf/leg_model.urdf` 中 Mesh 使用绝对路径，例如：

```xml
<mesh filename="file:///home/llq/leg_control/install/leg_model/share/leg_model/meshes/base_link.STL" />
```

如更换用户或移动目录，需更新这些路径。

- `mimic` 关节在 Gazebo 中不会自动驱动，已通过 `mimic_joint_plugin` 处理。若膝关节不随动：

```bash
colcon build --packages-select leg_model
source install/setup.bash
```

---

## 常见问题

- **找不到 mesh 或显示为空**：检查 URDF 中 `file://` 绝对路径是否正确。
- **控制无响应**：确认已 `source install/setup.bash` 且控制器已加载。
- **/motor_cmd 被丢弃**：数组长度必须严格等于关节数 × 5。
- **仿真关节抖动**：尝试降低 `Kp`、提高 `Kd`、调小 `filter_alpha`。

---

## 快速验证命令

```bash
ros2 topic echo /motor_feedback
```

```bash
ros2 topic pub /motor_cmd std_msgs/msg/Float64MultiArray \
"{data: [3.21, 0.0, 0.0, 3.0, 0.1,  0.0, 0.0, 0.0, 8.0, 0.5,  0.0, 0.0, 0.0, 8.0, 0.5]}" --once
```