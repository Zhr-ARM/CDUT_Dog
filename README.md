# arm_bringup ROS 2 工作空间

该工作空间目前包含了机械臂项目的所有核心功能包，支持从正逆运动学控制、轨迹规划（如箱子搬运）到仿真与真实硬件后端的电机控制。

## 功能特性

- **逆运动学解算 (IK)**：支持4自由度机械臂（`base_yaw`, `shoulder_pitch`, `elbow_pitch`, `wrist_pitch`）的逆运动学求解，可精准运行到目标空间点。
- **环境兼容**：无缝切换 Gazebo 仿真 (`sim_motor_controller`) 与真实硬件 (`real_motor_controller`) 后端。
- **任务轨迹规划**：内置吸盘取箱子等复杂装配动作的时间序列规划。
- **底层电机透传**：支持发送包含 `[p, v, kp, kd, t_ff]` 的 MIT 控制命令以直接驱动达妙 (DM) 等电机。

## 包含的软件包

- `arm_bringup`: 包含启动文件 (launch files)、URDF 描述、三维网格模型、Gazebo 模型文件、RViz 配置文件及其相关基础配置。
- `arm_controller`: 核心控制节点，负责逆向运动学解算、接收目标坐标并发布 MIT 电机指令、箱子搬运动作生成。
- `sim_motor_controller`: 针对 Gazebo 仿真环境的电机控制后端适配层。
- `real_motor_controller`: 针对真实物理硬件的串口总线控制后端。

## 编译方法

```bash
cd /home/zhr/robot_arm
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 运行指令

在 RViz 中预览机器人模型：
```bash
ros2 launch arm_bringup display.launch.py
```

启动 Gazebo 仿真与 RViz：
```bash
ros2 launch arm_bringup sim.launch.py
```

启动电机控制后端（二选一）：
```bash
# 启动仿真控制后端
ros2 launch arm_bringup control.launch.py backend:=sim

# 启动真实硬件控制后端
ros2 launch arm_bringup control.launch.py backend:=real serial_port:=/dev/ttyACM0
```

## 核心话题交互 (Topics Interface)

- **目标点控制** (供 `arm_controller` 订阅)：
  发布目标坐标点使机械臂解算逆运动学并移动：
  ```bash
  ros2 topic pub /arm_target_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 0.3, y: 0.0, z: 0.1}}" -1
  ```
- **启动内置搬箱子序列**：
  ```bash
  ros2 topic pub /arm_box_motion/start std_msgs/msg/Empty {} -1
  ```
- **吸盘控制**：
  ```bash
  ros2 topic pub /suction/enable std_msgs/msg/Bool "{data: true}" -1
  ```
- **底层电机指令发布** (通常由 `arm_controller` 自动对它进行发布)：
  话题：`/dm_j4340/mit_commands` (类型：`std_msgs/msg/Float32MultiArray`)  
  数据格式：`[p, v, kp, kd, t_ff] * 4`（依次对应4个关节的数据）。

## 重要参数配置 (Parameters)

运行节点前，可通过 ROS 2 参数 (比如在 launch 配置的 YAML 文件中) 微调 `arm_controller_node` 的底层执行逻辑。常用的高频参数说明如下：

### 1. 机械臂运动学与物理参数
| 参数名 | 默认值示例 | 说明 |
|---|---|---|
| `joint_origins` | `[0.0, ..., 0.057]` (包含12个浮点数) | 4个关节相对前置关节坐标系的原点平移向量 `[x,y,z]` |
| `joint_axes` | `[0.0, 0.0, 1.0, ...]` | 各关节旋转轴的单位向量 |
| `joint_lower_limits` | `[-3.14, ...]` | 关节活动受限下限范围 (rad) |
| `joint_upper_limits` | `[3.14, ...]` | 关节活动受限上限范围 (rad) |
| `tool_offset` | `[0.16, -0.08, -0.016]` | 末端执行器（例如吸盘）相对于最后一个法兰的坐标偏移 |

### 2. PID 与速度控制参数
| 参数名 | 说明 |
|---|---|
| `kp` / `kd` | 运动轨迹跟随期间，下发给电机的刚度 (Kp) 和阻尼 (Kd) 数组设置。 |
| `hold_kp` / `hold_kd` | 到达目标静止悬停时的位置刚度和阻尼数组，通常用于降低过冲和静差震荡。 |
| `command_rate_hz` | 控制指令发布频率，默认推荐 `50.0` Hz。 |
| `max_joint_speed_rad_s` | 关节最大运行角速度 (rad/s)，控制插补时的平滑程度。 |
| `position_tolerance_m` | 逆运动学算法的位置收敛容差。 |

### 3. 箱子搬运任务专属参数
用于微调通过 `/arm_box_motion/start` 触发的轨迹系列点。
| 参数名 | 说明 |
|---|---|
| `platform_height_m` | 操作平台高度 (m) |
| `box_width_m`, `box_depth_m`, `box_height_m` | 箱子的三维物理边界尺寸 (m) |
| `box_suction_target_x_m` | 吸取目标在箱体表面的X轴坐标落点 (m) |
| `box_approach_offset_m` | 机械臂接触箱体前的接近预备距离 (m) |
| `box_lift_height_m` | 吸附后向上提升箱体离开平面的高度 (m) |
