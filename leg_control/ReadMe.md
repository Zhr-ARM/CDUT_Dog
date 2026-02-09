# Leg Control Project

本项目包含单腿机器人的仿真模型、控制器及硬件驱动。基于 ROS 2 和 Gazebo 构建。

## 目录结构

*   **`leg_model`**: 包含机器人的 URDF 描述文件、Mesh 网格文件、RViz 配置及 Launch 启动文件。
*   **`leg_mit_controller`**: 自定义 `ros2_control` 控制器，实现了 MIT 运控模式（位置+速度+前馈力矩+Kp+Kd）。
*   **`deep_motor_ros`**: 宇树/DeepRobotics J60 电机驱动节点，用于通过 CAN 总线控制真实硬件。

---

## 1. 快速开始 (仿真)

### 1.1 编译工作空间

在工作空间根目录下执行：

```bash
cd ~/leg_control
colcon build --symlink-install
source install/setup.bash
```

### 1.2 启动 Gazebo 仿真

启动 Gazebo 环境，加载机器人模型并自动启动 `mit_controller` 控制器：

```bash
ros2 launch leg_model gazebo.launch.py
```

此时你应该能看到 Gazebo 窗口，并且机器人在其中保持站立或受控状态（取决于此时是否有控制指令输入）。

### 1.3 启动 RViz 可视化 (可选)

仅查看模型结构或关节状态：

```bash
ros2 launch leg_model display.launch.py
```

---

## 2. 配置说明 (重要)

由于 URDF 文件中加载 Mesh 文件的路径可能包含绝对路径，如果你更换了用户或移动了项目位置，需要修改 URDF 文件。

**文件路径**: `src/leg_model/urdf/leg_model.urdf`

请检查文件中的 `filename` 属性，确保路径指向正确的网格文件位置。例如：

```xml
<mesh filename="file:///home/YOUR_USERNAME/leg_control/install/leg_model/share/leg_model/meshes/base_link.STL" />
```

将 `YOUR_USERNAME` 替换为你的实际用户名 (当前为 `llq`)。

---

## 3. 控制接口说明

仿真环境使用了 `leg_mit_controller`，它订阅并发布以下话题：

### 3.1 控制指令 (`/motor_cmd`)

*   **话题类型**: `std_msgs/Float64MultiArray` (根据具体实现调整，此处基于 MIT Controller 代码推断，通常为自定义消息或数组)
*   **控制模式**: MIT 模式 ($\tau = K_p(p_{des} - p_{cur}) + K_d(v_{des} - v_{cur}) + \tau_{ff}$)

### 3.2 状态反馈 (`/motor_feedback`)

*   **话题类型**: 包含关节的位置、速度、力矩信息。

---

## 4. 真实硬件驱动 (`deep_motor_ros`)

如果要连接真实电机：
1. 确保 CAN 硬件已连接并配置为 `can0` (默认)。
2. 修改 `src/driver/deep_motor_ros/src/motor_node.cpp` 中的电机 ID 配置（如需）。
3. 编译并运行驱动节点。

2. **启动 Gazebo 仿真**
   ```bash
   source install/setup.bash
   ros2 launch leg_model_1 gazebo.launch.py
   ```

3. **发送控制命令**
   仿真启动后，通过以下命令使电机运动：
   ```bash
   ros2 topic pub -r 10 /motor_cmd std_msgs/msg/Float64MultiArray "{data: [3.210, 0.0, 0.0, 5.0, 0.5, 0.0, 0.0, 0.0, 19.0, 1.0, 0.88, 0.0, 0.0, 20.0, 1.0]}"
   ```

---

## 5. 控制器使用说明

### 5.1 控制模式介绍
本控制器支持两种模式，可在 `src/leg_model_1/config/ros2_controllers.yaml` 中修改 `mode` 参数：

1. **sim_effort_pd (默认)**
   - **适用环境**：Gazebo 仿真。
   - **原理**：控制器内部计算力矩公式 $\tau = K_p(p_{des} - p_{cur}) + K_d(v_{des} - v_{cur}) + \tau_{ff}$，仅向 Gazebo 发送力矩指令。

2. **passthrough (透传模式)**
   - **适用环境**：真实机器人（需配合 DeepRobotics J60 等智能驱动器）。
   - **原理**：控制器不进行计算，直接将所有指令参数（位置、速度、前馈力矩、Kp、Kd）转发给底层电机驱动器，由硬件执行高频闭环控制。

### 5.2 发送控制指令
启动仿真后，可以通过话题 `/motor_cmd` 发送控制指令。
协议格式为扁平的一维数组，每 5 个数为一组对应一个关节：
`[Pos, Vel, Torque, Kp, Kd, ...]`

**示例命令（3个关节）：**
```bash
ros2 topic pub -r 10 /motor_cmd std_msgs/msg/Float64MultiArray "{data: [1.57, 0.0, 0.0, 5.0, 0.5, 3.014, 0.0, 0.0, 50.0, 10, 0.0, 0.0, 0.0, 20.0, 1.0]}"
```
*   **每五个数据对应**： 位置、速度、前馈力矩、Kp、Kd
*   **第1组 (Hip)**: Pos=1.57, Kp=5.0, Kd=0.5
*   **第2组 (big_leg)**: Pos=3.014, Kp=50.0, Kd=10.0
*   **第3组 (thigh(带动小腿))**: Pos=0.0, Kp=20.0, Kd=1.0

> **注意**：控制器设有看门狗保护。如果超过 200ms 未收到指令，控制器会自动进入安全保护模式（输出归零）；一旦收到新指令，将立即自动恢复控制。

---

## 6. 常见问题（排查思路）
- **找不到 STL / mesh 显示为空**
  - 多数是 URDF 里的 `username` 未替换或路径不正确（请确认 12 处都已修改）。
- **launch 找不到包 / 命令无效**
  - 确认已执行：`source install/setup.bash`
  - 确认当前终端位于同一个工作空间（`~/leg_control`）下构建过。