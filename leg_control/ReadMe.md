# 使用方法（ROS 2 + Gazebo）

## 1. 目录要求（重要）
- **leg_control 必须位于你的 Home 目录下**：`~/leg_control`
- 先确认目录位置正确：

```bash
cd ~
ls
```

看到 `leg_control` 目录即为正确。

---

## 2. 配置 URDF 中的 mesh 绝对路径
1. 打开 URDF 文件：
   - 路径：`src/leg_model1/urdf/leg_model_1.urdf`

2. 在该 URDF 内找到所有类似下面的引用（**共 12 个**）：
   - `file:///home/username/leg_control/install/leg_model_1/share/leg_model_1/meshes/Knee_link.STL`

3. 将其中的 `username` 替换为你自己的用户名（例如 `zhr`）：
   - 示例：`file:///home/zhr/leg_control/install/leg_model_1/share/leg_model_1/meshes/Knee_link.STL`

> 提示：如果你不确定用户名，可用 `whoami` 查看。

---

## 3. 编译与环境加载
在工作空间根目录执行：

```bash
cd ~/leg_control
colcon build
source install/setup.bash
```

---

## 4. 启动仿真与控制
> 建议开启三个终端，分别执行以下命令。

1. **启动显示（RViz）**
   ```bash
   cd ~/leg_control
   source install/setup.bash
   ros2 launch leg_model_1 display.launch.py
   ```

2. **启动 Gazebo 仿真**
   ```bash
   source install/setup.bash
   ros2 launch leg_model_1 gazebo.launch.py
   ```

3. **发送控制命令**
   仿真启动后，通过以下命令使电机运动：
   ```bash
   ros2 topic pub -r 10 /motor_cmd std_msgs/msg/Float64MultiArray "{data: [1.57, 0.0, 0.0, 5.0, 0.5, 3.014, 0.0, 0.0, 50.0, 10, 0.0, 0.0, 0.0, 20.0, 1.0]}"
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