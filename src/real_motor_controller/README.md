# real_motor_controller

这是一个 ROS 2 Humble 功能包，用于通过达妙 USB-CAN 串口适配器控制 4 个
DM-J4340-2EC 电机。

电机到 URDF 关节的映射固定为：

- 1: `base_yaw_joint`
- 2: `shoulder_pitch_joint`
- 3: `elbow_pitch_joint`
- 4: `wrist_pitch_joint`

当前控制流程很简单：

1. 先扫描配置好的电机 ID。
2. 再启动电机并切换到 MIT 控制模式。
3. 向 `/dm_j4340/mit_commands` 发布 MIT 控制指令。
4. 通过 `/dm_j4340/status` 和 `/dm_j4340/joint_states` 查看状态。

## 功能

- 按 CAN ID 查询电机是否在线。
- 启动时将全部电机配置为 MIT 控制模式。
- 支持清故障、清零、使能、失能全部电机。
- 订阅 4 个电机的 MIT 控制话题。
- 如果没有收到新指令，或者指令超时，自动回到零 MIT 指令。
- 发布关节反馈和简洁的文本状态。

## 编译

如果还没有安装串口依赖，先安装：

```bash
sudo apt install python3-serial
```

在仓库根目录编译：

```bash
cd /home/zhr/robot_arm
colcon build --packages-select real_motor_controller
source install/setup.bash
```

修改 Python、launch、yaml 或 package 文件后，需要重新编译并 source：

```bash
colcon build --packages-select real_motor_controller
source install/setup.bash
```

## 启动顺序

建议分两步启动。不要同时运行查询节点和启动节点，因为它们都会打开同一个串口。

### 1. 查询电机

```bash
ros2 launch real_motor_controller real_query_motor.launch.py
```

如果串口不是默认的 `/dev/ttyACM0`，可以覆盖参数：

```bash
ros2 launch real_motor_controller real_query_motor.launch.py serial_port:=/dev/ttyACM0
```

查询节点会向配置的 CAN ID 发送失能探测帧，并等待电机反馈。它不会使能电机，也不会清零。

### 2. 启动 MIT 控制

```bash
ros2 launch real_motor_controller real_motor_controller.launch.py
```

启动节点会按下面顺序执行：

1. 打开 USB-CAN 串口。
2. 按配置设置 CAN 波特率。
3. 先失能全部电机。
4. 写入 `CTRL_MODE=MIT`。
5. 清除故障。
6. 设置当前位置为零点。
7. 等待全部配置电机都有反馈。
8. 使能全部电机。
9. 按 `command_rate_hz` 周期发送 MIT 指令。

## MIT 控制话题

话题名：

```text
/dm_j4340/mit_commands
```

消息类型：

```text
std_msgs/msg/Float32MultiArray
```

数据格式：

```text
[p, v, kp, kd, t_ff] * motor_count
```

默认 `motor_ids: [1, 2, 3, 4]`，所以需要发布 20 个 float：

```text
[
  motor1_p, motor1_v, motor1_kp, motor1_kd, motor1_t_ff,
  motor2_p, motor2_v, motor2_kp, motor2_kd, motor2_t_ff,
  motor3_p, motor3_v, motor3_kp, motor3_kd, motor3_t_ff,
  motor4_p, motor4_v, motor4_kp, motor4_kd, motor4_t_ff
]
```

单位：

- `p`：位置，单位 rad。
- `v`：速度，单位 rad/s。
- `kp`：MIT 位置增益。
- `kd`：MIT 速度阻尼增益。
- `t_ff`：前馈力矩，单位 Nm。

### 发布零指令

```bash
ros2 topic pub -r 100 /dm_j4340/mit_commands std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 让 3 号电机转到 30 度

30 度约等于 `0.5236` rad。下面的例子只控制 3 号电机，其它电机保持零 MIT 指令：

```bash
ros2 topic pub -r 100 /dm_j4340/mit_commands std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5236, 0.0, 30.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

第 3 组数据是 3 号电机的 MIT 指令：

```text
p = 0.5236
v = 0.0
kp = 30.0
kd = 0.6
t_ff = 0.0
```

如果方向反了，把 `0.5236` 改成 `-0.5236`。

## 指令超时

`mit_command_timeout_s` 表示一条 MIT 指令的有效时间，默认是 `0.2` 秒。

如果超过这个时间没有收到新指令，节点会自动发送零 MIT 指令：

```text
p = 0, v = 0, kp = 0, kd = 0, t_ff = 0
```

做连续控制时，发布频率要高于超时时间。建议先用：

```bash
-r 100
```

也就是 100 Hz。

## 状态和服务

查看关节反馈：

```bash
ros2 topic echo /dm_j4340/joint_states
```

查看文本状态：

```bash
ros2 topic echo /dm_j4340/status
```

常见状态字段：

- `enabled=1`：节点正在发送 MIT 指令。
- `mode=topic_mit`：当前正在使用话题中的新指令。
- `mode=zero_mit`：没有新指令或指令超时，正在发送零 MIT 指令。
- `cmd_torque=[...]`：最近一次发送的 `t_ff`。
- `idN=missing`：没有收到 N 号电机反馈。
- `fresh=0`：收到过反馈，但反馈已经超过 `feedback_timeout_s`。

常用服务：

```bash
ros2 service call /dm_j4340/enable_all std_srvs/srv/Trigger {}
ros2 service call /dm_j4340/disable_all std_srvs/srv/Trigger {}
ros2 service call /dm_j4340/set_zero_all std_srvs/srv/Trigger {}
ros2 service call /dm_j4340/clear_faults_all std_srvs/srv/Trigger {}
```

急停建议直接调用失能服务：

```bash
ros2 service call /dm_j4340/disable_all std_srvs/srv/Trigger {}
```

如果 `disable_on_shutdown: true`，按 `Ctrl+C` 退出启动节点时也会自动发送失能命令。

## 配置文件

主要文件：

- `config/real_query_motor.yaml`：电机扫描参数。
- `config/real_motor_controller.yaml`：启动参数、MIT 话题、超时和量化范围。
- `launch/real_query_motor.launch.py`：启动 `real_query_motor_node`。
- `launch/real_motor_controller.launch.py`：启动 `real_motor_controller_node`。

`real_motor_controller.yaml` 里的关键参数：

```yaml
motor_ids: [1, 2, 3, 4]
joint_names: [base_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint]
mit_command_topic: /dm_j4340/mit_commands
dm_joint_states_topic: /dm_j4340/joint_states
robot_joint_states_topic: /joint_states
publish_robot_joint_states: true
mit_command_timeout_s: 0.2
command_rate_hz: 200.0
feedback_timeout_s: 0.5
p_min: -12.5
p_max: 12.5
v_min: -30.0
v_max: 30.0
kp_min: 0.0
kp_max: 500.0
kd_min: 0.0
kd_max: 5.0
t_min: -10.0
t_max: 10.0
```

## 代码架构

```text
real_motor_controller/
  config/
    real_query_motor.yaml          电机扫描参数
    real_motor_controller.yaml     MIT 启动和控制参数
  launch/
    real_query_motor.launch.py     启动 real_query_motor_node
    real_motor_controller.launch.py 启动 real_motor_controller_node
  src/real_motor_controller/
    query_motor_node.py            一次性电机在线扫描节点
    start_motor_node.py            启动流程、MIT 话题控制、状态发布和服务
    dm_protocol.py                 DM 协议解析、MIT 指令打包、寄存器帧打包
    usbcan_serial.py               USB-CAN 串口收发和回传帧解析
    ros_params.py                  ROS 参数声明和读取工具
```

运行时数据流：

```text
/dm_j4340/mit_commands
        |
        v
real_motor_controller_node
        |
        v
MIT 指令帧 -> USB-CAN 串口 -> CAN 总线 -> DM-J4340 电机
        ^
        |
反馈帧 <- USB-CAN 串口 <- CAN 总线 <- DM-J4340 电机
        |
        v
/dm_j4340/joint_states
/dm_j4340/status
```

模块职责：

- `dm_protocol.py`：负责电机协议，包括 MIT 指令打包、反馈解析、寄存器读写帧构造。
- `usbcan_serial.py`：负责 USB-CAN 适配器协议，包括串口写入、后台接收线程、回传帧解析。
- `real_query_motor_node.py`：负责发送安全探测帧，并报告哪些电机 ID 在线。
- `real_motor_controller_node.py`：负责启动序列、MIT 指令话题订阅、反馈缓存、周期发送和 ROS 服务。

## 安全建议

- 第一次测试时，从小的 `kp`、`kd`、`t_ff` 开始。
- 不要一上来发布最大力矩。
- 控制时保持发布频率高于 `mit_command_timeout_s`。
- 调试时保留一个终端，随时准备调用 `/dm_j4340/disable_all`。
- 如果某个电机方向反了，先反向该电机的 `p`、`v` 或 `t_ff` 符号。
