Single Leg Hopping Control (Algorithm Layer)

该功能包是单腿机器人的 核心运控算法层 (Algorithm Layer)。它基于 Pinocchio 动力学库构建，通过有限状态机 (FSM) 实现单腿的连续跳跃，并向底层驱动提供基于 MIT 模式的混合控制指令。
📋 功能特性

    高精度动力学解算：集成 Pinocchio 刚体动力学库，实时计算雅可比矩阵 (Jacobian) 与重力补偿项 (Gravity Bias)。

    混合控制策略 (Hybrid Control)：

        🚀 空中相 (Flight)：输出位置控制指令 (Position PD)，利用底层高频环路快速收腿并保持姿态。

        🏋️ 支撑相 (Stance)：输出前馈力矩指令 (Pure Torque)，基于虚拟功原理实现柔顺着陆与爆发蹬地。

    架构解耦：核心算法 (locomotion_core.py) 与 ROS 通信层完全分离，支持脱离 ROS 环境进行算法验证。

    标准协议对接：完美适配 deep_motor_ros 驱动节点，使用标准的 [p, v, t, kp, kd] 协议。

🛠️ 编译与安装
1. 依赖项

    Ubuntu 22.04 + ROS 2 Humble

    Pinocchio: sudo apt install ros-humble-pinocchio (或通过 pip 安装)

    Python 库: numpy

2. 编译

请在工作空间根目录下执行：
Bash

# 仅编译本算法包
colcon build --packages-select single_leg_hop
source install/setup.bash

3. ⚠️ 配置说明

本节点需要加载机器人的 URDF 模型以计算动力学。 默认情况下，节点会自动在 single_leg_sim 包中查找 single_leg.urdf。如果您使用自定义路径，请在运行时通过参数指定：
Bash

--ros-args -p urdf_path:="/path/to/your/custom.urdf"

🚀 快速启动
1. 启动步骤

为了让机器人动起来，你需要先启动底层驱动，再启动本算法节点。

Step 1: 启动底层驱动 (确保 CAN 已配置)
Bash

ros2 run deep_motor_ros deep_motor_node

Step 2: 启动跳跃算法
Bash

ros2 run single_leg_hop hop_node

正常启动日志示例：
Plaintext

[INFO] >>> 正在启动跳跃算法节点 (Algorithm Layer)...
[INFO] Loading Model: .../share/single_leg_sim/urdf/single_leg.urdf
[INFO] 算法层就绪，等待底层反馈数据...
[INFO] [123.456] [EVENT] Touch Down! Z_foot=0.021

📡 接口文档 (驱动对接)

本节点作为上层控制器，与底层驱动通过以下话题交互。
1. 接收反馈 (Input)

    话题: /motor_feedback

    来源: deep_motor_ros

    类型: std_msgs/msg/Float64MultiArray

数据协议： 接收底层回传的一维数组，解析逻辑如下：
索引偏移 (Offset)	参数	说明
0	Pos	实际角度 (rad)
1	Vel	实际角速度 (rad/s)
2	Torque	实际力矩 (N·m)
3	Temp	温度 (℃)
4	Error	错误码

(注：数据按电机 ID 顺序排列，例如 Motor 2 的 Pos 索引为 5)
2. 发送指令 (Output)

    话题: /motor_cmd

    去向: deep_motor_ros

    类型: std_msgs/msg/Float64MultiArray

    频率: 500Hz

控制策略映射：
状态 (FSM)	控制模式	发送参数说明	物理含义
空中 (Flight)	Position PD	Kp > 0, Kd > 0, T_ff = 0	位置伺服：让关节像弹簧一样迅速回到预设姿态 (0, -0.7, 1.4)。
支撑 (Stance)	Force Control	Kp = 0, Kd ≈ 1, T_ff = J^T · F	纯力控：关闭位置环，直接映射笛卡尔空间蹬地力，实现爆发跳跃。
🧠 核心算法逻辑

代码位于 single_leg_hop/locomotion_core.py，核心逻辑如下：
Python

def update(self, q, v):
    # 1. 动力学更新
    pin.computeAllTerms(model, data, q, v)
    
    # 2. 状态机流转
    if state == FLIGHT and touch_down:
        state = STANCE
    elif state == STANCE and time_out:
        state = FLIGHT
        
    # 3. 计算力矩 (MIT Mode)
    if state == STANCE:
        # 雅可比转置力控: τ = J^T * F_push
        tau = J.T @ F_push + Gravity
        cmd = [0, 0, tau, 0, 1.0] # Kp=0 (纯力)
    else:
        # PD 位置控制
        cmd = [q_des, 0, 0, 80, 3.0] # Kp=80 (高刚度)
        
    return cmd

🧪 调试与测试

1. 监控算法输出 查看算法计算出的实时力矩和刚度指令：
Bash

ros2 topic echo /motor_cmd

2. 仅测试算法逻辑 (无需真机) 由于算法层与通信层解耦，你可以直接运行 Python 脚本测试数学计算是否正常：
Bash

# 进入包目录
cd src/control/single_leg_hop/single_leg_hop
python3 locomotion_core.py
# (注：需在 core 文件中添加简单的 main 函数实例化测试)

👤 维护者

    Maintainer: Li Xiang (Algorithm Group)