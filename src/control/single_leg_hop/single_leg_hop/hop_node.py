#!/usr/bin/env python3
"""
* 项目名称: Single Leg Hopping Node (ROS2 Adapter)
* 维护者: 李想 (算法组)
* 最后修改: 2025-12-29
* ======================================================================================
* 架构说明:
* 本节点充当 "算法适配层 (Adapter Layer)" 的角色。
* * 数据流向:
* [C++驱动层] ---> /motor_feedback (原始数据) ---> [本节点]
* | (解包 & 状态估计)
* v
* [SingleLegController] (核心计算)
* | (计算 MIT 指令)
* v
* [C++驱动层] <--- /motor_cmd (数组协议)    <--- [本节点]
* ======================================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
import time

# 引入解耦的核心算法库
from .locomotion_core import SingleLegController 

class FSMHopNode(Node):
    def __init__(self):
        super().__init__('fsm_hop_node')
        self.get_logger().info(">>> 正在启动跳跃算法节点 (Algorithm Layer)...")

        # ========================================================================
        # 1. 资源路径配置
        # ========================================================================
        # 优先尝试从 launch 文件参数获取 URDF 路径
        self.declare_parameter('urdf_path', '')
        urdf_param = self.get_parameter('urdf_path').get_parameter_value().string_value
        
        if not urdf_param:
            # 回退逻辑: 自动查找 share 目录
            try:
                pkg_share = get_package_share_directory('single_leg_sim')
                urdf_path = os.path.join(pkg_share, 'urdf', 'single_leg.urdf')
            except Exception as e:
                self.get_logger().error(f"无法找到 URDF 文件: {e}")
                return
        else:
            urdf_path = urdf_param

        self.get_logger().info(f"Loading Model: {urdf_path}")

        # ========================================================================
        # 2. 算法核心实例化
        # ========================================================================
        # 初始化 Pinocchio 向量 (假设 4 自由度: 1 Slider + 3 Revolute)
        q_init = np.zeros(4) 
        self.controller = SingleLegController(urdf_path, q_init)

        # 缓存变量 (用于存储来自驱动层的最新反馈)
        self.motor_q = np.zeros(3)
        self.motor_v = np.zeros(3)
        self.base_height = 0.5 # [TODO] 真实机器人需接入动捕或状态估计器
        self.has_feedback = False

        # ========================================================================
        # 3. 通信接口定义
        # ========================================================================
        
        # 订阅: 来自底层的电机反馈
        # Protocol: [p1, v1, t1, temp1, err1, p2, v2...]
        self.sub_feedback = self.create_subscription(
            Float64MultiArray,
            '/motor_feedback',
            self.feedback_callback,
            10
        )

        # 发布: 发给底层的控制指令
        # Protocol: [p1, v1, t1, kp1, kd1, p2, v2...] (长度 15)
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            '/motor_cmd',
            10
        )

        # 定时器: 500Hz 控制循环 (dt = 0.002s)
        self.dt = 0.002
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("算法层就绪，等待底层反馈数据...")

    def feedback_callback(self, msg):
        """
        回调函数: 解析底层协议
        """
        data = np.array(msg.data)
        # 校验数据长度 (3个电机 * 5个状态数据 = 15)
        if len(data) < 15: 
            return 

        # 数据解包 (Unpacking): 提取每个电机的 P 和 V
        # 对应驱动层的: feedback_msg.data.push_back(p, v, t, ...)
        # Motor 1 (Index 0-4)
        self.motor_q[0] = data[0]
        self.motor_v[0] = data[1]
        # Motor 2 (Index 5-9)
        self.motor_q[1] = data[5]
        self.motor_v[1] = data[6]
        # Motor 3 (Index 10-14)
        self.motor_q[2] = data[10]
        self.motor_v[2] = data[11]
        
        self.has_feedback = True

    def control_loop(self):
        """
        主控制循环
        """
        # 安全检查: 如果还没收到电机数据，不发送指令，防止飞车
        if not self.has_feedback:
            return

        # --------------------------------------------------------------------
        # 1. 状态估计 (State Estimation)
        # --------------------------------------------------------------------
        # Pinocchio 需要全状态向量 q_full (Base + Joints)
        q_full = np.zeros(4)
        v_full = np.zeros(4)
        
        # [模拟] 基座高度
        # 在真实实验中，这里应该读取 Vicon/OptiTrack 或 IMU 积分数据
        q_full[0] = self.base_height 
        
        # 填入关节真实数据
        q_full[1:] = self.motor_q
        v_full[1:] = self.motor_v

        # --------------------------------------------------------------------
        # 2. 算法迭代 (Algorithm Update)
        # --------------------------------------------------------------------
        # 调用纯 Python 核心库，计算下一时刻的控制指令
        # 返回 cmds 形状: (3, 5)
        cmds = self.controller.update(q_full, v_full, self.get_clock().now().nanoseconds/1e9)

        # --------------------------------------------------------------------
        # 3. 协议打包与下发 (Protocol Packing)
        # --------------------------------------------------------------------
        ros_msg = Float64MultiArray()
        
        # 展平数组 (Flatten): 变成一维数组 [15]
        # 顺序: [m1_p, m1_v, m1_t, m1_kp, m1_kd, m2_p...]
        ros_msg.data = cmds.flatten().tolist() 
        
        self.pub_cmd.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FSMHopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()