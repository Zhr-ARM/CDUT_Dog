#!/usr/bin/env python3
"""
* 项目名称: Single Leg Hopping Node (ROS2 Adapter)
* 维护者: 李想 (算法组)
* 最后修改: 2026-02-01
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
* [C++驱动层] <--- /motor_cmd (单电机协议, 长度=5) <--- [本节点]
* ======================================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

# 引入解耦的核心算法库
from .locomotion_core import SingleLegController


class FSMHopNode(Node):
    def __init__(self):
        super().__init__('fsm_hop_node')
        self.get_logger().info(">>> 正在启动跳跃算法节点 (Algorithm Layer)...")

        # ========================================================================
        # 1. 资源路径配置
        # ========================================================================
        self.declare_parameter('urdf_path', '')
        urdf_param = self.get_parameter('urdf_path').get_parameter_value().string_value

        if not urdf_param:
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
        # Pinocchio 状态: 1 个 base + 3 个关节
        q_init = np.zeros(4)
        self.controller = SingleLegController(urdf_path, q_init)

        # ========================================================================
        # 3. 状态缓存
        # ========================================================================
        self.motor_q = np.zeros(3)
        self.motor_v = np.zeros(3)
        self.base_height = 0.5   # TODO: 真实机器人接入状态估计
        self.has_feedback = False

        # ========================================================================
        # 4. 通信接口定义
        # ========================================================================

        # ---- 反馈订阅 ----
        # Protocol: [p1, v1, t1, temp1, err1, p2, v2...]
        self.sub_feedback = self.create_subscription(
            Float64MultiArray,
            '/motor_feedback',
            self.feedback_callback,
            10
        )

        # ---- 控制指令发布（按电机）----
        self.motor_ids = [1, 2, 3]
        self.pub_cmd = {}

        for mid in self.motor_ids:
            self.pub_cmd[mid] = self.create_publisher(
                Float64MultiArray,
                '/motor_cmd',
                10
            )

        # ========================================================================
        # 5. 控制循环
        # ========================================================================
        self.dt = 0.002  # 500 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("算法层就绪，等待底层反馈数据...")

    # ========================================================================
    # 回调：解析底层反馈
    # ========================================================================
    def feedback_callback(self, msg):
        data = np.array(msg.data)

        # 至少需要 3 * 5 = 15 个数据
        if len(data) < 15:
            return

        # Motor 1
        self.motor_q[0] = data[0]
        self.motor_v[0] = data[1]

        # Motor 2
        self.motor_q[1] = data[5]
        self.motor_v[1] = data[6]

        # Motor 3
        self.motor_q[2] = data[10]
        self.motor_v[2] = data[11]

        self.has_feedback = True

    # ========================================================================
    # 主控制循环
    # ========================================================================
    def control_loop(self):
        # 安全保护：未收到反馈不下发指令
        if not self.has_feedback:
            return

        # --------------------------------------------------------------------
        # 1. 状态估计
        # --------------------------------------------------------------------
        q_full = np.zeros(4)
        v_full = np.zeros(4)

        q_full[0] = self.base_height
        q_full[1:] = self.motor_q
        v_full[1:] = self.motor_v

        # --------------------------------------------------------------------
        # 2. 算法更新
        # --------------------------------------------------------------------
        # 返回 cmds: shape = (3, 5)
        # [p, v, t, kp, kd]
        cmds = self.controller.update(
            q_full,
            v_full,
            self.get_clock().now().nanoseconds / 1e9
        )

        # --------------------------------------------------------------------
        # 3. 协议拆包 & 下发（5 个一组）
        # --------------------------------------------------------------------
        for i, motor_id in enumerate(self.motor_ids):
            ros_msg = Float64MultiArray()
            ros_msg.data = cmds[i, :].tolist()
            self.pub_cmd[motor_id].publish(ros_msg)


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
