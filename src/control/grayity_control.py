#  项目名称: 单腿姿态控制算法（未完成）
#  维护者: 李想
#  最后修改: 2025-12-22

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
import numpy as np
import os
import math # 引入数学库计算正弦波

class TrajectoryTrackingNode(Node):
    def __init__(self):
        super().__init__('trajectory_tracking_node')

        # 1. 模型加载 (保持不变)
        urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf/single_leg.urdf")
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        self.q = np.zeros(self.model.nq)
        self.v = np.zeros(self.model.nv)

        self.joint_names = ['hip_joint', 'thigh_joint', 'calf_joint']
        self.joint_ids = [self.model.getJointId(name) for name in self.joint_names]
        
        # 2. 控制参数 (稍微调柔顺一点)
        self.kp = np.array([40.0, 40.0, 40.0]) 
        self.kd = np.array([3.0,  3.0,  3.0])  

        # 3. 记录程序启动时间
        self.start_time = self.get_clock().now()

        # ROS 通讯
        self.sub = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/leg_effort_controller/commands', 10)

        self.get_logger().info("轨迹跟踪控制器启动！腿部即将开始运动...")

    def state_callback(self, msg):
        # 更新当前状态 q 和 v (保持不变)
        for i, name in enumerate(self.joint_names):
            try:
                idx_in_msg = msg.name.index(name)
                q_idx = self.model.joints[self.joint_ids[i]].idx_q
                v_idx = self.model.joints[self.joint_ids[i]].idx_v
                self.q[q_idx] = msg.position[idx_in_msg]
                self.v[v_idx] = msg.velocity[idx_in_msg]
            except ValueError:
                pass

        # ==========================================
        # 4. 生成动态轨迹 (核心变化点)
        # ==========================================
        
        # 计算当前运行了多少秒
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9  # 换算成秒
        
        # 定义正弦波参数
        # 目标位置 = 基准位置 + 幅度 * sin(2 * pi * 频率 * t)
        amplitude = 0.5  # 摆动幅度 (弧度)
        frequency = 0.5  # 频率 (0.5Hz, 每2秒摆动一次)
        
        # 我们让大腿 (thigh) 和 小腿 (calf) 动起来
        q_des = np.zeros(3)
        q_des[0] = 0.0 # 髋关节保持不动
        q_des[1] = -0.8 + amplitude * math.sin(2 * math.pi * frequency * t) # 大腿前后摆
        q_des[2] = 1.6 + amplitude * math.cos(2 * math.pi * frequency * t)  # 小腿伸缩 (用cos产生相位差，更像自然动作)

        # 理论上的目标速度 (对位置求导)
        v_des = np.zeros(3)
        v_des[1] = amplitude * (2 * math.pi * frequency) * math.cos(2 * math.pi * frequency * t)
        v_des[2] = -amplitude * (2 * math.pi * frequency) * math.sin(2 * math.pi * frequency * t)
        
        # ==========================================

        # 5. 计算重力项 g(q)
        g = pin.computeGeneralizedGravity(self.model, self.data, self.q)

        # 6. PD + 重力前馈 控制律
        # tau = g + Kp*(q_des - q) + Kd*(v_des - v)
        tau_total = g + self.kp * (q_des - self.q) + self.kd * (v_des - self.v)

        # 7. 发布指令
        tau_msg = Float64MultiArray()
        cmd_torques = []
        for i in range(len(self.joint_names)):
             v_idx = self.model.joints[self.joint_ids[i]].idx_v
             cmd_torques.append(tau_total[v_idx])
        
        tau_msg.data = cmd_torques
        self.pub.publish(tau_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()