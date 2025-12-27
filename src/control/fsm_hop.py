#  项目名称: 单腿姿态控制算法（未跟电机关联）
#  维护者: 李想
#  最后修改: 2025-12-27

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
import numpy as np
import os
import time

class FSMHopNode(Node):
    def __init__(self):
        super().__init__('fsm_hop_node')

        # ==========================================
        # 1. 路径与模型加载 (Pinocchio Setup)
        # ==========================================
        # 自动寻找 URDF 文件路径
        # 假设结构为: src/single_leg_sim/urdf/single_leg.urdf
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        # 注意：这里如果你的 urdf 是由 xacro 生成的，确保该目录下有 .urdf 文件
        # 如果没有，你需要先运行 xacro source.xacro > target.urdf
        urdf_path = os.path.join(package_dir, 'urdf', 'single_leg.urdf')
        
        self.get_logger().info(f"正在加载 URDF: {urdf_path}")
        
        try:
            # 加载模型
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
        except Exception as e:
            self.get_logger().error(f"无法加载 URDF，请检查路径或文件是否生成: {e}")
            return

        # 获取足端 Link ID (根据你的URDF，最末端是 link3)
        if self.model.existFrame("link3"):
            self.foot_id = self.model.getFrameId("link3")
        else:
            self.foot_id = self.model.nframes - 1
            self.get_logger().warn("找不到 'link3'，使用最后一个 Frame 作为足端")

        # ==========================================
        # 2. 关节定义与状态初始化
        # ==========================================
        # 关键点：你的 URDF 有 4 个关节 (world_joint + 3个腿关节)
        # Pinocchio 的 q 向量长度为 4
        self.q = pin.neutral(self.model) 
        self.v = np.zeros(self.model.nv)
        
        # 所有的关节名称 (对应 URDF)
        # 顺序必须和 Pinocchio 解析的一致，通常是树形结构顺序
        # 0: world_joint (被动, Z轴滑动)
        # 1: hip_joint   (主动)
        # 2: thigh_joint (主动)
        # 3: calf_joint  (主动)
        self.active_joints = ['hip_joint', 'thigh_joint', 'calf_joint']
        
        # 用于 PD 控制的当前状态 (只存3个主动关节)
        self.active_q = np.zeros(3)
        self.active_v = np.zeros(3)

        # 状态机变量
        self.state = "FLIGHT" 
        self.state_start_time = self.get_clock().now()

        # ==========================================
        # 3. 控制参数
        # ==========================================
        # 空中相 PD 参数 (保持姿态)
        self.kp_flight = np.array([150.0, 150.0, 150.0]) 
        self.kd_flight = np.array([5.0, 5.0, 5.0])
        
        # 空中目标姿态 (收腿准备落地)
        self.q_des_flight = np.array([0.0, -0.7, 1.4]) 

        # 支撑相 蹬地参数
        # 想要跳得高，就把 Z 轴负值加大，例如 -200
        self.push_force_cartesian = np.array([0.0, 0.0, -180.0]) 

        # ==========================================
        # 4. ROS 通信
        # ==========================================
        # 订阅关节状态
        self.sub_joint_state = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 发布力矩命令
        # 修正：根据你的 controllers.yaml，名字是 leg_effort_controller
        self.pub_commands = self.create_publisher(
            Float64MultiArray,
            '/leg_effort_controller/commands', 
            10
        )

        # 1kHz 控制循环
        self.timer = self.create_timer(0.001, self.control_loop)
        
        self.get_logger().info("控制器已启动，等待 Gazebo 关节数据...")

    def joint_state_callback(self, msg):
        """
        读取 Gazebo 发来的真实状态，填入 Pinocchio 的模型向量
        """
        try:
            # 1. 处理被动关节 (world_joint) - 它是 Pinocchio 里的第 0 个关节
            if 'world_joint' in msg.name:
                idx = msg.name.index('world_joint')
                self.q[0] = msg.position[idx]
                self.v[0] = msg.velocity[idx]

            # 2. 处理主动关节 (Leg joints) - 对应 Pinocchio 里的 1, 2, 3
            for i, name in enumerate(self.active_joints):
                if name in msg.name:
                    idx = msg.name.index(name)
                    # 更新 Pinocchio 向量 (偏移量+1)
                    self.q[i+1] = msg.position[idx]
                    self.v[i+1] = msg.velocity[idx]
                    
                    # 更新本地控制向量
                    self.active_q[i] = msg.position[idx]
                    self.active_v[i] = msg.velocity[idx]
        except ValueError:
            pass

    def control_loop(self):
        # 如果还没收到数据，不进行计算
        if np.all(self.q == 0):
            return

        # 1. 动力学更新
        pin.computeAllTerms(self.model, self.data, self.q, self.v)
        pin.updateFramePlacements(self.model, self.data)

        # 获取足端位置和速度 (用于状态机判断)
        foot_pos = self.data.oMf[self.foot_id].translation
        
        now = self.get_clock().now()
        tau_ff = np.zeros(3) # 前馈力矩

        # ==================== FSM 状态机 ====================
        
        if self.state == "FLIGHT":
            # --- 空中相：PD 控制归位 ---
            tau_ff = self.kp_flight * (self.q_des_flight - self.active_q) + \
                     self.kd_flight * (0 - self.active_v)
            
            # 切换条件：足端高度 < 0.03m (触地)
            # 注意：world_joint 初始高度是 0.9，如果盒体高0.9，你需要看具体 z 坐标
            # 假设地面是 z=0，foot_pos[2] 是绝对高度
            if foot_pos[2] < 0.03: 
                self.state = "STANCE"
                self.state_start_time = now
                self.get_logger().info(f"触地 (Z={foot_pos[2]:.2f}) -> 进入 STANCE")

        elif self.state == "STANCE":
            # --- 支撑相：雅可比转置力控 (虚拟弹簧/蹬地) ---
            
            # 计算雅可比 (3x4 矩阵)
            J = pin.computeFrameJacobian(self.model, self.data, self.q, self.foot_id, pin.LOCAL_WORLD_ALIGNED)
            # 取线速度部分 (前3行)，且只取对应主动关节的列 (后3列)
            J_leg = J[:3, 1:] 
            
            # Tau = J^T * F
            tau_ff = J_leg.T @ self.push_force_cartesian

            # 切换条件：蹬地时间超过 0.15s
            duration = (now - self.state_start_time).nanoseconds / 1e9
            if duration > 0.15:
                self.state = "FLIGHT"
                self.get_logger().info("起跳 -> 进入 FLIGHT")

        # ====================================================

        # 2. 重力补偿 (核心)
        # self.data.g 包含4个元素，第0个是滑块的重力，我们要后3个
        tau_gravity = self.data.g[1:]

        # 3. 合成总力矩
        tau_output = tau_ff + tau_gravity

        # 4. 安全限幅 (根据你的URDF limit effort=100)
        tau_output = np.clip(tau_output, -50.0, 50.0)

        # 5. 发布命令
        self.publish_command(tau_output)

    def publish_command(self, tau):
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.pub_commands.publish(msg)

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