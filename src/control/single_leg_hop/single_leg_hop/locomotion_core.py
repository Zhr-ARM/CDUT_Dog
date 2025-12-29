"""
* 项目名称: Single Leg Hopping Control (Core Solver)
* 维护者: 李想 (算法组)
* 最后修改: 2025-12-29
* ======================================================================================
* 架构说明:
* 本模块是纯 Python 实现的运动控制核心 (Locomotion Kernel)。
* 它与 ROS 解耦，仅依赖 Pinocchio 和 Numpy。
* * 功能职责:
* 1. 动力学解算: 基于 Pinocchio 计算雅可比矩阵 (Jacobian) 和重力项 (Gravity Bias)。
* 2. 状态机管理: 维护 FLIGHT (腾空) 和 STANCE (支撑) 的状态切换。
* 3. 运控协议生成: 输出符合 MIT 模式的 [P, V, T, Kp, Kd] 指令。
* ======================================================================================
"""

import pinocchio as pin
import numpy as np

class SingleLegController:
    """
    单腿机器人核心运控类
    """
    def __init__(self, urdf_path, q0):
        # ========================================================================
        # 1. 动力学模型初始化 (Physics Engine)
        # ========================================================================
        # 加载 URDF 模型，构建刚体动力学树
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # 关节索引映射 (Joint Mapping)
        # Pinocchio 的 q 向量通常包含浮动基 (7维或1维) + 关节角度。
        # 假设 q 结构: [Slider_Z (被动), Hip, Thigh, Calf]
        # 对应的电机 ID: 1, 2, 3
        self.active_idx_q = [1, 2, 3] # 在 q 向量中的索引
        self.active_idx_v = [1, 2, 3] # 在 v 向量中的索引
        
        # ========================================================================
        # 2. 状态机与控制参数 (FSM & Gains)
        # ========================================================================
        self.state = "FLIGHT"  # 初始状态
        self.state_time = 0.0  # 状态持续计时器
        self.dt = 0.002        # 控制周期 (对应 ROS timer)

        # 参数字典 (支持在线调参)
        self.params = {
            # --- 空中相参数 (Position Control) ---
            # 目标: 快速收腿，准备着陆
            'kp_flight': np.array([80.0, 80.0, 80.0]), # 高刚度，位置闭环
            'kd_flight': np.array([3.0, 3.0, 3.0]),    # 适量阻尼，防止震荡
            'q_des_flight': np.array([0.0, -0.7, 1.4]),# 标称构型
            
            # --- 支撑相参数 (Force Control) ---
            # 目标: 像弹簧一样压缩，然后猛烈蹬地
            'push_force': np.array([0.0, 0.0, -150.0]), # 笛卡尔空间蹬地力 (N), Z轴向下为负
            'stance_duration': 0.2, # 蹬地持续时间 (s)
            'touch_threshold': 0.03 # 触地判定高度 (m)
        }

    def update(self, q_full, v_full, clock_time):
        """
        核心计算步 (Core Step)
        @param q_full: 完整位置向量 (包含基座 + 关节)
        @param v_full: 完整速度向量
        @param clock_time: 当前系统时间 (用于调试打印)
        @return: cmd_output (3x5 矩阵，对应三个电机的 MIT 指令)
        """
        
        # --------------------------------------------------------------------
        # A. 动力学更新 (Kinematics & Dynamics Update)
        # --------------------------------------------------------------------
        # 这一步计算所有关节的位置、速度、雅可比、重力项
        pin.computeAllTerms(self.model, self.data, q_full, v_full)
        pin.updateFramePlacements(self.model, self.data)
        
        # 获取足端位置 (Forward Kinematics)
        # 假设 URDF 中最后一个 Frame 是足端
        foot_id = self.model.nframes - 1
        foot_pos = self.data.oMf[foot_id].translation
        
        # --------------------------------------------------------------------
        # B. 有限状态机流转 (Finite State Machine)
        # --------------------------------------------------------------------
        if self.state == "FLIGHT":
            # [切换条件]: 足端高度 < 阈值 且 正在向下运动
            if foot_pos[2] < self.params['touch_threshold'] and v_full[0] < 0:
                self.state = "STANCE"
                self.state_time = 0.0
                print(f"[{clock_time:.3f}] [EVENT] Touch Down! Z_foot={foot_pos[2]:.3f}")

        elif self.state == "STANCE":
            self.state_time += self.dt
            # [切换条件]: 蹬地时间结束 (Time-based State Switch)
            if self.state_time > self.params['stance_duration']:
                self.state = "FLIGHT"
                print(f"[{clock_time:.3f}] [EVENT] Liftoff! (Thrust complete)")

        # --------------------------------------------------------------------
        # C. 混合控制指令生成 (Hybrid Control Generation)
        # --------------------------------------------------------------------
        # 初始化输出矩阵: 3行(电机数) x 5列(协议参数)
        # 协议格式: [P_des, V_des, T_ff, Kp, Kd]
        cmd_output = np.zeros((3, 5)) 

        if self.state == "FLIGHT":
            # === 空中相: 位置 PD 控制 ===
            # 策略: 让底层驱动器执行高频 PD 闭环，将腿拉回这一帧的目标位置
            for i in range(3):
                cmd_output[i] = [
                    self.params['q_des_flight'][i], # P_des: 期望角度
                    0.0,                            # V_des: 期望静止
                    0.0,                            # T_ff:  前馈为0 (或加重力补偿)
                    self.params['kp_flight'][i],    # Kp:    开启刚度
                    self.params['kd_flight'][i]     # Kd:    开启阻尼
                ]
                
        elif self.state == "STANCE":
            # === 支撑相: 纯力控 (Pure Force Control) ===
            # 策略: 关闭底层的位置环 (Kp=0)，直接发送计算好的力矩
            
            # 1. 计算雅可比矩阵 (J)
            # frameJacobian 返回 6xN 矩阵 (前3行线速度，后3行角速度)
            J = pin.computeFrameJacobian(self.model, self.data, q_full, foot_id, pin.LOCAL_WORLD_ALIGNED)
            # 截取我们需要的部分: 线速度行(0:3) + 主动关节列(active_idx_v)
            J_leg = J[:3, self.active_idx_v] 
            
            # 2. 虚拟力映射 (Virtual Work Principle)
            # 公式: Tau = J^T * F_cartesian
            tau_push = J_leg.T @ self.params['push_force']
            
            # 3. 重力补偿 (Gravity Compensation)
            # data.g 包含了维持当前姿态抗重力所需的力矩
            tau_gravity = self.data.g[self.active_idx_v]
            
            # 4. 总力矩合成
            tau_total = tau_push + tau_gravity
            
            # 5. 填充指令
            for i in range(3):
                cmd_output[i] = [
                    0.0, 0.0,           # P, V: 在纯力控模式下忽略
                    tau_total[i],       # T_ff: 发送计算好的总力矩
                    0.0,                # Kp:   设为0 (切断位置环，模拟纯力源)
                    1.0                 # Kd:   给一点微小阻尼，防止电机空转飞车
                ]

        return cmd_output