/**
 * @file gait_controller.cpp
 * @brief 步态控制器节点 - 基于位控初始算法实现 Gazebo 仿真步态
 *
 * 将 posotion_control.c 中的逆运动学和轨迹规划算法集成为 ROS2 节点，
 * 通过 /motor_cmd 话题向 mit_controller 发送控制指令。
 *
 * ========================================================================
 * 平行四边形连杆运动学:
 * ========================================================================
 *   knee_crank_joint = α  (驱动)
 *   → crank_pivot_joint = -α  (mimic, ×-1)
 *     → knee_pivot_joint = -α  (mimic, ×1)
 *       → knee_joint = -α      (mimic, ×1)
 *
 * 位控算法映射 (去掉减速比 6.33):
 *   thigh_joint      ← theta2 (大腿摆角)
 *   knee_crank_joint ← theta1 (曲柄角, 通过连杆传动到膝关节)
 *
 * ========================================================================
 * 控制接口 /motor_cmd (Float64MultiArray, 15个元素):
 * ========================================================================
 *   [hip_p, hip_v, hip_t, hip_kp, hip_kd,
 *    thigh_p, thigh_v, thigh_t, thigh_kp, thigh_kd,
 *    knee_crank_p, knee_crank_v, knee_crank_t, knee_crank_kp, knee_crank_kd]
 *
 * ========================================================================
 * 抖动消除策略:
 * ========================================================================
 * 1. 适中的 Kp + 充足的 Kd → 临界阻尼或过阻尼
 * 2. 速度前馈 v_des → 减小跟踪误差，降低 Kp 需求
 * 3. 指令平滑滤波 → 消除离散化阶跃引起的冲击
 * 4. 重力补偿前馈力矩 → 减轻 PD 负担，降低所需增益
 */

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// ============================================================================
// 运动学参数
// ============================================================================
static constexpr double L1 = 0.200;  // 大腿长度 (m)
static constexpr double L2 = 0.243;  // 小腿长度 (m)
static constexpr double PI = M_PI;

// ============================================================================
// 逆运动学 - 移植自 Inverse_Calculation()
// ============================================================================
struct IKResult {
    double theta2;  // 大腿摆角 → thigh_joint
    double theta1;  // 曲柄角度 → knee_crank_joint
};

/**
 * @brief 2-DOF 平面腿部逆运动学求解
 * @param x 足端在髋关节坐标系下的水平位移 (m)，前向为正
 * @param y 足端在髋关节坐标系下的竖直位移 (m)，向下为正
 * @return IKResult 包含大腿摆角 theta2 与曲柄角 theta1
 */
static IKResult inverse_kinematics(double x, double y)
{
    double L_sq = x * x + y * y;
    double L = std::sqrt(L_sq);

    // 安全限幅
    const double L_min = std::abs(L1 - L2) + 0.001;
    const double L_max = L1 + L2 - 0.001;
    if (L < L_min) L = L_min;
    if (L > L_max) L = L_max;
    L_sq = L * L;

    // 膝关节弯曲角
    double cos_knee = (L1 * L1 + L2 * L2 - L_sq) / (2.0 * L1 * L2);
    cos_knee = std::clamp(cos_knee, -1.0, 1.0);
    double theta1 = PI - std::acos(cos_knee);

    // 大腿摆角
    double phi1 = std::atan2(-x, y);
    double cos_phi2 = (L_sq + L1 * L1 - L2 * L2) / (2.0 * L * L1);
    cos_phi2 = std::clamp(cos_phi2, -1.0, 1.0);
    double phi2 = std::acos(cos_phi2);
    double theta2 = phi2 + phi1;

    return {theta2, theta1};
}

struct FootState {
    double x, y;    // 位置 (m)
    double vx, vy;  // 速度 (m/s)
};

/**
 * @brief 摆线轨迹 (Trajectory1) 与解析速度
 * @param tau 归一化相位，范围 [0, 1]
 * @param step_length 步长 (m)，完整摆动过程中足端前进距离
 * @param step_height 抬腿高度 (m)，足端轨迹最高点高度
 * @param start_x 起始水平位置 (m)
 * @param start_y 起始竖直位置 (m)
 * @param period 摆动相持续时间 (s)，用于速度求解
 * @return FootState 足端位置与速度，落地时垂直速度为 0
 */
static FootState trajectory_cycloid(double tau, double step_length, double step_height,
                                     double start_x, double start_y, double period)
{
    FootState s;
    s.x = start_x + step_length * (tau - std::sin(2.0 * PI * tau) / (2.0 * PI));
    s.y = start_y + step_height * (1.0 - std::cos(2.0 * PI * tau)) / 2.0;

    if (period > 0.0) {
        double dtau = 1.0 / period;
        s.vx = step_length * (1.0 - std::cos(2.0 * PI * tau)) * dtau;
        s.vy = step_height * PI * std::sin(2.0 * PI * tau) * dtau;
    } else {
        s.vx = 0.0;
        s.vy = 0.0;
    }
    return s;
}

// ============================================================================
// 雅可比矩阵 → 关节角速度 (速度前馈)
// ============================================================================
struct JointVel {
    double v_theta2;  // 大腿关节角速度 (rad/s)
    double v_theta1;  // 曲柄关节角速度 (rad/s)
};

/**
 * @brief 由足端速度计算关节角速度 (雅可比逆)
 * @param x 足端水平位移 (m)
 * @param y 足端竖直位移 (m)
 * @param vx 足端水平速度 (m/s)
 * @param vy 足端竖直速度 (m/s)
 * @return JointVel 关节角速度，作为速度前馈
 */
static JointVel compute_joint_velocity(double x, double y, double vx, double vy)
{
    double L_sq = x * x + y * y;
    double L = std::sqrt(L_sq);
    if (L < 0.01) return {0.0, 0.0};

    const double L_min = std::abs(L1 - L2) + 0.001;
    const double L_max = L1 + L2 - 0.001;
    if (L < L_min) L = L_min;
    if (L > L_max) L = L_max;
    L_sq = L * L;

    // dtheta1/dL
    double cos_knee = (L1 * L1 + L2 * L2 - L_sq) / (2.0 * L1 * L2);
    cos_knee = std::clamp(cos_knee, -0.999, 0.999);
    double sin_knee = std::sqrt(1.0 - cos_knee * cos_knee);
    if (sin_knee < 1e-6) sin_knee = 1e-6;
    double dtheta1_dL = L / (L1 * L2 * sin_knee);

    // dphi2/dL
    double cos_phi2 = (L_sq + L1 * L1 - L2 * L2) / (2.0 * L * L1);
    cos_phi2 = std::clamp(cos_phi2, -0.999, 0.999);
    double sin_phi2 = std::sqrt(1.0 - cos_phi2 * cos_phi2);
    if (sin_phi2 < 1e-6) sin_phi2 = 1e-6;
    double dphi2_dL = -(L_sq - L1 * L1 + L2 * L2) / (2.0 * L_sq * L1 * sin_phi2);

    double dL_dx = x / L;
    double dL_dy = y / L;
    double dphi1_dx = -y / L_sq;
    double dphi1_dy = -x / L_sq;

    double dtheta2_dx = dphi1_dx + dphi2_dL * dL_dx;
    double dtheta2_dy = dphi1_dy + dphi2_dL * dL_dy;
    double dtheta1_dx = dtheta1_dL * dL_dx;
    double dtheta1_dy = dtheta1_dL * dL_dy;

    return {
        dtheta2_dx * vx + dtheta2_dy * vy,
        dtheta1_dx * vx + dtheta1_dy * vy
    };
}

// ============================================================================
// 一阶低通滤波器 (指令平滑，消除阶跃冲击)
// ============================================================================
/**
 * @brief 一阶低通滤波器，用于平滑期望角度与速度指令
 */
class LowPassFilter {
public:
    LowPassFilter(double alpha = 0.3) : alpha_(alpha), initialized_(false), value_(0.0) {}

    /**
     * @brief 进行一次滤波更新
     * @param input 当前输入值
     * @return 滤波后的输出值
     */
    double filter(double input) {
        if (!initialized_) {
            value_ = input;
            initialized_ = true;
        } else {
            value_ = alpha_ * input + (1.0 - alpha_) * value_;
        }
        return value_;
    }

    /**
     * @brief 设置滤波系数
     * @param alpha 取值范围 (0,1]，越小越平滑但延迟越大
     */
    void set_alpha(double alpha) { alpha_ = std::clamp(alpha, 0.01, 1.0); }

private:
    double alpha_;
    bool initialized_;
    double value_;
};

// ============================================================================
// 步态控制器节点
// ============================================================================
/**
 * @brief ROS2 步态控制器节点
 *
 * 通过参数配置步态、PD 增益与滤波器，周期性发布 `/motor_cmd`。
 * hip 关节保持固定角度，thigh/knee 依据足端轨迹生成期望角与速度前馈。
 */
class GaitController : public rclcpp::Node
{
public:
    GaitController() : Node("gait_controller")
    {
        // 声明参数
        this->declare_parameter("step_length", 0.1);   // 步长 (m)
        this->declare_parameter("step_height", 0.05);  // 抬腿高度 (m)
        this->declare_parameter("gait_period", 5.0);   // 完整步态周期 (s)
        this->declare_parameter("stance_depth", 0.35); // 站立时足端深度 (m)
        this->declare_parameter("control_rate", 50.0); // 控制频率 (Hz)
        this->declare_parameter("swing_ratio", 0.8);   // 摆动相占比 [0,1]

        // 零位对齐
        this->declare_parameter("auto_zero_offsets", true); // 自动对齐零位
        this->declare_parameter("thigh_offset", 0.0);       // 大腿零位偏置 (rad)
        this->declare_parameter("knee_offset", 0.0);        // 小腿零位偏置 (rad)

        // PD 增益 - 抖动消除调优
        // 原则: 降低 Kp 减小刚度，增大 Kd 增加阻尼
        // 注意：过大的 Kd 会在数值仿真中引起高频噪声放大，反而导致剧烈抖动
        // 建议 Kd 取值为 Critically Damped 或更小
        this->declare_parameter("hip_kp", 3.0);          // hip 位置环刚度
        this->declare_parameter("hip_kd", 0.1);          // hip 速度阻尼
        this->declare_parameter("thigh_kp", 12.0);       // thigh 位置环刚度
        this->declare_parameter("thigh_kd", 0.6);        // thigh 速度阻尼
        this->declare_parameter("knee_crank_kp", 10.0);  // knee_crank 位置环刚度
        this->declare_parameter("knee_crank_kd", 0.5);   // knee_crank 速度阻尼

        // hip 固定角度
        this->declare_parameter("hip_angle", 3.21);  // hip 固定角 (rad)

        // 滤波器系数 (0~1, 越小越平滑但延迟越大)
        this->declare_parameter("filter_alpha", 0.44); // 低通滤波系数 (0,1]

        // 读取参数
        step_length_ = this->get_parameter("step_length").as_double();
        step_height_ = this->get_parameter("step_height").as_double();
        gait_period_ = this->get_parameter("gait_period").as_double();
        stance_depth_ = this->get_parameter("stance_depth").as_double();
        control_rate_ = this->get_parameter("control_rate").as_double();
        swing_ratio_ = this->get_parameter("swing_ratio").as_double();

        auto_zero_offsets_ = this->get_parameter("auto_zero_offsets").as_bool();
        thigh_offset_ = this->get_parameter("thigh_offset").as_double();
        knee_offset_ = this->get_parameter("knee_offset").as_double();

        hip_kp_ = this->get_parameter("hip_kp").as_double();
        hip_kd_ = this->get_parameter("hip_kd").as_double();
        thigh_kp_ = this->get_parameter("thigh_kp").as_double();
        thigh_kd_ = this->get_parameter("thigh_kd").as_double();
        knee_crank_kp_ = this->get_parameter("knee_crank_kp").as_double();
        knee_crank_kd_ = this->get_parameter("knee_crank_kd").as_double();
        hip_angle_ = this->get_parameter("hip_angle").as_double();

        double filter_alpha = this->get_parameter("filter_alpha").as_double();

        // 初始化滤波器
        filter_theta2_.set_alpha(filter_alpha);
        filter_theta1_.set_alpha(filter_alpha);
        filter_v_theta2_.set_alpha(filter_alpha);
        filter_v_theta1_.set_alpha(filter_alpha);

        // 步态状态
        swing_period_ = gait_period_ * swing_ratio_;
        stance_period_ = gait_period_ * (1.0 - swing_ratio_);
        phase_time_ = 0.0;
        is_swing_ = true;

        if (auto_zero_offsets_) {
            double start_x = -step_length_ / 2.0;
            double start_y = stance_depth_;
            IKResult ik0 = inverse_kinematics(start_x, start_y);
            thigh_offset_ = -ik0.theta2;
            knee_offset_ = -ik0.theta1;
        }

        // 重力补偿估算 (简化: 假设大腿+小腿质量集中在大腿中点)
        // 从 URDF: thigh_Link mass=0.756, knee_Link mass=0.210
        // 总质量约 0.97 kg, 等效力臂约 L1/2
        // 重力矩 ≈ m*g*L_eff
        gravity_torque_thigh_ = 0.97 * 9.81 * L1 / 2.0;

        // 记录上一次时间戳 (用仿真时间驱动相位)
        last_time_initialized_ = false;

        // 发布者
        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor_cmd", 10);

        // 定时器 - 使用 create_wall_timer 触发回调，
        // 但相位推进使用 ROS 时钟 (仿真时间) 计算真实 dt
        double dt = 1.0 / control_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&GaitController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "步态控制器已启动 (C++ position_control)");
        RCLCPP_INFO(this->get_logger(), "  步长: %.3fm, 步高: %.3fm, 周期: %.2fs",
                     step_length_, step_height_, gait_period_);
        RCLCPP_INFO(this->get_logger(), "  站立深度: %.3fm, 控制频率: %.1fHz",
                     stance_depth_, control_rate_);
        RCLCPP_INFO(this->get_logger(), "  PD: thigh(Kp=%.1f,Kd=%.1f) knee_crank(Kp=%.1f,Kd=%.1f)",
                     thigh_kp_, thigh_kd_, knee_crank_kp_, knee_crank_kd_);
        RCLCPP_INFO(this->get_logger(), "  滤波系数: %.2f, 重力补偿: %.2f N·m",
                     filter_alpha, gravity_torque_thigh_);
        RCLCPP_INFO(this->get_logger(), "  零位偏置: thigh=%.3f, knee=%.3f (auto=%s)",
                 thigh_offset_, knee_offset_, auto_zero_offsets_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "============================================================");
    }

private:
    /**
     * @brief 控制主循环：计算足端目标、逆解关节、滤波并发布指令
     */
    void control_loop()
    {
        // 使用 ROS 时钟 (仿真时间) 计算真实 dt
        // 这样即使 Gazebo 跑得比实时慢，相位推进也与仿真同步
        rclcpp::Time now = this->now();
        double dt;
        if (!last_time_initialized_) {
            dt = 0.0;  // 第一次调用不推进相位
            last_time_ = now;
            last_time_initialized_ = true;
        } else {
            dt = (now - last_time_).seconds();
            last_time_ = now;
        }
        // 防止异常大的 dt (例如仿真暂停后恢复)
        if (dt < 0.0) dt = 0.0;
        if (dt > 0.1) dt = 0.1;

        // 当前相位
        double current_period = is_swing_ ? swing_period_ : stance_period_;
        double tau = (current_period > 0.0) ? (phase_time_ / current_period) : 0.0;
        tau = std::clamp(tau, 0.0, 1.0);

        // ================================================================
        // 计算足端目标位置和速度
        // ================================================================
        double foot_x, foot_y, vx_foot, vy_foot;

        if (is_swing_) {
            // 摆动相: 摆线轨迹
            double start_x = -step_length_ / 2.0;
            FootState fs = trajectory_cycloid(
                tau, step_length_, step_height_,
                start_x, 0.0, current_period);
            foot_x = fs.x;
            foot_y = stance_depth_ - fs.y;  // 抬腿时 y 减小
            vx_foot = fs.vx;
            vy_foot = -fs.vy;
        } else {
            // 支撑相: 线性后移
            double front_x = step_length_ / 2.0;
            double back_x = -step_length_ / 2.0;
            foot_x = front_x + (back_x - front_x) * tau;
            foot_y = stance_depth_;
            vx_foot = (current_period > 0.0) ? ((back_x - front_x) / current_period) : 0.0;
            vy_foot = 0.0;
        }

        // ================================================================
        // 逆运动学
        // ================================================================
        IKResult ik = inverse_kinematics(foot_x, foot_y);

        // ================================================================
        // 关节角速度 (速度前馈)
        // ================================================================
        JointVel jv = compute_joint_velocity(foot_x, foot_y, vx_foot, vy_foot);

        // ================================================================
        // 低通滤波 (消除离散阶跃冲击 → 消除抖动)
        // ================================================================
        double cmd_theta2 = filter_theta2_.filter(ik.theta2 + thigh_offset_);
        double cmd_theta1 = filter_theta1_.filter(ik.theta1 + knee_offset_);
        double cmd_v_theta2 = filter_v_theta2_.filter(jv.v_theta2);
        double cmd_v_theta1 = filter_v_theta1_.filter(jv.v_theta1);

        // ================================================================
        // 重力补偿前馈力矩
        // ================================================================
        // thigh_joint 重力矩 ≈ m*g*L_eff*cos(theta2)
        double t_ff_thigh = gravity_torque_thigh_ * std::cos(cmd_theta2);
        // knee_crank 重力矩较小，简化为 0
        double t_ff_knee = 0.0;

        // ================================================================
        // 发布控制指令
        // ================================================================
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(15);

        // hip_joint: 保持固定
        msg.data[0]  = hip_angle_;
        msg.data[1]  = 0.0;
        msg.data[2]  = 0.0;
        msg.data[3]  = hip_kp_;
        msg.data[4]  = hip_kd_;

        // thigh_joint: 大腿摆角 + 速度前馈 + 重力补偿
        msg.data[5]  = cmd_theta2;
        msg.data[6]  = cmd_v_theta2;
        msg.data[7]  = t_ff_thigh;
        msg.data[8]  = thigh_kp_;
        msg.data[9]  = thigh_kd_;

        // knee_crank_joint: 曲柄角度 + 速度前馈
        msg.data[10] = cmd_theta1;
        msg.data[11] = cmd_v_theta1;
        msg.data[12] = t_ff_knee;
        msg.data[13] = knee_crank_kp_;
        msg.data[14] = knee_crank_kd_;

        cmd_pub_->publish(msg);

        // ================================================================
        // 更新相位
        // ================================================================
        phase_time_ += dt;
        if (phase_time_ >= current_period) {
            phase_time_ = 0.0;
            is_swing_ = !is_swing_;
            RCLCPP_INFO(this->get_logger(), "切换到 %s",
                         is_swing_ ? "摆动相" : "支撑相");
        }
    }

    // 参数
    double step_length_, step_height_, gait_period_;      // 步态几何与周期
    double stance_depth_, control_rate_, swing_ratio_;    // 站立深度/控制频率/摆动占比
    double hip_kp_, hip_kd_;                              // hip PD 增益
    double thigh_kp_, thigh_kd_;                          // thigh PD 增益
    double knee_crank_kp_, knee_crank_kd_;                // knee_crank PD 增益
    double hip_angle_;                                    // hip 固定角度 (rad)
    double gravity_torque_thigh_;                         // thigh 重力补偿力矩 (N·m)
    bool auto_zero_offsets_;                              // 自动对齐零位
    double thigh_offset_;                                 // 大腿零位偏置 (rad)
    double knee_offset_;                                  // 小腿零位偏置 (rad)

    // 步态状态
    double swing_period_, stance_period_;  // 摆动/支撑相持续时间
    double phase_time_;                    // 当前相位累计时间
    bool is_swing_;                        // 当前是否处于摆动相

    // 仿真时间追踪
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME}; // 上一次时间戳
    bool last_time_initialized_;                 // 是否已完成初始化

    // 低通滤波器
    LowPassFilter filter_theta2_{0.25};   // 大腿角度
    LowPassFilter filter_theta1_{0.25};   // 曲柄角度
    LowPassFilter filter_v_theta2_{0.25}; // 大腿角速度
    LowPassFilter filter_v_theta1_{0.25}; // 曲柄角速度

    // ROS 接口
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GaitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
