/**
 * @file mpc_gait_controller.cpp
 * @brief 轻量级步态控制器，支持 walk / run / jump 三种步态模式
 *
 * ============================================================================
 * 整体控制流程（每个控制周期执行一次）:
 * ============================================================================
 *
 *   ┌─────────────┐     ┌──────────────┐     ┌──────────────┐
 *   │ 步态状态机    │ --> │ 足端轨迹生成   │ --> │ 逆运动学(IK)  │
 *   │ (摆动/支撑)   │     │ (摆线/线性)    │     │ 足端→关节角   │
 *   └─────────────┘     └──────────────┘     └──────────────┘
 *                                                    │
 *                                                    v
 *   ┌─────────────┐     ┌──────────────┐     ┌──────────────┐
 *   │ 发布motor_cmd │ <-- │ PD+前馈力矩   │ <-- │ 雅可比速度映射 │
 *   │ (15维MIT命令) │     │ (重力补偿)     │     │ 足端速度→角速度│
 *   └─────────────┘     └──────────────┘     └──────────────┘
 *
 * 说明:
 *   - 当前实现采用 "足端轨迹规划 + 逆运动学 + PD控制 + 重力前馈" 方案
 *   - 输出接口与 leg_mit_controller 对齐，发布15维 /motor_cmd 话题
 *   - 支持通过 /gait_mode 话题实时切换 walk / run / jump 步态
 *   - 配置参数通过 config/mpc_gait_controller.yaml 加载
 *
 * 坐标系约定:
 *   - x轴: 水平方向（前进为正）
 *   - y轴: 竖直方向（向下为正，即足端在髋关节下方时 y > 0）
 *
 * 腿部结构示意:
 *          髋关节(Hip) ──── 固定角度，不参与步态
 *            │
 *          大腿(Thigh) ──── 长度 L1，关节角 theta2
 *            │
 *          膝关节(Knee)
 *            │
 *          小腿(Shank) ──── 长度 L2，关节角 theta1
 *            │
 *          足端(Foot)
 * ============================================================================
 */

/* ======================== 标准库头文件 ======================== */
#include <algorithm>   // std::clamp, std::max — 数值截断与取最大值
#include <array>       // std::array — 固定大小数组（预留）
#include <chrono>      // std::chrono — 定时器时间间隔
#include <cmath>       // std::sqrt, std::sin, std::cos, std::atan2, std::acos, M_PI
#include <memory>      // std::shared_ptr — 智能指针管理节点生命周期
#include <string>      // std::string — 步态模式字符串

/* ======================== ROS2 头文件 ======================== */
#include "rclcpp/rclcpp.hpp"                       // ROS2 C++ 客户端核心库
#include "std_msgs/msg/float64_multi_array.hpp"    // 发布 /motor_cmd 使用的消息类型
#include "std_msgs/msg/string.hpp"                 // 订阅 /gait_mode 使用的消息类型

/* ========================================================================== */
/*                        匿名命名空间：内部工具函数与数据结构                      */
/* ========================================================================== */
namespace {

/* -------------------- 物理/数学常量 -------------------- */
constexpr double PI = M_PI;         // 圆周率 π ≈ 3.14159
constexpr double GRAVITY = 9.81;    // 重力加速度 [m/s²]

/* -------------------- 数据结构定义 -------------------- */

/**
 * @brief 逆运动学求解结果
 *
 * 给定足端位置 (x, y)，通过逆运动学计算出两个关节角度：
 *   theta2 — 大腿关节角（髋关节处，大腿相对于竖直方向的偏转角）
 *   theta1 — 膝关节角（膝关节处，小腿相对于大腿的弯曲角）
 */
struct IKResult {
  double theta2;   // 大腿关节角 [rad]
  double theta1;   // 膝关节角 [rad]
};

/**
 * @brief 足端状态（位置 + 速度）
 *
 * 描述足端在笛卡尔空间中的运动状态，用于轨迹生成的输出。
 */
struct FootState {
  double x, y;     // 足端位置 [m]（x=水平, y=竖直抬高量）
  double vx, vy;   // 足端速度 [m/s]
};

/**
 * @brief 关节角速度
 *
 * 通过雅可比矩阵将足端速度映射到关节空间后的结果。
 */
struct JointVel {
  double v_theta2;  // 大腿关节角速度 [rad/s]
  double v_theta1;  // 膝关节角速度 [rad/s]
};

/**
 * @brief 步态参数集合
 *
 * 每种步态模式（walk/run/jump）各有一组独立的参数，控制足端轨迹的形状和节奏。
 *
 * 步态周期示意（以 swing_ratio=0.6 为例）:
 *   |<--- 摆动相 (60%) --->|<--- 支撑相 (40%) --->|
 *   |  足端抬起并前摆       |  足端着地并后拖        |
 */
struct GaitParams {
  double step_length;   // 步长 [m] — 一个摆动相中足端水平移动的总距离
  double step_height;   // 步高 [m] — 摆动相中足端抬起的最大高度
  double period;        // 步态周期 [s] — 一个完整的摆动+支撑周期时长
  double swing_ratio;   // 摆动相占比 [0~1] — 摆动相时间 / 总周期
  double stance_depth;  // 站立深度 [m] — 足端在支撑相时距髋关节的竖直距离（腿伸展程度）
};

/* ======================== 核心算法函数 ======================== */

/**
 * @brief 二连杆逆运动学求解 (Inverse Kinematics)
 *
 * 已知足端在髋关节坐标系下的位置 (x, y)，求解大腿关节角 theta2 和膝关节角 theta1。
 *
 * 几何原理示意:
 *
 *      髋关节 O ──────────────── x轴 (水平正方向)
 *           │\  phi1
 *           │ \
 *           │  \ L1 (大腿)
 *      y轴  │   \
 *     (向下) │    ● 膝关节
 *           │   / theta1 (膝弯角)
 *           │  / L2 (小腿)
 *           │ /
 *           ●  足端 P(x, y)
 *
 *   L = sqrt(x² + y²)  — 髋关节到足端的直线距离
 *
 *   步骤1: 用余弦定理求膝关节角 theta1
 *     cos(π - theta1) = (L1² + L2² - L²) / (2·L1·L2)
 *     => theta1 = π - arccos(...)
 *
 *   步骤2: 用余弦定理 + atan2 求大腿关节角 theta2
 *     phi1 = atan2(-x, y)  — 足端方向角（相对于竖直向下）
 *     phi2 = arccos((L² + L1² - L2²) / (2·L·L1))  — 大腿偏移量
 *     theta2 = phi1 + phi2
 *
 * @param x   足端水平位置 [m]（前方为正）
 * @param y   足端竖直位置 [m]（向下为正）
 * @param l1  大腿连杆长度 [m]
 * @param l2  小腿连杆长度 [m]
 * @return    IKResult{theta2, theta1} 两个关节角 [rad]
 */
IKResult inverse_kinematics(double x, double y, double l1, double l2) {
  // 计算髋关节到足端的距离 L
  double Lsq = x * x + y * y;        // L² = x² + y²
  double L = std::sqrt(Lsq);

  // 安全截断：确保 L 在可达范围内 (|l1-l2| < L < l1+l2)
  // 避免因数值误差导致 arccos 输入超出 [-1, 1] 范围
  L = std::clamp(L, std::abs(l1 - l2) + 0.001, l1 + l2 - 0.001);
  Lsq = L * L;  // 用截断后的 L 重新计算 L²

  // ---- 步骤1：求膝关节角 theta1 ----
  // 余弦定理: cos(π-theta1) = (l1² + l2² - L²) / (2·l1·l2)
  double ck = std::clamp((l1 * l1 + l2 * l2 - Lsq) / (2.0 * l1 * l2), -1.0, 1.0);
  double theta1 = PI - std::acos(ck);  // 膝关节弯曲角（完全伸直时为0）

  // ---- 步骤2：求大腿关节角 theta2 ----
  // phi1: 足端方向与竖直向下的夹角
  double phi1 = std::atan2(-x, y);
  // phi2: 大腿连杆与"髋-足端连线"的夹角（余弦定理求解）
  double cp = std::clamp((Lsq + l1 * l1 - l2 * l2) / (2.0 * L * l1), -1.0, 1.0);
  double phi2 = std::acos(cp);
  // theta2 = phi1 + phi2（大腿相对于竖直方向的总偏转角）
  // double theta2 = phi2 + phi1;
  double theta2 = phi1 - phi2;  // 修改为 phi1 - phi2，使站立位时 theta2 接近0（更符合直立姿态）

  return {theta2, theta1};
}

/**
 * @brief 摆线轨迹生成 (Cycloid Trajectory)
 *
 * 在摆动相中，使用摆线（cycloid）曲线生成足端的运动轨迹。
 * 摆线的优势在于：起止速度为零，加速度连续，运动非常平滑，不会产生冲击。
 *
 * 轨迹形状示意 (tau 从 0→1):
 *
 *   高度 y ▲
 *          │      ╭──╮          ← step_height (最高点, tau=0.5)
 *          │    ╱      ╲
 *          │  ╱          ╲
 *          │╱              ╲
 *   ───────●────────────────●──► 水平 x
 *        start_x          start_x + step_length
 *        (tau=0)                (tau=1)
 *
 * 数学公式:
 *   x(τ) = start_x + step_length × [τ - sin(2πτ)/(2π)]
 *   y(τ) = start_y + step_height × [1 - cos(2πτ)] / 2
 *
 * 速度（对时间求导，利用链式法则 dτ/dt = 1/period）:
 *   vx(τ) = step_length × [1 - cos(2πτ)] × (1/period)
 *   vy(τ) = step_height × π × sin(2πτ) × (1/period)
 *
 * @param tau          归一化相位 [0, 1]（0=摆动开始, 1=摆动结束）
 * @param step_length  步长 [m] — 水平方向总移动距离
 * @param step_height  步高 [m] — 足端抬起的最大高度
 * @param start_x      水平起始位置 [m]
 * @param start_y      竖直起始位置 [m]（通常为 0）
 * @param period       当前相的时间长度 [s]（用于计算速度）
 * @return             FootState{x, y, vx, vy} 足端位置和速度
 */
FootState trajectory_cycloid(double tau, double step_length, double step_height,
                             double start_x, double start_y, double period) {
  FootState s;
  // 摆线位置公式：水平方向使用修正摆线，竖直方向使用升余弦
  s.x = start_x + step_length * (tau - std::sin(2.0 * PI * tau) / (2.0 * PI));
  s.y = start_y + step_height * (1.0 - std::cos(2.0 * PI * tau)) / 2.0;

  // 摆线速度公式：对位置公式求时间导数
  if (period > 0.0) {
    double dtau = 1.0 / period;  // dτ/dt = 1/T（τ从0到1经过的时间为period秒）
    s.vx = step_length * (1.0 - std::cos(2.0 * PI * tau)) * dtau;
    s.vy = step_height * PI * std::sin(2.0 * PI * tau) * dtau;
  } else {
    s.vx = 0.0;
    s.vy = 0.0;
  }
  return s;
}

/**
 * @brief 通过解析雅可比矩阵计算关节角速度
 *
 * 核心思想：已知足端的笛卡尔速度 (vx, vy)，利用逆运动学的解析微分（雅可比矩阵），
 * 将足端速度映射到关节角速度空间。
 *
 * 数学原理:
 *   关节角 θ 是足端位置 (x, y) 的函数: θ = f(x, y)
 *   对时间求导（链式法则）:
 *     dθ/dt = (∂θ/∂x)·(dx/dt) + (∂θ/∂y)·(dy/dt)
 *           = (∂θ/∂x)·vx + (∂θ/∂y)·vy
 *
 *   即雅可比矩阵 J:
 *     [v_theta2]   [∂θ2/∂x  ∂θ2/∂y] [vx]
 *     [v_theta1] = [∂θ1/∂x  ∂θ1/∂y] [vy]
 *
 *   各偏导数通过对 IK 公式逐项微分获得（使用中间变量 L, phi1, phi2 的导数链）。
 *
 * @param x   足端当前水平位置 [m]
 * @param y   足端当前竖直位置 [m]
 * @param vx  足端水平速度 [m/s]
 * @param vy  足端竖直速度 [m/s]
 * @param l1  大腿连杆长度 [m]
 * @param l2  小腿连杆长度 [m]
 * @return    JointVel{v_theta2, v_theta1} 关节角速度 [rad/s]
 */
JointVel compute_joint_velocity(double x, double y, double vx, double vy, double l1, double l2) {
  // ---- 计算中间量 L（髋到足端距离）----
  double Lsq = x * x + y * y;
  double L = std::sqrt(Lsq);
  if (L < 1e-3) return {0.0, 0.0};  // 奇异点保护：足端太靠近髋关节

  // 安全截断，与 IK 函数保持一致
  L = std::clamp(L, std::abs(l1 - l2) + 0.001, l1 + l2 - 0.001);
  Lsq = L * L;

  // ---- 计算 dtheta1/dL（膝关节角对 L 的偏导数）----
  // theta1 = π - arccos(ck)，其中 ck = (l1²+l2²-L²)/(2·l1·l2)
  double ck = std::clamp((l1 * l1 + l2 * l2 - Lsq) / (2.0 * l1 * l2), -0.999, 0.999);
  double sk = std::max(std::sqrt(1.0 - ck * ck), 1e-6);  // sin(arccos(ck))，避免除零
  double dtheta1_dL = L / (l1 * l2 * sk);  // 链式法则: dθ1/dL = L/(l1·l2·sin(θ_knee))

  // ---- 计算 dphi2/dL（大腿偏移角对 L 的偏导数）----
  // phi2 = arccos(cp)，其中 cp = (L²+l1²-l2²)/(2·L·l1)
  double cp = std::clamp((Lsq + l1 * l1 - l2 * l2) / (2.0 * L * l1), -0.999, 0.999);
  double sp = std::max(std::sqrt(1.0 - cp * cp), 1e-6);  // sin(arccos(cp))
  double dphi2_dL = -(Lsq - l1 * l1 + l2 * l2) / (2.0 * Lsq * l1 * sp);

  // ---- 计算 L 和 phi1 对 (x, y) 的偏导数 ----
  // L = sqrt(x²+y²)  =>  ∂L/∂x = x/L,  ∂L/∂y = y/L
  double dL_dx = x / L;
  double dL_dy = y / L;
  // phi1 = atan2(-x, y)  =>  ∂phi1/∂x = -y/L²,  ∂phi1/∂y = -x/L²
  double dphi1_dx = -y / Lsq;
  double dphi1_dy = -x / Lsq;

  // ---- 组合偏导数：计算雅可比矩阵元素 ----
  // theta2 = phi1 + phi2(L)  =>  ∂θ2/∂x = ∂phi1/∂x + (∂phi2/∂L)·(∂L/∂x)
  double dtheta2_dx = dphi1_dx + dphi2_dL * dL_dx;
  double dtheta2_dy = dphi1_dy + dphi2_dL * dL_dy;
  // theta1 只依赖于 L  =>  ∂θ1/∂x = (∂θ1/∂L)·(∂L/∂x)
  double dtheta1_dx = dtheta1_dL * dL_dx;
  double dtheta1_dy = dtheta1_dL * dL_dy;

  // ---- 雅可比映射：足端速度 → 关节角速度 ----
  // v_theta = J · [vx, vy]^T
  return {dtheta2_dx * vx + dtheta2_dy * vy, dtheta1_dx * vx + dtheta1_dy * vy};
}

/**
 * @brief 一阶低通滤波器 (First-order Low-Pass Filter)
 *
 * 用于平滑关节角度和角速度命令，消除离散计算带来的高频抖动。
 *
 * 滤波公式（指数移动平均 EMA）:
 *   y[k] = α · x[k] + (1 - α) · y[k-1]
 *
 * 其中:
 *   x[k] — 当前输入值（原始信号）
 *   y[k] — 当前输出值（滤波后信号）
 *   α    — 滤波系数，范围 [0.01, 1.0]
 *            α 越大 → 响应越快，但越不平滑（α=1 时无滤波）
 *            α 越小 → 越平滑，但延迟越大（α→0 时几乎不更新）
 *            推荐值: 0.2~0.4
 *
 * 使用示例:
 *   LowPassFilter f(0.25);          // 创建滤波器，α=0.25
 *   double smoothed = f.filter(raw); // 每个周期输入原始值，输出滤波值
 */
class LowPassFilter {
public:
  /** @brief 构造函数 @param alpha 滤波系数，默认0.3 */
  explicit LowPassFilter(double alpha = 0.3) : alpha_(alpha) {}

  /** @brief 动态修改滤波系数（会被截断到 [0.01, 1.0]） */
  void set_alpha(double alpha) { alpha_ = std::clamp(alpha, 0.01, 1.0); }

  /**
   * @brief 执行一次滤波
   * @param input 当前原始输入值
   * @return 滤波后的平滑值
   */
  double filter(double input) {
    if (!initialized_) {
      // 首次调用时，直接使用输入值初始化（避免从0开始的跳变）
      value_ = input;
      initialized_ = true;
    } else {
      // EMA 滤波: 新值 = α×输入 + (1-α)×旧值
      value_ = alpha_ * input + (1.0 - alpha_) * value_;
    }
    return value_;
  }

private:
  double alpha_;                // 滤波系数 [0.01, 1.0]
  bool initialized_{false};    // 是否已初始化（首次调用标志）
  double value_{0.0};          // 上一次滤波输出值（内部状态）
};

}  // namespace（匿名命名空间结束）

/* ========================================================================== */
/*                         MPC 步态控制器 ROS2 节点                            */
/* ========================================================================== */

/**
 * @class MpcGaitController
 * @brief 步态控制器的核心 ROS2 节点类
 *
 * 继承自 rclcpp::Node，实现了完整的步态控制流水线:
 *   1. 从 yaml 参数文件加载机器人物理参数和步态配置
 *   2. 维护步态状态机（摆动相 ↔ 支撑相 交替切换）
 *   3. 按设定频率（默认200Hz）执行控制循环
 *   4. 发布15维 MIT 格式电机命令到 /motor_cmd 话题
 *   5. 接收 /gait_mode 话题实现步态在线切换
 *
 * ROS2 接口:
 *   发布: /motor_cmd  (std_msgs/Float64MultiArray, 15维)
 *   订阅: /gait_mode  (std_msgs/String, "walk"/"run"/"jump")
 */
class MpcGaitController : public rclcpp::Node {
public:
  /**
   * @brief 构造函数 — 初始化所有参数、话题和定时器
   *
   * 执行流程:
   *   1) 声明并读取所有 ROS2 参数（declare_parameter + get_parameter）
   *   2) 加载三种步态参数集（walk/run/jump）
   *   3) 计算零位偏置（使足端在站立位时关节角为0）
   *   4) 创建发布者、订阅者和控制定时器
   *   5) 打印启动信息
   */
  MpcGaitController() : Node("mpc_gait_controller") {

    /* ========== 第1步：声明 ROS2 参数（设置默认值） ========== */
    // 声明的参数可通过 yaml 文件或命令行覆盖

    // --- 机器人物理参数 ---
    declare_parameter("link_length_thigh", 0.200);      // 大腿连杆长度 [m]
    declare_parameter("link_length_shank", 0.243);       // 小腿连杆长度 [m]
    declare_parameter("leg_mass", 1.05);                 // 腿部等效质量 [kg]
    declare_parameter("leg_com_ratio", 0.45);            // 质心在大腿上的位置比例
    declare_parameter("enable_knee_gravity_comp", true);  // 是否启用膝关节重力补偿

    // --- 控制频率 ---
    declare_parameter("control_rate", 200.0);            // 控制循环频率 [Hz]

    // --- 步态模式与参数 ---
    declare_parameter("gait_mode", "walk");              // 默认步态模式
    // walk 步态参数
    declare_parameter("walk.step_length", 0.08);         // 步长 [m]
    declare_parameter("walk.step_height", 0.05);         // 步高 [m]
    declare_parameter("walk.period", 0.80);              // 周期 [s]
    declare_parameter("walk.swing_ratio", 0.60);         // 摆动相占比
    declare_parameter("walk.stance_depth", 0.29);        // 站立深度 [m]
    // run 步态参数
    declare_parameter("run.step_length", 0.12);
    declare_parameter("run.step_height", 0.06);
    declare_parameter("run.period", 0.50);
    declare_parameter("run.swing_ratio", 0.60);
    declare_parameter("run.stance_depth", 0.28);
    // jump 步态参数
    declare_parameter("jump.step_length", 0.00);         // 跳跃无水平位移
    declare_parameter("jump.step_height", 0.12);         // 跳跃抬腿高度较大
    declare_parameter("jump.period", 0.65);
    declare_parameter("jump.swing_ratio", 0.45);
    declare_parameter("jump.stance_depth", 0.30);

    // --- PD 控制增益（MIT 控制模式参数）---
    declare_parameter("hip_angle", 3.21);                // 髋关节固定角度 [rad]
    declare_parameter("hip_kp", 3.0);                    // 髋关节比例增益
    declare_parameter("hip_kd", 0.8);                    // 髋关节微分增益
    declare_parameter("thigh_kp", 12.0);                 // 大腿关节比例增益
    declare_parameter("thigh_kd", 4.0);                  // 大腿关节微分增益
    declare_parameter("knee_crank_kp", 12.0);            // 膝曲柄关节比例增益
    declare_parameter("knee_crank_kd", 4.0);             // 膝曲柄关节微分增益

    // --- 零位偏置与滤波 ---
    declare_parameter("auto_zero_offsets", true);        // 是否自动计算零位偏置
    declare_parameter("thigh_offset", 0.0);              // 大腿角度偏置 [rad]
    declare_parameter("knee_offset", 0.0);               // 膝关节角度偏置 [rad]
    declare_parameter("filter_alpha", 0.25);             // 低通滤波系数

    /* ========== 第2步：读取参数值到成员变量 ========== */

    // 读取机器人物理参数
    L1_ = get_parameter("link_length_thigh").as_double();
    L2_ = get_parameter("link_length_shank").as_double();
    leg_mass_ = get_parameter("leg_mass").as_double();
    com_ratio_ = get_parameter("leg_com_ratio").as_double();
    knee_gc_ = get_parameter("enable_knee_gravity_comp").as_bool();

    // 读取控制频率
    ctrl_rate_ = get_parameter("control_rate").as_double();

    // 读取 PD 增益
    hip_angle_ = get_parameter("hip_angle").as_double();
    hip_kp_ = get_parameter("hip_kp").as_double();
    hip_kd_ = get_parameter("hip_kd").as_double();
    thigh_kp_ = get_parameter("thigh_kp").as_double();
    thigh_kd_ = get_parameter("thigh_kd").as_double();
    knee_kp_ = get_parameter("knee_crank_kp").as_double();
    knee_kd_ = get_parameter("knee_crank_kd").as_double();

    // 读取零位与滤波参数
    auto_zero_offsets_ = get_parameter("auto_zero_offsets").as_bool();
    thigh_offset_ = get_parameter("thigh_offset").as_double();
    knee_offset_ = get_parameter("knee_offset").as_double();

    // 设置四个低通滤波器的系数（角度×2 + 角速度×2）
    double filter_alpha = get_parameter("filter_alpha").as_double();
    filter_theta2_.set_alpha(filter_alpha);
    filter_theta1_.set_alpha(filter_alpha);
    filter_v_theta2_.set_alpha(filter_alpha);
    filter_v_theta1_.set_alpha(filter_alpha);

    /* ========== 第3步：加载步态参数并设置初始步态 ========== */
    walk_ = read_gait("walk");    // 从参数服务器读取 walk 步态
    run_ = read_gait("run");      // 从参数服务器读取 run 步态
    jump_ = read_gait("jump");    // 从参数服务器读取 jump 步态
    set_gait(get_parameter("gait_mode").as_string());  // 激活默认步态

    /* ========== 第4步：计算零位偏置 ========== */
    // 目的：让足端在站立位（x=0, y=stance_depth）时，关节命令角度为 0
    // 原理：先求站立位的 IK 角度，然后取负值作为偏置
    if (auto_zero_offsets_) {
      IKResult ik_neutral = inverse_kinematics(0.0, ag_.stance_depth, L1_, L2_);
      thigh_offset_ = -ik_neutral.theta2;   // 大腿偏置 = -站立位角度
      knee_offset_ = -ik_neutral.theta1;    // 膝关节偏置 = -站立位角度
    }

    /* ========== 第5步：创建 ROS2 通信接口 ========== */

    // 发布者：向 /motor_cmd 发布15维电机命令
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/motor_cmd", 10);

    // 订阅者：从 /gait_mode 接收步态切换指令
    gait_sub_ = create_subscription<std_msgs::msg::String>(
        "/gait_mode", 10,
        std::bind(&MpcGaitController::gait_callback, this, std::placeholders::_1));

    // 定时器：按 control_rate 频率周期性调用 control_loop()
    double dt = 1.0 / std::max(1.0, ctrl_rate_);  // 防止除零
    timer_ = create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&MpcGaitController::control_loop, this));

    /* ========== 第6步：打印启动信息 ========== */
    RCLCPP_INFO(get_logger(), "============================================================");
    RCLCPP_INFO(get_logger(), "mpc_gait_controller 已启动 (轨迹+IK 控制)");
    RCLCPP_INFO(get_logger(), "  gait=%s, 控制频率=%.1fHz", current_mode_.c_str(), ctrl_rate_);
    RCLCPP_INFO(get_logger(), "  站立深度=%.3f, 步长=%.3f, 步高=%.3f", ag_.stance_depth, ag_.step_length, ag_.step_height);
    RCLCPP_INFO(get_logger(), "  PD: thigh(Kp=%.1f,Kd=%.1f) knee(Kp=%.1f,Kd=%.1f)",
                thigh_kp_, thigh_kd_, knee_kp_, knee_kd_);
    RCLCPP_INFO(get_logger(), "  零位偏置: thigh=%.3f, knee=%.3f (auto=%s)",
                thigh_offset_, knee_offset_, auto_zero_offsets_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "============================================================");
  }

private:
  /* ======================== 私有方法 ======================== */

  /**
   * @brief 从 ROS2 参数服务器读取一组步态参数
   * @param prefix 步态名前缀（"walk" / "run" / "jump"）
   * @return 填充好的 GaitParams 结构体
   *
   * 例: read_gait("walk") 会读取 walk.step_length, walk.step_height, ...
   */
  GaitParams read_gait(const std::string &prefix) {
    GaitParams g;
    g.step_length = get_parameter(prefix + ".step_length").as_double();
    g.step_height = get_parameter(prefix + ".step_height").as_double();
    g.period = get_parameter(prefix + ".period").as_double();
    g.swing_ratio = get_parameter(prefix + ".swing_ratio").as_double();
    g.stance_depth = get_parameter(prefix + ".stance_depth").as_double();
    return g;
  }

  /**
   * @brief 切换步态模式
   * @param mode 目标步态模式字符串（"walk" / "run" / "jump"）
   *
   * 切换时会:
   *   1. 将对应的步态参数复制到当前活动参数 ag_
   *   2. 重新计算摆动/支撑相的时间长度
   *   3. 重置相位状态机（从支撑相中段开始，避免切换瞬间的跳变）
   *   4. 重新计算零位偏置（因为不同步态的 stance_depth 不同）
   */
  void set_gait(const std::string &mode) {
    // 根据模式名选择对应的参数集
    if (mode == "walk") {
      ag_ = walk_;
      current_mode_ = mode;
    } else if (mode == "run") {
      ag_ = run_;
      current_mode_ = mode;
    } else if (mode == "jump") {
      ag_ = jump_;
      current_mode_ = mode;
    } else {
      RCLCPP_WARN(get_logger(), "未知步态模式: %s", mode.c_str());
      return;
    }

    // 计算摆动相和支撑相的各自时长
    swing_period_ = ag_.period * ag_.swing_ratio;             // 摆动相时长
    stance_period_ = ag_.period * (1.0 - ag_.swing_ratio);    // 支撑相时长

    // 重置状态机：从支撑相的中段开始（避免切换瞬间的位置跳变）
    is_swing_ = false;
    phase_time_ = 0.5 * stance_period_;   // 从支撑相50%处开始
    last_time_initialized_ = false;       // 重置时间戳

    // 重新计算零位偏置（因为不同步态的站立深度可能不同）
    if (auto_zero_offsets_) {
      IKResult ik_neutral = inverse_kinematics(0.0, ag_.stance_depth, L1_, L2_);
      thigh_offset_ = -ik_neutral.theta2;
      knee_offset_ = -ik_neutral.theta1;
    }

    RCLCPP_INFO(get_logger(), "步态模式切换: %s", current_mode_.c_str());
  }

  /**
   * @brief /gait_mode 话题的回调函数
   * @param msg 接收到的步态模式字符串消息
   *
   * 用法: ros2 topic pub /gait_mode std_msgs/msg/String "{data: run}" -1
   */
  void gait_callback(const std_msgs::msg::String::SharedPtr msg) {
    set_gait(msg->data);
  }

  /**
   * @brief 主控制循环 — 由定时器按 control_rate 频率周期性调用
   *
   * 每次调用执行以下流水线:
   *   1. 计算时间步长 dt
   *   2. 根据步态状态机，计算当前归一化相位 tau
   *   3. 生成足端参考轨迹（摆动相用摆线，支撑相用线性）
   *   4. 逆运动学：足端位置 → 关节角度
   *   5. 雅可比映射：足端速度 → 关节角速度
   *   6. 低通滤波平滑命令
   *   7. 计算重力补偿前馈力矩
   *   8. 组装并发布15维 MIT 电机命令
   *   9. 更新相位时间，必要时切换摆动/支撑相
   */
  void control_loop() {
    /* ---- 步骤1：计算时间步长 dt ---- */
    rclcpp::Time t = this->now();
    double dt = 0.0;
    if (!last_time_initialized_) {
      // 首次调用：仅记录时间戳，dt 保持为 0
      last_time_ = t;
      last_time_initialized_ = true;
    } else {
      dt = (t - last_time_).seconds();
      last_time_ = t;
    }
    // 安全保护：防止时间倒流或异常大的 dt（例如仿真暂停后恢复）
    if (dt < 0.0) dt = 0.0;
    if (dt > 0.1) dt = 0.1;   // 最大 100ms，避免相位跳变

    /* ---- 步骤2：计算归一化相位 tau ∈ [0, 1] ---- */
    // tau 表示当前相（摆动或支撑）已完成的比例
    double current_period = is_swing_ ? swing_period_ : stance_period_;
    double tau = (current_period > 0.0) ? (phase_time_ / current_period) : 0.0;
    tau = std::clamp(tau, 0.0, 1.0);

    /* ---- 步骤3：生成足端参考轨迹 ---- */
    double foot_x, foot_y, vx_foot, vy_foot;
    if (is_swing_) {
      // === 摆动相：使用摆线轨迹，足端从后方抬起摆到前方 ===
      //
      //   足端水平范围: [-step_length/2, +step_length/2]
      //   足端竖直范围: [stance_depth, stance_depth - step_height]
      //
      double start_x = -ag_.step_length / 2.0;  // 摆动起点（后方）
      FootState fs = trajectory_cycloid(tau, ag_.step_length, ag_.step_height,
                                        start_x, 0.0, current_period);
      foot_x = fs.x;                     // 水平位置直接使用
      foot_y = ag_.stance_depth - fs.y;  // 竖直：站立深度 - 抬腿高度（y向下为正）
      vx_foot = fs.vx;                   // 水平速度
      vy_foot = -fs.vy;                  // 竖直速度取反（轨迹 y 向上，坐标系 y 向下）
    } else {
      // === 支撑相：足端线性从前方滑到后方（模拟地面支撑） ===
      //
      //   足端从 +step_length/2 线性移动到 -step_length/2
      //   竖直保持在 stance_depth（贴地）
      //
      double front_x = ag_.step_length / 2.0;    // 前方极限
      double back_x = -ag_.step_length / 2.0;    // 后方极限
      foot_x = front_x + (back_x - front_x) * tau;  // 线性插值
      foot_y = ag_.stance_depth;                      // 竖直位置不变
      vx_foot = (current_period > 0.0) ? ((back_x - front_x) / current_period) : 0.0;
      vy_foot = 0.0;   // 支撑相无竖直运动
    }

    /* ---- 步骤4 & 5：逆运动学 + 雅可比速度映射 ---- */
    IKResult ik = inverse_kinematics(foot_x, foot_y, L1_, L2_);           // 足端位置 → 关节角
    JointVel jv = compute_joint_velocity(foot_x, foot_y, vx_foot, vy_foot, L1_, L2_);  // 足端速度 → 角速度

    /* ---- 步骤6：低通滤波 + 叠加零位偏置 ---- */
    double cmd_theta2 = filter_theta2_.filter(ik.theta2 + thigh_offset_);   // 大腿目标角度（滤波后）
    double cmd_theta1 = filter_theta1_.filter(ik.theta1 + knee_offset_);    // 膝关节目标角度（滤波后）
    double cmd_v_theta2 = filter_v_theta2_.filter(jv.v_theta2);             // 大腿目标角速度（滤波后）
    double cmd_v_theta1 = filter_v_theta1_.filter(jv.v_theta1);             // 膝关节目标角速度（滤波后）

    /* ---- 步骤7：重力补偿前馈力矩 ---- */
    // 大腿前馈: τ = m·g·(L1·com_ratio)·cos(θ2)
    // 物理含义: 补偿腿部质心因重力产生的力矩，减轻 PD 控制器的负担
    double t_ff_thigh = leg_mass_ * GRAVITY * (L1_ * com_ratio_) * std::cos(cmd_theta2);
    double t_ff_knee = 0.0;
    if (knee_gc_) {
      // 膝关节前馈: τ = m·g·(L2·0.5)·cos(θ2-θ1)
      t_ff_knee = leg_mass_ * GRAVITY * (L2_ * 0.5) * std::cos(cmd_theta2 - cmd_theta1);
    }

    /* ---- 步骤8：组装15维 MIT 电机命令并发布 ---- */
    // MIT 命令格式（每个关节5个值）:
    //   [目标角度, 目标角速度, 前馈力矩, Kp, Kd]
    //
    // 电机端执行的力矩公式:
    //   τ_out = Kp·(q_target - q_actual) + Kd·(qd_target - qd_actual) + τ_ff
    //
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(15);

    // 关节1: 髋关节 (Hip) — 固定角度，不参与步态运动
    msg.data[0] = hip_angle_;      // 目标角度 [rad]（固定值）
    msg.data[1] = 0.0;             // 目标角速度 [rad/s]（0，保持静止）
    msg.data[2] = 0.0;             // 前馈力矩 [Nm]（无前馈）
    msg.data[3] = hip_kp_;         // 比例增益 Kp
    msg.data[4] = hip_kd_;         // 微分增益 Kd

    // 关节2: 大腿关节 (Thigh) — 控制大腿摆动
    msg.data[5] = cmd_theta2;      // 目标角度 [rad]
    msg.data[6] = cmd_v_theta2;    // 目标角速度 [rad/s]
    msg.data[7] = t_ff_thigh;      // 重力补偿前馈力矩 [Nm]
    msg.data[8] = thigh_kp_;       // 比例增益 Kp
    msg.data[9] = thigh_kd_;       // 微分增益 Kd

    // 关节3: 膝曲柄关节 (Knee Crank) — 控制膝关节弯曲
    msg.data[10] = cmd_theta1;     // 目标角度 [rad]
    msg.data[11] = cmd_v_theta1;   // 目标角速度 [rad/s]
    msg.data[12] = t_ff_knee;      // 重力补偿前馈力矩 [Nm]
    msg.data[13] = knee_kp_;       // 比例增益 Kp
    msg.data[14] = knee_kd_;       // 微分增益 Kd

    cmd_pub_->publish(msg);  // 发布到 /motor_cmd 话题

    /* ---- 步骤9：更新相位时间 & 状态机切换 ---- */
    phase_time_ += dt;  // 累加相位时间
    // 当相位时间超过当前相的周期时，切换到下一相
    if (phase_time_ >= current_period && current_period > 0.0) {
      phase_time_ = 0.0;          // 重置相位时间
      is_swing_ = !is_swing_;     // 摆动 ↔ 支撑 切换
    }
  }

  /* ======================== 成员变量 ======================== */

  // --- 机器人物理参数 ---
  double L1_{0.200};           // 大腿连杆长度 [m]
  double L2_{0.243};           // 小腿连杆长度 [m]
  double leg_mass_{1.05};      // 腿部等效质量 [kg]（用于重力补偿）
  double com_ratio_{0.45};     // 质心在大腿连杆上的位置比例 [0~1]
  bool knee_gc_{true};         // 是否启用膝关节重力补偿前馈

  // --- 控制参数 ---
  double ctrl_rate_{200.0};    // 控制循环频率 [Hz]

  // --- PD 增益（MIT 控制模式）---
  double hip_angle_{3.21};     // 髋关节固定目标角度 [rad]
  double hip_kp_{3.0};         // 髋关节比例增益
  double hip_kd_{0.8};         // 髋关节微分增益
  double thigh_kp_{12.0};      // 大腿关节比例增益
  double thigh_kd_{4.0};       // 大腿关节微分增益
  double knee_kp_{12.0};       // 膝曲柄关节比例增益
  double knee_kd_{4.0};        // 膝曲柄关节微分增益

  // --- 零位偏置 ---
  bool auto_zero_offsets_{true};   // 是否自动计算零位（true=站立位为零）
  double thigh_offset_{0.0};      // 大腿角度偏置 [rad]
  double knee_offset_{0.0};       // 膝关节角度偏置 [rad]

  // --- 步态参数存储 ---
  GaitParams walk_{};              // walk 步态参数集
  GaitParams run_{};               // run 步态参数集
  GaitParams jump_{};              // jump 步态参数集
  GaitParams ag_{};                // 当前激活的步态参数（active gait）
  std::string current_mode_{"walk"};  // 当前步态模式名称

  // --- 步态状态机 ---
  double swing_period_{0.0};       // 摆动相时长 [s] = period × swing_ratio
  double stance_period_{0.0};      // 支撑相时长 [s] = period × (1 - swing_ratio)
  double phase_time_{0.0};         // 当前相内已经过的时间 [s]
  bool is_swing_{false};           // 当前是否处于摆动相（false=支撑相）

  // --- 时间管理 ---
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};  // 上一次控制循环的时间戳
  bool last_time_initialized_{false};             // 时间戳是否已初始化

  // --- 低通滤波器（平滑关节命令，减少离散抖动）---
  LowPassFilter filter_theta2_{0.25};      // 大腿角度滤波器
  LowPassFilter filter_theta1_{0.25};      // 膝关节角度滤波器
  LowPassFilter filter_v_theta2_{0.25};    // 大腿角速度滤波器
  LowPassFilter filter_v_theta1_{0.25};    // 膝关节角速度滤波器

  // --- ROS2 通信接口 ---
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;   // 电机命令发布者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_sub_;         // 步态模式订阅者
  rclcpp::TimerBase::SharedPtr timer_;    // 控制循环定时器
};

/* ========================================================================== */
/*                                程序入口                                     */
/* ========================================================================== */

/**
 * @brief 主函数 — ROS2 节点的启动入口
 *
 * 执行流程:
 *   1. rclcpp::init()     — 初始化 ROS2 运行时环境
 *   2. 创建 MpcGaitController 节点实例（构造函数中完成所有初始化）
 *   3. rclcpp::spin()     — 进入事件循环，持续执行定时器回调和话题回调
 *   4. rclcpp::shutdown() — 收到终止信号后清理资源并退出
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);                                  // 初始化 ROS2
  auto node = std::make_shared<MpcGaitController>();          // 创建控制器节点
  rclcpp::spin(node);                                        // 进入事件循环（阻塞）
  rclcpp::shutdown();                                        // 清理退出
  return 0;
}
