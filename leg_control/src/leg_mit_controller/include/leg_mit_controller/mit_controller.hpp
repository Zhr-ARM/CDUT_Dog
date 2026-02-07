#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace leg_mit_controller
{

/**
 * @brief 针对 ros2_control 的 MIT 控制策略实现
 *
 * 实现了足式机器人常用的混合控制模式（位置、速度、力矩以及PD增益），
 * 通常被称为 MIT Cheetah 控制模式。
 *
 * 控制律：
 *   tau = kp * (p_des - p_curr) + kd * (v_des - v_curr) + t_ff
 *
 * 支持两种模式：
 * 1. "sim_effort_pd": PD计算在此控制器内部完成，输出为力矩（effort）。
 * 2. "passthrough": 所有5个命令直接传递给硬件接口（如果硬件支持）。
 */
class MitController : public controller_interface::ControllerInterface
{
public:
  MitController() = default;

  /**
   * @brief 初始化控制器并声明参数
   *
   * @return controller_interface::CallbackReturn::SUCCESS 如果初始化成功
   * @return controller_interface::CallbackReturn::ERROR 否则
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief 定义此控制器所需的命令接口
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief 定义此控制器所需的状态接口
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief 配置控制器（读取参数，设置缓冲区）
   *
   * @param previous_state 上一个生命周期状态
   * @return controller_interface::CallbackReturn
   */
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 激活控制器（订阅话题，设置发布者）
   *
   * @param previous_state 上一个生命周期状态
   * @return controller_interface::CallbackReturn
   */
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 停用控制器（停止通信，重置输出）
   *
   * @param previous_state 上一个生命周期状态
   * @return controller_interface::CallbackReturn
   */
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 主控制循环函数
   *
   * 执行控制计算并更新命令接口。
   *
   * @param time 当前时间
   * @param period 自上次更新以来的持续时间
   * @return controller_interface::return_type
   */
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /**
   * @brief 每个关节的5个基本MIT控制命令结构体
   */
  struct Command5
  {
    double p{0.0};    ///< 期望位置
    double v{0.0};    ///< 期望速度
    double t_ff{0.0}; ///< 前馈力矩
    double kp{0.0};   ///< 位置增益
    double kd{0.0};   ///< 速度增益
  };

  /**
   * @brief 接收电机命令的回调函数
   *
   * @param msg 包含所有关节的 [p, v, t_ff, kp, kd] 的 Float64MultiArray 消息
   */
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // 参数
  std::vector<std::string> joints_;     ///< 关节名称列表
  std::string mode_{"sim_effort_pd"};   ///< 控制模式

  std::string command_topic_{"motor_cmd"};          ///< 接收命令的话题名称
  std::string feedback_topic_{"motor_feedback"};    ///< 发布反馈的话题名称
  bool publish_feedback_{true};                     ///< 是否发布反馈
  int command_timeout_ms_{200};                     ///< 命令超时时间（毫秒）
  double feedback_rate_hz_{20.0};                   ///< 反馈发布频率

  // 实时缓冲区
  realtime_tools::RealtimeBuffer<std::vector<Command5>> rt_command_buffer_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_feedback_time_;

  // ROS 接口
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_raw_;
};

}  // namespace leg_mit_controller
