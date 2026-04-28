#ifndef DEEP_MOTOR_ROS_DEEP_MOTOR_NODE_HPP_
#define DEEP_MOTOR_ROS_DEEP_MOTOR_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "deep_motor_ros/can_interface_manager.hpp"
#include "deep_motor_ros/motor_types.hpp"

namespace deep_motor_ros
{

class DeepMotorNode : public rclcpp::Node
{
public:
  DeepMotorNode();
  ~DeepMotorNode() override;

private:
  void declare_parameters();
  void load_parameters();
  void setup_can_interfaces();
  void setup_ros_interfaces();

  std::vector<int64_t> expand_motor_ids(
    const std::vector<int64_t> & configured_motor_ids) const;
  std::vector<std::string> resolve_joint_names() const;

  void initialize_buses();
  bool enable_and_verify_motor(BusContext & bus, MotorContext & motor);
  bool set_home_for_motor(BusContext & bus, MotorContext & motor);
  void normalize_shutdown_positions();
  void move_to_shutdown_pose_if_requested();

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void control_loop();
  void publish_feedback();
  void log_status_summary() const;

  std::vector<BusContext> buses_;
  std::vector<std::string> can_interfaces_;
  std::vector<int64_t> motor_ids_;
  std::vector<std::string> joint_names_;
  size_t total_motor_count_{0};
  int motors_per_can_{3};
  std::string command_topic_;
  std::string feedback_topic_;
  std::string joint_state_topic_;
  bool configure_can_{false};
  int can_bitrate_{1000000};
  int can_txqueuelen_{1000};
  double can_interface_wait_sec_{5.0};
  int can_configure_retries_{3};
  double control_rate_hz_{400.0};
  double status_log_period_sec_{5.0};
  bool auto_set_home_on_startup_{false};
  bool require_home_on_startup_{false};
  int home_command_retries_{3};
  double home_command_delay_sec_{0.05};
  bool shutdown_move_on_exit_{false};
  std::vector<double> shutdown_positions_;
  double shutdown_duration_sec_{1.5};
  double shutdown_rate_hz_{50.0};
  int shutdown_command_timeout_ms_{5};
  double shutdown_kp_{8.0};
  double shutdown_kd_{1.0};
  double shutdown_torque_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace deep_motor_ros

#endif  // DEEP_MOTOR_ROS_DEEP_MOTOR_NODE_HPP_
