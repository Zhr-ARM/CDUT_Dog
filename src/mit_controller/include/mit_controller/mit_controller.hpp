#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mit_controller
{

class MitController : public controller_interface::ControllerInterface
{
public:
  MitController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Command5
  {
    double p{0.0};
    double v{0.0};
    double t_ff{0.0};
    double kp{0.0};
    double kd{0.0};
  };

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::vector<std::string> joints_;
  std::string mode_{"sim_effort_pd"};
  std::string command_topic_{"motor_cmd"};
  std::string feedback_topic_{"motor_feedback"};
  bool publish_feedback_{true};
  int command_timeout_ms_{200};
  double feedback_rate_hz_{20.0};
  std::vector<double> effort_limits_;

  realtime_tools::RealtimeBuffer<std::vector<Command5>> rt_command_buffer_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_feedback_time_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_raw_;
};

}  // namespace mit_controller
