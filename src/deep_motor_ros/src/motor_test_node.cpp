#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr size_t kValuesPerJoint = 5;
constexpr double kTwoPi = 6.28318530717958647692;

}  // namespace

class MotorTestNode : public rclcpp::Node
{
public:
  MotorTestNode()
  : Node("motor_test_node")
  {
    declare_parameter<std::string>("command_topic", "/motor_cmd");
    declare_parameter<int>("motor_count", 12);
    declare_parameter<int>("active_motor_index", 0);
    declare_parameter<double>("publish_rate_hz", 100.0);
    declare_parameter<double>("start_delay_sec", 1.0);
    declare_parameter<double>("run_duration_sec", 0.0);
    declare_parameter<double>("center_position", 0.0);
    declare_parameter<double>("amplitude", 0.15);
    declare_parameter<double>("frequency_hz", 0.25);
    declare_parameter<double>("kp", 6.0);
    declare_parameter<double>("kd", 0.3);
    declare_parameter<double>("hold_kd", 0.8);
    declare_parameter<double>("torque_ff", 0.0);

    command_topic_ = get_parameter("command_topic").as_string();
    motor_count_ = std::max(1, static_cast<int>(get_parameter("motor_count").as_int()));
    active_motor_index_ = std::clamp(
      static_cast<int>(get_parameter("active_motor_index").as_int()), 0, motor_count_ - 1);
    publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    start_delay_sec_ = std::max(0.0, get_parameter("start_delay_sec").as_double());
    run_duration_sec_ = std::max(0.0, get_parameter("run_duration_sec").as_double());
    center_position_ = get_parameter("center_position").as_double();
    amplitude_ = std::abs(get_parameter("amplitude").as_double());
    frequency_hz_ = std::max(0.0, get_parameter("frequency_hz").as_double());
    kp_ = std::max(0.0, get_parameter("kp").as_double());
    kd_ = std::max(0.0, get_parameter("kd").as_double());
    hold_kd_ = std::max(0.0, get_parameter("hold_kd").as_double());
    torque_ff_ = get_parameter("torque_ff").as_double();

    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);

    start_time_ = now();

    const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(publish_period, std::bind(&MotorTestNode::publish_command, this));
    log_timer_ = create_wall_timer(1s, std::bind(&MotorTestNode::log_target_state, this));

    RCLCPP_WARN(
      get_logger(),
      "Motor test node is active. motor_count=%d active_motor_index=%d amplitude=%.3f rad "
      "frequency=%.3f Hz kp=%.2f kd=%.2f",
      motor_count_, active_motor_index_, amplitude_, frequency_hz_, kp_, kd_);
  }

private:
  void publish_command()
  {
    const double elapsed_sec = (now() - start_time_).seconds();
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.assign(static_cast<size_t>(motor_count_) * kValuesPerJoint, 0.0);

    for (int motor_index = 0; motor_index < motor_count_; ++motor_index)
    {
      const size_t base = static_cast<size_t>(motor_index) * kValuesPerJoint;
      msg.data[base + 0] = center_position_;
      msg.data[base + 1] = 0.0;
      msg.data[base + 2] = 0.0;
      msg.data[base + 3] = 0.0;
      msg.data[base + 4] = hold_kd_;
    }

    const size_t active_base = static_cast<size_t>(active_motor_index_) * kValuesPerJoint;
    const bool motion_enabled =
      elapsed_sec >= start_delay_sec_ &&
      (run_duration_sec_ <= 0.0 || elapsed_sec <= start_delay_sec_ + run_duration_sec_);

    double position_target = center_position_;
    double velocity_target = 0.0;
    if (motion_enabled)
    {
      const double motion_time = elapsed_sec - start_delay_sec_;
      position_target = center_position_ + amplitude_ * std::sin(kTwoPi * frequency_hz_ * motion_time);
      velocity_target = amplitude_ * kTwoPi * frequency_hz_ *
        std::cos(kTwoPi * frequency_hz_ * motion_time);
    }
    else if (run_duration_sec_ > 0.0 && !motion_complete_logged_ &&
      elapsed_sec > start_delay_sec_ + run_duration_sec_)
    {
      motion_complete_logged_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Motor test motion window finished. Holding motor %d at %.3f rad with damping only.",
        active_motor_index_, center_position_);
    }

    msg.data[active_base + 0] = position_target;
    msg.data[active_base + 1] = velocity_target;
    msg.data[active_base + 2] = torque_ff_;
    msg.data[active_base + 3] = kp_;
    msg.data[active_base + 4] = kd_;

    last_target_position_ = position_target;
    last_target_velocity_ = velocity_target;
    cmd_pub_->publish(msg);
  }

  void log_target_state() const
  {
    const double elapsed_sec = (now() - start_time_).seconds();
    RCLCPP_INFO(
      get_logger(),
      "[MotorTest] t=%.2fs topic=%s motor=%d target_pos=%.3f target_vel=%.3f",
      elapsed_sec, command_topic_.c_str(), active_motor_index_, last_target_position_,
      last_target_velocity_);
  }

  std::string command_topic_;
  int motor_count_{12};
  int active_motor_index_{0};
  double publish_rate_hz_{100.0};
  double start_delay_sec_{1.0};
  double run_duration_sec_{0.0};
  double center_position_{0.0};
  double amplitude_{0.15};
  double frequency_hz_{0.25};
  double kp_{6.0};
  double kd_{0.3};
  double hold_kd_{0.8};
  double torque_ff_{0.0};
  bool motion_complete_logged_{false};
  double last_target_position_{0.0};
  double last_target_velocity_{0.0};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
