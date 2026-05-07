#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <string>
#include <vector>

class DogTeleopNode : public rclcpp::Node
{
public:
  DogTeleopNode() : Node("dog_teleop_node"), last_joy_time_(this->now())
  {
    // Declare parameters
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("gait_action_topic", "/gait_action");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("output_mode", "cmd_vel");
    const int legacy_linear_axis = this->declare_parameter<int>("linear_axis", 1);
    const int legacy_angular_axis = this->declare_parameter<int>("angular_axis", 0);
    const bool legacy_invert_linear = this->declare_parameter<bool>("invert_linear", false);
    const bool legacy_invert_angular = this->declare_parameter<bool>("invert_angular", false);
    this->declare_parameter<int>("forward_axis", legacy_linear_axis);
    this->declare_parameter<int>("lateral_axis", legacy_angular_axis);
    this->declare_parameter<int>("turn_axis", 3);
    this->declare_parameter<bool>("invert_forward", legacy_invert_linear);
    this->declare_parameter<bool>("invert_lateral", legacy_invert_angular);
    this->declare_parameter<bool>("invert_turn", false);
    this->declare_parameter<double>("deadzone_linear", 0.1);
    this->declare_parameter<double>("deadzone_angular", 0.1);
    this->declare_parameter<double>("diagonal_min_ratio", 0.55);
    this->declare_parameter<double>("max_linear_velocity", 0.5);
    this->declare_parameter<double>("max_lateral_velocity", 0.5);
    this->declare_parameter<double>("max_angular_velocity", 1.0);
    this->declare_parameter<double>("joy_timeout", 0.5);
    this->declare_parameter<int>("emergency_stop_button", 0);
    this->declare_parameter<int>("crouch_button", 2);
    this->declare_parameter<int>("stand_button", 3);
    this->declare_parameter<bool>("debug_joy_axes", false);
    this->declare_parameter<double>("debug_log_period", 0.5);

    std::string joy_topic = this->get_parameter("joy_topic").as_string();
    std::string gait_action_topic = this->get_parameter("gait_action_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    output_mode_ = this->get_parameter("output_mode").as_string();
    forward_axis_ = static_cast<int>(this->get_parameter("forward_axis").as_int());
    lateral_axis_ = static_cast<int>(this->get_parameter("lateral_axis").as_int());
    turn_axis_ = static_cast<int>(this->get_parameter("turn_axis").as_int());
    invert_forward_ = this->get_parameter("invert_forward").as_bool();
    invert_lateral_ = this->get_parameter("invert_lateral").as_bool();
    invert_turn_ = this->get_parameter("invert_turn").as_bool();
    deadzone_linear_ = this->get_parameter("deadzone_linear").as_double();
    deadzone_angular_ = this->get_parameter("deadzone_angular").as_double();
    diagonal_min_ratio_ = std::clamp(
      this->get_parameter("diagonal_min_ratio").as_double(), 0.0, 1.0);
    max_linear_velocity_ = std::max(0.0, this->get_parameter("max_linear_velocity").as_double());
    max_lateral_velocity_ = std::max(0.0, this->get_parameter("max_lateral_velocity").as_double());
    max_angular_velocity_ = std::max(0.0, this->get_parameter("max_angular_velocity").as_double());
    joy_timeout_ = this->get_parameter("joy_timeout").as_double();
    emergency_stop_button_ = static_cast<int>(
      this->get_parameter("emergency_stop_button").as_int());
    crouch_button_ = static_cast<int>(this->get_parameter("crouch_button").as_int());
    stand_button_ = static_cast<int>(this->get_parameter("stand_button").as_int());
    debug_joy_axes_ = this->get_parameter("debug_joy_axes").as_bool();
    debug_log_period_ = std::max(0.05, this->get_parameter("debug_log_period").as_double());

    // Create subscription to joystick input
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, 10,
        std::bind(&DogTeleopNode::joyCallback, this, std::placeholders::_1));

    // Create publisher for high-level gait action commands.
    action_publisher_ =
        this->create_publisher<std_msgs::msg::String>(gait_action_topic, 10);
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // Create timer for disconnection protection
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DogTeleopNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Dog teleop node initialized");
    RCLCPP_INFO(this->get_logger(), "Output mode: %s", output_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gait action topic: %s", gait_action_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel topic: %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Axis mapping - forward: axes[%d], lateral: axes[%d], turn: axes[%d]",
      forward_axis_, lateral_axis_, turn_axis_);
    RCLCPP_INFO(this->get_logger(), "Deadzone - Linear: %.2f, Angular: %.2f", deadzone_linear_, deadzone_angular_);
    RCLCPP_INFO(this->get_logger(), "Diagonal min ratio: %.2f", diagonal_min_ratio_);
    RCLCPP_INFO(
      this->get_logger(),
      "Buttons - emergency/stand: buttons[%d], crouch: buttons[%d], stand up: buttons[%d]",
      emergency_stop_button_, crouch_button_, stand_button_);
    RCLCPP_INFO(this->get_logger(), "Joystick timeout: %.2f seconds", joy_timeout_);
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = this->now();
    timeout_reported_ = false;

    // Check emergency stop button (A button by default).
    if (buttonPressed(*msg, emergency_stop_button_)) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
      publishCmdVel(0.0, 0.0, 0.0, true);
      publishAction("stand", true);
      updatePreviousButtons(*msg);
      return;
    }

    if (buttonRisingEdge(*msg, crouch_button_)) {
      RCLCPP_INFO(this->get_logger(), "Crouch button pressed.");
      publishCmdVel(0.0, 0.0, 0.0, true);
      publishAction("crouch", true);
      updatePreviousButtons(*msg);
      return;
    }

    if (buttonRisingEdge(*msg, stand_button_)) {
      RCLCPP_INFO(this->get_logger(), "Stand button pressed.");
      publishCmdVel(0.0, 0.0, 0.0, true);
      publishAction("stand", true);
      updatePreviousButtons(*msg);
      return;
    }

    double forward = axisValue(*msg, forward_axis_);
    double lateral = axisValue(*msg, lateral_axis_);
    double turn = axisValue(*msg, turn_axis_);
    if (invert_forward_) {
      forward = -forward;
    }
    if (invert_lateral_) {
      lateral = -lateral;
    }
    if (invert_turn_) {
      turn = -turn;
    }

    if (output_mode_ == "cmd_vel") {
      const double filtered_forward = applyDeadzone(forward, deadzone_linear_);
      const double filtered_lateral = applyDeadzone(lateral, deadzone_linear_);
      const double filtered_turn = applyDeadzone(turn, deadzone_angular_);
      publishCmdVel(
        filtered_forward * max_linear_velocity_,
        filtered_lateral * max_lateral_velocity_,
        filtered_turn * max_angular_velocity_);
      maybeLogJoy(
        *msg, filtered_forward, filtered_lateral, filtered_turn,
        formatCmdVelAction(filtered_forward, filtered_lateral, filtered_turn));
    } else {
      const double turn_abs = std::abs(turn);
      std::string action = "stand";

      if (turn_abs > deadzone_angular_) {
        action = turn > 0.0 ? "turn_left" : "turn_right";
      } else {
        action = selectTranslationAction(forward, lateral);
      }

      maybeLogJoy(*msg, forward, lateral, turn, action);
      publishAction(action);
    }
    updatePreviousButtons(*msg);
  }

  void timerCallback()
  {
    // Check if joystick connection is lost
    rclcpp::Time now = this->now();
    double time_since_last_joy = (now - last_joy_time_).seconds();

    if (time_since_last_joy > joy_timeout_) {
      // No joystick signal - stop the robot.
      publishCmdVel(0.0, 0.0, 0.0, true);
      publishAction("stand", true);

      if (!timeout_reported_) {
        RCLCPP_WARN(this->get_logger(), "Joystick disconnected! Robot stopped.");
        timeout_reported_ = true;
      }
    }
  }

  double axisValue(const sensor_msgs::msg::Joy & msg, int axis_index) const
  {
    if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg.axes.size()) {
      return 0.0;
    }
    const double value = msg.axes[static_cast<size_t>(axis_index)];
    return std::clamp(value, -1.0, 1.0);
  }

  bool buttonPressed(const sensor_msgs::msg::Joy & msg, int button_index) const
  {
    return button_index >= 0 &&
      static_cast<size_t>(button_index) < msg.buttons.size() &&
      msg.buttons[static_cast<size_t>(button_index)] != 0;
  }

  bool buttonRisingEdge(const sensor_msgs::msg::Joy & msg, int button_index) const
  {
    if (!buttonPressed(msg, button_index)) {
      return false;
    }
    const size_t index = static_cast<size_t>(button_index);
    return index >= previous_buttons_.size() || previous_buttons_[index] == 0;
  }

  void updatePreviousButtons(const sensor_msgs::msg::Joy & msg)
  {
    previous_buttons_ = msg.buttons;
  }

  double applyDeadzone(double value, double deadzone) const
  {
    if (std::abs(value) <= deadzone) {
      return 0.0;
    }

    const double sign = value >= 0.0 ? 1.0 : -1.0;
    const double scaled = (std::abs(value) - deadzone) / std::max(1.0 - deadzone, 1e-6);
    return sign * std::clamp(scaled, 0.0, 1.0);
  }

  std::string selectTranslationAction(double forward, double lateral) const
  {
    const double forward_abs = std::abs(forward);
    const double lateral_abs = std::abs(lateral);
    const bool forward_active = forward_abs > deadzone_linear_;
    const bool lateral_active = lateral_abs > deadzone_linear_;

    if (forward_active && lateral_active) {
      const double dominant = std::max(forward_abs, lateral_abs);
      const double secondary = std::min(forward_abs, lateral_abs);
      const bool diagonal_requested =
        dominant <= 1e-6 || secondary / dominant >= diagonal_min_ratio_;

      if (diagonal_requested) {
        if (forward > 0.0) {
          return lateral > 0.0 ? "forward_left" : "forward_right";
        }
        return lateral > 0.0 ? "backward_left" : "backward_right";
      }

      if (forward_abs > lateral_abs) {
        return forward > 0.0 ? "forward" : "backward";
      }
      return lateral > 0.0 ? "lateral_left" : "lateral_right";
    }

    if (forward_active) {
      return forward > 0.0 ? "forward" : "backward";
    }
    if (lateral_active) {
      return lateral > 0.0 ? "lateral_left" : "lateral_right";
    }
    return "stand";
  }

  std::string formatCmdVelAction(double forward, double lateral, double turn) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "cmd_vel(x=" << forward
           << ", y=" << lateral
           << ", yaw=" << turn << ")";
    return stream.str();
  }

  std::string formatAxes(const sensor_msgs::msg::Joy & msg) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << "[";
    for (size_t i = 0; i < msg.axes.size(); ++i) {
      if (i > 0) {
        stream << ", ";
      }
      stream << i << ":" << msg.axes[i];
    }
    stream << "]";
    return stream.str();
  }

  void maybeLogJoy(
    const sensor_msgs::msg::Joy & msg,
    double forward,
    double lateral,
    double turn,
    const std::string & action)
  {
    if (!debug_joy_axes_) {
      return;
    }

    const rclcpp::Time now = this->now();
    const bool period_elapsed =
      !last_debug_log_time_valid_ ||
      (now - last_debug_log_time_).seconds() >= debug_log_period_;
    if (!period_elapsed && action == last_debug_action_) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Joy axes=%s | selected forward[%d]=%.2f lateral[%d]=%.2f turn[%d]=%.2f -> %s",
      formatAxes(msg).c_str(),
      forward_axis_, forward,
      lateral_axis_, lateral,
      turn_axis_, turn,
      action.c_str());
    last_debug_log_time_ = now;
    last_debug_log_time_valid_ = true;
    last_debug_action_ = action;
  }

  void publishAction(const std::string & action, bool force = false)
  {
    if (!force && action == last_action_) {
      return;
    }

    auto msg = std_msgs::msg::String();
    msg.data = action;
    action_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publish gait action: %s", action.c_str());
    last_action_ = action;
  }

  void publishCmdVel(double linear_x, double linear_y, double angular_z, bool force = false)
  {
    (void) force;
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z = angular_z;
    cmd_vel_publisher_->publish(msg);

    last_cmd_vel_linear_x_ = linear_x;
    last_cmd_vel_linear_y_ = linear_y;
    last_cmd_vel_angular_z_ = angular_z;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_joy_time_;
  std::string output_mode_;
  std::string last_action_;
  int forward_axis_;
  int lateral_axis_;
  int turn_axis_;
  int emergency_stop_button_;
  int crouch_button_;
  int stand_button_;
  bool invert_forward_;
  bool invert_lateral_;
  bool invert_turn_;
  bool debug_joy_axes_;
  bool last_debug_log_time_valid_{false};
  bool timeout_reported_{false};
  double deadzone_linear_;
  double deadzone_angular_;
  double diagonal_min_ratio_;
  double max_linear_velocity_;
  double max_lateral_velocity_;
  double max_angular_velocity_;
  double joy_timeout_;
  double debug_log_period_;
  double last_cmd_vel_linear_x_{0.0};
  double last_cmd_vel_linear_y_{0.0};
  double last_cmd_vel_angular_z_{0.0};
  rclcpp::Time last_debug_log_time_;
  std::string last_debug_action_;
  std::vector<int> previous_buttons_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DogTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
