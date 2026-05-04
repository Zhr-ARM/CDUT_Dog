#include <algorithm>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class DogTeleopNode : public rclcpp::Node
{
public:
  DogTeleopNode() : Node("dog_teleop_node"), last_joy_time_(this->now())
  {
    // Declare parameters
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("gait_action_topic", "/gait_action");
    this->declare_parameter<int>("linear_axis", 1);
    this->declare_parameter<int>("angular_axis", 0);
    this->declare_parameter<bool>("invert_linear", false);
    this->declare_parameter<bool>("invert_angular", false);
    this->declare_parameter<double>("deadzone_linear", 0.1);
    this->declare_parameter<double>("deadzone_angular", 0.1);
    this->declare_parameter<double>("joy_timeout", 0.5);
    this->declare_parameter<int>("emergency_stop_button", 0);

    std::string joy_topic = this->get_parameter("joy_topic").as_string();
    std::string gait_action_topic = this->get_parameter("gait_action_topic").as_string();
    linear_axis_ = static_cast<int>(this->get_parameter("linear_axis").as_int());
    angular_axis_ = static_cast<int>(this->get_parameter("angular_axis").as_int());
    invert_linear_ = this->get_parameter("invert_linear").as_bool();
    invert_angular_ = this->get_parameter("invert_angular").as_bool();
    deadzone_linear_ = this->get_parameter("deadzone_linear").as_double();
    deadzone_angular_ = this->get_parameter("deadzone_angular").as_double();
    joy_timeout_ = this->get_parameter("joy_timeout").as_double();
    emergency_stop_button_ = static_cast<int>(
      this->get_parameter("emergency_stop_button").as_int());

    // Create subscription to joystick input
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, 10,
        std::bind(&DogTeleopNode::joyCallback, this, std::placeholders::_1));

    // Create publisher for high-level gait action commands.
    action_publisher_ =
        this->create_publisher<std_msgs::msg::String>(gait_action_topic, 10);

    // Create timer for disconnection protection
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DogTeleopNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Dog teleop node initialized");
    RCLCPP_INFO(this->get_logger(), "Gait action topic: %s", gait_action_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Deadzone - Linear: %.2f, Angular: %.2f", deadzone_linear_, deadzone_angular_);
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
      publishAction("stand", true);
      return;
    }

    double linear = axisValue(*msg, linear_axis_);
    double angular = axisValue(*msg, angular_axis_);
    if (invert_linear_) {
      linear = -linear;
    }
    if (invert_angular_) {
      angular = -angular;
    }

    const double linear_abs = std::abs(linear);
    const double angular_abs = std::abs(angular);
    std::string action = "stand";

    if (linear_abs > deadzone_linear_ && linear_abs >= angular_abs) {
      action = linear > 0.0 ? "forward" : "backward";
    } else if (angular_abs > deadzone_angular_) {
      action = angular > 0.0 ? "turn_left" : "turn_right";
    }

    publishAction(action);
  }

  void timerCallback()
  {
    // Check if joystick connection is lost
    rclcpp::Time now = this->now();
    double time_since_last_joy = (now - last_joy_time_).seconds();

    if (time_since_last_joy > joy_timeout_) {
      // No joystick signal - stop the robot.
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

  void publishAction(const std::string & action, bool force = false)
  {
    if (!force && action == last_action_) {
      return;
    }

    auto msg = std_msgs::msg::String();
    msg.data = action;
    action_publisher_->publish(msg);
    last_action_ = action;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_joy_time_;
  std::string last_action_;
  int linear_axis_;
  int angular_axis_;
  int emergency_stop_button_;
  bool invert_linear_;
  bool invert_angular_;
  bool timeout_reported_{false};
  double deadzone_linear_;
  double deadzone_angular_;
  double joy_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DogTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
