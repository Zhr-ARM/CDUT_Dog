#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

class DogTeleopNode : public rclcpp::Node
{
public:
  DogTeleopNode() : Node("dog_teleop_node"), last_joy_time_(this->now())
  {
    // Declare parameters
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    this->declare_parameter<double>("max_linear_vel", 1.0);
    this->declare_parameter<double>("max_angular_vel", 1.0);
    this->declare_parameter<double>("deadzone_linear", 0.1);
    this->declare_parameter<double>("deadzone_angular", 0.1);
    this->declare_parameter<double>("joy_timeout", 0.5);

    std::string joy_topic = this->get_parameter("joy_topic").as_string();
    std::string cmd_topic = this->get_parameter("cmd_topic").as_string();
    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    deadzone_linear_ = this->get_parameter("deadzone_linear").as_double();
    deadzone_angular_ = this->get_parameter("deadzone_angular").as_double();
    joy_timeout_ = this->get_parameter("joy_timeout").as_double();

    // Create subscription to joystick input
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, 10,
        std::bind(&DogTeleopNode::joyCallback, this, std::placeholders::_1));

    // Create publisher for command velocity
    cmd_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    // Create timer for disconnection protection
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DogTeleopNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Dog teleop node initialized");
    RCLCPP_INFO(this->get_logger(), "Deadzone - Linear: %.2f, Angular: %.2f", deadzone_linear_, deadzone_angular_);
    RCLCPP_INFO(this->get_logger(), "Joystick timeout: %.2f seconds", joy_timeout_);
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = this->now();
    auto cmd = geometry_msgs::msg::Twist();

    // Check emergency stop button (A button = buttons[0])
    if (!msg->buttons.empty() && msg->buttons[0]) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
      cmd_publisher_->publish(cmd);  // Publish zero velocity
      return;
    }

    // Map joystick axes to robot commands
    // Assuming standard Xbox/PS4 joystick layout
    // Left stick Y-axis for forward/backward (axis 1)
    // Right stick X-axis for rotation (axis 2)
    if (msg->axes.size() >= 3) {
      double linear = msg->axes[1] * max_linear_vel_;
      double angular = msg->axes[2] * max_angular_vel_;

      // Apply deadzone
      if (std::fabs(linear) < deadzone_linear_) {
        linear = 0.0;
      }
      if (std::fabs(angular) < deadzone_angular_) {
        angular = 0.0;
      }

      cmd.linear.x = linear;
      cmd.angular.z = angular;
    }

    cmd_publisher_->publish(cmd);
  }

  void timerCallback()
  {
    // Check if joystick connection is lost
    rclcpp::Time now = this->now();
    double time_since_last_joy = (now - last_joy_time_).seconds();

    if (time_since_last_joy > joy_timeout_) {
      // No joystick signal - stop the robot
      auto cmd = geometry_msgs::msg::Twist();
      cmd_publisher_->publish(cmd);

      if (time_since_last_joy < joy_timeout_ + 0.5) {
        RCLCPP_WARN(this->get_logger(), "Joystick disconnected! Robot stopped.");
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_joy_time_;
  double max_linear_vel_;
  double max_angular_vel_;
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
