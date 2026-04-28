#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "deep_motor_ros/deep_motor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<deep_motor_ros::DeepMotorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
