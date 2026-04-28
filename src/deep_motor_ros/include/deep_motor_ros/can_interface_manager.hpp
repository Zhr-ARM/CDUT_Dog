#ifndef DEEP_MOTOR_ROS_CAN_INTERFACE_MANAGER_HPP_
#define DEEP_MOTOR_ROS_CAN_INTERFACE_MANAGER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace deep_motor_ros
{

struct CanInterfaceConfig
{
  int bitrate{1000000};
  int txqueuelen{1000};
  double wait_sec{5.0};
  int configure_retries{3};
};

class CanInterfaceManager
{
public:
  CanInterfaceManager(rclcpp::Logger logger, CanInterfaceConfig config);

  void wait_for_interfaces(const std::vector<std::string> & can_interfaces) const;
  void configure_interfaces(const std::vector<std::string> & can_interfaces) const;
  void log_interface_snapshot(
    const std::string & can_interface,
    const std::string & stage) const;

private:
  bool run_setup_command(const std::string & can_interface, const std::string & cmd) const;
  std::string read_command_output(const std::string & cmd) const;

  rclcpp::Logger logger_;
  CanInterfaceConfig config_;
};

}  // namespace deep_motor_ros

#endif  // DEEP_MOTOR_ROS_CAN_INTERFACE_MANAGER_HPP_
