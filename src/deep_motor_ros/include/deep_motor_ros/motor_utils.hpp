#ifndef DEEP_MOTOR_ROS_MOTOR_UTILS_HPP_
#define DEEP_MOTOR_ROS_MOTOR_UTILS_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace deep_motor_ros
{

std::vector<std::string> make_default_joint_names(size_t can_count, size_t motors_per_can);
std::string join_strings(const std::vector<std::string> & values);
std::string trim_copy(const std::string & value);
std::string shell_single_quote(const std::string & value);
bool interface_exists(const std::string & can_interface);

}  // namespace deep_motor_ros

#endif  // DEEP_MOTOR_ROS_MOTOR_UTILS_HPP_
