#include "deep_motor_ros/motor_utils.hpp"

#include <array>
#include <filesystem>
#include <sstream>

namespace deep_motor_ros
{

namespace
{

const std::array<std::string, 4> kLegOrder = {"LF", "LH", "RF", "RH"};
const std::array<std::string, 3> kJointOrder = {"HAA", "HFE", "KFE"};

}  // namespace

std::vector<std::string> make_default_joint_names(size_t can_count, size_t motors_per_can)
{
  std::vector<std::string> names;
  names.reserve(can_count * motors_per_can);

  if (can_count == kLegOrder.size() && motors_per_can == kJointOrder.size())
  {
    for (size_t leg_index = 0; leg_index < can_count; ++leg_index)
    {
      for (size_t joint_index = 0; joint_index < motors_per_can; ++joint_index)
      {
        names.push_back(kLegOrder[leg_index] + "_" + kJointOrder[joint_index]);
      }
    }
    return names;
  }

  for (size_t index = 0; index < can_count * motors_per_can; ++index)
  {
    names.push_back("joint_" + std::to_string(index));
  }
  return names;
}

std::string join_strings(const std::vector<std::string> & values)
{
  std::ostringstream oss;
  for (size_t i = 0; i < values.size(); ++i)
  {
    if (i != 0)
    {
      oss << ", ";
    }
    oss << values[i];
  }
  return oss.str();
}

std::string trim_copy(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
  {
    return {};
  }

  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string shell_single_quote(const std::string & value)
{
  std::string escaped = "'";
  for (const char ch : value)
  {
    if (ch == '\'')
    {
      escaped += "'\\''";
    }
    else
    {
      escaped.push_back(ch);
    }
  }
  escaped += "'";
  return escaped;
}

bool interface_exists(const std::string & can_interface)
{
  return std::filesystem::exists("/sys/class/net/" + can_interface);
}

}  // namespace deep_motor_ros
