#ifndef DEEP_MOTOR_ROS_MOTOR_TYPES_HPP_
#define DEEP_MOTOR_ROS_MOTOR_TYPES_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace deep_motor_ros
{

constexpr size_t kValuesPerJoint = 5;

struct ControlCommand
{
  double p{0.0};
  double v{0.0};
  double t{0.0};
  double kp{0.0};
  double kd{0.0};
};

struct MotorState
{
  double p{0.0};
  double v{0.0};
  double t{0.0};
  double motor_temp{0.0};
  double driver_temp{0.0};
};

struct MotorContext
{
  size_t global_index{0};
  int local_slot{0};
  int id{0};
  std::string joint_name;
  bool is_connected{false};
  bool home_set{false};
  bool has_feedback{false};
  void * cmd_handle{nullptr};
  void * data_handle{nullptr};
  ControlCommand current_cmd;
  MotorState current_state;
};

struct BusContext
{
  std::string can_interface;
  void * can_handle{nullptr};
  std::vector<MotorContext> motors;
};

struct MotorIoResult
{
  int ret{0};
  unsigned int reply_motor_id{0};
  unsigned int reply_cmd{0};
  bool temperature_is_motor{false};
  MotorState state;
};

}  // namespace deep_motor_ros

#endif  // DEEP_MOTOR_ROS_MOTOR_TYPES_HPP_
