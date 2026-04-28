#ifndef DEEP_MOTOR_ROS_MOTOR_SDK_BRIDGE_HPP_
#define DEEP_MOTOR_ROS_MOTOR_SDK_BRIDGE_HPP_

#include "deep_motor_ros/motor_types.hpp"

namespace deep_motor_ros
{

bool open_can_bus(BusContext & bus, bool show_log);
void close_can_bus(BusContext & bus);

bool create_motor_io(MotorContext & motor);
void destroy_motor_io(MotorContext & motor);

MotorIoResult send_enable_motor(BusContext & bus, MotorContext & motor);
MotorIoResult send_disable_motor(BusContext & bus, MotorContext & motor);
MotorIoResult send_set_home(BusContext & bus, MotorContext & motor);
MotorIoResult send_control_motor(BusContext & bus, MotorContext & motor);
MotorIoResult send_control_motor_with_timeout(
  BusContext & bus,
  MotorContext & motor,
  int timeout_ms);

bool send_recv_ok(const MotorIoResult & result);
bool reply_matches(const MotorIoResult & result, const MotorContext & motor);
bool enable_reply_ok(const MotorIoResult & result, const MotorContext & motor);
bool set_home_reply_ok(const MotorIoResult & result, const MotorContext & motor);

}  // namespace deep_motor_ros

#endif  // DEEP_MOTOR_ROS_MOTOR_SDK_BRIDGE_HPP_
