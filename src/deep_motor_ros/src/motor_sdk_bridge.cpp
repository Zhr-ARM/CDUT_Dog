#include "deep_motor_ros/motor_sdk_bridge.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>

extern "C" {
#include "deep_motor_ros/deep_motor_sdk.h"
}

namespace deep_motor_ros
{

namespace
{

DrMotorCan * can_handle(BusContext & bus)
{
  return static_cast<DrMotorCan *>(bus.can_handle);
}

MotorCMD * cmd_handle(MotorContext & motor)
{
  return static_cast<MotorCMD *>(motor.cmd_handle);
}

MotorDATA * data_handle(MotorContext & motor)
{
  return static_cast<MotorDATA *>(motor.data_handle);
}

void reset_motor_data(MotorDATA * data)
{
  if (!data)
  {
    return;
  }

  std::memset(data, 0, sizeof(MotorDATA));
  data->error_ = kMotorNoError;
}

MotorIoResult make_result(int ret, const MotorDATA * data)
{
  MotorIoResult result;
  result.ret = ret;
  if (!data)
  {
    return result;
  }

  result.reply_motor_id = data->motor_id_;
  result.reply_cmd = data->cmd_;
  result.state.p = data->position_;
  result.state.v = data->velocity_;
  result.state.t = data->torque_;
  result.temperature_is_motor = data->flag_;
  if (data->flag_)
  {
    result.state.motor_temp = data->temp_;
  }
  else
  {
    result.state.driver_temp = data->temp_;
  }
  return result;
}

bool parsed_reply_matches(const MotorDATA & data, const MotorCMD & cmd)
{
  return data.motor_id_ == cmd.motor_id_ && data.cmd_ == cmd.cmd_;
}

void drain_pending_frames(DrMotorCan * can)
{
  if (!can)
  {
    return;
  }

  for (int i = 0; i < 64; ++i)
  {
    struct can_frame stale_frame;
    pthread_mutex_lock(&can->rw_mutex);
    const ssize_t nbytes = read(can->can_socket_, &stale_frame, sizeof(stale_frame));
    const int read_errno = errno;
    pthread_mutex_unlock(&can->rw_mutex);

    if (nbytes == static_cast<ssize_t>(sizeof(stale_frame)))
    {
      continue;
    }

    if (nbytes < 0 && (read_errno == EAGAIN || read_errno == EWOULDBLOCK))
    {
      return;
    }

    return;
  }
}

int read_expected_reply(
  DrMotorCan * can,
  const MotorCMD & cmd,
  MotorDATA * data,
  int timeout_ms)
{
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);

  while (clock::now() < deadline)
  {
    const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - clock::now()).count();
    const int wait_ms = std::max(1, static_cast<int>(remaining_ms));

    struct epoll_event events;
    const int epoll_wait_result = epoll_wait(can->epoll_fd_, &events, 1, wait_ms);
    if (epoll_wait_result == 0)
    {
      return kRecvTimeoutError;
    }
    if (epoll_wait_result == -1)
    {
      if (errno == EINTR)
      {
        continue;
      }
      return kRecvEpollError;
    }

    while (true)
    {
      struct can_frame recv_frame;
      pthread_mutex_lock(&can->rw_mutex);
      const ssize_t nbytes = read(can->can_socket_, &recv_frame, sizeof(recv_frame));
      const int read_errno = errno;
      pthread_mutex_unlock(&can->rw_mutex);

      if (nbytes == static_cast<ssize_t>(sizeof(recv_frame)))
      {
        MotorDATA candidate;
        reset_motor_data(&candidate);
        ParseRecvFrame(&recv_frame, &candidate);
        if (parsed_reply_matches(candidate, cmd))
        {
          *data = candidate;
          return kNoSendRecvError;
        }

        continue;
      }

      if (nbytes < 0 && (read_errno == EAGAIN || read_errno == EWOULDBLOCK))
      {
        break;
      }

      return kRecvLengthError;
    }
  }

  return kRecvTimeoutError;
}

int send_recv_expected(DrMotorCan * can, const MotorCMD * cmd, MotorDATA * data, int timeout_ms)
{
  if (!can || !cmd || !data)
  {
    return kRecvEpollError;
  }

  reset_motor_data(data);
  drain_pending_frames(can);

  struct can_frame send_frame{};
  MakeSendFrame(cmd, &send_frame);

  pthread_mutex_lock(&can->rw_mutex);
  const ssize_t nbytes = write(can->can_socket_, &send_frame, sizeof(send_frame));
  pthread_mutex_unlock(&can->rw_mutex);
  if (nbytes != static_cast<ssize_t>(sizeof(send_frame)))
  {
    return kSendLengthError;
  }

  return read_expected_reply(can, *cmd, data, std::max(1, timeout_ms));
}

MotorIoResult make_error_result(int ret)
{
  MotorIoResult result;
  result.ret = ret;
  return result;
}

MotorIoResult send_normal_command(BusContext & bus, MotorContext & motor, uint8_t command)
{
  auto * can = can_handle(bus);
  auto * cmd = cmd_handle(motor);
  auto * data = data_handle(motor);
  if (!can || !cmd || !data)
  {
    return make_error_result(-1);
  }

  SetNormalCMD(cmd, motor.id, command);
  const int ret = send_recv_expected(can, cmd, data, 50);
  return make_result(ret, data);
}

}  // namespace

bool open_can_bus(BusContext & bus, bool show_log)
{
  close_can_bus(bus);
  bus.can_handle = DrMotorCanCreate(bus.can_interface.c_str(), show_log);
  return bus.can_handle != nullptr;
}

void close_can_bus(BusContext & bus)
{
  if (bus.can_handle)
  {
    DrMotorCanDestroy(can_handle(bus));
    bus.can_handle = nullptr;
  }
}

bool create_motor_io(MotorContext & motor)
{
  destroy_motor_io(motor);
  motor.cmd_handle = MotorCMDCreate();
  motor.data_handle = MotorDATACreate();
  return motor.cmd_handle != nullptr && motor.data_handle != nullptr;
}

void destroy_motor_io(MotorContext & motor)
{
  if (motor.cmd_handle)
  {
    MotorCMDDestroy(cmd_handle(motor));
    motor.cmd_handle = nullptr;
  }
  if (motor.data_handle)
  {
    MotorDATADestroy(data_handle(motor));
    motor.data_handle = nullptr;
  }
}

MotorIoResult send_enable_motor(BusContext & bus, MotorContext & motor)
{
  return send_normal_command(bus, motor, ENABLE_MOTOR);
}

MotorIoResult send_disable_motor(BusContext & bus, MotorContext & motor)
{
  return send_normal_command(bus, motor, DISABLE_MOTOR);
}

MotorIoResult send_set_home(BusContext & bus, MotorContext & motor)
{
  return send_normal_command(bus, motor, SET_HOME);
}

MotorIoResult send_control_motor(BusContext & bus, MotorContext & motor)
{
  return send_control_motor_with_timeout(bus, motor, 50);
}

MotorIoResult send_control_motor_with_timeout(
  BusContext & bus,
  MotorContext & motor,
  int timeout_ms)
{
  auto * can = can_handle(bus);
  auto * cmd = cmd_handle(motor);
  auto * data = data_handle(motor);
  if (!can || !cmd || !data)
  {
    return make_error_result(-1);
  }

  SetMotionCMD(
    cmd, motor.id, CONTROL_MOTOR, motor.current_cmd.p, motor.current_cmd.v, motor.current_cmd.t,
    motor.current_cmd.kp, motor.current_cmd.kd);

  const int ret = send_recv_expected(can, cmd, data, timeout_ms);
  return make_result(ret, data);
}

bool send_recv_ok(const MotorIoResult & result)
{
  return result.ret == kNoSendRecvError;
}

bool reply_matches(const MotorIoResult & result, const MotorContext & motor)
{
  return send_recv_ok(result) && result.reply_motor_id == static_cast<unsigned int>(motor.id);
}

bool enable_reply_ok(const MotorIoResult & result, const MotorContext & motor)
{
  return reply_matches(result, motor) && result.reply_cmd == ENABLE_MOTOR;
}

bool set_home_reply_ok(const MotorIoResult & result, const MotorContext & motor)
{
  return reply_matches(result, motor) && result.reply_cmd == SET_HOME;
}

}  // namespace deep_motor_ros
