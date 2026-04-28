#include "deep_motor_ros/deep_motor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

#include "deep_motor_ros/motor_sdk_bridge.hpp"
#include "deep_motor_ros/motor_utils.hpp"

using namespace std::chrono_literals;

namespace deep_motor_ros
{

DeepMotorNode::DeepMotorNode()
: Node("deep_motor_node")
{
  declare_parameters();
  load_parameters();

  motor_ids_ = expand_motor_ids(get_parameter("motor_ids").as_integer_array());
  if (motor_ids_.empty())
  {
    RCLCPP_ERROR(get_logger(), "No valid motor_ids configuration found. Node will stay idle.");
    return;
  }

  joint_names_ = resolve_joint_names();
  total_motor_count_ = motor_ids_.size();
  normalize_shutdown_positions();

  setup_can_interfaces();
  initialize_buses();
  setup_ros_interfaces();
}

DeepMotorNode::~DeepMotorNode()
{
  move_to_shutdown_pose_if_requested();

  for (auto & bus : buses_)
  {
    for (auto & motor : bus.motors)
    {
      if (bus.can_handle && motor.is_connected && motor.cmd_handle && motor.data_handle)
      {
        send_disable_motor(bus, motor);
      }
      destroy_motor_io(motor);
    }

    close_can_bus(bus);
  }
}

void DeepMotorNode::declare_parameters()
{
  declare_parameter<std::vector<std::string>>("can_interfaces", std::vector<std::string>{});
  declare_parameter<std::string>("can_interface", "can0");
  declare_parameter<int>("motors_per_can", 3);
  declare_parameter<std::vector<int64_t>>("motor_ids", {0, 1, 2});
  declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
  declare_parameter<std::string>("command_topic", "/motor_cmd");
  declare_parameter<std::string>("feedback_topic", "/motor_feedback");
  declare_parameter<std::string>("joint_state_topic", "/joint_states");
  declare_parameter<bool>("configure_can", true);
  declare_parameter<int>("can_bitrate", 1000000);
  declare_parameter<int>("can_txqueuelen", 1000);
  declare_parameter<double>("can_interface_wait_sec", 5.0);
  declare_parameter<int>("can_configure_retries", 3);
  declare_parameter<double>("control_rate_hz", 400.0);
  declare_parameter<double>("status_log_period_sec", 5.0);
  declare_parameter<bool>("auto_set_home_on_startup", false);
  declare_parameter<bool>("require_home_on_startup", false);
  declare_parameter<int>("home_command_retries", 3);
  declare_parameter<double>("home_command_delay_sec", 0.05);
  declare_parameter<bool>("shutdown_move_on_exit", false);
  declare_parameter<std::vector<double>>("shutdown_positions", std::vector<double>{});
  declare_parameter<double>("shutdown_duration_sec", 1.5);
  declare_parameter<double>("shutdown_rate_hz", 50.0);
  declare_parameter<int>("shutdown_command_timeout_ms", 5);
  declare_parameter<double>("shutdown_kp", 8.0);
  declare_parameter<double>("shutdown_kd", 1.0);
  declare_parameter<double>("shutdown_torque", 0.0);
}

void DeepMotorNode::load_parameters()
{
  can_interfaces_ = get_parameter("can_interfaces").as_string_array();
  if (can_interfaces_.empty())
  {
    can_interfaces_.push_back(get_parameter("can_interface").as_string());
  }

  motors_per_can_ = std::max(1, static_cast<int>(get_parameter("motors_per_can").as_int()));
  command_topic_ = get_parameter("command_topic").as_string();
  feedback_topic_ = get_parameter("feedback_topic").as_string();
  joint_state_topic_ = get_parameter("joint_state_topic").as_string();
  configure_can_ = get_parameter("configure_can").as_bool();
  can_bitrate_ = get_parameter("can_bitrate").as_int();
  can_txqueuelen_ = get_parameter("can_txqueuelen").as_int();
  can_interface_wait_sec_ = std::max(0.0, get_parameter("can_interface_wait_sec").as_double());
  can_configure_retries_ =
    std::max(1, static_cast<int>(get_parameter("can_configure_retries").as_int()));
  control_rate_hz_ = std::max(1.0, get_parameter("control_rate_hz").as_double());
  status_log_period_sec_ = std::max(0.0, get_parameter("status_log_period_sec").as_double());
  auto_set_home_on_startup_ = get_parameter("auto_set_home_on_startup").as_bool();
  require_home_on_startup_ = get_parameter("require_home_on_startup").as_bool();
  home_command_retries_ =
    std::max(1, static_cast<int>(get_parameter("home_command_retries").as_int()));
  home_command_delay_sec_ = std::max(0.0, get_parameter("home_command_delay_sec").as_double());
  shutdown_move_on_exit_ = get_parameter("shutdown_move_on_exit").as_bool();
  shutdown_positions_ = get_parameter("shutdown_positions").as_double_array();
  shutdown_duration_sec_ = std::max(0.0, get_parameter("shutdown_duration_sec").as_double());
  shutdown_rate_hz_ = std::max(1.0, get_parameter("shutdown_rate_hz").as_double());
  shutdown_command_timeout_ms_ =
    std::clamp(static_cast<int>(get_parameter("shutdown_command_timeout_ms").as_int()), 1, 50);
  shutdown_kp_ = std::max(0.0, get_parameter("shutdown_kp").as_double());
  shutdown_kd_ = std::max(0.0, get_parameter("shutdown_kd").as_double());
  shutdown_torque_ = get_parameter("shutdown_torque").as_double();
}

void DeepMotorNode::setup_can_interfaces()
{
  if (configure_can_ && geteuid() != 0)
  {
    RCLCPP_INFO(
      get_logger(),
      "configure_can is enabled and this process is not running as root. "
      "The node will use embedded sudo password fallback for CAN setup.");
  }

  CanInterfaceConfig config;
  config.bitrate = can_bitrate_;
  config.txqueuelen = can_txqueuelen_;
  config.wait_sec = can_interface_wait_sec_;
  config.configure_retries = can_configure_retries_;

  const CanInterfaceManager can_manager(get_logger(), config);
  can_manager.wait_for_interfaces(can_interfaces_);

  if (configure_can_)
  {
    can_manager.configure_interfaces(can_interfaces_);
  }
}

void DeepMotorNode::setup_ros_interfaces()
{
  feedback_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(feedback_topic_, 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
  cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    command_topic_, 10,
    std::bind(&DeepMotorNode::command_callback, this, std::placeholders::_1));

  const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / control_rate_hz_));
  timer_ = create_wall_timer(control_period, std::bind(&DeepMotorNode::control_loop, this));

  if (status_log_period_sec_ > 0.0)
  {
    const auto status_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(status_log_period_sec_));
    status_timer_ =
      create_wall_timer(status_period, std::bind(&DeepMotorNode::log_status_summary, this));
  }
}

std::vector<int64_t> DeepMotorNode::expand_motor_ids(
  const std::vector<int64_t> & configured_motor_ids) const
{
  const size_t bus_count = can_interfaces_.size();
  const size_t expected_total = bus_count * static_cast<size_t>(motors_per_can_);

  if (configured_motor_ids.size() == expected_total)
  {
    return configured_motor_ids;
  }

  if (configured_motor_ids.size() == static_cast<size_t>(motors_per_can_) && bus_count > 1)
  {
    std::vector<int64_t> expanded;
    expanded.reserve(expected_total);
    for (size_t bus_index = 0; bus_index < bus_count; ++bus_index)
    {
      expanded.insert(expanded.end(), configured_motor_ids.begin(), configured_motor_ids.end());
    }
    return expanded;
  }

  RCLCPP_ERROR(
    get_logger(),
    "motor_ids size mismatch. Got %zu values, expected %zu values or %d per CAN.",
    configured_motor_ids.size(), expected_total, motors_per_can_);
  return {};
}

std::vector<std::string> DeepMotorNode::resolve_joint_names() const
{
  auto configured_joint_names = get_parameter("joint_names").as_string_array();
  if (configured_joint_names.size() == motor_ids_.size())
  {
    return configured_joint_names;
  }

  if (!configured_joint_names.empty())
  {
    RCLCPP_WARN(
      get_logger(),
      "joint_names size (%zu) does not match motor_ids size (%zu). Falling back to defaults.",
      configured_joint_names.size(), motor_ids_.size());
  }

  return make_default_joint_names(can_interfaces_.size(), static_cast<size_t>(motors_per_can_));
}

void DeepMotorNode::initialize_buses()
{
  buses_.clear();
  buses_.reserve(can_interfaces_.size());

  CanInterfaceConfig config;
  config.bitrate = can_bitrate_;
  config.txqueuelen = can_txqueuelen_;
  config.wait_sec = can_interface_wait_sec_;
  config.configure_retries = can_configure_retries_;
  const CanInterfaceManager can_manager(get_logger(), config);

  size_t global_index = 0;
  size_t total_online = 0;
  size_t total_homed = 0;

  for (const auto & can_interface : can_interfaces_)
  {
    BusContext bus;
    bus.can_interface = can_interface;
    bus.motors.reserve(static_cast<size_t>(motors_per_can_));

    std::vector<std::string> bus_joint_names;
    std::vector<std::string> bus_motor_ids;
    bus_joint_names.reserve(static_cast<size_t>(motors_per_can_));
    bus_motor_ids.reserve(static_cast<size_t>(motors_per_can_));

    for (int local_slot = 0; local_slot < motors_per_can_; ++local_slot, ++global_index)
    {
      MotorContext motor;
      motor.global_index = global_index;
      motor.local_slot = local_slot;
      motor.id = static_cast<int>(motor_ids_[global_index]);
      motor.joint_name = joint_names_[global_index];
      bus_joint_names.push_back(motor.joint_name);
      bus_motor_ids.push_back(std::to_string(motor.id));
      bus.motors.push_back(std::move(motor));
    }

    RCLCPP_INFO(
      get_logger(),
      "[LinkMap] %s -> joints=[%s], motor_ids=[%s], cmd=%s, feedback=%s, joint_states=%s",
      bus.can_interface.c_str(), join_strings(bus_joint_names).c_str(),
      join_strings(bus_motor_ids).c_str(), command_topic_.c_str(), feedback_topic_.c_str(),
      joint_state_topic_.c_str());

    if (!interface_exists(bus.can_interface))
    {
      RCLCPP_ERROR(
        get_logger(),
        "[Fatal] CAN interface %s does not exist. Check USB-CAN connection, udev rules, "
        "and whether the adapter driver loaded.",
        bus.can_interface.c_str());
      buses_.push_back(std::move(bus));
      continue;
    }

    can_manager.log_interface_snapshot(bus.can_interface, "pre-open");

    if (!open_can_bus(bus, false))
    {
      RCLCPP_ERROR(
        get_logger(), "[Fatal] CAN interface initialization failed: %s",
        bus.can_interface.c_str());
      buses_.push_back(std::move(bus));
      continue;
    }

    size_t bus_online = 0;
    size_t bus_homed = 0;
    for (auto & motor : bus.motors)
    {
      if (!create_motor_io(motor))
      {
        RCLCPP_ERROR(
          get_logger(), "[Fatal] Memory allocation failed for %s(id=%d).",
          motor.joint_name.c_str(), motor.id);
        continue;
      }

      if (enable_and_verify_motor(bus, motor))
      {
        if (auto_set_home_on_startup_)
        {
          if (set_home_for_motor(bus, motor))
          {
            motor.home_set = true;
            ++bus_homed;
          }
          else
          {
            motor.home_set = false;
            if (require_home_on_startup_)
            {
              motor.is_connected = false;
              RCLCPP_ERROR(
                get_logger(),
                "[Home] %s slot=%d joint=%s motor_id=%d failed to set zero point. "
                "Motor will stay offline because require_home_on_startup=true.",
                bus.can_interface.c_str(), motor.local_slot, motor.joint_name.c_str(), motor.id);
              continue;
            }

            RCLCPP_WARN(
              get_logger(),
              "[Home] %s slot=%d joint=%s motor_id=%d failed to set zero point, "
              "but communication is OK. Keeping motor online.",
              bus.can_interface.c_str(), motor.local_slot, motor.joint_name.c_str(), motor.id);
          }
        }
        ++bus_online;
      }
    }

    total_online += bus_online;
    total_homed += bus_homed;
    if (auto_set_home_on_startup_)
    {
      RCLCPP_INFO(
        get_logger(), "[LinkSummary] %s online=%zu/%zu homed=%zu/%zu",
        bus.can_interface.c_str(), bus_online, bus.motors.size(), bus_homed, bus.motors.size());
    }
    else
    {
      RCLCPP_INFO(
        get_logger(), "[LinkSummary] %s online=%zu/%zu", bus.can_interface.c_str(), bus_online,
        bus.motors.size());
    }
    buses_.push_back(std::move(bus));
  }

  if (auto_set_home_on_startup_)
  {
    RCLCPP_INFO(
      get_logger(),
      "[RobotLinkSummary] online=%zu/%zu, homed=%zu/%zu, buses=%zu, control_rate=%.1fHz",
      total_online, total_motor_count_, total_homed, total_motor_count_, buses_.size(),
      control_rate_hz_);
  }
  else
  {
    RCLCPP_INFO(
      get_logger(),
      "[RobotLinkSummary] online=%zu/%zu, buses=%zu, control_rate=%.1fHz",
      total_online, total_motor_count_, buses_.size(), control_rate_hz_);
  }
}

bool DeepMotorNode::enable_and_verify_motor(BusContext & bus, MotorContext & motor)
{
  bool is_verified = false;
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    const MotorIoResult result = send_enable_motor(bus, motor);
    if (enable_reply_ok(result, motor))
    {
      is_verified = true;
      break;
    }
    std::this_thread::sleep_for(10ms);
  }

  motor.is_connected = is_verified;
  if (!is_verified)
  {
    RCLCPP_WARN(
      get_logger(),
      "[LinkCheck] %s slot=%d joint=%s motor_id=%d -> OFFLINE",
      bus.can_interface.c_str(), motor.local_slot, motor.joint_name.c_str(), motor.id);
  }
  return is_verified;
}

bool DeepMotorNode::set_home_for_motor(BusContext & bus, MotorContext & motor)
{
  for (int attempt = 1; attempt <= home_command_retries_; ++attempt)
  {
    const MotorIoResult result = send_set_home(bus, motor);
    if (set_home_reply_ok(result, motor))
    {
      motor.current_cmd = {};
      motor.current_state = {};
      RCLCPP_INFO(
        get_logger(), "[Home] %s slot=%d joint=%s motor_id=%d zero point set.",
        bus.can_interface.c_str(), motor.local_slot, motor.joint_name.c_str(), motor.id);
      if (home_command_delay_sec_ > 0.0)
      {
        std::this_thread::sleep_for(std::chrono::duration<double>(home_command_delay_sec_));
      }
      return true;
    }

    if (!send_recv_ok(result))
    {
      RCLCPP_WARN(
        get_logger(),
        "[Home] %s slot=%d joint=%s motor_id=%d set zero attempt %d/%d failed: ret=%d",
        bus.can_interface.c_str(), motor.local_slot, motor.joint_name.c_str(), motor.id,
        attempt, home_command_retries_, result.ret);
    }
    else
    {
      RCLCPP_WARN(
        get_logger(),
        "[Home] %s slot=%d joint=%s motor_id=%d set zero attempt %d/%d reply mismatch: "
        "reply_id=%u reply_cmd=%u",
        bus.can_interface.c_str(), motor.local_slot, motor.joint_name.c_str(), motor.id,
        attempt, home_command_retries_, result.reply_motor_id, result.reply_cmd);
    }
    if (home_command_delay_sec_ > 0.0)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(home_command_delay_sec_));
    }
  }

  return false;
}

void DeepMotorNode::normalize_shutdown_positions()
{
  if (total_motor_count_ == 0)
  {
    shutdown_positions_.clear();
    return;
  }

  if (shutdown_positions_.empty())
  {
    shutdown_positions_.assign(total_motor_count_, 0.0);
    return;
  }

  if (shutdown_positions_.size() == total_motor_count_)
  {
    return;
  }

  if (
    shutdown_positions_.size() == static_cast<size_t>(motors_per_can_) &&
    can_interfaces_.size() > 1)
  {
    const auto per_bus_positions = shutdown_positions_;
    shutdown_positions_.clear();
    shutdown_positions_.reserve(total_motor_count_);
    for (size_t bus_index = 0; bus_index < can_interfaces_.size(); ++bus_index)
    {
      shutdown_positions_.insert(
        shutdown_positions_.end(), per_bus_positions.begin(), per_bus_positions.end());
    }
    return;
  }

  RCLCPP_WARN(
    get_logger(),
    "shutdown_positions size (%zu) does not match motor count (%zu). "
    "Shutdown pose move is disabled.",
    shutdown_positions_.size(), total_motor_count_);
  shutdown_move_on_exit_ = false;
}

void DeepMotorNode::move_to_shutdown_pose_if_requested()
{
  if (!shutdown_move_on_exit_ || shutdown_positions_.size() != total_motor_count_)
  {
    return;
  }

  std::vector<double> start_positions(total_motor_count_, 0.0);
  size_t online_count = 0;
  for (const auto & bus : buses_)
  {
    for (const auto & motor : bus.motors)
    {
      if (!bus.can_handle || !motor.is_connected || !motor.cmd_handle || !motor.data_handle)
      {
        continue;
      }

      ++online_count;
      start_positions[motor.global_index] =
        motor.has_feedback ? motor.current_state.p : motor.current_cmd.p;
    }
  }

  if (online_count == 0)
  {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "[ShutdownPose] Moving %zu online motors to shutdown pose over %.2fs, "
    "command_timeout=%dms.",
    online_count, shutdown_duration_sec_, shutdown_command_timeout_ms_);

  using clock = std::chrono::steady_clock;
  const auto start_time = clock::now();
  const auto step_period = std::chrono::duration<double>(1.0 / shutdown_rate_hz_);
  auto next_step_time = start_time;

  while (true)
  {
    const auto now = clock::now();
    const double phase =
      shutdown_duration_sec_ <= 1e-6 ?
      1.0 :
      std::clamp(std::chrono::duration<double>(now - start_time).count() / shutdown_duration_sec_,
        0.0, 1.0);
    const double alpha = phase * phase * (3.0 - 2.0 * phase);

    for (auto & bus : buses_)
    {
      if (!bus.can_handle)
      {
        continue;
      }

      for (auto & motor : bus.motors)
      {
        if (!motor.is_connected || !motor.cmd_handle || !motor.data_handle)
        {
          continue;
        }

        const double start = start_positions[motor.global_index];
        const double target = shutdown_positions_[motor.global_index];
        motor.current_cmd.p = start + (target - start) * alpha;
        motor.current_cmd.v = 0.0;
        motor.current_cmd.t = shutdown_torque_;
        motor.current_cmd.kp = shutdown_kp_;
        motor.current_cmd.kd = shutdown_kd_;
        const MotorIoResult result =
          send_control_motor_with_timeout(bus, motor, shutdown_command_timeout_ms_);
        if (send_recv_ok(result) && reply_matches(result, motor))
        {
          motor.current_state = result.state;
          motor.has_feedback = true;
        }
      }
    }

    if (phase >= 1.0)
    {
      break;
    }

    next_step_time += std::chrono::duration_cast<clock::duration>(step_period);
    const auto sleep_until_time = std::max(next_step_time, clock::now());
    std::this_thread::sleep_until(sleep_until_time);
  }
}

void DeepMotorNode::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  const size_t expected_size = total_motor_count_ * kValuesPerJoint;
  if (msg->data.size() != expected_size)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[Protocol Error] %s length mismatch. expected=%zu actual=%zu",
      command_topic_.c_str(), expected_size, msg->data.size());
    return;
  }

  for (auto & bus : buses_)
  {
    for (auto & motor : bus.motors)
    {
      const size_t base = motor.global_index * kValuesPerJoint;
      motor.current_cmd.p = msg->data[base + 0];
      motor.current_cmd.v = msg->data[base + 1];
      motor.current_cmd.t = msg->data[base + 2];
      motor.current_cmd.kp = msg->data[base + 3];
      motor.current_cmd.kd = msg->data[base + 4];
    }
  }
}

void DeepMotorNode::control_loop()
{
  for (auto & bus : buses_)
  {
    if (!bus.can_handle)
    {
      continue;
    }

    for (auto & motor : bus.motors)
    {
      if (!motor.is_connected || !motor.cmd_handle || !motor.data_handle)
      {
        continue;
      }

      const MotorIoResult result = send_control_motor(bus, motor);
      if (!send_recv_ok(result))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[CommWarn] %s joint=%s motor_id=%d SendRecv failed: %d",
          bus.can_interface.c_str(), motor.joint_name.c_str(), motor.id, result.ret);
        continue;
      }

      if (!reply_matches(result, motor))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[CommWarn] %s joint=%s expected motor_id=%d but got %u",
          bus.can_interface.c_str(), motor.joint_name.c_str(), motor.id,
          result.reply_motor_id);
        continue;
      }

      motor.current_state.p = result.state.p;
      motor.current_state.v = result.state.v;
      motor.current_state.t = result.state.t;
      motor.has_feedback = true;
      if (result.temperature_is_motor)
      {
        motor.current_state.motor_temp = result.state.motor_temp;
      }
      else
      {
        motor.current_state.driver_temp = result.state.driver_temp;
      }
    }
  }

  publish_feedback();
}

void DeepMotorNode::publish_feedback()
{
  auto feedback_msg = std_msgs::msg::Float64MultiArray();
  feedback_msg.data.resize(total_motor_count_ * kValuesPerJoint, 0.0);

  auto joint_msg = sensor_msgs::msg::JointState();
  joint_msg.header.stamp = now();

  for (const auto & bus : buses_)
  {
    for (const auto & motor : bus.motors)
    {
      const size_t base = motor.global_index * kValuesPerJoint;
      feedback_msg.data[base + 0] = motor.current_state.p;
      feedback_msg.data[base + 1] = motor.current_state.v;
      feedback_msg.data[base + 2] = motor.current_state.t;
      feedback_msg.data[base + 3] = motor.current_state.motor_temp;
      feedback_msg.data[base + 4] = motor.current_state.driver_temp;

      if (motor.is_connected)
      {
        joint_msg.name.push_back(motor.joint_name);
        joint_msg.position.push_back(motor.current_state.p);
        joint_msg.velocity.push_back(motor.current_state.v);
        joint_msg.effort.push_back(motor.current_state.t);
      }
    }
  }

  feedback_pub_->publish(feedback_msg);
  if (!joint_msg.name.empty())
  {
    joint_state_pub_->publish(joint_msg);
  }
}

void DeepMotorNode::log_status_summary() const
{
  if (buses_.empty())
  {
    return;
  }

  size_t online_total = 0;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  for (size_t bus_index = 0; bus_index < buses_.size(); ++bus_index)
  {
    const auto & bus = buses_[bus_index];
    if (bus_index != 0)
    {
      oss << " || ";
    }

    size_t bus_online = 0;
    oss << bus.can_interface << ": ";
    for (size_t motor_index = 0; motor_index < bus.motors.size(); ++motor_index)
    {
      const auto & motor = bus.motors[motor_index];
      if (motor_index != 0)
      {
        oss << " | ";
      }
      if (motor.is_connected)
      {
        ++bus_online;
        ++online_total;
      }
      if (motor.is_connected)
      {
        oss
          << motor.joint_name << "(id=" << motor.id << ",slot=" << motor.local_slot << ")"
          << " cur=" << motor.current_state.p
          << " target=" << motor.current_cmd.p;
      }
      else
      {
        oss
          << motor.joint_name << "(id=" << motor.id << ",slot=" << motor.local_slot << ")"
          << " OFFLINE";
      }
    }
    oss << " [" << bus_online << "/" << bus.motors.size() << "]";
  }

  if (online_total == total_motor_count_)
  {
    RCLCPP_INFO(
      get_logger(), "[JointAngles] online=%zu/%zu current/target(rad) | %s", online_total,
      total_motor_count_, oss.str().c_str());
  }
  else
  {
    RCLCPP_INFO(
      get_logger(), "[LinkStatus] online=%zu/%zu | %s", online_total, total_motor_count_,
      oss.str().c_str());
  }
}

}  // namespace deep_motor_ros
