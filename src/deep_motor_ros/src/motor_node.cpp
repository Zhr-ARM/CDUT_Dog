#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

extern "C" {
#include "deep_motor_ros/deep_motor_sdk.h"
}

using namespace std::chrono_literals;

namespace
{

constexpr size_t kValuesPerJoint = 5;
const std::array<std::string, 4> kLegOrder = {"LF", "LH", "RF", "RH"};
const std::array<std::string, 3> kJointOrder = {"HAA", "HFE", "KFE"};

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

}  // namespace

class DeepMotorNode : public rclcpp::Node
{
public:
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
    MotorCMD * cmd_struct{nullptr};
    MotorDATA * data_struct{nullptr};
    ControlCommand current_cmd;
    MotorState current_state;
  };

  struct BusContext
  {
    std::string can_interface;
    DrMotorCan * can_ctx{nullptr};
    std::vector<MotorContext> motors;
  };

  DeepMotorNode()
  : Node("deep_motor_node")
  {
    declare_parameter<std::vector<std::string>>("can_interfaces", std::vector<std::string>{});
    declare_parameter<std::string>("can_interface", "can0");
    declare_parameter<int>("motors_per_can", 3);
    declare_parameter<std::vector<int64_t>>("motor_ids", {0, 1, 2});
    declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    declare_parameter<std::string>("command_topic", "/motor_cmd");
    declare_parameter<std::string>("feedback_topic", "/motor_feedback");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<bool>("configure_can", false);
    declare_parameter<int>("can_bitrate", 1000000);
    declare_parameter<int>("can_txqueuelen", 1000);
    declare_parameter<double>("control_rate_hz", 400.0);
    declare_parameter<double>("status_log_period_sec", 5.0);

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
    control_rate_hz_ = std::max(1.0, get_parameter("control_rate_hz").as_double());
    status_log_period_sec_ = std::max(0.0, get_parameter("status_log_period_sec").as_double());

    motor_ids_ = expand_motor_ids(get_parameter("motor_ids").as_integer_array());
    if (motor_ids_.empty())
    {
      RCLCPP_ERROR(get_logger(), "No valid motor_ids configuration found. Node will stay idle.");
      return;
    }

    joint_names_ = resolve_joint_names();
    total_motor_count_ = motor_ids_.size();

    if (configure_can_)
    {
      configure_can_interfaces();
    }

    initialize_buses();

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

  ~DeepMotorNode() override
  {
    for (auto & bus : buses_)
    {
      for (auto & motor : bus.motors)
      {
        if (bus.can_ctx && motor.is_connected && motor.cmd_struct && motor.data_struct)
        {
          SetNormalCMD(motor.cmd_struct, motor.id, DISABLE_MOTOR);
          SendRecv(bus.can_ctx, motor.cmd_struct, motor.data_struct);
        }
        if (motor.cmd_struct)
        {
          MotorCMDDestroy(motor.cmd_struct);
        }
        if (motor.data_struct)
        {
          MotorDATADestroy(motor.data_struct);
        }
      }

      if (bus.can_ctx)
      {
        DrMotorCanDestroy(bus.can_ctx);
      }
    }
  }

private:
  std::vector<int64_t> expand_motor_ids(const std::vector<int64_t> & configured_motor_ids) const
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

  std::vector<std::string> resolve_joint_names() const
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

  void configure_can_interfaces()
  {
    for (const auto & can_interface : can_interfaces_)
    {
      auto run_step = [this, &can_interface](const std::string & cmd) {
          const int ret = std::system(cmd.c_str());
          if (ret != 0)
          {
            RCLCPP_WARN(
              get_logger(),
              "CAN configure command failed on %s: %s",
              can_interface.c_str(), cmd.c_str());
          }
        };

      run_step("ip link set " + can_interface + " down");
      run_step("ip link set " + can_interface + " type can bitrate " + std::to_string(can_bitrate_));
      run_step(
        "ip link set " + can_interface + " txqueuelen " + std::to_string(can_txqueuelen_));
      run_step("ip link set " + can_interface + " up");
    }
  }

  void initialize_buses()
  {
    buses_.clear();
    buses_.reserve(can_interfaces_.size());

    size_t global_index = 0;
    size_t total_online = 0;

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

      bus.can_ctx = DrMotorCanCreate(bus.can_interface.c_str(), false);
      if (!bus.can_ctx)
      {
        RCLCPP_ERROR(
          get_logger(), "[Fatal] CAN interface initialization failed: %s",
          bus.can_interface.c_str());
        buses_.push_back(std::move(bus));
        continue;
      }

      size_t bus_online = 0;
      for (auto & motor : bus.motors)
      {
        motor.cmd_struct = MotorCMDCreate();
        motor.data_struct = MotorDATACreate();

        if (!motor.cmd_struct || !motor.data_struct)
        {
          RCLCPP_ERROR(
            get_logger(), "[Fatal] Memory allocation failed for %s(id=%d).",
            motor.joint_name.c_str(), motor.id);
          continue;
        }

        if (enable_and_verify_motor(bus, motor))
        {
          ++bus_online;
        }
      }

      total_online += bus_online;
      RCLCPP_INFO(
        get_logger(), "[LinkSummary] %s online=%zu/%zu", bus.can_interface.c_str(), bus_online,
        bus.motors.size());
      buses_.push_back(std::move(bus));
    }

    RCLCPP_INFO(
      get_logger(),
      "[RobotLinkSummary] online=%zu/%zu, buses=%zu, control_rate=%.1fHz",
      total_online, total_motor_count_, buses_.size(), control_rate_hz_);
  }

  bool enable_and_verify_motor(BusContext & bus, MotorContext & motor)
  {
    bool is_verified = false;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
      SetNormalCMD(motor.cmd_struct, motor.id, ENABLE_MOTOR);
      const int ret = SendRecv(bus.can_ctx, motor.cmd_struct, motor.data_struct);
      if (ret == kNoSendRecvError && motor.data_struct->motor_id_ == motor.id)
      {
        is_verified = true;
        break;
      }
      std::this_thread::sleep_for(10ms);
    }

    motor.is_connected = is_verified;
    if (is_verified)
    {
      RCLCPP_INFO(
        get_logger(),
        "[LinkCheck] %s slot=%d joint=%s motor_id=%d -> ONLINE",
        bus.can_interface.c_str(), motor.local_slot, motor.joint_name.c_str(), motor.id);
    }
    else
    {
      RCLCPP_WARN(
        get_logger(),
        "[LinkCheck] %s slot=%d joint=%s motor_id=%d -> OFFLINE",
        bus.can_interface.c_str(), motor.local_slot, motor.joint_name.c_str(), motor.id);
    }
    return is_verified;
  }

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
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

  void control_loop()
  {
    for (auto & bus : buses_)
    {
      if (!bus.can_ctx)
      {
        continue;
      }

      for (auto & motor : bus.motors)
      {
        if (!motor.is_connected || !motor.cmd_struct || !motor.data_struct)
        {
          continue;
        }

        SetMotionCMD(
          motor.cmd_struct, motor.id, CONTROL_MOTOR, motor.current_cmd.p, motor.current_cmd.v,
          motor.current_cmd.t, motor.current_cmd.kp, motor.current_cmd.kd);

        const int ret = SendRecv(bus.can_ctx, motor.cmd_struct, motor.data_struct);
        if (ret != kNoSendRecvError)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[CommWarn] %s joint=%s motor_id=%d SendRecv failed: %d",
            bus.can_interface.c_str(), motor.joint_name.c_str(), motor.id, ret);
          continue;
        }

        if (motor.data_struct->motor_id_ != motor.id)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[CommWarn] %s joint=%s expected motor_id=%d but got %u",
            bus.can_interface.c_str(), motor.joint_name.c_str(), motor.id,
            static_cast<unsigned>(motor.data_struct->motor_id_));
          continue;
        }

        motor.current_state.p = motor.data_struct->position_;
        motor.current_state.v = motor.data_struct->velocity_;
        motor.current_state.t = motor.data_struct->torque_;

        if (motor.data_struct->flag_)
        {
          motor.current_state.motor_temp = motor.data_struct->temp_;
        }
        else
        {
          motor.current_state.driver_temp = motor.data_struct->temp_;
        }
      }
    }

    publish_feedback();
  }

  void publish_feedback()
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

  void log_status_summary() const
  {
    if (buses_.empty())
    {
      return;
    }

    size_t online_total = 0;
    std::ostringstream oss;
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
        oss
          << motor.joint_name << "(id=" << motor.id << ",slot=" << motor.local_slot << ")"
          << (motor.is_connected ? " ONLINE" : " OFFLINE");
      }
      oss << " [" << bus_online << "/" << bus.motors.size() << "]";
    }

    RCLCPP_INFO(
      get_logger(), "[LinkStatus] online=%zu/%zu | %s", online_total, total_motor_count_,
      oss.str().c_str());
  }

  std::vector<BusContext> buses_;
  std::vector<std::string> can_interfaces_;
  std::vector<int64_t> motor_ids_;
  std::vector<std::string> joint_names_;
  size_t total_motor_count_{0};
  int motors_per_can_{3};
  std::string command_topic_;
  std::string feedback_topic_;
  std::string joint_state_topic_;
  bool configure_can_{false};
  int can_bitrate_{1000000};
  int can_txqueuelen_{1000};
  double control_rate_hz_{400.0};
  double status_log_period_sec_{5.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeepMotorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
