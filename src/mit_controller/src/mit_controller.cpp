#include "mit_controller/mit_controller.hpp"

#include <algorithm>
#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace mit_controller
{

// 这个控制器是高层步态节点与 ros2_control 之间的协议适配层。
// 它接收 MIT 风格的 5 元组命令:
//   [position, velocity, torque_feedforward, kp, kd]
// 然后根据运行模式把这组命令映射到 ros2_control 的命令接口上。
//
// - sim_effort_pd:
//   这是 Gazebo 仿真中的主要模式。控制器在本地读取关节位置/速度，
//   按 MIT PD 公式计算力矩，再把结果写入 effort 接口。
// - passthrough:
//   这是“透传”模式，直接把 5 个字段写入底层硬件接口。适用于底层
//   硬件接口自己就支持 MIT 五元组语义的场景。

/// 在生命周期早期统一声明参数。
/// 这样做有两个目的:
/// 1. 让 lifecycle 节点在 configure/activate 阶段可以稳定读取参数。
/// 2. 把控制器支持的外部配置项集中在一个入口，便于排查运行时行为。
controller_interface::CallbackReturn MitController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("mode", "sim_effort_pd");
    auto_declare<std::string>("command_topic", "motor_cmd");
    auto_declare<std::string>("feedback_topic", "motor_feedback");
    auto_declare<bool>("publish_feedback", true);
    auto_declare<int>("command_timeout_ms", 200);
    auto_declare<double>("feedback_rate_hz", 20.0);
    auto_declare<std::vector<double>>("effort_limits", {});
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/// 读取并校验参数，随后初始化实时命令缓冲区。
///
/// 这里重点做三件事:
/// 1. 解析 joints / mode / topic / timeout / effort limit 等运行参数。
/// 2. 校验配置是否自洽，避免控制器进入激活态后才暴露参数错误。
/// 3. 用全零命令填充 realtime buffer，保证 update() 在收到第一帧
///    外部命令之前也有一份确定的默认输入。
controller_interface::CallbackReturn MitController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  mode_ = get_node()->get_parameter("mode").as_string();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  feedback_topic_ = get_node()->get_parameter("feedback_topic").as_string();
  publish_feedback_ = get_node()->get_parameter("publish_feedback").as_bool();
  command_timeout_ms_ = get_node()->get_parameter("command_timeout_ms").as_int();
  feedback_rate_hz_ = get_node()->get_parameter("feedback_rate_hz").as_double();
  effort_limits_ = get_node()->get_parameter("effort_limits").as_double_array();

  if (joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (mode_ != "sim_effort_pd" && mode_ != "passthrough")
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Unsupported mode '%s'. Use 'sim_effort_pd' or 'passthrough'.", mode_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!effort_limits_.empty() && effort_limits_.size() != 1 && effort_limits_.size() != joints_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'effort_limits' must be empty, contain 1 value, or match joints size (%zu).",
      joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (effort_limits_.size() == 1 && joints_.size() > 1)
  {
    effort_limits_.assign(joints_.size(), effort_limits_.front());
  }

  std::vector<Command5> init_cmd(joints_.size());
  rt_command_buffer_.writeFromNonRT(init_cmd);

  return controller_interface::CallbackReturn::SUCCESS;
}

/// 声明状态接口需求。
/// 对于当前控制器来说，只需要 position 和 velocity 两类状态量，
/// 这已经足够在仿真里计算 MIT PD 力矩。
controller_interface::InterfaceConfiguration MitController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }

  return config;
}

/// 根据模式声明命令接口。
///
/// - passthrough:
///   直接导出 position / velocity / effort / kp / kd 五类接口，
///   由下层硬件接管 MIT 命令的解释。
/// - sim_effort_pd:
///   只导出 effort，因为位置误差、速度误差和增益都在本控制器内
///   部完成计算，最终对仿真硬件只需要写入力矩。
controller_interface::InterfaceConfiguration MitController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (mode_ == "passthrough")
  {
    for (const auto & joint : joints_)
    {
      config.names.push_back(joint + "/position");
      config.names.push_back(joint + "/velocity");
      config.names.push_back(joint + "/effort");
      config.names.push_back(joint + "/kp");
      config.names.push_back(joint + "/kd");
    }
  }
  else
  {
    for (const auto & joint : joints_)
    {
      config.names.push_back(joint + "/effort");
    }
  }

  return config;
}

/// 激活阶段建立 ROS 通信对象。
///
/// 设计上采用“非实时订阅 + 实时缓冲区”的常见模式:
/// - command_callback() 运行在 ROS executor 线程里，负责接收话题数据。
/// - update() 运行在 ros2_control 的实时循环里，只从 realtime buffer
///   读取已经整理好的命令，避免在实时线程里做不确定的 ROS 操作。
controller_interface::CallbackReturn MitController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_command_time_ = get_node()->now();
  last_feedback_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());

  command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    command_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&MitController::command_callback, this, std::placeholders::_1));

  if (publish_feedback_)
  {
    feedback_pub_raw_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      feedback_topic_, rclcpp::SystemDefaultsQoS());
    feedback_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      feedback_pub_raw_);
    feedback_pub_->msg_.data.resize(joints_.size() * 5);
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "MitController active. mode=%s, joints=%zu, cmd_topic=%s",
    mode_.c_str(), joints_.size(), command_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

/// 失活时释放 ROS 通信资源，并把所有命令接口清零。
/// 这样可以避免控制器退出后底层仍然保留上一帧输出，属于一个基础的
/// 安全收尾动作。
controller_interface::CallbackReturn MitController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_sub_.reset();
  feedback_pub_.reset();
  feedback_pub_raw_.reset();

  for (auto & cmd_if : command_interfaces_)
  {
    cmd_if.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/// 解析来自上层步态节点的扁平化 MIT 命令数组。
///
/// 数组布局约定为:
///   joint_0: [p, v, t_ff, kp, kd]
///   joint_1: [p, v, t_ff, kp, kd]
///   ...
///
/// 这个回调本身不直接写 command interface，只做两件事:
/// 1. 把消息拆包为按关节组织的 Command5 向量。
/// 2. 更新“最近一次收到命令”的时间戳，并写入 realtime buffer。
void MitController::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  const size_t expected_size = joints_.size() * 5;
  if (msg->data.size() != expected_size)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "motor_cmd size mismatch. expected=%zu actual=%zu",
      expected_size, msg->data.size());
    return;
  }

  std::vector<Command5> cmd(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    const size_t base = i * 5;
    cmd[i].p = msg->data[base + 0];
    cmd[i].v = msg->data[base + 1];
    cmd[i].t_ff = msg->data[base + 2];
    cmd[i].kp = msg->data[base + 3];
    cmd[i].kd = msg->data[base + 4];
  }

  last_command_time_ = get_node()->now();
  rt_command_buffer_.writeFromNonRT(cmd);
}

/// ros2_control 的实时更新入口。
///
/// 这个函数承担了控制器的核心实时职责:
/// 1. 从 realtime buffer 读取最新目标命令。
/// 2. 根据超时策略决定是否继续执行命令。
/// 3. 按 mode 选择“透传五元组”还是“本地算力矩”。
/// 4. 以较低频率发布反馈，供外部调试或上层观测使用。
///
/// 安全策略:
/// - 一旦命令超时，不保持旧目标，而是输出零值，避免执行陈旧控制。
/// - 力矩限幅发生在 PD 计算之后，语义上更接近执行器饱和。
controller_interface::return_type MitController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const auto cmd_ptr = rt_command_buffer_.readFromRT();
  if (!cmd_ptr)
  {
    return controller_interface::return_type::OK;
  }

  const rclcpp::Duration timeout(0, static_cast<int64_t>(command_timeout_ms_) * 1000000LL);
  const bool is_timed_out = (time - last_command_time_) > timeout;

  if (mode_ == "passthrough")
  {
    // 透传模式下，控制器不再解释 MIT 命令的物理含义，而是把五元组
    // 原样写到下层接口。超时后统一写 0，防止底层继续执行陈旧目标。
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      const auto & c = (*cmd_ptr)[i];
      const double p = is_timed_out ? 0.0 : c.p;
      const double v = is_timed_out ? 0.0 : c.v;
      const double t = is_timed_out ? 0.0 : c.t_ff;
      const double kp = is_timed_out ? 0.0 : c.kp;
      const double kd = is_timed_out ? 0.0 : c.kd;

      const size_t base = i * 5;
      command_interfaces_[base + 0].set_value(p);
      command_interfaces_[base + 1].set_value(v);
      command_interfaces_[base + 2].set_value(t);
      command_interfaces_[base + 3].set_value(kp);
      command_interfaces_[base + 4].set_value(kd);
    }
  }
  else
  {
    // 仿真模式下，本控制器本地实现 MIT PD:
    //   tau = kp * (p_des - p_cur) + kd * (v_des - v_cur) + t_ff
    // 最后只把计算后的力矩写入 effort 接口。
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      const double p_cur = state_interfaces_[2 * i + 0].get_value();
      const double v_cur = state_interfaces_[2 * i + 1].get_value();
      const auto & c = (*cmd_ptr)[i];
      const double tau =
        is_timed_out ? 0.0 : (c.kp * (c.p - p_cur) + c.kd * (c.v - v_cur) + c.t_ff);
      const double effort_limit =
        effort_limits_.empty() ? 0.0 : std::max(0.0, effort_limits_[i]);
      const double clamped_tau =
        effort_limit > 0.0 ? std::clamp(tau, -effort_limit, effort_limit) : tau;

      command_interfaces_[i].set_value(clamped_tau);
    }
  }

  if (publish_feedback_ && feedback_pub_)
  {
    // 反馈发布频率独立于控制频率配置，避免每个 update tick 都做发布，
    // 同时保持调试数据的带宽可控。
    const double min_period = (feedback_rate_hz_ <= 0.0) ? 0.0 : (1.0 / feedback_rate_hz_);
    if (min_period <= 0.0 || (time - last_feedback_time_).seconds() >= min_period)
    {
      if (feedback_pub_->trylock())
      {
        auto & data = feedback_pub_->msg_.data;

        for (size_t i = 0; i < joints_.size(); ++i)
        {
          const double p_cur = state_interfaces_[2 * i + 0].get_value();
          const double v_cur = state_interfaces_[2 * i + 1].get_value();
          double tau = 0.0;

          if (mode_ == "sim_effort_pd" && !is_timed_out)
          {
            // 反馈里的 tau 与真正写入 effort 接口的计算逻辑保持一致，
            // 这样外部看到的是“控制器实际打算施加的力矩”，而不是仅仅
            // 把原始 t_ff 原样回显。
            const auto & c = (*cmd_ptr)[i];
            tau = c.kp * (c.p - p_cur) + c.kd * (c.v - v_cur) + c.t_ff;
            const double effort_limit =
              effort_limits_.empty() ? 0.0 : std::max(0.0, effort_limits_[i]);
            if (effort_limit > 0.0)
            {
              tau = std::clamp(tau, -effort_limit, effort_limit);
            }
          }

          const size_t base = i * 5;
          data[base + 0] = p_cur;
          data[base + 1] = v_cur;
          data[base + 2] = tau;
          data[base + 3] = 0.0;
          data[base + 4] = 0.0;
        }

        feedback_pub_->unlockAndPublish();
        last_feedback_time_ = time;
      }
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace mit_controller

PLUGINLIB_EXPORT_CLASS(mit_controller::MitController, controller_interface::ControllerInterface)
