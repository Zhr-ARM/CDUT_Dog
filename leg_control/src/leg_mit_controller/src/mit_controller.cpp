#include "leg_mit_controller/mit_controller.hpp"

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace leg_mit_controller
{

controller_interface::CallbackReturn MitController::on_init()
{
  try
  {
    // 定义参数及其默认值
    // joints: 要控制的关节名称列表
    auto_declare<std::vector<std::string>>("joints", {});
    // mode: "sim_effort_pd" (控制器计算扭矩) 或 "passthrough" (直接发送所有指令给硬件)
    auto_declare<std::string>("mode", "sim_effort_pd");
    // command_topic: 接收控制指令的话题名称
    auto_declare<std::string>("command_topic", "motor_cmd");
    // feedback_topic: 发布状态反馈的话题名称
    auto_declare<std::string>("feedback_topic", "motor_feedback");
    // publish_feedback: 启用/禁用反馈发布
    auto_declare<bool>("publish_feedback", true);
    // command_timeout_ms: 命令超时安全看门狗（毫秒）。
    // 若在此时间内未收到新命令，输出将归零保护；
    // 一旦收到新命令，控制器将立即自动恢复工作。
    auto_declare<int>("command_timeout_ms", 200);
    // feedback_rate_hz: 反馈发布频率
    auto_declare<double>("feedback_rate_hz", 20.0);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 从参数服务器获取参数
  joints_ = get_node()->get_parameter("joints").as_string_array();
  mode_ = get_node()->get_parameter("mode").as_string();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  feedback_topic_ = get_node()->get_parameter("feedback_topic").as_string();
  publish_feedback_ = get_node()->get_parameter("publish_feedback").as_bool();
  command_timeout_ms_ = get_node()->get_parameter("command_timeout_ms").as_int();
  feedback_rate_hz_ = get_node()->get_parameter("feedback_rate_hz").as_double();

  // 验证参数有效性
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

  // 使用默认值初始化实时命令缓冲区
  // 防止在第一个更新周期中访问未初始化的内存
  std::vector<Command5> init_cmd;
  init_cmd.resize(joints_.size());
  rt_command_buffer_.writeFromNonRT(init_cmd);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MitController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 请求每个关节的位置和速度状态接口
  // 这些是 PD 控制循环计算所需的
  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }

  return config;
}

controller_interface::InterfaceConfiguration MitController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 根据选择的模式定义命令接口
  if (mode_ == "passthrough")
  {
    // 在透传模式下，我们将所有 5 个参数发送到硬件接口
    // 这假设硬件接口具有自定义接口，名称为 'kp', 'kd', 't_ff' 等
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
    // 在 sim_effort_pd 模式下，控制器计算力矩并仅发送 'effort'
    // 这是用于仿真或简单的力矩控制机器人的标准操作模式
    for (const auto & joint : joints_)
    {
      config.names.push_back(joint + "/effort");
    }
  }

  return config;
}

controller_interface::CallbackReturn MitController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 将时间戳重置为当前时间，以避免立即超时或陈旧的反馈
  last_command_time_ = get_node()->now();
  last_feedback_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());

  // 订阅控制话题
  command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    command_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&MitController::command_callback, this, std::placeholders::_1));

  // 如果启用，初始化反馈发布者
  if (publish_feedback_)
  {
    feedback_pub_raw_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      feedback_topic_, rclcpp::SystemDefaultsQoS());
    // 创建实时安全发布者以避免阻塞控制循环
    feedback_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      feedback_pub_raw_);
    
    // 预先分配消息内存，避免在实时循环中动态分配
    feedback_pub_->msg_.data.resize(joints_.size() * 5);
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "MitController active. mode=%s, joints=%zu, cmd_topic=%s", mode_.c_str(), joints_.size(),
    command_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 清理 ROS 通信资源
  command_sub_.reset();
  feedback_pub_.reset();
  feedback_pub_raw_.reset();

  // 在停止前为了安全将所有命令接口归零
  for (auto & cmd_if : command_interfaces_)
  {
    cmd_if.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void MitController::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // 检查消息大小是否正确
  // 期望布局: [joint1_p, joint1_v, joint1_t, joint1_kp, joint1_kd, joint2_p, ...]
  // 总大小 = 关节数量 * 5
  const size_t expected_size = joints_.size() * 5;
  if (msg->data.size() != expected_size)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "motor_cmd size mismatch. expected=%zu actual=%zu", expected_size, msg->data.size());
    return;
  }

  // 将消息解析为 Command5 结构体并写入实时缓冲区
  // 此操作必须是实时线程无锁可读的
  std::vector<Command5> cmd;
  cmd.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    const size_t base = i * 5;
    cmd[i].p = msg->data[base + 0];
    cmd[i].v = msg->data[base + 1];
    cmd[i].t_ff = msg->data[base + 2];
    cmd[i].kp = msg->data[base + 3];
    cmd[i].kd = msg->data[base + 4];
  }

  // 更新最后命令时间以进行超时监控
  last_command_time_ = get_node()->now();
  rt_command_buffer_.writeFromNonRT(cmd);
}

controller_interface::return_type MitController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // 从实时缓冲区读取最新指令
  const auto cmd_ptr = rt_command_buffer_.readFromRT();
  if (!cmd_ptr)
  {
    return controller_interface::return_type::OK;
  }

  // 安全超时检查：如果在超时时间内未收到新指令，输出归零
  const rclcpp::Duration timeout(0, static_cast<int64_t>(command_timeout_ms_) * 1000000LL);
  const bool is_timed_out = (time - last_command_time_) > timeout;

  if (mode_ == "passthrough")
  {
    // 透传模式：直接将指令映射到硬件接口
    // 假设硬件接口为：position, velocity, effort, kp, kd
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      const auto & c = (*cmd_ptr)[i];
      // 如果超时则输出0，否则使用指令值
      const double p = is_timed_out ? 0.0 : c.p;
      const double v = is_timed_out ? 0.0 : c.v;
      const double t = is_timed_out ? 0.0 : c.t_ff;
      const double kp = is_timed_out ? 0.0 : c.kp;
      const double kd = is_timed_out ? 0.0 : c.kd;

      // 设置命令接口值（顺序与 command_interface_configuration 匹配）
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
    // 仿真力矩 PD 模式：使用 PD 控制律计算力矩并发送到 effort 接口
    // 适用于仿真或仅接受力矩命令的硬件
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      // 从状态接口获取当前状态
      const double p_cur = state_interfaces_[2 * i + 0].get_value();
      const double v_cur = state_interfaces_[2 * i + 1].get_value();

      const auto & c = (*cmd_ptr)[i];
      // 力矩 = Kp*(期望位置 - 当前位置) + Kd*(期望速度 - 当前速度) + 前馈力矩
      const double tau = is_timed_out ? 0.0 : (c.kp * (c.p - p_cur) + c.kd * (c.v - v_cur) + c.t_ff);
      
      // 设置 effort 命令接口
      command_interfaces_[i].set_value(tau);
    }
  }

  // 反馈发布（按限制频率）
  if (publish_feedback_ && feedback_pub_)
  {
    // 检查是否自上次发布以来经过了足够的时间
    const double min_period = (feedback_rate_hz_ <= 0.0) ? 0.0 : (1.0 / feedback_rate_hz_);
    if (min_period <= 0.0 || (time - last_feedback_time_).seconds() >= min_period)
    {
      // 尝试获取发布锁（非阻塞）
      if (feedback_pub_->trylock())
      {
        auto & data = feedback_pub_->msg_.data;

        for (size_t i = 0; i < joints_.size(); ++i)
        {
          const double p_cur = state_interfaces_[2 * i + 0].get_value();
          const double v_cur = state_interfaces_[2 * i + 1].get_value();
          double tau = 0.0;
          
          // 如果在 PD 模式下，报告计算出的力矩
          if (mode_ == "sim_effort_pd" && !is_timed_out)
          {
            const auto & c = (*cmd_ptr)[i];
            tau = c.kp * (c.p - p_cur) + c.kd * (c.v - v_cur) + c.t_ff;
          }

          // 填充反馈消息：[p, v, tau, 0, 0]
          const size_t base = i * 5;
          data[base + 0] = p_cur;
          data[base + 1] = v_cur;
          data[base + 2] = tau;
          data[base + 3] = 0.0; // 预留
          data[base + 4] = 0.0; // 预留
        }

        feedback_pub_->unlockAndPublish();
        last_feedback_time_ = time;
      }
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace leg_mit_controller

PLUGINLIB_EXPORT_CLASS(leg_mit_controller::MitController, controller_interface::ControllerInterface)
