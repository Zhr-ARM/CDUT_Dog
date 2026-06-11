#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "arm_control/srv/plan_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using PlanMove = arm_control::srv::PlanMove;

namespace
{
struct BoxTarget
{
  std::string label;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  bool use_level_constraint{false};
  bool enable_suction_after{false};
  bool disable_suction_after{false};
};

struct BusyGuard
{
  explicit BusyGuard(std::atomic_bool & busy_flag) : busy(busy_flag) {}
  ~BusyGuard() { busy.store(false); }

  std::atomic_bool & busy;
};

enum class BoxPlaceMode
{
  Ground,
  Stack,
};

std::string normalized_mode(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

const char * mode_name(const BoxPlaceMode mode)
{
  switch (mode) {
    case BoxPlaceMode::Ground:
      return "ground";
    case BoxPlaceMode::Stack:
      return "stack";
  }
  return "ground";
}
}  // namespace

volatile std::sig_atomic_t g_shutdown_signal_requested = 0;

void shutdown_signal_handler(int)
{
  g_shutdown_signal_requested = 1;
}

bool shutdown_signal_requested()
{
  return g_shutdown_signal_requested != 0;
}

class BoxMotionNode : public rclcpp::Node
{
public:
  explicit BoxMotionNode(const rclcpp::NodeOptions & options)
  : Node("box_motion_node", options)
  {
    declare_parameters();
    read_parameters();

    plan_client_ = create_client<PlanMove>(plan_service_name_);
    suction_publisher_ = create_publisher<std_msgs::msg::Bool>(suction_command_topic_, 10);
    vision_mode_publisher_ =
      create_publisher<std_msgs::msg::Bool>(vision_mode_topic_, rclcpp::QoS(1).transient_local());
    home_publisher_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_trajectory_topic_, 10);

    box_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      box_motion_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_box_motion(default_box_place_mode_);
      });

    home_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      home_motion_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_home_motion();
      });

    vision_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      vision_motion_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_vision_motion();
      });

    box_ground_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      box_motion_ground_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_box_motion(BoxPlaceMode::Ground);
      });

    box_stack_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      box_motion_stack_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_box_motion(BoxPlaceMode::Stack);
      });

    box_suction_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      box_suction_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_suction_motion();
      });

    box_lift_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      box_lift_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_lift_motion();
      });

    box_place_ground_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      box_place_ground_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_place_motion(BoxPlaceMode::Ground);
      });

    box_place_stack_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      box_place_stack_start_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        start_place_motion(BoxPlaceMode::Stack);
      });

    shutdown_watch_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (!shutdown_signal_requested()) {
          return;
        }
        shutdown_to_home();
        rclcpp::shutdown();
      });

    if (start_in_vision_mode_) {
      startup_vision_timer_ = create_wall_timer(
        duration_from_seconds(vision_start_delay_s_),
        [this]() {
          startup_vision_timer_->cancel();
          start_vision_motion();
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "box_motion_node ready: box_start=%s (%s), ground_start=%s, stack_start=%s, "
      "suction_start=%s, lift_start=%s, place_ground_start=%s, place_stack_start=%s, "
      "home_start=%s, vision_start=%s, suction=%s, vision_mode=%s",
      box_motion_start_topic_.c_str(),
      mode_name(default_box_place_mode_),
      box_motion_ground_start_topic_.c_str(),
      box_motion_stack_start_topic_.c_str(),
      box_suction_start_topic_.c_str(),
      box_lift_start_topic_.c_str(),
      box_place_ground_start_topic_.c_str(),
      box_place_stack_start_topic_.c_str(),
      home_motion_start_topic_.c_str(),
      vision_motion_start_topic_.c_str(),
      suction_command_topic_.c_str(),
      vision_mode_topic_.c_str());
  }

  ~BoxMotionNode() override
  {
    stop_requested_.store(true);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void shutdown_to_home()
  {
    if (shutdown_home_started_.exchange(true)) {
      return;
    }

    RCLCPP_WARN(get_logger(), "收到退出信号，停止当前箱子动作并回原位。");
    stop_requested_.store(true);
    publish_suction_command(false);
    publish_home_trajectory();

    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (worker_.joinable()) {
        worker_.join();
      }
    }

    std::this_thread::sleep_for(duration_from_seconds(home_duration_s_));
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("plan_service_name", "plan_moveit");
    declare_parameter<std::string>("box_motion_start_topic", "/arm_box_motion/start");
    declare_parameter<std::string>(
      "box_motion_ground_start_topic", "/arm_box_motion/start_ground");
    declare_parameter<std::string>("box_motion_stack_start_topic", "/arm_box_motion/start_stack");
    declare_parameter<std::string>("box_suction_start_topic", "/arm_box_motion/suction");
    declare_parameter<std::string>("box_lift_start_topic", "/arm_box_motion/lift");
    declare_parameter<std::string>(
      "box_place_ground_start_topic", "/arm_box_motion/place_ground");
    declare_parameter<std::string>("box_place_stack_start_topic", "/arm_box_motion/place_stack");
    declare_parameter<std::string>("home_motion_start_topic", "/arm_home_motion/start");
    declare_parameter<std::string>("vision_motion_start_topic", "/arm_vision_motion/start");
    declare_parameter<std::string>("suction_command_topic", "/suction/enable");
    declare_parameter<std::string>("vision_mode_topic", "/arm_vision_mode/enabled");
    declare_parameter<std::string>("joint_trajectory_topic", "/arm_controller/joint_trajectory");
    declare_parameter<std::string>("default_box_place_mode", "ground");

    declare_parameter<std::vector<double>>(
      "home_joint_positions", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    declare_parameter<double>("home_duration_s", 4.0);
    declare_parameter<bool>("start_in_vision_mode", true);
    declare_parameter<std::vector<double>>(
      "vision_joint_positions", std::vector<double>{0.0, 0.0, 0.0, -1.57079632679});
    declare_parameter<double>("vision_duration_s", 3.0);
    declare_parameter<double>("vision_start_delay_s", 3.0);

    declare_parameter<double>("box_center_distance_m", 0.42);
    declare_parameter<double>("platform_height_m", 0.0);
    declare_parameter<double>("box_ground_z_m", 0.0);
    declare_parameter<double>("box_height_m", 0.25);
    declare_parameter<double>("box_motion_y_m", 0.0);
    declare_parameter<double>("box_place_x_m", 0.0);
    declare_parameter<double>("box_place_y_m", 0.0);
    declare_parameter<double>("box_approach_offset_m", 0.02);
    declare_parameter<double>("box_suction_contact_offset_m", 0.01);
    declare_parameter<double>("box_touch_push_m", 0.005);
    declare_parameter<double>("box_suction_target_x_m", 0.0);
    declare_parameter<double>("box_suction_settle_s", 0.8);
    declare_parameter<double>("box_release_settle_s", 0.8);
    declare_parameter<double>("box_lift_height_m", 0.08);
    declare_parameter<double>("box_lift_retract_m", 0.04);
    declare_parameter<double>("box_min_approach_x_m", 0.14);
    declare_parameter<double>("stack_base_box_height_m", 0.25);
    declare_parameter<double>("stack_carry_clearance_m", 0.08);
    declare_parameter<double>("box_motion_step_wait_s", 0.5);
    declare_parameter<double>("box_motion_position_tolerance_m", 0.04);
    declare_parameter<bool>("box_use_level_constraint", true);
    declare_parameter<double>("box_wrist_level_offset_rad", 0.0);
    declare_parameter<bool>("box_use_target_orientation", true);
    declare_parameter<std::vector<double>>(
      "box_target_orientation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    declare_parameter<double>("box_target_orientation_tolerance_rad", 0.20);
    declare_parameter<double>("service_wait_timeout_s", 5.0);
    declare_parameter<double>("plan_response_timeout_s", 60.0);
  }

  void read_parameters()
  {
    plan_service_name_ = get_parameter("plan_service_name").as_string();
    box_motion_start_topic_ = get_parameter("box_motion_start_topic").as_string();
    box_motion_ground_start_topic_ =
      get_parameter("box_motion_ground_start_topic").as_string();
    box_motion_stack_start_topic_ = get_parameter("box_motion_stack_start_topic").as_string();
    box_suction_start_topic_ = get_parameter("box_suction_start_topic").as_string();
    box_lift_start_topic_ = get_parameter("box_lift_start_topic").as_string();
    box_place_ground_start_topic_ = get_parameter("box_place_ground_start_topic").as_string();
    box_place_stack_start_topic_ = get_parameter("box_place_stack_start_topic").as_string();
    home_motion_start_topic_ = get_parameter("home_motion_start_topic").as_string();
    vision_motion_start_topic_ = get_parameter("vision_motion_start_topic").as_string();
    suction_command_topic_ = get_parameter("suction_command_topic").as_string();
    vision_mode_topic_ = get_parameter("vision_mode_topic").as_string();
    joint_trajectory_topic_ = get_parameter("joint_trajectory_topic").as_string();
    default_box_place_mode_ = read_box_place_mode_parameter("default_box_place_mode");

    home_joint_positions_ = get_parameter("home_joint_positions").as_double_array();
    if (home_joint_positions_.size() != 4) {
      RCLCPP_WARN(
        get_logger(),
        "home_joint_positions 必须包含 4 个关节角，当前配置无效，已回退为全 0。");
      home_joint_positions_ = {0.0, 0.0, 0.0, 0.0};
    }

    home_duration_s_ = positive_parameter("home_duration_s", 0.1);
    start_in_vision_mode_ = get_parameter("start_in_vision_mode").as_bool();
    vision_joint_positions_ = get_parameter("vision_joint_positions").as_double_array();
    if (vision_joint_positions_.size() != 4) {
      RCLCPP_WARN(
        get_logger(),
        "vision_joint_positions 必须包含 4 个关节角，当前配置无效，已回退为仅腕部 -90 度。");
      vision_joint_positions_ = {0.0, 0.0, 0.0, -1.57079632679};
    }
    vision_duration_s_ = positive_parameter("vision_duration_s", 0.1);
    vision_start_delay_s_ = non_negative_parameter("vision_start_delay_s");
    box_center_distance_m_ = positive_parameter("box_center_distance_m", 0.01);
    platform_height_m_ = get_parameter("platform_height_m").as_double();
    box_ground_z_m_ = get_parameter("box_ground_z_m").as_double();
    box_height_m_ = positive_parameter("box_height_m", 0.01);
    box_motion_y_m_ = get_parameter("box_motion_y_m").as_double();
    box_place_x_m_ = get_parameter("box_place_x_m").as_double();
    box_place_y_m_ = get_parameter("box_place_y_m").as_double();
    box_approach_offset_m_ = non_negative_parameter("box_approach_offset_m");
    box_suction_contact_offset_m_ = non_negative_parameter("box_suction_contact_offset_m");
    box_touch_push_m_ = non_negative_parameter("box_touch_push_m");
    box_suction_target_x_m_ = get_parameter("box_suction_target_x_m").as_double();
    if (box_suction_target_x_m_ <= 0.0) {
      box_suction_target_x_m_ = box_center_distance_m_;
    }
    box_suction_settle_s_ = non_negative_parameter("box_suction_settle_s");
    box_release_settle_s_ = non_negative_parameter("box_release_settle_s");
    box_lift_height_m_ = non_negative_parameter("box_lift_height_m");
    box_lift_retract_m_ = non_negative_parameter("box_lift_retract_m");
    box_min_approach_x_m_ = positive_parameter("box_min_approach_x_m", 0.01);
    stack_base_box_height_m_ = positive_parameter("stack_base_box_height_m", 0.01);
    stack_carry_clearance_m_ = non_negative_parameter("stack_carry_clearance_m");
    box_motion_step_wait_s_ = non_negative_parameter("box_motion_step_wait_s");
    box_motion_position_tolerance_m_ =
      positive_parameter("box_motion_position_tolerance_m", 0.005);
    box_use_level_constraint_ = get_parameter("box_use_level_constraint").as_bool();
    box_wrist_level_offset_rad_ = get_parameter("box_wrist_level_offset_rad").as_double();
    box_use_target_orientation_ = get_parameter("box_use_target_orientation").as_bool();
    box_target_orientation_xyzw_ =
      get_parameter("box_target_orientation_xyzw").as_double_array();
    if (box_target_orientation_xyzw_.size() != 4) {
      RCLCPP_WARN(
        get_logger(),
        "box_target_orientation_xyzw 必须包含 4 个数，当前配置无效，已回退为 [0,0,0,1]。");
      box_target_orientation_xyzw_ = {0.0, 0.0, 0.0, 1.0};
    }
    box_target_orientation_tolerance_rad_ =
      positive_parameter("box_target_orientation_tolerance_rad", 0.01);
    service_wait_timeout_s_ = positive_parameter("service_wait_timeout_s", 0.1);
    plan_response_timeout_s_ = positive_parameter("plan_response_timeout_s", 0.1);
  }

  double positive_parameter(const std::string & name, const double minimum) const
  {
    return std::max(minimum, get_parameter(name).as_double());
  }

  double non_negative_parameter(const std::string & name) const
  {
    return std::max(0.0, get_parameter(name).as_double());
  }

  BoxPlaceMode read_box_place_mode_parameter(const std::string & name)
  {
    const auto value = normalized_mode(get_parameter(name).as_string());
    if (value == "stack" || value == "stacked" || value == "pile") {
      return BoxPlaceMode::Stack;
    }
    if (value != "ground" && value != "floor") {
      RCLCPP_WARN(
        get_logger(),
        "未知箱子放置模式 '%s'，已使用 ground。可选值：ground/stack。",
        value.c_str());
    }
    return BoxPlaceMode::Ground;
  }

  void start_box_motion(const BoxPlaceMode mode)
  {
    start_worker("完整箱子任务", [this, mode]() { run_box_motion(mode); });
  }

  void start_suction_motion()
  {
    start_worker("吸箱子动作", [this]() {
      if (execute_suction_motion()) {
        RCLCPP_INFO(get_logger(), "吸箱子动作完成，吸泵保持开启。");
      }
    });
  }

  void start_lift_motion()
  {
    start_worker("举箱子动作", [this]() {
      if (execute_lift_motion()) {
        RCLCPP_INFO(get_logger(), "举箱子动作完成，已进入适合移动运输的姿态。");
      }
    });
  }

  void start_place_motion(const BoxPlaceMode mode)
  {
    start_worker("放箱子动作", [this, mode]() {
      if (execute_place_motion(mode)) {
        RCLCPP_INFO(get_logger(), "放箱子动作完成，已回到 vision 模式。");
      }
    });
  }

  void start_worker(const char * label, std::function<void()> work)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (busy_.load()) {
      RCLCPP_WARN(get_logger(), "箱子任务正在执行，忽略新的%s启动请求。", label);
      return;
    }

    stop_requested_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    busy_.store(true);
    worker_ = std::thread([this, label, work = std::move(work)]() {
      BusyGuard guard(busy_);
      try {
        work();
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "%s异常停止：%s", label, ex.what());
      }
    });
  }

  void start_home_motion()
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (busy_.load()) {
      stop_requested_.store(true);
      RCLCPP_WARN(get_logger(), "当前箱子任务正在执行，已请求停止；请在当前段结束后再次触发回原位。");
      return;
    }

    publish_suction_command(false);
    publish_home_trajectory();
  }

  void start_vision_motion()
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (busy_.load()) {
      RCLCPP_WARN(get_logger(), "当前箱子任务正在执行，忽略 vision 姿态请求。");
      return;
    }

    publish_vision_trajectory();
  }

  void run_box_motion(const BoxPlaceMode mode)
  {
    RCLCPP_INFO(
      get_logger(),
      "完整箱子任务开始：模式=%s。执行 vision -> 吸箱子 -> 举箱子 -> 放箱子 -> vision。",
      mode_name(mode));

    if (!execute_vision_motion()) {
      RCLCPP_WARN(get_logger(), "完整箱子任务在 vision 阶段停止。");
      return;
    }

    if (!execute_suction_motion()) {
      RCLCPP_WARN(get_logger(), "完整箱子任务在吸箱子阶段停止。");
      return;
    }

    if (!execute_lift_motion()) {
      RCLCPP_WARN(get_logger(), "完整箱子任务在举箱子阶段停止。");
      return;
    }

    if (!execute_place_motion(mode)) {
      RCLCPP_WARN(get_logger(), "完整箱子任务在放箱子阶段停止。");
      return;
    }

    RCLCPP_INFO(get_logger(), "完整箱子任务完成。");
  }

  bool execute_vision_motion()
  {
    publish_vision_trajectory();
    sleep_with_stop(vision_duration_s_);
    return !stop_requested_.load();
  }

  bool execute_suction_motion()
  {
    update_box_source_geometry();
    publish_vision_mode(false);

    RCLCPP_INFO(
      get_logger(),
      "吸箱子动作开始：吸附点 x=%.3f y=%.3f z=%.3f。",
      box_suction_target_x_,
      box_motion_y_m_,
      box_source_top_z_);

    publish_suction_command(false);
    return execute_targets(make_suction_targets(), "吸箱子动作");
  }

  bool execute_lift_motion()
  {
    update_box_source_geometry();
    publish_vision_mode(false);

    RCLCPP_INFO(
      get_logger(),
      "举箱子动作开始：运输姿态 x=%.3f y=%.3f z=%.3f，吸泵保持开启。",
      box_transport_x_,
      box_motion_y_m_,
      box_carry_z_);

    publish_suction_command(true);
    return execute_targets(make_lift_targets(), "举箱子动作");
  }

  bool execute_place_motion(const BoxPlaceMode mode)
  {
    update_box_place_geometry(mode);
    publish_vision_mode(false);

    RCLCPP_INFO(
      get_logger(),
      "放箱子动作开始：模式=%s，放置点 x=%.3f y=%.3f z=%.3f，运输高度 z=%.3f。",
      mode_name(mode),
      box_place_x_,
      box_place_y_,
      box_place_top_z_,
      box_carry_z_);

    publish_suction_command(true);
    if (!execute_targets(make_place_targets(mode), "放箱子动作")) {
      return false;
    }
    return execute_vision_motion();
  }

  bool execute_targets(const std::vector<BoxTarget> & targets, const char * action_label)
  {
    for (std::size_t index = 0; index < targets.size(); ++index) {
      if (stop_requested_.load()) {
        RCLCPP_WARN(get_logger(), "%s已停止。", action_label);
        return false;
      }

      if (!call_plan_service(targets[index])) {
        RCLCPP_WARN(get_logger(), "%s在第 %zu 步停止。", action_label, index + 1);
        return false;
      }

      if (targets[index].enable_suction_after) {
        publish_suction_command(true);
        sleep_with_stop(box_suction_settle_s_);
      } else if (targets[index].disable_suction_after) {
        publish_suction_command(false);
        sleep_with_stop(box_release_settle_s_);
      } else {
        sleep_with_stop(box_motion_step_wait_s_);
      }
    }

    return !stop_requested_.load();
  }

  void update_box_source_geometry()
  {
    box_suction_target_x_ = box_suction_target_x_m_;
    box_source_top_z_ = box_ground_z_m_ + box_height_m_ - platform_height_m_;

    box_transport_x_ =
      std::max(box_min_approach_x_m_, box_suction_target_x_ - box_lift_retract_m_);
    const double stack_place_top_z =
      box_ground_z_m_ + stack_base_box_height_m_ + box_height_m_ - platform_height_m_;
    box_carry_z_ = std::max(
      box_source_top_z_ + box_lift_height_m_,
      stack_place_top_z + stack_carry_clearance_m_);
  }

  void update_box_place_geometry(const BoxPlaceMode mode)
  {
    update_box_source_geometry();

    box_place_x_ = box_place_x_m_ > 0.0 ? box_place_x_m_ : box_transport_x_;
    box_place_y_ = box_place_y_m_;
    box_place_top_z_ = box_source_top_z_;
    if (mode == BoxPlaceMode::Stack) {
      box_place_top_z_ =
        box_ground_z_m_ + stack_base_box_height_m_ + box_height_m_ - platform_height_m_;
    }
  }

  std::vector<BoxTarget> make_suction_targets() const
  {
    const double source_approach_z = box_source_top_z_ + box_approach_offset_m_;
    const double source_pre_contact_z = box_source_top_z_ + box_suction_contact_offset_m_;
    const double source_contact_z = box_source_top_z_ - box_touch_push_m_;

    return {
      {"approach_box", box_suction_target_x_, box_motion_y_m_, source_approach_z, false},
      {"pre_suction", box_suction_target_x_, box_motion_y_m_, source_pre_contact_z, false},
      {"suction_contact", box_suction_target_x_, box_motion_y_m_, source_contact_z, true, true},
    };
  }

  std::vector<BoxTarget> make_lift_targets() const
  {
    return {
      {"lift_box", box_suction_target_x_, box_motion_y_m_, box_carry_z_, true},
      {"transport_ready", box_transport_x_, box_motion_y_m_, box_carry_z_, true},
    };
  }

  std::vector<BoxTarget> make_place_targets(const BoxPlaceMode mode) const
  {
    const double place_pre_contact_z = box_place_top_z_ + box_suction_contact_offset_m_;
    const double place_contact_z = box_place_top_z_ - box_touch_push_m_;
    const double retreat_z =
      mode == BoxPlaceMode::Stack ? box_carry_z_ : box_place_top_z_ + box_approach_offset_m_;

    return {
      {"carry_to_place", box_place_x_, box_place_y_, box_carry_z_, true},
      {"pre_place", box_place_x_, box_place_y_, place_pre_contact_z, true},
      {"place", box_place_x_, box_place_y_, place_contact_z, true, false, true},
      {"retreat", box_place_x_, box_place_y_, retreat_z, false},
    };
  }

  bool call_plan_service(const BoxTarget & target)
  {
    if (!plan_client_->wait_for_service(duration_from_seconds(service_wait_timeout_s_))) {
      RCLCPP_ERROR(
        get_logger(),
        "等待 MoveIt 规划服务 %s 超时。",
        plan_service_name_.c_str());
      return false;
    }

    auto request = std::make_shared<PlanMove::Request>();
    request->x = target.x;
    request->y = target.y;
    request->z = target.z;
    request->execute = true;
    request->position_tolerance =
      target.use_level_constraint ? box_motion_position_tolerance_m_ : 0.0;
    request->use_level_constraint =
      box_use_level_constraint_ && target.use_level_constraint;
    request->wrist_level_offset = box_wrist_level_offset_rad_;
    request->use_target_orientation = box_use_target_orientation_ && !box_use_level_constraint_;
    request->orientation_x = box_target_orientation_xyzw_[0];
    request->orientation_y = box_target_orientation_xyzw_[1];
    request->orientation_z = box_target_orientation_xyzw_[2];
    request->orientation_w = box_target_orientation_xyzw_[3];
    request->orientation_tolerance = box_target_orientation_tolerance_rad_;

    RCLCPP_INFO(
      get_logger(),
      "箱子动作 %s: x=%.3f y=%.3f z=%.3f",
      target.label.c_str(),
      target.x,
      target.y,
      target.z);

    auto future = plan_client_->async_send_request(request);
    const auto timeout = duration_from_seconds(plan_response_timeout_s_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!stop_requested_.load() && future.wait_for(std::chrono::milliseconds(50)) !=
                                      std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() >= deadline) {
        RCLCPP_ERROR(get_logger(), "MoveIt 规划/执行响应超时。");
        return false;
      }
    }

    if (stop_requested_.load()) {
      RCLCPP_WARN(get_logger(), "已请求停止，放弃等待 MoveIt 响应。");
      return false;
    }

    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "MoveIt 规划/执行响应超时。");
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(get_logger(), "MoveIt 执行失败：%s", response->message.c_str());
      return false;
    }

    return true;
  }

  void publish_suction_command(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    suction_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "吸泵命令：%s。", enabled ? "开启" : "关闭");
  }

  void publish_vision_mode(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    vision_mode_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "机械臂视觉模式：%s。", enabled ? "开启" : "关闭");
  }

  void publish_home_trajectory()
  {
    publish_vision_mode(false);
    publish_joint_trajectory("回原位", home_joint_positions_, home_duration_s_);
  }

  void publish_vision_trajectory()
  {
    publish_vision_mode(true);
    publish_joint_trajectory("vision 姿态", vision_joint_positions_, vision_duration_s_);
  }

  void publish_joint_trajectory(
    const std::string & label,
    const std::vector<double> & joint_positions,
    const double duration_s)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = {
      "base_yaw_joint",
      "shoulder_pitch_joint",
      "elbow_pitch_joint",
      "wrist_pitch_joint",
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    const auto duration_ns = duration_from_seconds(duration_s);
    point.time_from_start.sec =
      static_cast<int32_t>(duration_ns.count() / 1000000000LL);
    point.time_from_start.nanosec =
      static_cast<uint32_t>(duration_ns.count() % 1000000000LL);
    trajectory.points.push_back(point);

    home_publisher_->publish(trajectory);
    RCLCPP_INFO(
      get_logger(),
      "%s轨迹已发布：q=[%.3f, %.3f, %.3f, %.3f], duration=%.2fs。",
      label.c_str(),
      joint_positions[0],
      joint_positions[1],
      joint_positions[2],
      joint_positions[3],
      duration_s);
  }

  void sleep_with_stop(const double seconds) const
  {
    const auto end_time = std::chrono::steady_clock::now() + duration_from_seconds(seconds);
    while (!stop_requested_.load() && std::chrono::steady_clock::now() < end_time) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  static std::chrono::nanoseconds duration_from_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  std::string plan_service_name_;
  std::string box_motion_start_topic_;
  std::string box_motion_ground_start_topic_;
  std::string box_motion_stack_start_topic_;
  std::string box_suction_start_topic_;
  std::string box_lift_start_topic_;
  std::string box_place_ground_start_topic_;
  std::string box_place_stack_start_topic_;
  std::string home_motion_start_topic_;
  std::string vision_motion_start_topic_;
  std::string suction_command_topic_;
  std::string vision_mode_topic_;
  std::string joint_trajectory_topic_;
  BoxPlaceMode default_box_place_mode_{BoxPlaceMode::Ground};

  std::vector<double> home_joint_positions_{0.0, 0.0, 0.0, 0.0};
  double home_duration_s_{4.0};
  bool start_in_vision_mode_{true};
  std::vector<double> vision_joint_positions_{0.0, 0.0, 0.0, -1.57079632679};
  double vision_duration_s_{3.0};
  double vision_start_delay_s_{3.0};

  double box_center_distance_m_{0.42};
  double platform_height_m_{0.0};
  double box_ground_z_m_{0.0};
  double box_height_m_{0.25};
  double box_motion_y_m_{0.0};
  double box_place_x_m_{0.0};
  double box_place_y_m_{0.0};
  double box_approach_offset_m_{0.02};
  double box_suction_contact_offset_m_{0.01};
  double box_touch_push_m_{0.005};
  double box_suction_target_x_m_{0.42};
  double box_suction_settle_s_{0.8};
  double box_release_settle_s_{0.8};
  double box_lift_height_m_{0.08};
  double box_lift_retract_m_{0.04};
  double box_min_approach_x_m_{0.14};
  double stack_base_box_height_m_{0.25};
  double stack_carry_clearance_m_{0.08};
  double box_motion_step_wait_s_{0.5};
  double box_motion_position_tolerance_m_{0.04};
  bool box_use_level_constraint_{true};
  double box_wrist_level_offset_rad_{0.0};
  bool box_use_target_orientation_{true};
  std::vector<double> box_target_orientation_xyzw_{0.0, 0.0, 0.0, 1.0};
  double box_target_orientation_tolerance_rad_{0.20};
  double service_wait_timeout_s_{5.0};
  double plan_response_timeout_s_{60.0};

  double box_suction_target_x_{0.42};
  double box_transport_x_{0.38};
  double box_source_top_z_{0.25};
  double box_place_x_{0.38};
  double box_place_y_{0.0};
  double box_place_top_z_{0.25};
  double box_carry_z_{0.33};

  std::atomic_bool busy_{false};
  std::atomic_bool stop_requested_{false};
  std::atomic_bool shutdown_home_started_{false};
  std::mutex worker_mutex_;
  std::thread worker_;

  rclcpp::Client<PlanMove>::SharedPtr plan_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_mode_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr home_publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_ground_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_stack_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_suction_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_lift_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_place_ground_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_place_stack_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr home_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr vision_start_subscription_;
  rclcpp::TimerBase::SharedPtr startup_vision_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_watch_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, shutdown_signal_handler);
  std::signal(SIGTERM, shutdown_signal_handler);

  rclcpp::NodeOptions options;

  auto node = std::make_shared<BoxMotionNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  if (shutdown_signal_requested()) {
    node->shutdown_to_home();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
