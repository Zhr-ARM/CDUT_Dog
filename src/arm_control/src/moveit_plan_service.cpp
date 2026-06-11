#include <functional>
#include <memory>
#include <mutex>
#include <array>
#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <vector>

#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "arm_control/srv/plan_move.hpp"
#include "moveit/kinematics_base/kinematics_base.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"

using PlanMove = arm_control::srv::PlanMove;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

namespace
{
    constexpr const char *kPlanningGroup = "arm";
    constexpr const char *kReferenceFrame = "world";
    constexpr const char *kEndEffectorLink = "end_effector_fixed_joint";
    constexpr double kIkTimeout = 0.5;          // 最大IK求解时间
    constexpr double kPositionTolerance = 0.01; // 位置误差容忍度，单位为米
    constexpr double kDefaultOrientationTolerance = 0.35; // 固定姿态误差容忍度，单位为弧度
    constexpr double kToolDirectionTolerance = 0.20; // 工具方向误差容忍度，单位为弧度
    constexpr double kToolDirectionWeight = 0.20; // 工具方向误差在数值 IK 中的权重
    constexpr int kToolDirectionMaxIterations = 120;
    constexpr double kDamping = 0.08;
    constexpr double kMaxSolverStep = 0.12;
    constexpr double kFiniteDifferenceStep = 1.0e-5;

    const std::array<std::string, 4> kJointNames = {
        "base_yaw_joint",
        "shoulder_pitch_joint",
        "elbow_pitch_joint",
        "wrist_pitch_joint",
    };

    double quaternion_angle_error(
        const Eigen::Quaterniond &target,
        const Eigen::Quaterniond &actual)
    {
        const double dot = std::abs(target.normalized().dot(actual.normalized()));
        const double clamped_dot = std::clamp(dot, -1.0, 1.0);
        return 2.0 * std::acos(clamped_dot);
    }

    double direction_angle_error(
        const Eigen::Vector3d &target,
        const Eigen::Vector3d &actual)
    {
        const double dot = std::clamp(target.normalized().dot(actual.normalized()), -1.0, 1.0);
        return std::acos(dot);
    }
} // namespace

class MoveitPlanService : public rclcpp::Node
{
public:
    MoveitPlanService(const rclcpp::NodeOptions &options) : Node("moveit_plan_service", options)
    {
        service_ = this->create_service<PlanMove>("plan_moveit", std::bind(&MoveitPlanService::handle_plan_request, this, std::placeholders::_1, std::placeholders::_2));
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",                                                                  // 创建对joint_states话题的订阅
            rclcpp::SensorDataQoS(),                                                         // 使用适合传感器数据的QoS设置
            std::bind(&MoveitPlanService::joint_state_callback, this, std::placeholders::_1) // 绑定回调函数
        );
        RCLCPP_INFO(this->get_logger(), "MoveIt planning service is ready.");
    }
    void init_moveit(const rclcpp::Node::SharedPtr &node)
    {
        move_group_ = std::make_shared<MoveGroupInterface>(node, kPlanningGroup);

        move_group_->setPoseReferenceFrame(kReferenceFrame);
        move_group_->setEndEffectorLink(kEndEffectorLink);

        move_group_->setPlanningTime(6.0);
        move_group_->setNumPlanningAttempts(20);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);

        move_group_->setGoalPositionTolerance(0.01);
        move_group_->startStateMonitor(0.0);
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_.clear();

        for (size_t index = 0; index < msg->name.size() && index < msg->position.size(); ++index)
        {
            latest_joint_state_[msg->name[index]] = msg->position[index];
        }

        has_joint_state_ = true;
    }

    moveit::core::RobotState make_start_state()
    {
        moveit::core::RobotState start_state(move_group_->getRobotModel());
        start_state.setToDefaultValues();

        {
            // 优先使用本节点直接订阅到的 /joint_states。这样不依赖 MoveGroupInterface
            // 按“当前时间戳”等待状态，可避免真实硬件反馈略滞后时反复打印取状态失败。
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            if (has_joint_state_)
            {
                for (const auto &joint_name : kJointNames)
                {
                    const auto joint_it = latest_joint_state_.find(joint_name);
                    if (joint_it != latest_joint_state_.end())
                    {
                        start_state.setVariablePosition(joint_name, joint_it->second);
                    }
                }
                start_state.update();
                return start_state;
            }
        }

        auto current_state = move_group_->getCurrentState(1.0);
        if (current_state)
        {
            start_state = *current_state;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "未收到 joint_states，使用模型默认状态作为规划起点。");
        }

        start_state.update();
        return start_state;
    }
    /*************这是一个静态工具函数，将 IK 求解出的关节角度数组格式化为人类可读的日志字符串**********/
    static std::string format_joints(const std::vector<double> &joints)
    {
        std::ostringstream stream; // 创建一个字符串流对象，用于构建输出字符串

        for (size_t index = 0; index < joints.size() && index < kJointNames.size(); ++index)
        {
            if (index != 0)
            {
                stream << ", "; // 不是第一个元素就先输出一个逗号和空格分隔
            }
            stream << kJointNames[index] << "=" << joints[index]; // 输出关节名和对应的角度值，例如"base_yaw_joint=0.5"
        }

        return stream.str();
    }

    std::array<double, 4> read_lower_bounds() const
    {
        std::array<double, 4> lower{};
        const auto robot_model = move_group_->getRobotModel();
        for (size_t index = 0; index < kJointNames.size(); ++index)
        {
            lower[index] = robot_model->getVariableBounds(kJointNames[index]).min_position_;
        }
        return lower;
    }

    std::array<double, 4> read_upper_bounds() const
    {
        std::array<double, 4> upper{};
        const auto robot_model = move_group_->getRobotModel();
        for (size_t index = 0; index < kJointNames.size(); ++index)
        {
            upper[index] = robot_model->getVariableBounds(kJointNames[index]).max_position_;
        }
        return upper;
    }

    static std::vector<double> clamp_joints(
        std::vector<double> joints,
        const std::array<double, 4> &lower,
        const std::array<double, 4> &upper)
    {
        for (size_t index = 0; index < joints.size() && index < lower.size(); ++index)
        {
            joints[index] = std::clamp(joints[index], lower[index], upper[index]);
        }
        return joints;
    }

    Eigen::Matrix<double, 6, 1> tool_direction_state(
        const moveit::core::RobotState &start_state,
        const moveit::core::JointModelGroup *joint_model_group,
        const std::vector<double> &joints,
        const double direction_weight) const
    {
        moveit::core::RobotState state(start_state);
        state.setJointGroupPositions(joint_model_group, joints);
        state.update();

        const Eigen::Isometry3d transform = state.getGlobalLinkTransform(kEndEffectorLink);
        Eigen::Matrix<double, 6, 1> result;
        result.segment<3>(0) = transform.translation();
        // 箱顶吸取要求吸盘面平行于地面。这里约束末端局部 Z 轴竖直，
        // 等价于约束末端局部 XY 平面水平。
        result.segment<3>(3) = direction_weight * transform.linear().col(2).normalized();
        return result;
    }

    bool solve_tool_direction_joint_targets(
        const moveit::core::RobotState &start_state,
        const moveit::core::JointModelGroup *joint_model_group,
        const Eigen::Vector3d &target_position,
        const double position_tolerance,
        const double tool_yaw_offset,
        const double direction_tolerance,
        std::vector<double> &joint_targets,
        double &final_position_error,
        double &final_direction_error,
        int &final_iterations)
    {
        std::vector<double> current_joints;
        start_state.copyJointGroupPositions(joint_model_group, current_joints);
        if (current_joints.size() != kJointNames.size())
        {
            return false;
        }

        const auto lower = read_lower_bounds();
        const auto upper = read_upper_bounds();

        const double yaw = std::atan2(target_position.y(), std::max(0.001, target_position.x()));
        (void)target_position;
        (void)tool_yaw_offset;
        const Eigen::Vector3d target_tool_z = Eigen::Vector3d::UnitZ();

        std::vector<std::vector<double>> seeds;
        seeds.push_back(current_joints);
        auto yaw_current = current_joints;
        yaw_current[0] = yaw;
        seeds.push_back(yaw_current);
        seeds.push_back({yaw, 0.35, 1.2, -0.6});
        seeds.push_back({yaw, 0.8, 1.4, -0.8});
        seeds.push_back({
            yaw,
            0.5 * (lower[1] + upper[1]),
            0.5 * (lower[2] + upper[2]),
            std::clamp(-0.6, lower[3], upper[3]),
        });

        const double tolerance =
            direction_tolerance > 0.0 ? direction_tolerance : kToolDirectionTolerance;
        const double damping_squared = kDamping * kDamping;

        bool solved = false;
        double best_position_error = std::numeric_limits<double>::infinity();
        double best_direction_error = std::numeric_limits<double>::infinity();
        int best_iterations = 0;
        std::vector<double> best_joints = current_joints;

        for (auto seed : seeds)
        {
            std::vector<double> joints = clamp_joints(seed, lower, upper);
            int iterations = 0;

            for (; iterations < kToolDirectionMaxIterations; ++iterations)
            {
                const auto state = tool_direction_state(
                    start_state, joint_model_group, joints, kToolDirectionWeight);
                const Eigen::Vector3d position = state.segment<3>(0);
                const Eigen::Vector3d tool_z = state.segment<3>(3) / kToolDirectionWeight;
                const double position_error = (target_position - position).norm();
                const double direction_error = direction_angle_error(target_tool_z, tool_z);

                if (position_error < best_position_error ||
                    (position_error <= best_position_error &&
                     direction_error < best_direction_error))
                {
                    best_position_error = position_error;
                    best_direction_error = direction_error;
                    best_iterations = iterations;
                    best_joints = joints;
                }

                if (position_error <= position_tolerance &&
                    direction_error <= tolerance)
                {
                    solved = true;
                    best_position_error = position_error;
                    best_direction_error = direction_error;
                    best_iterations = iterations;
                    best_joints = joints;
                    break;
                }

                Eigen::Matrix<double, 6, 1> target_state;
                target_state.segment<3>(0) = target_position;
                target_state.segment<3>(3) = kToolDirectionWeight * target_tool_z;
                const Eigen::Matrix<double, 6, 1> error = target_state - state;

                Eigen::Matrix<double, 6, 4> jacobian;
                for (size_t column = 0; column < kJointNames.size(); ++column)
                {
                    auto plus = joints;
                    auto minus = joints;
                    plus[column] = std::min(upper[column], plus[column] + kFiniteDifferenceStep);
                    minus[column] = std::max(lower[column], minus[column] - kFiniteDifferenceStep);

                    const double denominator = plus[column] - minus[column];
                    if (std::abs(denominator) < 1.0e-12)
                    {
                        jacobian.col(static_cast<int>(column)).setZero();
                        continue;
                    }

                    const auto plus_state = tool_direction_state(
                        start_state, joint_model_group, plus, kToolDirectionWeight);
                    const auto minus_state = tool_direction_state(
                        start_state, joint_model_group, minus, kToolDirectionWeight);
                    jacobian.col(static_cast<int>(column)) =
                        (plus_state - minus_state) / denominator;
                }

                auto damped = (jacobian * jacobian.transpose()).eval();
                damped.diagonal().array() += damping_squared;
                Eigen::Vector4d delta = jacobian.transpose() * damped.ldlt().solve(error);

                if (!delta.allFinite())
                {
                    break;
                }
                if (delta.norm() > kMaxSolverStep)
                {
                    delta *= kMaxSolverStep / delta.norm();
                }

                for (size_t index = 0; index < joints.size(); ++index)
                {
                    joints[index] =
                        std::clamp(joints[index] + delta(static_cast<int>(index)),
                                   lower[index],
                                   upper[index]);
                }
            }

            if (solved)
            {
                break;
            }
        }

        joint_targets = best_joints;
        final_position_error = best_position_error;
        final_direction_error = best_direction_error;
        final_iterations = best_iterations;
        return solved;
    }

    bool plan_and_execute(
        const moveit::core::RobotState &start_state,
        const std::vector<double> &joint_targets,
        const bool execute,
        PlanMove::Response &response)
    {
        move_group_->setStartState(start_state);
        move_group_->clearPathConstraints();
        move_group_->clearPoseTargets();

        if (!move_group_->setJointValueTarget(joint_targets))
        {
            response.success = false;
            response.message = "Failed to set MoveIt joint target.";
            RCLCPP_WARN(this->get_logger(), "%s", response.message.c_str());
            return false;
        }

        MoveGroupInterface::Plan plan;
        auto plan_result = move_group_->plan(plan);

        move_group_->clearPoseTargets();

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            response.success = false;
            response.message = "MoveIt planning failed.";
            RCLCPP_WARN(this->get_logger(), "%s", response.message.c_str());
            return false;
        }

        if (!execute)
        {
            response.success = true;
            response.message = "Planning succeeded, execution skipped.";
            RCLCPP_INFO(this->get_logger(), "%s", response.message.c_str());
            return true;
        }

        auto execute_result = move_group_->execute(plan);
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            response.success = true;
            response.message = "Planning and execution succeeded.";
            RCLCPP_INFO(this->get_logger(), "%s", response.message.c_str());
            return true;
        }

        response.success = false;
        response.message = "Planning succeeded, but execution failed.";
        RCLCPP_WARN(this->get_logger(), "%s", response.message.c_str());
        return false;
    }

    void handle_plan_request(const std::shared_ptr<PlanMove::Request> request, std::shared_ptr<PlanMove::Response> response)
    {
        std::lock_guard<std::mutex> lock(planning_mutex_); // 加锁保证同一时间只有一个规划请求在处理，避免MoveIt接口的线程安全问题
        if (!move_group_)                                  // moveit 接口未初始化，无法处理请求，返回错误信息
        {
            response->success = false;
            response->message = "MoveGroupInterface is not initialized.";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Received planning request: x=%.3f, y=%.3f, z=%.3f, execute=%s, fixed_orientation=%s, level=%s",
            request->x,
            request->y,
            request->z,
            request->execute ? "true" : "false",
            request->use_target_orientation ? "true" : "false",
            request->use_level_constraint ? "true" : "false");

        const auto robot_model = move_group_->getRobotModel();
        const auto *joint_model_group = robot_model->getJointModelGroup(kPlanningGroup);
        if (joint_model_group == nullptr)
        {
            response->success = false;
            response->message = "MoveIt joint model group not found.";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        } // 找到规划组对应的关节模型组，如果找不到就返回错误信息

        // 复制规划起点，作为 IK 求解的初始状态。
        moveit::core::RobotState start_state = make_start_state();
        const Eigen::Vector3d requested_position(request->x, request->y, request->z);
        const double position_tolerance =
            request->position_tolerance > 0.0 ? request->position_tolerance : kPositionTolerance;
        std::vector<double> joint_targets;

        if (request->use_level_constraint)
        {
            double position_error = 0.0;
            double direction_error = 0.0;
            int iterations = 0;
            if (!solve_tool_direction_joint_targets(
                    start_state,
                    joint_model_group,
                    requested_position,
                    position_tolerance,
                    request->wrist_level_offset,
                    request->orientation_tolerance,
                    joint_targets,
                    position_error,
                    direction_error,
                    iterations))
            {
                response->success = false;
                response->message = "Tool-direction IK failed for level box motion.";
                RCLCPP_WARN(
                    this->get_logger(),
                    "%s position_error=%.4f m, tolerance=%.4f m, direction_error=%.4f rad after %d iterations",
                    response->message.c_str(),
                    position_error,
                    position_tolerance,
                    direction_error,
                    iterations);
                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Tool-direction IK solution position_error=%.4f m, direction_error=%.4f rad, target joints=[%s]",
                position_error,
                direction_error,
                format_joints(joint_targets).c_str());
            plan_and_execute(start_state, joint_targets, request->execute, *response);
            return;
        }

        moveit::core::RobotState ik_state(start_state);
        // 获取末端姿态四元数；普通目标点保持当前姿态，箱子动作可指定固定水平姿态。
        const Eigen::Isometry3d current_tip_transform = start_state.getGlobalLinkTransform(kEndEffectorLink);
        const Eigen::Quaterniond current_tip_orientation(current_tip_transform.rotation());
        Eigen::Quaterniond target_orientation = current_tip_orientation;
        double orientation_tolerance = kDefaultOrientationTolerance;
        if (request->use_target_orientation)
        {
            target_orientation = Eigen::Quaterniond(
                request->orientation_w,
                request->orientation_x,
                request->orientation_y,
                request->orientation_z);
            if (target_orientation.norm() < 1e-9)
            {
                response->success = false;
                response->message = "Target orientation quaternion is invalid.";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            target_orientation.normalize();
            if (request->orientation_tolerance > 0.0)
            {
                orientation_tolerance = request->orientation_tolerance;
            }
        }
        // 构造目标末端位姿，位置来自请求，姿态按请求选择当前姿态或固定姿态。
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = request->x;
        target_pose.position.y = request->y;
        target_pose.position.z = request->z;
        target_pose.orientation.x = target_orientation.x();
        target_pose.orientation.y = target_orientation.y();
        target_pose.orientation.z = target_orientation.z();
        target_pose.orientation.w = target_orientation.w();
        // 运行近似解
        kinematics::KinematicsQueryOptions ik_options;
        ik_options.return_approximate_solution = true;
        // 执行IK:从ik—state开始，尝试求解达到target_pose的关节角度，结果保存在ik_state中
        const bool ik_success = ik_state.setFromIK(
            joint_model_group,
            target_pose,
            kEndEffectorLink,
            kIkTimeout,
            moveit::core::GroupStateValidityCallbackFn(),
            ik_options);
        // 计算末端位置，还有距离误差
        const Eigen::Vector3d solved_position =
            ik_state.getGlobalLinkTransform(kEndEffectorLink).translation();
        const Eigen::Quaterniond solved_orientation(
            ik_state.getGlobalLinkTransform(kEndEffectorLink).rotation());
        const double position_error = (requested_position - solved_position).norm();
        const double orientation_error =
            quaternion_angle_error(target_orientation, solved_orientation);
        // 双重判定：IK 失败 OR 位置误差 > 1cm → 拒绝
        if (!ik_success || position_error > position_tolerance)
        {
            response->success = false;
            response->message = "MoveIt IK failed for target position.";
            RCLCPP_WARN(
                this->get_logger(),
                "%s ik_success=%s, position_error=%.4f m, tolerance=%.4f m",
                response->message.c_str(),
                ik_success ? "true" : "false",
                position_error,
                position_tolerance);
            return;
        }
        if (request->use_target_orientation && orientation_error > orientation_tolerance)
        {
            response->success = false;
            response->message = "MoveIt IK failed to satisfy target orientation.";
            RCLCPP_WARN(
                this->get_logger(),
                "%s orientation_error=%.4f rad, tolerance=%.4f rad",
                response->message.c_str(),
                orientation_error,
                orientation_tolerance);
            return;
        }
        // 提取求解出的关节角度，准备规划
        ik_state.copyJointGroupPositions(joint_model_group, joint_targets);
        RCLCPP_INFO(
            this->get_logger(),
            "MoveIt IK solution position_error=%.4f m, orientation_error=%.4f rad, target joints=[%s]",
            position_error,
            orientation_error,
            format_joints(joint_targets).c_str());
        plan_and_execute(start_state, joint_targets, request->execute, *response);
    }
    rclcpp::Service<PlanMove>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    std::shared_ptr<MoveGroupInterface> move_group_;
    std::mutex planning_mutex_;
    std::mutex joint_state_mutex_;
    std::map<std::string, double> latest_joint_state_;
    bool has_joint_state_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MoveitPlanService>(options);
    node->init_moveit(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
