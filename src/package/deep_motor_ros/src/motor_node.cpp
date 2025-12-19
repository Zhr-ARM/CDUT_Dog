/*
 * 项目名称: DeepRobotics J60 关节电机驱动节点 (ROS2 Hardware Interface)
 * 维护者: 张恒瑞 (底层控制组)
 * 最后修改: 2025-12-20
 * * ======================================================================================
 * 架构说明:
 * 本节点充当 "硬件抽象层 (HAL)" 的角色，负责屏蔽底层 CAN 通信细节，
 * 向算法层提供标准的 ROS2 话题接口。
 * * 数据流向:
 * [算法层/仿真器]  ---> /motor_cmd (控制指令) ---> [本节点] ---> CAN Bus ---> [J60电机]
 * [算法层/仿真器]  <--- /motor_feedback (状态) <--- [本节点] <--- CAN Bus <--- [J60电机]
 * ======================================================================================
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <cmath>
#include <cstdlib>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

extern "C" {
    #include "deep_motor_ros/deep_motor_sdk.h"
}

using namespace std::chrono_literals;

class DeepMotorNode : public rclcpp::Node
{
public:
    // ========================================================================
    // 1. 数据结构定义 (物理层)
    // ========================================================================

    /**
     * @brief 最小控制单元 (单电机指令)
     * @note 这里的参数对应 MIT 运控模式公式: 
     * T_out = Kp*(P_des - P_cur) + Kd*(V_des - V_cur) + T_ff
     */
    struct ControlCommand {
        double p = 0.0;  // [单位: rad]   期望关节角度 (Position)
        double v = 0.0;  // [单位: rad/s] 期望关节角速度 (Velocity)
        double t = 0.0;  // [单位: N·m]   前馈力矩 (Torque Feedforward)
        double kp = 0.0; // [单位: 无]    位置刚度系数 (Stiffness, 典型范围 0~100)
        double kd = 0.0; // [单位: 无]    速度阻尼系数 (Damping, 典型范围 0~5)
    };

    /**
     * @brief 最小反馈单元 (单电机状态)
     */
    struct MotorState {
        double p = 0.0;           // [单位: rad]   当前实际角度
        double v = 0.0;           // [单位: rad/s] 当前实际角速度
        double t = 0.0;           // [单位: N·m]   当前实际力矩
        double motor_temp = 0.0;  // [单位: ℃]    电机线圈温度 (过热保护阈值需查手册)
        double driver_temp = 0.0; // [单位: ℃]    驱动板温度
    };

    // 内部电机上下文管理 (底层实现细节，算法层无需关注)
    struct MotorContext {
        int id;
        bool is_connected = false;
        MotorCMD* cmd_struct = nullptr;
        MotorDATA* data_struct = nullptr;
        ControlCommand current_cmd;
        MotorState current_state;
    };

    // ========================================================================
    // 2. 节点初始化
    // ========================================================================
    DeepMotorNode() : Node("deep_motor_node")
    {
        RCLCPP_INFO(this->get_logger(), ">>> 正在启动 J60 电机驱动节点 (Driver Layer)...");

        // 读取参数: 决定了控制哪些电机，以及它们在数组中的顺序
        this->declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3});
        this->declare_parameter<std::string>("can_interface", "can0");
        
        std::string can_iface = this->get_parameter("can_interface").as_string();
        std::vector<int64_t> ids = this->get_parameter("motor_ids").as_integer_array();

        // SDK 初始化
        can_ctx_ = DrMotorCanCreate(const_cast<char*>(can_iface.c_str()), false);
        if (!can_ctx_) {
            RCLCPP_ERROR(this->get_logger(), "[Fatal] CAN 接口初始化失败，请检查硬件连接！");
            return;
        }

        // --------------------------------------------------------------------
        // 硬件自检流程 (Hardware Self-Check)
        // --------------------------------------------------------------------
        motors_.resize(ids.size());
        int connected_count = 0;

        for (size_t i = 0; i < ids.size(); ++i) {
            auto& motor = motors_[i];
            motor.id = static_cast<int>(ids[i]);
            motor.cmd_struct = MotorCMDCreate();
            motor.data_struct = MotorDATACreate();
            // 设置安全初始值 (小阻尼模式，防止上电乱动)
            motor.current_cmd = {0.0, 0.0, 0.0, 0.0, 1.0}; 

            RCLCPP_INFO(this->get_logger(), "[Check] 正在检测电机 ID: %d ...", motor.id);
            
            // 握手验证 (防止 CAN 缓存干扰)
            bool is_verified = false;
            for (int k = 0; k < 3; ++k) {
                SetNormalCMD(motor.cmd_struct, motor.id, ENABLE_MOTOR);
                int ret = SendRecv(can_ctx_, motor.cmd_struct, motor.data_struct);
                if (ret == kNoSendRecvError && motor.data_struct->motor_id_ == motor.id) {
                    is_verified = true;
                    break;
                }
                std::this_thread::sleep_for(10ms);
            }

            if (is_verified) {
                motor.is_connected = true;
                connected_count++;
                RCLCPP_INFO(this->get_logger(), "   -> [OK] 电机 (ID: %d) 在线，已使能。", motor.id);
            } else {
                motor.is_connected = false;
                RCLCPP_WARN(this->get_logger(), "   -> [Fail] 电机 (ID: %d) 未响应，将跳过控制。", motor.id);
            }
        }
        RCLCPP_INFO(this->get_logger(), ">>> 驱动层初始化完成。共挂载 %d/%ld 个电机。", connected_count, ids.size());


        // ====================================================================
        // 3. 接口定义 (算法层对接重点)
        // ====================================================================

        /**
         * @brief 接口 A: 电机状态反馈 (Publisher)
         * @topic /motor_feedback
         * @type std_msgs/msg/Float64MultiArray
         * @frequency 20Hz (取决于定时器设置)
         * * @details 数据协议定义:
         * 这是一个一维数组，数据按电机 ID 顺序平铺。
         * 数组总长度 = 电机数量 N * 5
         * * Index 分布示意:
         * [ 
         * // --- 电机 1 (Index 0~4) ---
         * 0: Pos (rad), 
         * 1: Vel (rad/s), 
         * 2: Torque (N·m), 
         * 3: Motor_Temp (℃), 
         * 4: Driver_Temp (℃),
         * * // --- 电机 2 (Index 5~9) ---
         * 5: Pos, 6: Vel, 7: Torque, 8: M_Temp, 9: D_Temp,
         * ...
         * ]
         */
        feedback_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor_feedback", 10);

        /**
         * @brief 接口 B: 关节状态可视化 (Publisher)
         * @topic /joint_states
         * @type sensor_msgs/msg/JointState
         * @note 标准 ROS 接口，仅用于 Rviz 显示或 MoveIt 规划，不含温度信息。
         */
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        /**
         * @brief 接口 C: 电机控制指令 (Subscriber)
         * @topic /motor_cmd
         * @type std_msgs/msg/Float64MultiArray
         * * @details 算法层需下发的数据协议:
         * 必须发送一个一维数组，长度严格等于 电机数量 N * 5。
         * * Index 分布示意:
         * [
         * // --- 电机 1 指令 ---
         * 0: P_des (期望角度 rad),
         * 1: V_des (期望角速度 rad/s),
         * 2: T_ff  (前馈力矩 N·m),
         * 3: Kp    (位置刚度),
         * 4: Kd    (速度阻尼),
         * * // --- 电机 2 指令 ---
         * 5: P_des, 6: V_des, 7: T_ff, 8: Kp, 9: Kd,
         * ...
         * ]
         * * @warning 如果数组长度不匹配，驱动层将丢弃该帧指令并打印警告。
         */
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "motor_cmd", 10,
            std::bind(&DeepMotorNode::command_callback, this, std::placeholders::_1));

        // 核心控制循环 (20Hz)
        timer_ = this->create_wall_timer(
            50ms, std::bind(&DeepMotorNode::control_loop, this));
    }

    ~DeepMotorNode()
    {
        // 析构逻辑: 确保节点退出时电机安全失能
        if (can_ctx_) {
            for (auto& motor : motors_) {
                if (motor.is_connected) {
                    SetNormalCMD(motor.cmd_struct, motor.id, DISABLE_MOTOR);
                    SendRecv(can_ctx_, motor.cmd_struct, motor.data_struct);
                }
                MotorCMDDestroy(motor.cmd_struct);
                MotorDATADestroy(motor.data_struct);
            }
            DrMotorCanDestroy(can_ctx_);
        }
    }

private:
    // 回调函数: 接收算法层的控制指令
    void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        size_t expected_size = motors_.size() * 5;
        // 安全检查: 协议长度校验
        if (msg->data.size() != expected_size) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "[Protocol Error] /motor_cmd 长度错误! 期望: %ld, 实际: %ld. 请检查算法层打包逻辑。", 
                expected_size, msg->data.size());
            return;
        }

        // 数据解包 (Unpacking): Array -> Struct
        for (size_t i = 0; i < motors_.size(); ++i) {
            size_t base = i * 5;
            motors_[i].current_cmd.p  = msg->data[base + 0];
            motors_[i].current_cmd.v  = msg->data[base + 1];
            motors_[i].current_cmd.t  = msg->data[base + 2];
            motors_[i].current_cmd.kp = msg->data[base + 3];
            motors_[i].current_cmd.kd = msg->data[base + 4];
        }
    }

    // 主循环: 执行底层 CAN 通信
    void control_loop()
    {
        if (!can_ctx_) return;

        for (auto& motor : motors_) {
            // 跳过离线电机 (保护机制)
            if (!motor.is_connected) continue;

            // 1. 下发指令 (Write to CAN)
            SetMotionCMD(motor.cmd_struct, motor.id, CONTROL_MOTOR, 
                         motor.current_cmd.p, motor.current_cmd.v, 
                         motor.current_cmd.t, motor.current_cmd.kp, motor.current_cmd.kd);

            // 2. 双向通信 (Blocking Call)
            int ret = SendRecv(can_ctx_, motor.cmd_struct, motor.data_struct);
            
            // 3. 读取反馈 (Read from CAN)
            // 严格校验 ID，防止数据串扰
            if (ret == kNoSendRecvError && motor.data_struct->motor_id_ == motor.id) {
                motor.current_state.p = motor.data_struct->position_;
                motor.current_state.v = motor.data_struct->velocity_;
                motor.current_state.t = motor.data_struct->torque_;
                
                // 温度数据多路复用解析 (SDK 每次只回传一种温度)
                if (motor.data_struct->flag_) {
                    motor.current_state.motor_temp = motor.data_struct->temp_;
                } else {
                    motor.current_state.driver_temp = motor.data_struct->temp_;
                }
            }
        }
        publish_feedback();
    }

    // 发布函数: 将底层状态打包上传给算法层
    void publish_feedback()
    {
        auto feedback_msg = std_msgs::msg::Float64MultiArray();
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();

        for (const auto& motor : motors_) {
            // 1. 打包自定义反馈协议 (即使离线也填充0值，保持数组对齐)
            feedback_msg.data.push_back(motor.current_state.p);
            feedback_msg.data.push_back(motor.current_state.v);
            feedback_msg.data.push_back(motor.current_state.t);
            feedback_msg.data.push_back(motor.current_state.motor_temp);
            feedback_msg.data.push_back(motor.current_state.driver_temp);

            // 2. 打包 Rviz 可视化数据 (仅包含在线电机)
            if (motor.is_connected) {
                joint_msg.name.push_back("joint_" + std::to_string(motor.id));
                joint_msg.position.push_back(motor.current_state.p);
                joint_msg.velocity.push_back(motor.current_state.v);
                joint_msg.effort.push_back(motor.current_state.t);
            }
        }
        
        feedback_pub_->publish(feedback_msg);
        if (!joint_msg.name.empty()) {
            joint_state_pub_->publish(joint_msg);
        }
    }

    DrMotorCan* can_ctx_ = nullptr;
    std::vector<MotorContext> motors_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // ========================================================================
    // 自动配置脚本 (Hardware Setup)
    // 功能: 调用 ip link 自动设置 CAN 波特率和队列长度
    // ========================================================================
    std::string sudo_password = "llq666"; 
    std::string cmd = "echo '" + sudo_password + "' | sudo -S sh -c '";
    cmd += "ip link set can0 down; ";
    cmd += "ip link set can0 type can bitrate 1000000; "; // 波特率: 1Mbps
    cmd += "ip link set can0 txqueuelen 1000; ";          // 队列: 1000 (防溢出)
    cmd += "ip link set can0 up";
    cmd += "' > /dev/null 2>&1";
    std::system(cmd.c_str());

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeepMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}