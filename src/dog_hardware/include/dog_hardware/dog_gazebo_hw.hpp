#ifndef DOG_HARDWARE__DOG_GAZEBO_HW_HPP__
#define DOG_HARDWARE__DOG_GAZEBO_HW_HPP__

#include <deque>
#include <map>
#include <vector>
#include <string>

// ROS 2 & gazebo_ros2_control
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Joint.hh>
#include "gazebo_ros2_control/gazebo_system_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
namespace dog_hardware
{
    using CallbackReturn = hardware_interface::CallbackReturn;

    class DogGazeboHW : public gazebo_ros2_control::GazeboSystemInterface
    {
    public:
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        bool initSim(
            rclcpp::Node::SharedPtr &node,
            gazebo::physics::ModelPtr model,
            const hardware_interface::HardwareInfo &hardware_info,
            sdf::ElementPtr sdf) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // 读写循环（由 Gazebo 物理引擎触发）
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // 定义参数混合控制结构
        struct HybridCommand
        {
            double pos_des = 0.0;
            double vel_des = 0.0;
            double kp = 0.0;
            double kd = 0.0;
            double ff = 0.0;
            rclcpp::Time stamp; // 用于计算延迟
        };

        // 关节数据
        struct JointData
        {
            std::string name;
            gazebo::physics::JointPtr gz_joint;
            double pos = 0.0, vel = 0.0, eff = 0.0; // 反馈
            HybridCommand cmd;                      // 命令
            std::deque<HybridCommand> cmd_buffer;   // 延迟缓冲区
        };
        std::vector<JointData> joints_;

        // IMU 数据
        struct ImuData
        {
            std::string name;
            gazebo::sensors::ImuSensorPtr gazebo_sensor;
            // 数据缓冲区
            double ori[4];     // x, y, z, w
            double ang_vel[3]; // x, y, z
            double lin_acc[3]; // x, y, z
            // 协方差
            std::array<double, 9> ori_cov;
            std::array<double, 9> ang_vel_cov;
            std::array<double, 9> lin_acc_cov;
        };
        ImuData imu_data_;

        // 触地检测
        struct ContactSensorData
        {
            std::string name;
            gazebo::sensors::ContactSensorPtr gazebo_sensor;
            double contact_value; // 0.0 表示未触地，1.0 表示触地
        };
        std::vector<ContactSensorData> contact_sensors_;

        // 参数
        double delay_ = 0.0;           // 秒
        rclcpp::Node::SharedPtr node_; // 存储节点指针用于日志
    };

} // namespace dog_hardware

#endif // DOG_HARDWARE__DOG_GAZEBO_HW_HPP__