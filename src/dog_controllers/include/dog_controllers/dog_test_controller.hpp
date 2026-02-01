#ifndef DOG_CONTROL__DOG_TEST_CONTROLLER_HPP__
#define DOG_CONTROL__DOG_TEST_CONTROLLER_HPP__

#include "controller_interface/controller_interface.hpp"
#include "dog_controllers/dog_data_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dog_controllers
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class DogTestController : public controller_interface::ControllerInterface
    {
    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        dog_controllers::DogDataBridge bridge_;

        // 存储从参数读取的名字
        std::vector<std::string> joint_names_;
        std::vector<std::string> contact_names_;
        std::string imu_name_;
    };
}

#endif