#include "dog_controllers/dog_test_controller.hpp"

namespace dog_controllers
{
    CallbackReturn DogTestController::on_init()
    {
        auto node = get_node();
        node->declare_parameter("joints", std::vector<std::string>());
        node->declare_parameter("contacts", std::vector<std::string>());
        node->declare_parameter("imu", "imu_sensor");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DogTestController::on_configure(const rclcpp_lifecycle::State &)
    {
        auto node = get_node();
        joint_names_ = node->get_parameter("joints").as_string_array();
        contact_names_ = node->get_parameter("contacts").as_string_array();
        imu_name_ = "imu_sensor";

        if (joint_names_.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "未配置关节名称列表！");
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration DogTestController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::ALL;
        return conf;
    }

    controller_interface::InterfaceConfiguration DogTestController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::ALL;
        return conf;
    }

    CallbackReturn DogTestController::on_activate(const rclcpp_lifecycle::State &)
    {
        if (!bridge_.setup(state_interfaces_, command_interfaces_, joint_names_, contact_names_, imu_name_))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "数据桥梁初始化失败！地址映射不匹配。");
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(get_node()->get_logger(), "DogTestController 激活成功，硬件地址已打通。");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DogTestController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // 【测试打印】：每 500ms 打印一次数据，验证链路
        static int count = 0;
        if (++count % 500 == 0)
        {
            // 访问示例：左前腿髋关节位置和 IMU X 轴加速度
            double fl_hip_pos = *bridge_.legs[0].hip.pos;
            double imu_acc_x = bridge_.imu.lin_acc[0];
            bool fl_touch = (*bridge_.legs[0].contact == 1.0);

            RCLCPP_INFO(get_node()->get_logger(),
                        "--- Debug Info ---\n"
                        "FL Hip Pos: %.3f rad\n"
                        "Body Acc X: %.3f m/s^2\n"
                        "FL Contact: %s\n"
                        "------------------",
                        fl_hip_pos, imu_acc_x, fl_touch ? "TOUCH" : "AIR");
        }

        // // 测试控制：让所有关节保持零位（写指令地址）
        // for (int i = 0; i < 4; ++i)
        // {
        //     for (int j = 0; j < 3; ++j)
        //     {
        //         *bridge_.legs[i].joints[j]->cmd_pos = 0.0;
        //         *bridge_.legs[i].joints[j]->cmd_kp = 50.0;
        //         *bridge_.legs[i].joints[j]->cmd_kd = 1.0;
        //     }
        // }

        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogTestController, controller_interface::ControllerInterface)