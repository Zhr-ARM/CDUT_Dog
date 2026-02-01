#include "dog_controllers/dog_test_controller.hpp"

namespace dog_controllers
{
    CallbackReturn DogTestController::on_init()
    {
        auto node = get_node();
        if (!node->has_parameter("joints"))
        {
            node->declare_parameter("joints", std::vector<std::string>());
        }
        // 检查 contacts
        if (!node->has_parameter("contacts"))
        {
            node->declare_parameter("contacts", std::vector<std::string>());
        }
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
        // 执行初始化
        if (!bridge_.setup(state_interfaces_, command_interfaces_, joint_names_, contact_names_, imu_name_))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "数据桥梁初始化失败！地址映射不匹配。");
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(get_node()->get_logger(), "DogTestController 激活成功，硬件地址已打通！");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DogTestController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        bridge_.read_from_hw();

        // for (int i = 0; i < 4; ++i)
        // {
        //     for (int j = 0; j < 3; ++j)
        //     {
        //         bridge_.legs[i].joints[j]->cmd_pos = 0.0;
        //         bridge_.legs[i].joints[j]->cmd_kp = 60.0;
        //         bridge_.legs[i].joints[j]->cmd_kd = 2.0;
        //         bridge_.legs[i].joints[j]->cmd_ff = 0.0;
        //         bridge_.legs[i].joints[j]->cmd_vel = 0.0;
        //     }
        // }

        static int print_count = 0;
        if (++print_count >= 100)
        {
            print_count = 0;
            RCLCPP_INFO(get_node()->get_logger(), "==================== DOG STATE SNAPSHOT ====================");

            // --- 关节数据 (Joints) ---
            const char *leg_names[] = {"FL", "FR", "HL", "HR"};
            const char *joint_names[] = {"Hip", "Thigh", "Calf"};

            for (int i = 0; i < 4; ++i)
            {
                RCLCPP_INFO(get_node()->get_logger(),
                            "[%s Leg] | Pos(rad): [%.3f, %.3f, %.3f] | Contact: %s",
                            leg_names[i],
                            bridge_.legs[i].hip.pos, bridge_.legs[i].thigh.pos, bridge_.legs[i].calf.pos,
                            bridge_.legs[i].contact > 0.5 ? "GROUND" : "AIR");
            }

            // --- IMU 数据 ---
            RCLCPP_INFO(get_node()->get_logger(), "------------------------------------------------------------");
            RCLCPP_INFO(get_node()->get_logger(),
                        "[IMU Ori] | W:%.3f X:%.3f Y:%.3f Z:%.3f",
                        bridge_.imu.ori[3], bridge_.imu.ori[0], bridge_.imu.ori[1], bridge_.imu.ori[2]);
            RCLCPP_INFO(get_node()->get_logger(),
                        "[IMU Acc] | X:%.3f Y:%.3f Z:%.3f m/s^2",
                        bridge_.imu.lin_acc[0], bridge_.imu.lin_acc[1], bridge_.imu.lin_acc[2]);
            RCLCPP_INFO(get_node()->get_logger(),
                        "[IMU Ang] | X:%.3f Y:%.3f Z:%.3f rad/s",
                        bridge_.imu.ang_vel[0], bridge_.imu.ang_vel[1], bridge_.imu.ang_vel[2]);

            RCLCPP_INFO(get_node()->get_logger(), "============================================================");
        }

        bridge_.write_to_hw();
        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogTestController, controller_interface::ControllerInterface)