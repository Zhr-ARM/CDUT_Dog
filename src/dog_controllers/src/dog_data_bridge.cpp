#include "dog_control/dog_data_bridge.hpp"
#include <algorithm>

namespace dog_controllers
{
    bool DogDataBridge::setup(
        std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
        std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces,
        const std::vector<std::string> &joint_names,
        const std::vector<std::string> &contact_names,
        const std::string &imu_name)
    {
        // 映射关节
        const std::string leg_prefixes[4] = {"FL", "FR", "HL", "HR"};
        for (int i = 0; i < 4; ++i)
        {
            legs[i].name = leg_prefixes[i];
            for (int j = 0; j < 3; ++j)
            {
                legs[i].joints[j]->name = joint_names[i * 3 + j];
                bind_joint_interface(*legs[i].joints[j], state_interfaces, command_interfaces);
            }
            const std::string &target_contact_name = contact_names[i];
            for (auto &si : state_interfaces)
            {
                if (si.get_prefix_name() == target_contact_name && si.get_interface_name() == "contact")
                {
                    legs[i].contact = &si.get_value();
                    break;
                }
            }
        }

        // IMU 指针及协方差绑定
        imu.name = imu_name;
        for (auto &si : state_interfaces)
        {
            if (si.get_prefix_name() != imu.name)
                continue;
            const std::string &inf = si.get_interface_name();

            if (inf == "orientation.x")
                imu.ori = &si.get_value();
            if (inf == "angular_velocity.x")
                imu.ang_vel = &si.get_value();
            if (inf == "linear_acceleration.x")
                imu.lin_acc = &si.get_value();

            if (inf == "orientation_covariance")
                imu.ori_cov = &si.get_value();
            if (inf == "angular_velocity_covariance")
                imu.ang_vel_cov = &si.get_value();
            if (inf == "linear_acceleration_covariance")
                imu.lin_acc_cov = &si.get_value();
        }
        return true;
    }

    void DogDataBridge::bind_joint_interface(
        JointData &j,
        std::vector<hardware_interface::LoanedStateInterface> &states,
        std::vector<hardware_interface::LoanedCommandInterface> &cmds)
    {
        // 绑定状态 (State)
        for (auto &si : states)
        {
            if (si.get_prefix_name() != j.name)
                continue;
            const std::string &inf = si.get_interface_name();

            if (inf == "position")
                j.pos = &si.get_value();
            else if (inf == "velocity")
                j.vel = &si.get_value();
            else if (inf == "effort")
                j.eff = &si.get_value();
            else if (inf == "driver_temperature")
                j.mos_tem = &si.get_value();
            else if (inf == "motor_temperature")
                j.motor_tem = &si.get_value();
            else if (inf == "error_code")
                j.error_code = &si.get_value();
        }

        // 绑定指令 (Command)
        for (auto &ci : cmds)
        {
            if (ci.get_prefix_name() != j.name)
                continue;
            const std::string &inf = ci.get_interface_name();

            double *addr = const_cast<double *>(&ci.get_value());
            if (inf == "position")
                j.cmd_pos = addr;
            else if (inf == "velocity")
                j.cmd_vel = addr;
            else if (inf == "kp")
                j.cmd_kp = addr;
            else if (inf == "kd")
                j.cmd_kd = addr;
            else if (inf == "effort")
                j.cmd_ff = addr;
            else if (inf == "mode")
                j.cmd_mode = addr;
        }
    }

} // namespace dog_control