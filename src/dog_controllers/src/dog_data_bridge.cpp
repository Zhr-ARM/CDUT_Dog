#include "dog_controllers/dog_data_bridge.hpp"

namespace dog_controllers
{
    bool DogDataBridge::setup(
        std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
        std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces,
        const std::vector<std::string> &joint_names,
        const std::vector<std::string> &contact_names,
        const std::string &imu_name)
    {
        // 1. 初始化结构
        const std::string leg_prefixes[4] = {"FL", "FR", "HL", "HR"};
        imu.name = imu_name;
        for (int i = 0; i < 4; ++i)
        {
            legs[i].name = leg_prefixes[i];
            for (int j = 0; j < 3; ++j)
                legs[i].joints[j]->name = joint_names[i * 3 + j];
        }

        read_tasks_.clear();
        write_tasks_.clear();

        // 2. 预映射状态读取任务 (State -> Local)
        for (auto &si : state_interfaces)
        {
            const std::string &prefix = si.get_prefix_name();
            const std::string &inf = si.get_interface_name();

            // 关节
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    if (prefix == legs[i].joints[j]->name)
                    {
                        if (inf == "position")
                            read_tasks_.push_back({&si, &legs[i].joints[j]->pos});
                        else if (inf == "velocity")
                            read_tasks_.push_back({&si, &legs[i].joints[j]->vel});
                        else if (inf == "effort")
                            read_tasks_.push_back({&si, &legs[i].joints[j]->eff});
                    }
                }
                // 触地
                if (prefix == contact_names[i] && inf == "contact")
                    read_tasks_.push_back({&si, &legs[i].contact});
            }
            // IMU
            if (prefix == imu.name)
            {
                if (inf == "orientation.x")
                    read_tasks_.push_back({&si, &imu.ori[0]});
                else if (inf == "orientation.y")
                    read_tasks_.push_back({&si, &imu.ori[1]});
                else if (inf == "orientation.z")
                    read_tasks_.push_back({&si, &imu.ori[2]});
                else if (inf == "orientation.w")
                    read_tasks_.push_back({&si, &imu.ori[3]});
                else if (inf == "angular_velocity.x")
                    read_tasks_.push_back({&si, &imu.ang_vel[0]});
                else if (inf == "angular_velocity.y")
                    read_tasks_.push_back({&si, &imu.ang_vel[1]});
                else if (inf == "angular_velocity.z")
                    read_tasks_.push_back({&si, &imu.ang_vel[2]});
                else if (inf == "linear_acceleration.x")
                    read_tasks_.push_back({&si, &imu.lin_acc[0]});
                else if (inf == "linear_acceleration.y")
                    read_tasks_.push_back({&si, &imu.lin_acc[1]});
                else if (inf == "linear_acceleration.z")
                    read_tasks_.push_back({&si, &imu.lin_acc[2]});
            }
        }

        // 3. 预映射指令写入任务 (Local -> Command)
        for (auto &ci : command_interfaces)
        {
            const std::string &prefix = ci.get_prefix_name();
            const std::string &inf = ci.get_interface_name();

            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    if (prefix == legs[i].joints[j]->name)
                    {
                        if (inf == "position")
                            write_tasks_.push_back({&ci, &legs[i].joints[j]->cmd_pos});
                        else if (inf == "kp")
                            write_tasks_.push_back({&ci, &legs[i].joints[j]->cmd_kp});
                        else if (inf == "kd")
                            write_tasks_.push_back({&ci, &legs[i].joints[j]->cmd_kd});
                        else if (inf == "velocity")
                            write_tasks_.push_back({&ci, &legs[i].joints[j]->cmd_vel});
                        else if (inf == "effort")
                            write_tasks_.push_back({&ci, &legs[i].joints[j]->cmd_ff});
                    }
                }
            }
        }
        if (write_tasks_.size() != 60 || read_tasks_.size() != 50)
            return false;
        return true;
    }

    void DogDataBridge::read_from_hw()
    {
        for (const auto &task : read_tasks_)
        {
            *task.local_var = task.hw_handle->get_value();
        }
    }

    void DogDataBridge::write_to_hw()
    {
        for (const auto &task : write_tasks_)
        {
            task.hw_handle->set_value(*task.local_var);
        }
    }
}