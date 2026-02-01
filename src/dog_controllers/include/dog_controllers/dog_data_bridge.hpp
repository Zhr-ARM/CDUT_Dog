#ifndef DOG_CONTROL__DOG_DATA_BRIDGE_HPP__
#define DOG_CONTROL__DOG_DATA_BRIDGE_HPP__

#include <vector>
#include <string>
#include <array>
#include <iostream>

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

namespace dog_controllers
{
    static double G_DUMMY_ZERO = 0.0;
    static double G_DUMMY_TEMP = 25.0;
    static double G_DUMMY_COV[9] = {0.0};

    struct JointData
    {
        std::string name;
        // 状态反馈 (由驱动更新)
        const double *pos = &G_DUMMY_ZERO;
        const double *vel = &G_DUMMY_ZERO;
        const double *eff = &G_DUMMY_ZERO;
        const double *mode = &G_DUMMY_ZERO;
        const double *driver_tem = &G_DUMMY_TEMP; // 驱动器温度
        const double *motor_tem = &G_DUMMY_TEMP;  // 电机温度
        const double *error_code = &G_DUMMY_ZERO;

        // 指令下发 (由控制器写入)
        double *cmd_pos = &G_DUMMY_ZERO;
        double *cmd_vel = &G_DUMMY_ZERO;
        double *cmd_kp = &G_DUMMY_ZERO;
        double *cmd_kd = &G_DUMMY_ZERO;
        double *cmd_ff = &G_DUMMY_ZERO;
        double *cmd_mode = &G_DUMMY_ZERO; // 模式切换指令
    };

    struct LegData
    {
        std::string name; // FL, FR, HL, HR
        JointData hip;    // 髋关节 (HipX)
        JointData thigh;  // 大腿 (HipY)
        JointData calf;   // 小腿 (Knee)
        const double *contact = &G_DUMMY_ZERO;
        JointData *joints[3] = {&hip, &thigh, &calf};
    };

    struct ImuData
    {
        std::string name;
        const double *ori = nullptr;     // 指向 x,y,z,w 连续内存
        const double *ang_vel = nullptr; // 指向 x,y,z 连续内存
        const double *lin_acc = nullptr; // 指向 x,y,z 连续内存

        const double *ori_cov = G_DUMMY_COV;
        const double *ang_vel_cov = G_DUMMY_COV;
        const double *lin_acc_cov = G_DUMMY_COV;
    };

    class DogDataBridge
    {
    public:
        DogDataBridge() = default;

        /**
         * @brief 绑定所有接口
         */
        bool setup(
            std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
            std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces,
            const std::vector<std::string> &joint_names,
            const std::vector<std::string> &contact_names,
            const std::string &imu_name);

        std::array<LegData, 4> legs;
        ImuData imu;

    private:
        void bind_joint_interface(JointData &j,
                                  std::vector<hardware_interface::LoanedStateInterface> &states,
                                  std::vector<hardware_interface::LoanedCommandInterface> &cmds);
    };

} // namespace dog_control

#endif