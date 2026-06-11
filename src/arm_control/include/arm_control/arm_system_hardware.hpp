#ifndef ARM_CONTROL__ARM_SYSTEM_HARDWARE_HPP_
#define ARM_CONTROL__ARM_SYSTEM_HARDWARE_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "dm_j4340_motor_driver/dm_j4340_protocol.hpp"
#include "dm_j4340_motor_driver/usbcan_serial.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arm_control {

class ArmSystemHardware : public hardware_interface::SystemInterface {
public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ~ArmSystemHardware() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  struct JointConfig {
    uint16_t motor_id = 0;
    double direction = 1.0;
    double zero_offset = 0.0;
    double gear_ratio = 1.0;
    double velocity_limit = 1.0;
    uint16_t force_position_velocity_limit = 1000;
    uint16_t force_position_current_limit = 1000;
    double lower_limit = 0.0;
    double upper_limit = 0.0;
    bool has_lower_limit = false;
    bool has_upper_limit = false;
  };

  double joint_to_motor_position(size_t joint_index,
                                 double joint_position) const;
  double motor_to_joint_position(size_t joint_index,
                                 double motor_position) const;
  double joint_to_motor_velocity(size_t joint_index,
                                 double joint_velocity) const;
  double motor_to_joint_velocity(size_t joint_index,
                                 double motor_velocity) const;
  bool send_frame(uint32_t can_id, const std::array<uint8_t, 8> &payload);
  std::optional<dm_j4340_motor_driver::MotorFeedback>
  read_motor_feedback(uint16_t motor_id);
  std::optional<dm_j4340_motor_driver::RegisterReply>
  read_motor_register(uint16_t motor_id, dm_j4340_motor_driver::Register reg);
  bool write_motor_register_uint32(uint16_t motor_id,
                                   dm_j4340_motor_driver::Register reg,
                                   uint32_t value, bool wait_reply);
  bool send_joint_position_command(size_t joint_index, double joint_position);
  void disable_motors();
  void move_to_zero_position();
  void run_stop_sequence(bool home_first, bool disable_after);

  std::vector<std::string> joint_names_;
  std::vector<JointConfig> joint_configs_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> previous_hw_positions_;

  dm_j4340_motor_driver::UsbCanSerial bus_;
  std::map<uint16_t, dm_j4340_motor_driver::MotorFeedback> latest_feedback_;
  dm_j4340_motor_driver::ControlMode control_mode_ =
      dm_j4340_motor_driver::ControlMode::PositionVelocity;

  std::string serial_port_ = "/dev/arm_motor";
  int serial_baudrate_ = 921600;
  int serial_open_retry_count_ = 10;
  int serial_open_retry_delay_ms_ = 300;
  int timeout_ms_ = 5;
  int retry_count_ = 1;
  uint8_t usbcan_command_ = 0x03;
  bool set_can_bitrate_on_open_ = false;
  uint8_t can_bitrate_index_ = 0;

  bool active_ = false;
  bool enable_on_activate_ = true;
  bool clear_error_on_activate_ = false;
  bool save_zero_on_activate_ = false;
  bool zero_saved_this_session_ = false;
  bool switch_mode_on_activate_ = true;
  bool require_register_ack_ = true;
  bool require_motors_on_configure_ = true;
  bool require_feedback_on_activate_ = true;
  bool disable_on_deactivate_ = false;
  bool disable_on_shutdown_ = false;
  bool read_feedback_in_read_ = false;
  bool home_on_deactivate_ = false;
  bool home_on_shutdown_ = false;
  bool stop_sequence_done_ = false;
  double shutdown_home_duration_s_ = 4.0;
  double shutdown_home_command_rate_hz_ = 50.0;
  int disable_command_repeat_count_ = 3;
  int disable_command_repeat_delay_ms_ = 20;
};

} // namespace arm_control

#endif // ARM_CONTROL__ARM_SYSTEM_HARDWARE_HPP_ 头文件保护
