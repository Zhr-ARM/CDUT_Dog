#include <array>
#include <chrono>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dm_j4340_motor_driver/dm_j4340_protocol.hpp"
#include "dm_j4340_motor_driver/usbcan_serial.hpp"

namespace dm_j4340_motor_driver
{
  namespace
  {

    /**
     * @brief 非侵入式连接扫描时读取的默认寄存器集合。
     */
    std::vector<Register> default_registers()
    {
      return {Register::MST_ID, Register::ESC_ID, Register::CTRL_MODE,
              Register::ACC, Register::DEC, Register::MAX_SPD,
              Register::PMAX, Register::VMAX, Register::TMAX,
              Register::can_br, Register::sw_ver};
    }

  } // namespace

  class DmJ4340ScanNode : public rclcpp::Node
  {
  public:
    /**
     * @brief 打开 USB-CAN 串口链路，扫描配置的电机 ID，并发布扫描结果。
     */
    DmJ4340ScanNode() : Node("dm_j4340_scan_node")
    {
      serial_port_ =
          declare_parameter<std::string>("serial_port", "/dev/arm_motor");
      serial_baudrate_ = declare_parameter<int>("serial_baudrate", 921600);
      retry_count_ = declare_parameter<int>("retry_count", 3);
      timeout_ms_ = declare_parameter<int>("timeout_ms", 200);
      usbcan_command_ = declare_parameter<int>("usbcan_command", 0x03);
      set_can_bitrate_on_start_ =
          declare_parameter<bool>("set_can_bitrate_on_start", false);
      can_bitrate_index_ = declare_parameter<int>("can_bitrate_index", 0);
      scan_period_s_ = declare_parameter<double>("scan_period_s", 0.0);

      const auto motor_ids_param = declare_parameter<std::vector<int64_t>>(
          "motor_ids", std::vector<int64_t>{1, 2, 3, 4});
      for (const auto id : motor_ids_param)
      {
        if (id >= 0 && id <= 0x7FF)
        {
          motor_ids_.push_back(static_cast<uint16_t>(id));
        }
        else
        {
          RCLCPP_WARN(get_logger(), "Ignoring invalid motor id: %ld",
                      static_cast<long>(id));
        }
      }

      result_pub_ = create_publisher<std_msgs::msg::String>(
          "/dm_j4340/scan_result", rclcpp::QoS(1).transient_local());

      try
      {
        bus_.open(serial_port_, serial_baudrate_);
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR(get_logger(), "Failed to open USB-CAN serial port: %s",
                     ex.what());
        return;
      }

      RCLCPP_INFO(get_logger(), "Opened USB-CAN serial port %s at %d baud.",
                  serial_port_.c_str(), serial_baudrate_);

      if (set_can_bitrate_on_start_)
      {
        if (!bus_.set_can_bitrate(static_cast<uint8_t>(can_bitrate_index_)))
        {
          RCLCPP_WARN(get_logger(), "Failed to send USB-CAN bitrate command.");
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Sent USB-CAN bitrate index %d command.",
                      can_bitrate_index_);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      scan_and_publish();

      if (scan_period_s_ > 0.0)
      {
        scan_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(scan_period_s_)),
            [this]()
            { scan_and_publish(); });
      }
    }

  private:
    struct MotorScanResult
    {
      uint16_t motor_id = 0;
      bool online = false;
      std::map<Register, RegisterReply> parameters;
      std::optional<MotorFeedback> feedback;
    };

    /**
     * @brief 扫描所有配置的电机 ID，并发布一条便于阅读的汇总字符串。
     */
    void scan_and_publish()
    {
      if (!bus_.is_open())
      {
        RCLCPP_ERROR(get_logger(), "USB-CAN serial port is not open.");
        return;
      }

      std::vector<MotorScanResult> results;
      results.reserve(motor_ids_.size());

      for (const auto motor_id : motor_ids_)
      {
        results.push_back(scan_motor(motor_id));
      }

      const auto summary = format_results(results);
      RCLCPP_INFO(get_logger(), "\n%s", summary.c_str());

      std_msgs::msg::String msg;
      msg.data = summary;
      result_pub_->publish(msg);
    }

    /**
     * @brief 读取指定电机的默认参数集合和一帧状态反馈。
     */
    MotorScanResult scan_motor(uint16_t motor_id)
    {
      MotorScanResult result;
      result.motor_id = motor_id;

      for (const auto reg : default_registers())
      {
        const auto reply = read_register(motor_id, reg);
        if (!reply.has_value())
        {
          continue;
        }
        result.online = true;
        result.parameters.emplace(reg, reply.value());
      }

      result.feedback = read_feedback(motor_id);
      return result;
    }

    /**
     * @brief 带有限重试地读取一个电机寄存器。
     */
    std::optional<RegisterReply> read_register(uint16_t motor_id, Register reg)
    {
      const auto payload = make_read_register_command(motor_id, reg);

      for (int attempt = 0; attempt < retry_count_; ++attempt)
      {
        bus_.discard_pending();
        if (!bus_.send_frame(kRegisterCanId, payload, 8,
                             static_cast<uint8_t>(usbcan_command_)))
        {
          RCLCPP_WARN(get_logger(), "Failed to send read register command.");
          continue;
        }

        UsbCanFrame frame;
        while (bus_.read_frame(frame, timeout_ms_))
        {
          const auto reply = parse_register_reply(frame);
          if (reply.has_value() && reply->motor_id == motor_id &&
              reply->reg == reg && reply->command == kRegisterReadCommand)
          {
            return reply;
          }
        }
      }

      return std::nullopt;
    }

    /**
     * @brief 请求并解算一帧实时状态反馈。
     */
    std::optional<MotorFeedback> read_feedback(uint16_t motor_id)
    {
      const std::array<uint8_t, 8> payload = {
          static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          0xCC,
          0x00,
          0x00,
          0x00,
          0x00,
          0x00};

      for (int attempt = 0; attempt < retry_count_; ++attempt)
      {
        bus_.discard_pending();
        if (!bus_.send_frame(kRegisterCanId, payload, 8,
                             static_cast<uint8_t>(usbcan_command_)))
        {
          RCLCPP_WARN(get_logger(), "Failed to send refresh feedback command.");
          continue;
        }

        UsbCanFrame frame;
        while (bus_.read_frame(frame, timeout_ms_))
        {
          const auto feedback = parse_feedback(frame);
          if (feedback.has_value() && feedback->motor_id == motor_id)
          {
            return feedback;
          }
        }
      }

      return std::nullopt;
    }

    /**
     * @brief 将扫描结果转换为紧凑的 ROS 日志和话题消息文本。
     */
    std::string
    format_results(const std::vector<MotorScanResult> &results) const
    {
      std::ostringstream ss;
      ss << "DM-J4340 scan result";

      for (const auto &result : results)
      {
        ss << "\nID " << result.motor_id << ": "
           << (result.online ? "online" : "missing");

        if (!result.online)
        {
          continue;
        }

        for (const auto &item : result.parameters)
        {
          ss << "\n  " << register_name(item.first) << "="
             << format_register_value(item.second);
        }

        if (result.feedback.has_value())
        {
          const auto &fb = result.feedback.value();
          ss << "\n  feedback: state=" << error_name(fb.error_code)
             << " pos=" << fb.position << " vel=" << fb.velocity
             << " torque=" << fb.torque
             << " mos=" << static_cast<int>(fb.mos_temperature)
             << " rotor=" << static_cast<int>(fb.rotor_temperature);
        }
      }

      return ss.str();
    }

    std::string serial_port_;
    int serial_baudrate_ = 921600;
    int retry_count_ = 3;
    int timeout_ms_ = 200;
    int usbcan_command_ = 0x03;
    bool set_can_bitrate_on_start_ = false;
    int can_bitrate_index_ = 0;
    double scan_period_s_ = 0.0;
    std::vector<uint16_t> motor_ids_;

    UsbCanSerial bus_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
    rclcpp::TimerBase::SharedPtr scan_timer_;
  };

} // namespace dm_j4340_motor_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dm_j4340_motor_driver::DmJ4340ScanNode>());
  rclcpp::shutdown();
  return 0;
}
