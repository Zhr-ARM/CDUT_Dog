#ifndef DM_J4340_MOTOR_DRIVER__USBCAN_SERIAL_HPP_
#define DM_J4340_MOTOR_DRIVER__USBCAN_SERIAL_HPP_

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace dm_j4340_motor_driver {

struct UsbCanFrame {
  uint8_t command = 0;
  uint32_t can_id = 0;
  std::array<uint8_t, 8> data{};
  uint8_t dlc = 0;
  bool is_extended = false;
  bool is_remote = false;
};

/**
 * @brief 达妙 USB 转 CAN 适配器的最小串口传输层。
 *
 * 适配器通过 921600 波特率串口接收 30 字节发送包，并返回 16 字节经典
 * CAN 接收包。本类只负责字节收发与适配器帧格式，电机协议编解码放在
 * dm_j4340_protocol.hpp 中。
 */
class UsbCanSerial {
public:
  UsbCanSerial() = default;

  UsbCanSerial(const std::string &port, int baudrate) { open(port, baudrate); }

  ~UsbCanSerial() { close(); }

  UsbCanSerial(const UsbCanSerial &) = delete;
  UsbCanSerial &operator=(const UsbCanSerial &) = delete;

  /**
   * @brief 打开串口，并配置为原始 8N1 模式。
   * @throws std::runtime_error 串口无法打开或配置失败时抛出。
   */
  void open(const std::string &port, int baudrate) {
    close();//开启串口前确保之前的串口已经关闭
                                      //不成为控制终端//非阻塞模式
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open serial port: " + port);
    }

    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
      close();
      throw std::runtime_error("Failed to read serial port attributes: " +
                               port);
    }

    cfmakeraw(&options);// 原始模式：无行缓冲、无回显、无信号处理
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);// 忽略调制解调器控制线，启用接收
    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    options.c_cflag |= CS8;     //8位数据位
    options.c_cflag &= static_cast<tcflag_t>(~PARENB);  //无校验位
    options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);  //1位停止位
#ifdef CRTSCTS
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS); //无硬件流控制
#endif  
    options.c_cc[VTIME] = 0;    //无字节间超时
    options.c_cc[VMIN] = 0;     //不等待最少字节数，有就返回

    const speed_t speed = baudrate_to_speed(baudrate);
    if (cfsetispeed(&options, speed) != 0 ||
        cfsetospeed(&options, speed) != 0) {
      close();
      throw std::runtime_error("Unsupported serial baudrate: " +
                               std::to_string(baudrate));
    }

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      close();
      throw std::runtime_error("Failed to configure serial port: " + port);
    }
  }

  /**
   * @brief 关闭串口，并清空尚未解析的接收缓存。
   */
  void close() {
    if (fd_ >= 0) {
      tcflush(fd_, TCIOFLUSH);
      ::close(fd_);
      fd_ = -1;
    }
    rx_buffer_.clear();
  }

  /**
   * @brief 判断当前串口设备是否已经打开。
   */
  bool is_open() const { return fd_ >= 0; }

  /**
   * @brief 发送适配器 CAN 波特率选择命令。
   *
   * index 的具体含义由适配器固件定义。除非已经确认当前 USB-CAN 适配器所需
   * CAN 总线波特率索引，否则不建议在扫描节点中主动调用。
   */
  bool set_can_bitrate(uint8_t index) {
    const std::array<uint8_t, 5> command = {0x55, 0x05, index, 0xAA, 0x55};
    return write_all(command.data(), command.size());
  }

  /**
   * @brief 通过串口适配器转发一帧经典 CAN 数据帧。
   *
   * @param can_id 标准帧或扩展帧 CAN ID。
   * @param data CAN 数据区，最多 8 字节。
   * @param dlc 经典 CAN 数据长度，范围 0 到 8。
   * @param command 适配器命令字，0x03 表示非反馈 CAN 转发。
   * @return 完整适配器发送包写入串口时返回 true。
   */
  bool send_frame(uint32_t can_id, const std::array<uint8_t, 8> &data,
                  uint8_t dlc = 8, uint8_t command = 0x03,
                  uint32_t send_times = 1, uint32_t send_interval_100us = 0,
                  bool is_extended = false, bool is_remote = false) {
    if (dlc > 8) {
      throw std::invalid_argument("Classic CAN DLC must be <= 8");
    }

    std::array<uint8_t, 30> frame{};
    frame[0] = 0x55;
    frame[1] = 0xAA;
    frame[2] = 0x1E;
    frame[3] = command;
    write_u32_le(frame.data() + 4, send_times);
    write_u32_le(frame.data() + 8, send_interval_100us);
    frame[12] = is_extended ? 0x01 : 0x00;
    write_u32_le(frame.data() + 13, can_id);
    frame[17] = is_remote ? 0x01 : 0x00;
    frame[18] = dlc;
    frame[19] = 0x00;
    frame[20] = 0x00;
    std::copy_n(data.begin(), dlc, frame.begin() + 21);
    frame[29] = crc8(frame.data(), frame.size() - 1);

    return write_all(frame.data(), frame.size());
  }

  /**
   * @brief 读取并解析一帧适配器返回的经典 CAN 帧。
   *
   * @param timeout_ms 最大等待时间，0 表示非阻塞轮询。
   * @return 超时前解析到完整 CAN 帧时返回 true。
   */
  bool read_frame(UsbCanFrame &frame, int timeout_ms) {
    if (!is_open()) {
      throw std::runtime_error("Serial port is not open");
    }

    if (try_parse_frame(frame)) {
      return true;
    }

    if (timeout_ms <= 0) {
      return read_available(0) && try_parse_frame(frame);
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (timeout_ms >= 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return false;
      }

      const auto remaining =
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
              .count();
      if (!read_available(static_cast<int>(remaining))) {
        return false;
      }

      if (try_parse_frame(frame)) {
        return true;
      }
    }

    return false;
  }

  /**
   * @brief 在发送新请求前丢弃当前已经到达的适配器帧。
   */
  void discard_pending() {
    UsbCanFrame ignored;
    while (read_frame(ignored, 0)) {
    }
  }

private:
  static speed_t baudrate_to_speed(int baudrate) {
    switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      throw std::invalid_argument("Unsupported serial baudrate");
    }
  }

  static void write_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    dst[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    dst[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
  }

  static uint32_t read_u32_le(const uint8_t *src) {
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) |
           (static_cast<uint32_t>(src[3]) << 24);
  }

  static uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
      crc ^= data[i];
      for (int bit = 0; bit < 8; ++bit) {
        if ((crc & 0x01) != 0) {
          crc = static_cast<uint8_t>((crc >> 1) ^ 0x8C);
        } else {
          crc = static_cast<uint8_t>(crc >> 1);
        }
      }
    }
    return crc;
  }

  bool write_all(const uint8_t *data, size_t length) {
    if (!is_open()) {
      throw std::runtime_error("Serial port is not open");
    }

    size_t written = 0;
    while (written < length) {
      const ssize_t ret = ::write(fd_, data + written, length - written);
      if (ret < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        return false;
      }
      if (ret == 0) {
        return false;
      }
      written += static_cast<size_t>(ret);
    }
    return true;
  }

  bool read_available(int timeout_ms) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);

    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int ready = ::select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return false;
    }

    std::array<uint8_t, 64> chunk{};
    const ssize_t count = ::read(fd_, chunk.data(), chunk.size());
    if (count <= 0) {
      return false;
    }

    rx_buffer_.insert(rx_buffer_.end(), chunk.begin(), chunk.begin() + count);
    return true;
  }

  bool try_parse_frame(UsbCanFrame &frame) {
    constexpr size_t kRxFrameSize = 16;

    while (rx_buffer_.size() >= kRxFrameSize) {
      const auto header = std::find(rx_buffer_.begin(), rx_buffer_.end(),
                                    static_cast<uint8_t>(0xAA));
      if (header == rx_buffer_.end()) {
        rx_buffer_.clear();
        return false;
      }
      rx_buffer_.erase(rx_buffer_.begin(), header);

      if (rx_buffer_.size() < kRxFrameSize) {
        return false;
      }

      if (rx_buffer_[15] != 0x55) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      const uint8_t format = rx_buffer_[2];
      const uint8_t dlc = static_cast<uint8_t>(format & 0x3F);
      if (dlc > 8) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      frame.command = rx_buffer_[1];
      frame.can_id = read_u32_le(rx_buffer_.data() + 3);
      frame.dlc = dlc;
      frame.is_extended = (format & 0x40) != 0;
      frame.is_remote = (format & 0x80) != 0;
      frame.data.fill(0);
      std::copy_n(rx_buffer_.begin() + 7, dlc, frame.data.begin());

      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kRxFrameSize);
      return true;
    }

    return false;
  }

  int fd_ = -1;
  std::vector<uint8_t> rx_buffer_;
};

} // 命名空间 dm_j4340_motor_driver

#endif // DM_J4340_MOTOR_DRIVER__USBCAN_SERIAL_HPP_ 头文件保护
