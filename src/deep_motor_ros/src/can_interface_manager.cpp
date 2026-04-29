#include "deep_motor_ros/can_interface_manager.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <unistd.h>

#include "deep_motor_ros/motor_utils.hpp"

using namespace std::chrono_literals;

namespace deep_motor_ros
{

namespace
{

constexpr const char * kEmbeddedSudoPassword = "zhr1778096";

}  // namespace

CanInterfaceManager::CanInterfaceManager(rclcpp::Logger logger, CanInterfaceConfig config)
: logger_(logger),
  config_(config)
{
}

void CanInterfaceManager::wait_for_interfaces(
  const std::vector<std::string> & can_interfaces) const
{
  if (can_interfaces.empty())
  {
    return;
  }

  const auto wait_deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(config_.wait_sec);

  while (true)
  {
    std::vector<std::string> missing_interfaces;
    for (const auto & can_interface : can_interfaces)
    {
      if (!interface_exists(can_interface))
      {
        missing_interfaces.push_back(can_interface);
      }
    }

    if (missing_interfaces.empty())
    {
      RCLCPP_INFO(
        logger_,
        "[CanDetect] Found CAN interfaces: [%s]",
        join_strings(can_interfaces).c_str());
      return;
    }

    if (config_.wait_sec <= 0.0 || std::chrono::steady_clock::now() >= wait_deadline)
    {
      RCLCPP_WARN(
        logger_,
        "[CanDetect] Missing CAN interfaces after waiting %.1fs: [%s]",
        config_.wait_sec, join_strings(missing_interfaces).c_str());
      return;
    }

    std::this_thread::sleep_for(100ms);
  }
}

void CanInterfaceManager::configure_interfaces(
  const std::vector<std::string> & can_interfaces) const
{
  for (const auto & can_interface : can_interfaces)
  {
    if (!interface_exists(can_interface))
    {
      RCLCPP_WARN(
        logger_,
        "[CanSetup] %s is missing before configuration. Check USB-CAN connection and udev rules.",
        can_interface.c_str());
      continue;
    }

    log_interface_snapshot(can_interface, "before-config");

    bool configured = false;
    for (int attempt = 1; attempt <= config_.configure_retries; ++attempt)
    {
      const bool ok_down = run_setup_command(
        can_interface, "ip link set " + can_interface + " down");
      const bool ok_type = run_setup_command(
        can_interface,
        "ip link set " + can_interface + " type can bitrate " +
        std::to_string(config_.bitrate));
      const bool ok_queue = run_setup_command(
        can_interface,
        "ip link set " + can_interface + " txqueuelen " +
        std::to_string(config_.txqueuelen));
      const bool ok_up = run_setup_command(
        can_interface, "ip link set " + can_interface + " up");

      configured = ok_down && ok_type && ok_queue && ok_up;
      log_interface_snapshot(can_interface, configured ? "after-config" : "after-config-failed");
      if (configured)
      {
        break;
      }

      RCLCPP_WARN(
        logger_,
        "[CanSetup] %s configuration attempt %d/%d did not fully succeed.",
        can_interface.c_str(), attempt, config_.configure_retries);
      std::this_thread::sleep_for(200ms);
    }
  }
}

void CanInterfaceManager::bring_interfaces_down(
  const std::vector<std::string> & can_interfaces) const
{
  for (const auto & can_interface : can_interfaces)
  {
    if (!interface_exists(can_interface))
    {
      continue;
    }

    const bool ok_down = run_setup_command(
      can_interface, "ip link set " + can_interface + " down");
    log_interface_snapshot(can_interface, ok_down ? "after-down" : "after-down-failed");
  }
}

void CanInterfaceManager::log_interface_snapshot(
  const std::string & can_interface,
  const std::string & stage) const
{
  if (!interface_exists(can_interface))
  {
    RCLCPP_WARN(
      logger_, "[CanState] %s %s -> missing", stage.c_str(), can_interface.c_str());
    return;
  }

  const auto snapshot =
    read_command_output("ip -br link show dev " + can_interface + " 2>/dev/null");
  if (snapshot.empty())
  {
    RCLCPP_INFO(
      logger_,
      "[CanState] %s %s -> detected, but ip link returned no summary",
      stage.c_str(), can_interface.c_str());
    return;
  }

  RCLCPP_INFO(logger_, "[CanState] %s %s", stage.c_str(), snapshot.c_str());
}

bool CanInterfaceManager::run_setup_command(
  const std::string & can_interface,
  const std::string & cmd) const
{
  const auto direct_cmd = cmd + " >/dev/null 2>&1";
  if (std::system(direct_cmd.c_str()) == 0)
  {
    return true;
  }

  if (geteuid() != 0)
  {
    const auto sudo_cmd =
      "echo " + shell_single_quote(kEmbeddedSudoPassword) +
      " | sudo -S sh -c " + shell_single_quote(cmd + " >/dev/null 2>&1") +
      " >/dev/null 2>&1";
    if (std::system(sudo_cmd.c_str()) == 0)
    {
      RCLCPP_INFO(
        logger_,
        "[CanSetup] %s succeeded via embedded sudo fallback: %s",
        can_interface.c_str(), cmd.c_str());
      return true;
    }
  }

  RCLCPP_WARN(
    logger_,
    "CAN configure command failed on %s: %s",
    can_interface.c_str(), cmd.c_str());
  return false;
}

std::string CanInterfaceManager::read_command_output(const std::string & cmd) const
{
  std::array<char, 256> buffer{};
  std::string output;
  FILE * pipe = popen(cmd.c_str(), "r");
  if (!pipe)
  {
    return output;
  }

  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr)
  {
    output += buffer.data();
  }
  pclose(pipe);
  return trim_copy(output);
}

}  // namespace deep_motor_ros
