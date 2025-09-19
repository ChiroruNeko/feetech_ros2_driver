#pragma once

#include <atomic>
#include <chrono>
#include <feetech_hardware_interface/communication_protocol.hpp>
#include <feetech_hardware_interface/serial_port.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

namespace feetech_ros2_driver {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FeetechHardwareInterface : public hardware_interface::SystemInterface {
 public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void torqueEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  hardware_interface::return_type setTorqueEnable(bool enable);

  std::unique_ptr<feetech_hardware_interface::CommunicationProtocol> communication_protocol_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_accelerations_;
  std::vector<double> state_hw_positions_;
  std::vector<double> state_hw_velocities_;
  std::vector<uint8_t> previous_hw_positions_;

  std::vector<uint8_t> joint_ids_;
  std::vector<double> joint_offsets_;  // rad

  // Torque control
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr torque_enable_sub_;
  bool torque_enabled_;
  std::mutex communication_mutex_;  // Protect serial communication

  // Connection management
  bool recoverSerialPort();
  bool initializeSerialPort();
  std::string usb_port_;                    // Store USB port path for reconnection
  std::atomic<bool> is_connected_{false};   // Track connection status
  std::atomic<bool> is_recovering_{false};  // Prevent multiple recovery attempts
  std::chrono::steady_clock::time_point last_recovery_attempt_;
  static constexpr std::chrono::milliseconds RECOVERY_RETRY_DELAY{1000};  // 1 second between recovery attempts
  static constexpr int MAX_RECOVERY_ATTEMPTS = 3;  // Maximum number of consecutive recovery attempts
  int consecutive_errors_ = 0;                     // Track consecutive errors
  static constexpr int ERROR_THRESHOLD = 5;        // Number of errors before attempting recovery
};
}  // namespace feetech_ros2_driver
