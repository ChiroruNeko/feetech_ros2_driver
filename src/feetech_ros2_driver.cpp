#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <feetech_hardware_interface/common.hpp>
#include <feetech_hardware_interface/communication_protocol.hpp>
#include <feetech_ros2_driver/feetech_ros2_driver.hpp>
#include <filesystem>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

namespace feetech_ros2_driver {
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Create ROS node for subscription
  node_ = rclcpp::Node::make_shared("feetech_hardware_interface_node");

  // Initialize torque state
  torque_enabled_ = true;

  // Create torque enable subscription
  torque_enable_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "torque_enable", 10, std::bind(&FeetechHardwareInterface::torqueEnableCallback, this, std::placeholders::_1));

  // Store USB port for reconnection
  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    spdlog::error(
        "FeetechHardware::on_init Hardware parameter [{}] not found!. "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>");
    return CallbackReturn::ERROR;
  }
  usb_port_ = usb_port_it->second;

  // Initialize joint parameters
  joint_ids_.resize(info_.joints.size(), 0);
  joint_offsets_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto& joint_params = info_.joints[i].parameters;
    joint_ids_[i] = std::stoi(joint_params.at("id"));
    joint_offsets_[i] = [&] {
      if (const auto offset_it = joint_params.find("offset"); offset_it != joint_params.end()) {
        return std::stod(offset_it->second);
      }
      spdlog::info("Joint '{}' does not specify an offset parameter - Setting it to 0", info_.joints[i].name);
      return 0.0;
    }();
  }

  // Initialize serial port
  if (!initializeSerialPort()) {
    spdlog::error("FeetechHardware::on_init -> Failed to initialize serial port");
    // Don't return ERROR here - allow the system to start and try recovery later
    is_connected_ = false;
  }

  return CallbackReturn::SUCCESS;
}

bool FeetechHardwareInterface::initializeSerialPort() {
  try {
    auto serial_port = std::make_unique<feetech_hardware_interface::SerialPort>(usb_port_);

    if (const auto result = serial_port->configure(); !result) {
      spdlog::error("FeetechHardwareInterface::initializeSerialPort -> {}", result.error());
      return false;
    }

    communication_protocol_ =
        std::make_unique<feetech_hardware_interface::CommunicationProtocol>(std::move(serial_port));

    is_connected_ = true;
    consecutive_errors_ = 0;
    spdlog::info("Serial port initialized successfully on {}", usb_port_);
    return true;
  } catch (const std::exception& e) {
    spdlog::error("FeetechHardwareInterface::initializeSerialPort -> Exception: {}", e.what());
    return false;
  }
}

std::vector<hardware_interface::StateInterface> FeetechHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_hw_positions_.resize(info_.joints.size(), 0.0);
  state_hw_velocities_.resize(info_.joints.size(), 0.0);
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FeetechHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  command_interfaces.reserve(info_.joints.size() * 3);
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_accelerations_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type FeetechHardwareInterface::read(const rclcpp::Time& /* time */,
                                                               const rclcpp::Duration& /* period */) {
  // Process ROS callbacks for torque enable subscription
  rclcpp::spin_some(node_);

  // Check if we need to recover the connection
  if (!is_connected_ && !is_recovering_) {
    if (recoverSerialPort()) {
      spdlog::info("Serial port recovered successfully");
      consecutive_errors_ = 0;
    } else {
      // Continue with degraded mode
      return hardware_interface::return_type::OK;
    }
  }

  // Skip reading if not connected
  if (!is_connected_) {
    return hardware_interface::return_type::OK;
  }

  // Uncomment and modify the following code when ready to implement reading
  /*
  std::vector<std::array<uint8_t, 4>> data;
  data.reserve(joint_ids_.size());

  {
    std::lock_guard<std::mutex> lock(communication_mutex_);
    if (auto result = communication_protocol_->sync_read(joint_ids_, HLS_PRESENT_POSITION_L, &data); !result) {
      spdlog::error("FeetechHardwareInterface::read -> {}", result.error());
      consecutive_errors_++;

      if (consecutive_errors_ >= ERROR_THRESHOLD) {
        spdlog::warn("Too many consecutive read errors ({}), attempting recovery", consecutive_errors_);
        is_connected_ = false;
      }

      return hardware_interface::return_type::OK;  // Continue operation even with errors
    }
  }

  consecutive_errors_ = 0;  // Reset error count on successful read

  ranges::for_each(data | ranges::views::enumerate, [&](const auto& values) {
    const auto& [index, readings] = values;
    // Get position
    state_hw_positions_[index] = feetech_hardware_interface::to_radians(feetech_hardware_interface::from_sts(
                                     feetech_hardware_interface::WordBytes{.low = readings[0], .high = readings[1]}))
                                     -
                                 joint_offsets_[index];
    // Get velocity
    const uint16_t raw_velocity = feetech_hardware_interface::from_sts(
        feetech_hardware_interface::WordBytes{.low = readings[2], .high = readings[3]});
    const int16_t decoded_velocity = feetech_hardware_interface::decode_feetech_velocity(raw_velocity);
    state_hw_velocities_[index] = feetech_hardware_interface::to_radians_per_second(decoded_velocity);
  });
  */

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  // Check if we need to recover the connection
  if (!is_connected_ && !is_recovering_) {
    if (recoverSerialPort()) {
      spdlog::info("Serial port recovered successfully");
      consecutive_errors_ = 0;
    } else {
      // Continue with degraded mode
      return hardware_interface::return_type::OK;
    }
  }

  // Skip writing if not connected or torque is disabled
  if (!is_connected_ || !torque_enabled_) {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(communication_mutex_);  // Protect serial communication

  const auto positions = ranges::views::zip(hw_positions_, joint_offsets_) |
                         ranges::views::transform([&](const auto tuple) {
                           auto [position, offset] = tuple;
                           return feetech_hardware_interface::from_radians(position + offset);
                         }) |
                         ranges::to_vector;

  const auto velocities = hw_velocities_ | ranges::views::transform([](const double radians_per_second) {
                            const int raw_velocity =
                                feetech_hardware_interface::from_radians_per_second(radians_per_second);
                            return static_cast<int>(feetech_hardware_interface::encode_feetech_velocity(raw_velocity));
                          }) |
                          ranges::to_vector;

  const auto accelerations = hw_accelerations_ |
                             ranges::views::transform(feetech_hardware_interface::from_radians_per_second_squared) |
                             ranges::to_vector;

  const auto write_result =
      communication_protocol_->sync_write_motion_control(joint_ids_, positions, velocities, accelerations);

  if (!write_result) {
    spdlog::error("FeetechHardwareInterface::write -> {}", write_result.error());
    consecutive_errors_++;

    if (consecutive_errors_ >= ERROR_THRESHOLD) {
      spdlog::warn("Too many consecutive write errors ({}), attempting recovery", consecutive_errors_);
      is_connected_ = false;

      // Try to recover immediately
      if (!recoverSerialPort()) {
        spdlog::error("Failed to recover serial port after write error");
      }
    }

    return hardware_interface::return_type::OK;  // Continue operation even with errors
  }

  consecutive_errors_ = 0;  // Reset error count on successful write

  // TODO(ChiroruNeko): Fix to use read() to update state_hw_positions_ and state_hw_velocities_
  state_hw_positions_ = hw_positions_;
  state_hw_velocities_ = hw_velocities_;

  return hardware_interface::return_type::OK;
}

CallbackReturn FeetechHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Ensure connection is established before activation
  if (!is_connected_) {
    spdlog::warn("Attempting to establish connection during activation...");
    if (!recoverSerialPort()) {
      spdlog::error("Failed to establish connection during activation");
      // Still allow activation to proceed - will retry later
    }
  }

  // Time/Duration are not used
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  // Set the initial command to current joint positions
  hw_positions_ = state_hw_positions_;
  // Initialize velocity and acceleration commands to zero
  ranges::fill(hw_velocities_, 0.0);
  ranges::fill(hw_accelerations_, 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Disable torque when deactivating
  // setTorqueEnable(false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /* previous_state */) {
  // Clean up resources
  is_connected_ = false;
  communication_protocol_.reset();
  return CallbackReturn::SUCCESS;
}

void FeetechHardwareInterface::torqueEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  spdlog::info("Torque enable callback: {}", msg->data ? "true" : "false");
  setTorqueEnable(msg->data);
}

hardware_interface::return_type FeetechHardwareInterface::setTorqueEnable(bool enable) {
  if (!is_connected_) {
    spdlog::warn("Cannot set torque - serial port not connected");
    torque_enabled_ = enable;  // Store the desired state for when connection is restored
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(communication_mutex_);  // Protect serial communication

  torque_enabled_ = enable;

  // Send torque enable/disable command to all servos
  for (const auto& joint_id : joint_ids_) {
    const uint8_t torque_value = enable ? 1 : 0;
    const auto result =
        communication_protocol_->write(joint_id, HLS_TORQUE_ENABLE, std::experimental::make_array(torque_value));

    if (!result) {
      spdlog::error("FeetechHardwareInterface::setTorqueEnable -> Failed to set torque for servo {}: {}",
                    joint_id,
                    result.error());
      // Don't return ERROR - continue trying other servos
    }
  }

  spdlog::info("Torque {} for all servos", enable ? "enabled" : "disabled");
  return hardware_interface::return_type::OK;
}

bool FeetechHardwareInterface::recoverSerialPort() {
  // Prevent multiple simultaneous recovery attempts
  bool expected = false;
  if (!is_recovering_.compare_exchange_strong(expected, true)) {
    return false;  // Recovery already in progress
  }

  // Ensure minimum time between recovery attempts
  auto now = std::chrono::steady_clock::now();
  if (now - last_recovery_attempt_ < RECOVERY_RETRY_DELAY) {
    is_recovering_ = false;
    return false;
  }
  last_recovery_attempt_ = now;

  spdlog::warn("Attempting to recover serial port {}...", usb_port_);

  // Close existing connection using the safe reset method
  try {
    if (communication_protocol_) {
      communication_protocol_->safeReset();
      // Use nullptr assignment instead of reset() to avoid any possible exceptions
      communication_protocol_ = nullptr;
    }
  } catch (...) {
    spdlog::warn("Caught exception during communication protocol cleanup - continuing recovery");
  }

  // Small delay to allow USB device to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Check if the device file exists
  if (!std::filesystem::exists(usb_port_)) {
    spdlog::warn("USB device {} not found. Waiting for device to appear...", usb_port_);
    is_recovering_ = false;
    return false;
  }

  // Try to re-initialize the serial port
  bool success = false;
  for (int attempt = 1; attempt <= MAX_RECOVERY_ATTEMPTS; ++attempt) {
    spdlog::info("Recovery attempt {}/{}", attempt, MAX_RECOVERY_ATTEMPTS);

    if (initializeSerialPort()) {
      spdlog::info("Serial port recovery successful on attempt {}", attempt);

      // Re-enable torque if it was enabled before disconnection
      // if (torque_enabled_) {
      //   setTorqueEnable(true);
      // }

      success = true;
      break;
    }

    if (attempt < MAX_RECOVERY_ATTEMPTS) {
      std::this_thread::sleep_for(RECOVERY_RETRY_DELAY);
    }
  }

  if (!success) {
    spdlog::error("Failed to recover serial port after {} attempts", MAX_RECOVERY_ATTEMPTS);
  }

  is_recovering_ = false;
  return success;
}

}  // namespace feetech_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_ros2_driver::FeetechHardwareInterface, hardware_interface::SystemInterface)
