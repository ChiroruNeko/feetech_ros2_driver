#include <spdlog/spdlog.h>

#include <cstdint>
#include <cstdlib>
#include <feetech_hardware_interface/communication_protocol.hpp>
#include <iostream>
#include <range/v3/all.hpp>
#include <thread>
#include <tl_expected/expected.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace feetech_hardware_interface;

void set_id(CommunicationProtocol& communication_protocol, uint8_t old_id, uint8_t new_id) {
  // unlock eprom
  communication_protocol.unlock_eprom(old_id);

  std::array<uint8_t, 1> buffer{new_id};
  if (!communication_protocol.write(old_id, HLS_ID, buffer)) {
    spdlog::error("Failed to set ID.", old_id);
  } else {
    spdlog::info("Servo ID {}: Setting new ID to {}", old_id, new_id);
  }

  // lock eprom
  communication_protocol.lock_eprom(new_id);
}

void set_position_limit(CommunicationProtocol& communication_protocol,
                        uint8_t id,
                        uint16_t min_position,
                        uint16_t max_position) {
  // unlock eprom
  communication_protocol.unlock_eprom(id);

  std::array<uint8_t, 4> buffer{};
  to_sts(&buffer[0], &buffer[1], min_position);
  to_sts(&buffer[2], &buffer[3], max_position);

  if (!communication_protocol.write(id, HLS_MIN_ANGLE_LIMIT_L, buffer)) {
    spdlog::error("Failed to set angle_limit.", id);
  } else {
    spdlog::info("Servo ID {}: Setting angle_limit to ({}, {})", id, min_position, max_position);
  }

  // lock eprom
  communication_protocol.lock_eprom(id);
}

void set_pid_gain(
    CommunicationProtocol& communication_protocol, uint8_t id, uint8_t p_gain, uint8_t d_gain, uint8_t i_gain) {
  // unlock eprom
  communication_protocol.unlock_eprom(id);

  std::array<uint8_t, 3> buffer{p_gain, d_gain, i_gain};
  if (!communication_protocol.write(id, HLS_P_COEF, buffer)) {
    spdlog::error("Failed to set PID gain.", id);
  } else {
    spdlog::info("Servo ID {}: Setting PID gain to p={} d={} i={}", id, p_gain, d_gain, i_gain);
  }

  // lock eprom
  communication_protocol.lock_eprom(id);
}

std::string get_input(const std::string_view prompt) {
  std::string input;
  spdlog::info("{}", prompt);
  std::getline(std::cin, input);
  return input;
}

void write_initial_position(CommunicationProtocol& communication_protocol, uint8_t id) {
  const auto sleep_time = 100ms;
  while (true) {
    const int target_angle = 180;  // deg
    const auto data = from_angle(target_angle);
    spdlog::info("Setting position to {} deg: {}", target_angle, data);

    if (!communication_protocol.write_position(id, data, 128, 0)) {  // Acc is max when 0 is set
      spdlog::error("Failed to set position");
    }

    // double position = -1.;
    // while (std::abs(position - target_angle) > 1) {
    //   position =
    //       to_angle(communication_protocol.read_position(id)
    //                    .or_else([](const std::string& error) -> Expected<int> { throw std::runtime_error(error); })
    //                    .value());
    //   spdlog::info("Current position: {:.3f}°", position);
    //   std::this_thread::sleep_for(sleep_time);
    // }
    break;
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    spdlog::error("Usage: <port_name>", argv[0]);
    return EXIT_FAILURE;
  }

  const std::string port_name = argv[1];

  auto serial_port = std::make_unique<SerialPort>(port_name);
  serial_port->configure().and_then([&] { return serial_port->open(); }).or_else([](const std::string& error) {
    throw std::runtime_error(error);
  });

  auto communication_protocol = CommunicationProtocol(std::move(serial_port));

  // Set parmeters
  while (true) {
    const auto old_id = std::stoi(get_input("Enter ID of target servo: "));
    const auto new_id = std::stoi(get_input("Enter new ID: "));

    set_id(communication_protocol, old_id, new_id);
    std::this_thread::sleep_for(100ms);
    set_position_limit(communication_protocol, new_id, 1, 4095);
    std::this_thread::sleep_for(100ms);
    set_pid_gain(communication_protocol, new_id, 32, 32, 0);
    std::this_thread::sleep_for(100ms);
    write_initial_position(communication_protocol, new_id);
    std::this_thread::sleep_for(100ms);
  }
}
