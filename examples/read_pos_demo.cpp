/*********************************************************************
 * Copyright (c) 2025 SoftBank Corp.
 *
 * <<licensetext>>
 ********************************************************************/

#include <spdlog/spdlog.h>

#include <feetech_hardware_interface/communication_protocol.hpp>
#include <iomanip>
#include <iostream>
#include <range/v3/all.hpp>
#include <thread>
#include <tl_expected/expected.hpp>
#include <unordered_map>

using namespace std::chrono_literals;
using namespace feetech_hardware_interface;

int main(int argc, char** argv) {
  // Get serial port name from command line arguments
  if (argc != 2) {
    spdlog::error("Usage: {} <port_name>", argv[0]);
    return EXIT_FAILURE;
  }

  const std::string port_name = argv[1];

  auto serial_port = std::make_unique<SerialPort>(port_name);
  serial_port->configure().and_then([&] { return serial_port->open(); }).or_else([](const std::string& error) {
    throw std::runtime_error(error);
  });

  // Read current position of servo with ID 1
  std::array<uint8_t, 8> write_buf{};
  write_buf[2] = 0x01;  // ID
  write_buf[3] = 0x04;  // Message length
  write_buf[4] = 0x02;  // Instruction: Read
  write_buf[5] = 0x38;  // Memory address: Present Position L
  write_buf[6] = 0x02;  // Length to read
  write_buf[7] = ~sum_bytes(write_buf);
  // Set these two after calculating the checksum
  write_buf[0] = 0xFF;
  write_buf[1] = 0xFF;

  auto next_time = std::chrono::steady_clock::now();
  const auto period = std::chrono::microseconds(250);  // 4kHz

  while (true) {
    next_time += period;
    serial_port->write(write_buf).or_else([](const std::string& error) { throw std::runtime_error(error); });
    std::this_thread::sleep_until(next_time);
  }
  //   // Read response from servo
  //   std::array<uint8_t, 8> read_buf{};
  //   if (auto result = serial_port->read(&read_buf); !result) {
  //     throw std::runtime_error(result.error());
  //   }

  //   // Print response packet in hexadecimal format
  //   std::cout << "Return packet: ";
  //   for (const auto& byte : read_buf) {
  //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
  //   }
  //   std::cout << std::endl;

  return 0;
}
