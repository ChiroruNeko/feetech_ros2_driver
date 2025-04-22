#include <spdlog/spdlog.h>

#include <cstdint>
#include <feetech_hardware_interface/communication_protocol.hpp>
#include <range/v3/all.hpp>
#include <tl_expected/expected.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace feetech_hardware_interface;

// Define a structure to group servo command parameters.
struct ServoCommand {
  uint8_t id;
  float position;
  int speed;
  int acceleration;
};

void sync_write_position(CommunicationProtocol& communication_protocol, const std::vector<ServoCommand>& commands) {
  // Create separate vectors for ids, positions, speeds, and accelerations.
  std::vector<uint8_t> ids;
  std::vector<int> positions;
  std::vector<int> speeds;
  std::vector<int> accelerations;

  for (const auto& cmd : commands) {
    ids.push_back(cmd.id);
    positions.push_back(from_angle(cmd.position));
    speeds.push_back(cmd.speed);
    accelerations.push_back(cmd.acceleration);
  }

  // Log each servo's command values.
  for (size_t i = 0; i < commands.size(); ++i) {
    spdlog::info(
        "Servo id {}: position {}, speed {}, acceleration {}", ids[i], positions[i], speeds[i], accelerations[i]);
  }

  // Send the synchronous write command once.
  communication_protocol.sync_write_position(ids, positions, speeds, accelerations)
      .or_else([&](const std::string& error) {
        throw std::runtime_error(
            fmt::format("Failed to set position for ids [{}] with error: {}", fmt::join(ids, ", "), error));
      });
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
  std::vector<ServoCommand> commands = {{1, 0, 100, 0}, {2, 0, 100, 0}};
  while (true) {
    commands = {{1, 0, 100, 0}, {2, 0, 100, 0}};
    sync_write_position(communication_protocol, commands);
    sleep(1);
    commands = {{1, 90, 100, 0}, {2, 90, 100, 0}};
    sync_write_position(communication_protocol, commands);
    sleep(1);
  }
}
