#include <spdlog/spdlog.h>

#include <feetech_hardware_interface/serial_port.hpp>
#include <tl_expected/expected.hpp>

namespace feetech_hardware_interface {

Expected<LibSerial::BaudRate> to_baudrate(const std::size_t baud) noexcept {
  using LibSerial::BaudRate;
  switch (baud) {
    case 50:
      return BaudRate::BAUD_50;
    case 75:
      return BaudRate::BAUD_75;
    case 110:
      return BaudRate::BAUD_110;
    case 134:
      return BaudRate::BAUD_134;
    case 150:
      return BaudRate::BAUD_150;
    case 200:
      return BaudRate::BAUD_200;
    case 300:
      return BaudRate::BAUD_300;
    case 600:
      return BaudRate::BAUD_600;
    case 1'200:
      return BaudRate::BAUD_1200;
    case 1'800:
      return BaudRate::BAUD_1800;
    case 2'400:
      return BaudRate::BAUD_2400;
    case 4'800:
      return BaudRate::BAUD_4800;
    case 9'600:
      return BaudRate::BAUD_9600;
    case 19'200:
      return BaudRate::BAUD_19200;
    case 38'400:
      return BaudRate::BAUD_38400;
    case 57'600:
      return BaudRate::BAUD_57600;
    case 115'200:
      return BaudRate::BAUD_115200;
    case 230'400:
      return BaudRate::BAUD_230400;
#ifdef __linux__
    case 460'800:
      return BaudRate::BAUD_460800;
    case 500'000:
      return BaudRate::BAUD_500000;
    case 576'000:
      return BaudRate::BAUD_576000;
    case 921'600:
      return BaudRate::BAUD_921600;
    case 1'000'000:
      return BaudRate::BAUD_1000000;
    case 1'152'000:
      return BaudRate::BAUD_1152000;
    case 1'500'000:
      return BaudRate::BAUD_1500000;
#if __MAX_BAUD > B2000000
    case 2'000'000:
      return BaudRate::BAUD_2000000;
    case 2'500'000:
      return BaudRate::BAUD_2500000;
    case 3'000'000:
      return BaudRate::BAUD_3000000;
    case 3'500'000:
      return BaudRate::BAUD_3500000;
    case 4'000'000:
      return BaudRate::BAUD_4000000;
#endif /* __MAX_BAUD */
#endif /* __linux__ */
  }

  return tl::make_unexpected(fmt::format("Invalid baud rate: [{}]", baud));
}

SerialPort::SerialPort(const std::string& dev) : dev_(dev) {
  spdlog::info("Connecting to port: {}", dev);
  // We'll create the port object when we need it
  port_ = nullptr;
}

SerialPort::~SerialPort() {
  spdlog::debug("SerialPort for {} being destroyed", dev_);

  // Mark as disconnected first
  is_port_connected_ = false;

  // Use safeCleanup to properly clean up resources
  safeCleanup();
}

Result SerialPort::configure(const LibSerial::BaudRate baud_rate) {
  if (auto result = open(); !result) {
    return result;
  }

  try {
    if (port_ != nullptr) {
      port_->SetBaudRate(baud_rate);
    } else {
      return tl::make_unexpected("Serial port object is not initialized");
    }
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(fmt::format("Configuring the serial port failed: [{}]", e.what()));
  }
  return {};
}

Result SerialPort::open() {
  try {
    // Create a new port if needed
    if (port_ == nullptr) {
      port_ = new LibSerial::SerialPort();
    }

    if (!port_->IsOpen()) {
      port_->Open(dev_);
      is_port_connected_ = true;
    }
  } catch (const std::exception& e) {
    port_ = nullptr;
    is_port_connected_ = false;

    const char* error_type = dynamic_cast<const LibSerial::OpenFailed*>(&e) ? "Open failed" : "Unexpected error";
    return tl::make_unexpected(fmt::format("Open [{}]: {} - {}", dev_.c_str(), error_type, e.what()));
  }

  return {};
}

Result SerialPort::close() {
  // Mark port as disconnected
  is_port_connected_ = false;

  if (port_ == nullptr) {
    // Port doesn't exist, nothing to do
    return {};
  }

  if (!port_->IsOpen()) {
    // Port is already closed, nothing to do
    return {};
  }

  try {
    port_->Close();
  } catch (const LibSerial::AlreadyOpen& e) {
    spdlog::warn("close [{}]: {}", dev_.c_str(), e.what());
    // Don't return an error - port might actually be disconnected
  } catch (const std::runtime_error& e) {
    spdlog::warn("close [{}]: {}", dev_.c_str(), e.what());
    // Don't return an error - port might actually be disconnected
  }
  return {};
}

Result SerialPort::check_port() const noexcept {
  if (port_ == nullptr || !port_->IsOpen()) {
    if (!is_port_connected_) {
      // Port was previously marked as disconnected
      // The const_cast is needed because check_port is const but we need to modify state
      auto* self = const_cast<SerialPort*>(this);
      return self->try_reconnect().and_then([]() -> Result { return {}; });
    }
    return tl::make_unexpected(fmt::format("Port [{}] is not open", dev_));
  }

  return {};
}

Result SerialPort::flushBuffer(bool isInput) noexcept {
  if (auto result = check_port(); !result) {
    return result;
  }

  try {
    if (port_ != nullptr) {
      if (isInput) {
        port_->FlushInputBuffer();
      } else {
        port_->FlushOutputBuffer();
      }
    } else {
      return tl::make_unexpected("Serial port object is not initialized");
    }
  } catch (const std::runtime_error& e) {
    is_port_connected_ = false;
    return tl::make_unexpected(e.what());
  }

  return {};
}

Result SerialPort::flashInputBuffer() noexcept { return flushBuffer(true); }

Result SerialPort::flashOutputBuffer() noexcept { return flushBuffer(false); }
}  // namespace feetech_hardware_interface
