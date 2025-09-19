#pragma once

#include <fmt/core.h>
#include <libserial/SerialPort.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <feetech_hardware_interface/common.hpp>
#include <filesystem>
#include <range/v3/all.hpp>
#include <string>
#include <thread>

namespace feetech_hardware_interface {

class SerialPort {
 public:
  explicit SerialPort(const std::string& /*dev*/);
  ~SerialPort();
  Result configure(LibSerial::BaudRate baud_rate = LibSerial::BaudRate::BAUD_1000000);
  Result open();
  Result close();
  Result flashInputBuffer() noexcept;
  Result flashOutputBuffer() noexcept;

 private:
  Result flushBuffer(bool isInput) noexcept;

 public:
  void safeCleanup() {
    is_port_connected_ = false;

    if (port_ != nullptr) {
      spdlog::debug("Safely nullifying serial port pointer without deletion");
      port_ = nullptr;
    }
  }

  Result read_byte(uint8_t* byte) {
    if (port_ == nullptr) {
      is_port_connected_ = false;
      return tl::make_unexpected("Serial port object is not initialized");
    }

    try {
      port_->ReadByte(*byte, static_cast<std::size_t>(timeout_.count()));
    } catch (const LibSerial::ReadTimeout& e) {
      return tl::make_unexpected(fmt::format("SerialPort::read_byte [{}]", e.what()));
    } catch (const std::runtime_error& e) {
      // Mark port as potentially disconnected
      is_port_connected_ = false;
      return tl::make_unexpected(fmt::format("SerialPort::read_byte [{}]", e.what()));
    }

    return {};
  }

  template <std::size_t N>
  Result read(std::array<uint8_t, N>* buffer) {
    return check_port().and_then([&]() -> Result {
      for (auto& byte : *buffer) {
        if (const auto result = read_byte(&byte); !result) {
          return tl::make_unexpected(fmt::format("SerialPort::read -> {}", result.error()));
        }
      }
      return {};
    });
  }

  template <std::size_t N>
  Result write(const std::array<uint8_t, N>& buffer) {
    return check_port().and_then([&]() -> Result {
      if (port_ == nullptr) {
        is_port_connected_ = false;
        return tl::make_unexpected("Serial port object is not initialized");
      }

      try {
        port_->Write(std::string(buffer.begin(), buffer.end()));
      } catch (const std::runtime_error& e) {
        // Mark port as potentially disconnected
        is_port_connected_ = false;
        return tl::make_unexpected(fmt::format("SerialPort::write [{}]", e.what()));
      }
      return {};
    });
  }

  Result try_reconnect() {
    safeCleanup();

    if (!std::filesystem::exists(dev_)) {
      is_port_connected_ = false;
      return tl::make_unexpected(fmt::format("Device {} not found", dev_));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    try {
      port_ = new LibSerial::SerialPort();
      port_->Open(dev_);
      is_port_connected_ = true;
      return {};
    } catch (...) {
      // Set pointer to nullptr on exception (memory leak is tolerated)
      port_ = nullptr;
      is_port_connected_ = false;
      return tl::make_unexpected(fmt::format("Reconnection failed for device {}", dev_));
    }
  }

 private:
  [[nodiscard]] Result check_port() const noexcept;
  std::string dev_;
  std::chrono::milliseconds timeout_ = std::chrono::milliseconds(10);

  // Use raw pointer instead of std::optional to have more control over destruction
  // This allows us to simply set to nullptr without triggering any destructor calls
  LibSerial::SerialPort* port_ = nullptr;

  bool is_port_connected_ = true;
};
}  // namespace feetech_hardware_interface
