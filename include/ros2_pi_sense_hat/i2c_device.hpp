#pragma once

#include <cstdint>
#include <string>

class I2CDevice {
public:
  I2CDevice(const std::string& bus, uint8_t address);
  ~I2CDevice();

  bool open();
  void close();
  bool write(const uint8_t* data, size_t length);

private:
  std::string bus_;
  uint8_t address_;
  int fd_;
};
