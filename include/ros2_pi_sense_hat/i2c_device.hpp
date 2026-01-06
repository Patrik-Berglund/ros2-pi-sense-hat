#pragma once

#include <cstdint>
#include <string>

class I2CDevice {
public:
  I2CDevice(const std::string& bus, uint8_t address);
  ~I2CDevice();

  bool open();
  void close();
  
  // Raw I2C operations
  bool write(const uint8_t* data, size_t length);
  bool read(uint8_t* data, size_t length);
  
  // Register operations for sensor devices
  bool writeReg(uint8_t reg, uint8_t value);
  bool readReg(uint8_t reg, uint8_t& value);
  
  // Multi-byte operations with auto-increment support
  bool readMultiReg(uint8_t reg, uint8_t* data, size_t length, bool auto_increment = false);
  
  // 16-bit register read (little endian)
  bool readReg16(uint8_t reg, int16_t& value, bool auto_increment = false);
  
  // Check if device is open
  bool isOpen() const;

private:
  std::string bus_;
  uint8_t address_;
  int fd_;
};
