#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

I2CDevice::I2CDevice(const std::string& bus, uint8_t address)
  : bus_(bus), address_(address), fd_(-1) {}

I2CDevice::~I2CDevice() {
  close();
}

bool I2CDevice::open() {
  fd_ = ::open(bus_.c_str(), O_RDWR);
  if (fd_ < 0) return false;
  
  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

void I2CDevice::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool I2CDevice::write(const uint8_t* data, size_t length) {
  if (fd_ < 0) return false;
  return ::write(fd_, data, length) == static_cast<ssize_t>(length);
}

bool I2CDevice::read(uint8_t* data, size_t length) {
  if (fd_ < 0) return false;
  return ::read(fd_, data, length) == static_cast<ssize_t>(length);
}

bool I2CDevice::writeReg(uint8_t reg, uint8_t value) {
  if (fd_ < 0) return false;
  uint8_t data[2] = {reg, value};
  return write(data, 2);
}

bool I2CDevice::readReg(uint8_t reg, uint8_t& value) {
  if (fd_ < 0) return false;
  if (!write(&reg, 1)) return false;  // Write register address
  return read(&value, 1);             // Read register value
}

bool I2CDevice::readMultiReg(uint8_t reg, uint8_t* data, size_t length, bool auto_increment) {
  if (fd_ < 0) return false;
  
  // Set MSB for auto-increment if requested (magnetometer style)
  uint8_t reg_addr = auto_increment ? (reg | 0x80) : reg;
  
  if (!write(&reg_addr, 1)) return false;  // Write register address
  return read(data, length);               // Read multiple bytes
}

bool I2CDevice::readReg16(uint8_t reg, int16_t& value, bool auto_increment) {
  uint8_t data[2];
  if (!readMultiReg(reg, data, 2, auto_increment)) return false;
  
  // LSM9DS1 uses little endian format
  value = static_cast<int16_t>(data[0] | (data[1] << 8));
  return true;
}

bool I2CDevice::isOpen() const {
  return fd_ >= 0;
}
