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
