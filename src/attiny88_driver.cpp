#include "ros2_pi_sense_hat/attiny88_driver.hpp"
#include <cstring>
#include <algorithm>

ATTiny88Driver::ATTiny88Driver() : i2c_("/dev/i2c-1", 0x46) {
  framebuffer_.fill(0);
}

bool ATTiny88Driver::init() {
  return i2c_.open();
}

void ATTiny88Driver::clear() {
  framebuffer_.fill(0);
}

void ATTiny88Driver::setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
  if (x >= 8 || y >= 8) return;
  // Hardware uses planar format: R R R R R R R R  G G G G G G G G  B B B B B B B B per row
  size_t row_offset = y * 24;  // 24 bytes per row
  framebuffer_[row_offset + x] = r & 0x1F;          // Red plane
  framebuffer_[row_offset + 8 + x] = g & 0x1F;      // Green plane
  framebuffer_[row_offset + 16 + x] = b & 0x1F;     // Blue plane
}

void ATTiny88Driver::setAll(const uint8_t* rgb_data, size_t length) {
  size_t copy_len = std::min(length, framebuffer_.size());
  std::memcpy(framebuffer_.data(), rgb_data, copy_len);
  // Mask to 5-bit per channel
  for (auto& val : framebuffer_) {
    val &= 0x1F;
  }
}

void ATTiny88Driver::update() {
  uint8_t buffer[193];
  buffer[0] = 0x00;  // Start register
  std::memcpy(&buffer[1], framebuffer_.data(), 192);
  i2c_.write(buffer, 193);
}
