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
  // Write all zeros to hardware
  uint8_t buffer[193];
  buffer[0] = 0x00;
  std::memset(&buffer[1], 0, 192);
  i2c_.write(buffer, 193);
}

void ATTiny88Driver::setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
  if (x >= 8 || y >= 8) return;
  
  // Calculate register addresses for this pixel
  size_t row_offset = y * 24;
  uint8_t r_addr = row_offset + x;
  uint8_t g_addr = row_offset + 8 + x;
  uint8_t b_addr = row_offset + 16 + x;
  
  // Write each color component directly to hardware
  uint8_t r_buf[2] = {r_addr, static_cast<uint8_t>(r & 0x1F)};
  uint8_t g_buf[2] = {g_addr, static_cast<uint8_t>(g & 0x1F)};
  uint8_t b_buf[2] = {b_addr, static_cast<uint8_t>(b & 0x1F)};
  
  i2c_.write(r_buf, 2);
  i2c_.write(g_buf, 2);
  i2c_.write(b_buf, 2);
  
  // Update local framebuffer
  framebuffer_[r_addr] = r & 0x1F;
  framebuffer_[g_addr] = g & 0x1F;
  framebuffer_[b_addr] = b & 0x1F;
}

void ATTiny88Driver::setAll(const uint8_t* rgb_data, size_t length) {
  size_t copy_len = std::min(length, framebuffer_.size());
  
  // Compare and update only changed pixels
  for (size_t i = 0; i < copy_len; i++) {
    uint8_t new_val = rgb_data[i] & 0x1F;
    if (framebuffer_[i] != new_val) {
      // Write changed byte directly to hardware
      uint8_t buffer[2] = {static_cast<uint8_t>(i), new_val};
      i2c_.write(buffer, 2);
      framebuffer_[i] = new_val;
    }
  }
}


