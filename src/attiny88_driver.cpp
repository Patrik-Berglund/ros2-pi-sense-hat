#include "ros2_pi_sense_hat/attiny88_driver.hpp"
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <unistd.h>

ATTiny88Driver::ATTiny88Driver() : i2c_("/dev/i2c-1", 0x46), gpio_chip_(nullptr), frame_int_line_(nullptr), frame_sync_enabled_(false) {
  framebuffer_.fill(0);
}

ATTiny88Driver::~ATTiny88Driver() {
  cleanupFrameSync();
}

bool ATTiny88Driver::init() {
  if (!i2c_.open()) return false;
  initFrameSync();
  return true;
}

bool ATTiny88Driver::initFrameSync() {
  // Open GPIO chip 0
  gpio_chip_ = gpiod_chip_open("/dev/gpiochip0");
  if (!gpio_chip_) {
    printf("FRAME_INT: Failed to open gpiochip0\n");
    return false;
  }
  
  // Get GPIO24 line
  frame_int_line_ = gpiod_chip_get_line(gpio_chip_, 24);
  if (!frame_int_line_) {
    printf("FRAME_INT: Failed to get GPIO24 line\n");
    cleanupFrameSync();
    return false;
  }
  
  // Request line for rising edge events (frame completion)
  int ret = gpiod_line_request_rising_edge_events(frame_int_line_, "sense_hat_frame_int");
  if (ret < 0) {
    printf("FRAME_INT: Failed to request GPIO24 events\n");
    cleanupFrameSync();
    return false;
  }
  
  frame_sync_enabled_ = true;
  printf("FRAME_INT: Enabled on GPIO24 via libgpiod\n");
  return true;
}

void ATTiny88Driver::waitForFrameSync() {
  if (!frame_sync_enabled_ || !frame_int_line_) return;
  
  // Wait up to 10ms for rising edge event (frame completion)
  struct timespec timeout = {0, 10000000}; // 10ms
  int ret = gpiod_line_event_wait(frame_int_line_, &timeout);
  
  if (ret == 1) {
    // Event occurred, read it to clear
    struct gpiod_line_event event;
    gpiod_line_event_read(frame_int_line_, &event);
  }
}

void ATTiny88Driver::cleanupFrameSync() {
  if (frame_int_line_) {
    gpiod_line_release(frame_int_line_);
    frame_int_line_ = nullptr;
  }
  if (gpio_chip_) {
    gpiod_chip_close(gpio_chip_);
    gpio_chip_ = nullptr;
  }
  frame_sync_enabled_ = false;
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
  
  // Update framebuffer
  size_t row_offset = y * 24;
  framebuffer_[row_offset + x] = r & 0x1F;      // Red plane
  framebuffer_[row_offset + 8 + x] = g & 0x1F;  // Green plane
  framebuffer_[row_offset + 16 + x] = b & 0x1F; // Blue plane
  
  // Blast entire frame
  setAll(framebuffer_.data(), framebuffer_.size());
}

void ATTiny88Driver::setAll(const uint8_t* rgb_data, size_t length) {
  size_t copy_len = std::min(length, framebuffer_.size());
  
  // Prepare buffer: register address (0x00) + 192 bytes of data
  uint8_t buffer[193];
  buffer[0] = 0x00;  // Start at register 0x00
  
  for (size_t i = 0; i < copy_len; i++) {
    buffer[i + 1] = rgb_data[i] & 0x1F;
  }
  
  // Wait for frame sync, then immediately write
  waitForFrameSync();
  i2c_.write(buffer, copy_len + 1);
  
  // Update local framebuffer
  std::memcpy(framebuffer_.data(), &buffer[1], copy_len);
}


