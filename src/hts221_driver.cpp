#include "ros2_pi_sense_hat/hts221_driver.hpp"
#include <unistd.h>

static const uint8_t HTS221_ADDR = 0x5F;
static const uint8_t WHO_AM_I = 0x0F;
static const uint8_t WHO_AM_I_VAL = 0xBC;
static const uint8_t AV_CONF = 0x10;
static const uint8_t CTRL_REG1 = 0x20;
static const uint8_t CTRL_REG2 = 0x21;
static const uint8_t STATUS_REG = 0x27;
static const uint8_t HUMIDITY_OUT_L = 0x28;
static const uint8_t TEMP_OUT_L = 0x2A;

HTS221Driver::HTS221Driver() 
  : device_("/dev/i2c-1", HTS221_ADDR) {}

HTS221Driver::~HTS221Driver() {
  device_.close();
}

bool HTS221Driver::init() {
  if (!device_.open()) return false;

  uint8_t who_am_i;
  if (!device_.readReg(WHO_AM_I, who_am_i) || who_am_i != WHO_AM_I_VAL) return false;

  if (!read_calibration()) return false;

  // Power on, BDU enabled, 1 Hz ODR
  device_.writeReg(CTRL_REG1, 0x85);
  usleep(10000);

  return true;
}

bool HTS221Driver::read_calibration() {
  uint8_t h0_rh_x2, h1_rh_x2;
  uint8_t t0_degc_x8, t1_degc_x8, t1_t0_msb;
  uint8_t buf[2];

  device_.readReg(0x30, h0_rh_x2);
  device_.readReg(0x31, h1_rh_x2);
  device_.readReg(0x32, t0_degc_x8);
  device_.readReg(0x33, t1_degc_x8);
  device_.readReg(0x35, t1_t0_msb);

  device_.readMultiReg(0x36, buf, 2);
  h0_t0_out_ = static_cast<int16_t>(buf[0] | (buf[1] << 8));
  
  device_.readMultiReg(0x3A, buf, 2);
  h1_t0_out_ = static_cast<int16_t>(buf[0] | (buf[1] << 8));
  
  device_.readMultiReg(0x3C, buf, 2);
  t0_out_ = static_cast<int16_t>(buf[0] | (buf[1] << 8));
  
  device_.readMultiReg(0x3E, buf, 2);
  t1_out_ = static_cast<int16_t>(buf[0] | (buf[1] << 8));

  h0_rh_ = h0_rh_x2 / 2.0f;
  h1_rh_ = h1_rh_x2 / 2.0f;
  
  uint16_t t0_degc_x8_full = ((t1_t0_msb & 0x03) << 8) | t0_degc_x8;
  uint16_t t1_degc_x8_full = ((t1_t0_msb & 0x0C) << 6) | t1_degc_x8;
  t0_degc_ = t0_degc_x8_full / 8.0f;
  t1_degc_ = t1_degc_x8_full / 8.0f;

  return true;
}

bool HTS221Driver::read_humidity(float& humidity) {
  uint8_t buf[2];
  if (!device_.readMultiReg(HUMIDITY_OUT_L, buf, 2)) return false;
  
  int16_t h_out = static_cast<int16_t>(buf[0] | (buf[1] << 8));
  humidity = h0_rh_ + (h_out - h0_t0_out_) * (h1_rh_ - h0_rh_) / (h1_t0_out_ - h0_t0_out_);
  
  if (humidity < 0.0f) humidity = 0.0f;
  if (humidity > 100.0f) humidity = 100.0f;
  
  return true;
}

bool HTS221Driver::read_temperature(float& temperature) {
  uint8_t buf[2];
  if (!device_.readMultiReg(TEMP_OUT_L, buf, 2)) return false;
  
  int16_t t_out = static_cast<int16_t>(buf[0] | (buf[1] << 8));
  temperature = t0_degc_ + (t_out - t0_out_) * (t1_degc_ - t0_degc_) / (t1_out_ - t0_out_);
  
  return true;
}

void HTS221Driver::set_odr(uint8_t odr) {
  uint8_t ctrl;
  device_.readReg(CTRL_REG1, ctrl);
  ctrl = (ctrl & 0xFC) | (odr & 0x03);
  device_.writeReg(CTRL_REG1, ctrl);
}

void HTS221Driver::set_avg_samples(uint8_t temp_avg, uint8_t hum_avg) {
  uint8_t val = ((temp_avg & 0x07) << 3) | (hum_avg & 0x07);
  device_.writeReg(AV_CONF, val);
}
