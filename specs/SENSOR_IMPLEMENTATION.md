# Sensor Implementation Summary

## Overview

Implemented complete sensor support for Raspberry Pi Sense HAT v2, including environmental sensors (temperature, humidity, pressure) and color/light sensor. All sensors use direct I2C register access with no external libraries.

**Latest Updates (2026-01-13):**
- ✅ Added software reset on initialization for all sensors
- ✅ Separated configuration from enable for clean initialization
- ✅ Added comprehensive parameter validation with descriptors
- ✅ Added signal handlers for graceful shutdown
- ✅ Added HTS221 heater control service
- ✅ Added LPS25H FIFO mean mode and averaging parameters
- ✅ Added TCS3400 wait time for power saving
- ✅ All parameters now self-documenting with range validation

## Implemented Components

### 1. HTS221 Humidity/Temperature Sensor

**Driver:** `hts221_driver.cpp` / `hts221_driver.hpp`

**Features:**
- Factory calibration coefficient reading and application
- Linear interpolation for humidity and temperature conversion
- Configurable output data rate (1-12.5 Hz)
- Configurable averaging samples for noise reduction
- I2C address: 0x5F

**Key Implementation Details:**
- Reads 16 calibration coefficients from registers 0x30-0x3F
- Applies two-point linear interpolation for both humidity and temperature
- Humidity clamped to 0-100% range
- Default: 1 Hz ODR, BDU enabled

**Node:** `environmental_node.cpp`

**Parameters (with validation):**
- `temperature_offset_hts221` (-50 to +50°C) - Calibration offset
- `hts221_odr` (0-3) - Output data rate: 0=one-shot, 1=1Hz, 2=7Hz, 3=12.5Hz
- `hts221_temp_avg` (0-7) - Temperature averaging samples
- `hts221_hum_avg` (0-7) - Humidity averaging samples

**Services:**
- `/sense_hat/set_heater` (std_srvs/SetBool) - Enable/disable built-in heater

### 2. LPS25H Pressure Sensor

**Driver:** `lps25h_driver.cpp` / `lps25h_driver.hpp`

**Features:**
- 24-bit pressure measurement (260-1260 hPa)
- 16-bit temperature measurement
- 0.01 hPa resolution
- Configurable output data rate (1-25 Hz)
- FIFO mean mode for temporal averaging
- Configurable averaging for both pressure and temperature
- I2C address: 0x5C

**Key Implementation Details:**
- Pressure conversion: raw / 4096.0 → hPa
- Temperature conversion: 42.5 + (raw / 480.0) → °C
- Sign extension for 24-bit pressure value
- Default: 1 Hz ODR, BDU enabled

**Node:** `environmental_node.cpp`

**Parameters (with validation):**
- `temperature_offset_lps25h` (-50 to +50°C) - Calibration offset
- `lps25h_odr` (0-4) - Output data rate: 0=one-shot, 1=1Hz, 2=7Hz, 3=12.5Hz, 4=25Hz
- `lps25h_press_avg` (0-3) - Pressure averaging: 0=8, 1=32, 2=128, 3=512 samples
- `lps25h_temp_avg` (0-3) - Temperature averaging: 0=8, 1=16, 2=32, 3=64 samples
- `lps25h_fifo_mean` (bool) - Enable FIFO mean mode for temporal averaging
- `lps25h_fifo_samples` (2-32) - Number of samples to average in FIFO

### 3. TCS3400 Color/Light Sensor

**Driver:** `tcs3400_driver.cpp` / `tcs3400_driver.hpp`

**Features:**
- RGBC (Red, Green, Blue, Clear) channel reading
- IR channel for light source detection
- Configurable integration time (2.78-712 ms)
- Configurable gain (1x, 4x, 16x, 64x)
- I2C address: 0x39

**Key Implementation Details:**
- Device ID verification (0x90-0x93 range)
- Power-on sequence with delays
- Separate IR channel access via register 0xC0
- Normalized RGB output (R/C, G/C, B/C)
- Default: 27.8 ms integration, 16x gain

**Node:** `color_node.cpp`

**Parameters (with validation):**
- `integration_time` (0-255) - ATIME register value (lower = longer integration)
- `gain` (0-3) - Gain setting: 0=1x, 1=4x, 2=16x, 3=64x
- `lux_calibration` (0.1-10.0) - Lux calculation multiplier
- `wait_enable` (bool) - Enable wait time for power saving
- `wait_time` (0-255) - WTIME register value (lower = longer wait)
- `wait_long` (bool) - 12x multiplier for wait time (up to 8.54s)

## ROS2 Integration

### Environmental Node

**Published Topics:**
- `/sense_hat/temperature/humidity_sensor` (sensor_msgs/Temperature)
- `/sense_hat/temperature/pressure_sensor` (sensor_msgs/Temperature)
- `/sense_hat/humidity` (sensor_msgs/RelativeHumidity)
- `/sense_hat/pressure` (sensor_msgs/FluidPressure)

**Parameters:**
- `publish_rate` - Publishing frequency in Hz (default: 1)
- `temperature_offset_hts221` - HTS221 temperature offset
- `temperature_offset_lps25h` - LPS25H temperature offset
- `hts221_odr` - HTS221 output data rate
- `lps25h_odr` - LPS25H output data rate

### Color Node

**Published Topics:**
- `/sense_hat/color/illuminance` (sensor_msgs/Illuminance)
- `/sense_hat/color/rgb` (std_msgs/ColorRGBA)

**Parameters:**
- `publish_rate` - Publishing frequency in Hz (default: 10)
- `integration_time` - Integration time register value (default: 0xF6)
- `gain` - Gain setting 0-3 (default: 2)

## Testing

### Demo Scripts

**test_environmental.py:**
- Subscribes to all environmental sensor topics
- Displays temperature, humidity, and pressure readings
- Usage: `python3 demo/test_environmental.py`

**test_color.py:**
- Subscribes to color sensor topics
- Displays illuminance and normalized RGB values
- Usage: `python3 demo/test_color.py`

### Launch File

**sense_hat.launch.py:**
- Launches all sensor nodes simultaneously
- Includes LED matrix, joystick, IMU, environmental, and color nodes
- Configurable parameters for each node
- Usage: `ros2 launch ros2_pi_sense_hat sense_hat.launch.py`

## Technical Notes

### I2C Communication

All sensors use polling-based I2C communication:
- No interrupt pins connected to Raspberry Pi GPIO
- Sensors polled at configured rates
- Block Data Update (BDU) enabled where available
- Multi-byte reads use atomic I2C transactions

### Calibration

**HTS221:**
- Factory calibrated - coefficients stored in non-volatile memory
- Two-point linear interpolation for both sensors
- No user calibration required

**LPS25H:**
- Factory calibrated - no user calibration required
- Optional temperature offset parameter for fine-tuning

**TCS3400:**
- No calibration required
- Normalized RGB output compensates for illuminance

### Performance

**Environmental Sensors:**
- Default 1 Hz update rate (configurable)
- Low power consumption
- Suitable for ambient monitoring

**Color Sensor:**
- Default 10 Hz update rate (configurable)
- Integration time affects sensitivity and speed
- Gain setting affects dynamic range

## Build System

### CMakeLists.txt Updates

Added:
- `environmental_node` executable
- `color_node` executable
- Driver source files for HTS221, LPS25H, TCS3400
- Launch file installation

### Dependencies

No additional dependencies required beyond existing:
- rclcpp
- sensor_msgs
- std_msgs
- std_srvs

## Code Quality

**Minimal Implementation:**
- Only essential functionality implemented
- No unnecessary features or verbose code
- Direct register access without abstraction layers
- Efficient I2C communication

**ROS2 Best Practices:**
- Standard message types used
- Configurable parameters
- Proper frame_id and timestamps
- Clean node lifecycle

**Error Handling:**
- Device ID verification on initialization
- I2C transaction error checking
- Graceful failure reporting

## Future Enhancements (Not Implemented)

The following were intentionally not implemented to keep code minimal:

- Interrupt-based data ready detection (no GPIO pins available)
- FIFO buffer usage (not needed for current rates)
- Advanced filtering (handled by ROS2 ecosystem)
- Dynamic reconfiguration (use parameters instead)
- Service interfaces (not required for sensor reading)

## Files Created

**Headers:**
- `include/ros2_pi_sense_hat/hts221_driver.hpp`
- `include/ros2_pi_sense_hat/lps25h_driver.hpp`
- `include/ros2_pi_sense_hat/tcs3400_driver.hpp`

**Implementation:**
- `src/hts221_driver.cpp`
- `src/lps25h_driver.cpp`
- `src/tcs3400_driver.cpp`
- `src/environmental_node.cpp`
- `src/color_node.cpp`

**Demo Scripts:**
- `demo/test_environmental.py`
- `demo/test_color.py`

**Launch Files:**
- `launch/sense_hat.launch.py`

**Documentation:**
- This summary document

## Verification

To verify the implementation:

1. Build the package:
   ```bash
   colcon build --packages-select ros2_pi_sense_hat
   source install/setup.bash
   ```

2. Check I2C devices are detected:
   ```bash
   i2cdetect -y 1
   # Should show: 0x39 (TCS3400), 0x5C (LPS25H), 0x5F (HTS221)
   ```

3. Run individual nodes:
   ```bash
   ros2 run ros2_pi_sense_hat environmental_node
   ros2 run ros2_pi_sense_hat color_node
   ```

4. Test with demo scripts:
   ```bash
   python3 demo/test_environmental.py
   python3 demo/test_color.py
   ```

5. Launch all nodes:
   ```bash
   ros2 launch ros2_pi_sense_hat sense_hat.launch.py
   ```
