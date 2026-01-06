# Implementation Plan

## Architecture Overview

Component-based architecture with individual nodes for each Sense HAT function, plus a main application node that orchestrates them.

## Components (Nodes)

### 1. IMU Node
**Purpose:** Read and publish orientation data from accelerometer, gyroscope, and magnetometer

**Hardware:** LSM9DS1 via direct I2C (0x6A/0x6B accel/gyro, 0x1C/0x1E mag)

**Constraints:**
- **Polling-based only** - No interrupt pins connected to Pi GPIO
- **I2C register access only** - No reset or trigger pins available
- **Basic sensor configuration** - Limited to I2C register setup

**Publishers:**
- `/sense_hat/imu` (sensor_msgs/Imu) - orientation, angular velocity, linear acceleration

**Parameters:**
- `publish_rate` (int, default: 10 Hz) - Polling frequency
- `enable_magnetometer` (bool, default: true)

**Services:**
- `/sense_hat/imu/calibrate` (std_srvs/Trigger) - calibrate sensors

### 2. Environmental Sensor Node
**Purpose:** Read and publish temperature, humidity, and pressure data

**Publishers:**
- `/sense_hat/temperature/humidity_sensor` (sensor_msgs/Temperature) - from HTS221
- `/sense_hat/temperature/pressure_sensor` (sensor_msgs/Temperature) - from LPS25H
- `/sense_hat/humidity` (sensor_msgs/RelativeHumidity) - from HTS221
- `/sense_hat/pressure` (sensor_msgs/FluidPressure) - from LPS25H

**Parameters:**
- `publish_rate` (int, default: 1 Hz)
- `temperature_offset_hts221` (float, default: 0.0) - calibration offset for HTS221
- `temperature_offset_lps25h` (float, default: 0.0) - calibration offset for LPS25H

### 3. Color Sensor Node
**Purpose:** Read and publish RGB color and ambient light data

**Publishers:**
- `/sense_hat/color` (sensor_msgs/Illuminance) - ambient light level
- `/sense_hat/color/rgb` (std_msgs/ColorRGBA) - RGB color values

**Parameters:**
- `publish_rate` (int, default: 10 Hz)
- `integration_time` (int, default: 100) - sensor integration time in ms
- `gain` (int, default: 1) - sensor gain multiplier

### 4. Joystick Node
**Purpose:** Read and publish joystick button events

**Publishers:**
- `/sense_hat/joystick` (sensor_msgs/Joy) - button states and events

**Parameters:**
- `publish_rate` (int, default: 50 Hz)

### 5. LED Matrix Node
**Purpose:** Control the 8x8 RGB LED display

**Subscribers:**
- `/sense_hat/led_matrix/image` (sensor_msgs/Image) - display 8x8 image
- `/sense_hat/led_matrix/text` (std_msgs/String) - scroll text

**Services:**
- `/sense_hat/led_matrix/set_pixel` (custom) - set individual pixel
- `/sense_hat/led_matrix/clear` (std_srvs/Trigger) - clear display
- `/sense_hat/led_matrix/set_rotation` (custom) - rotate display

**Parameters:**
- `brightness` (int, default: 128, range: 0-255)
- `rotation` (int, default: 0, values: 0, 90, 180, 270)

### 6. Main Application Node
**Purpose:** Demonstrate integration - read sensors and display on LED matrix

**Subscribers:**
- `/sense_hat/temperature/humidity_sensor`
- `/sense_hat/joystick`

**Publishers:**
- `/sense_hat/led_matrix/image`

**Behavior:**
- Display temperature as color gradient on LED matrix
- React to joystick input to change display mode
- Example of coordinating multiple components

## Implementation Phases

### Phase 1: Project Setup
- [x] Create package structure (package.xml, CMakeLists.txt)
- [x] Define custom message types (if needed)
- [x] Define custom service types
- [x] Set up include/src directory structure

### Phase 2: Low-Level Hardware Drivers

**Reference:** See `docs/datasheets/` folder for detailed register maps and specifications.

**Hardware Architecture:** All sensors connect directly to Pi I2C bus. ATTINY88 only handles LED matrix and joystick.

- [x] **I2C Base Class** - raw I2C communication (`/dev/i2c-1`, ioctl)
  - Open/close I2C bus
  - Read/write registers
  - Multi-byte read/write operations
- [ ] **LSM9DS1 Driver** (IMU) - I2C 0x6A/0x6B (accel/gyro), 0x1C/0x1E (mag) - **Direct to Pi**
  - Reference: `docs/datasheets/ST-LSM9DS1.md`
  - Register configuration
  - Raw data reading
  - Calibration and conversion to physical units
  - **Note:** No interrupts available (IMU interrupt pins not connected to Pi GPIO)
- [ ] **HTS221 Driver** (Humidity/Temperature) - I2C 0x5F - **Direct to Pi**
  - Reference: `docs/datasheets/ST-HTS221.md`
  - Read calibration coefficients
  - Raw ADC to physical units conversion
- [ ] **LPS25H Driver** (Pressure) - I2C 0x5C - **Direct to Pi**
  - Reference: `docs/datasheets/ST-LPS25H.md`
  - Register setup
  - Pressure and temperature reading
- [ ] **TCS3400 Driver** (Color/Light Sensor) - I2C 0x39 - **Direct to Pi**
  - Reference: `docs/datasheets/AMS-TCS3400.md`
  - RGB color channel reading
  - Clear/ambient light reading
  - Integration time and gain configuration
- [x] **ATTINY88 Driver** - I2C 0x46 - **LED matrix and joystick only**
  - Reference: `docs/ATTINY88_PROTOCOL.md`
  - Device identification (register 0xF0)
  - LED matrix control (registers 0x00-0xBF, 192 bytes RGB)
  - Joystick reading (register 0xF2)
  - GPIO interrupt handling (GPIO23 for joystick, GPIO24 for frame sync)
  - RGB888 to RGB555 conversion (5-bit per channel)
  - Frame-synchronized bulk updates via libgpiod

### Phase 3: Component Nodes
- [ ] Implement IMU node
- [ ] Implement Environmental sensor node
- [ ] Implement Color sensor node
- [ ] Implement Joystick node (using ATTINY88 driver)
- [x] **Implement LED matrix node (using ATTINY88 driver)**
- [x] **Test LED matrix node independently**

### Phase 4: Main Application
- [ ] Implement main application node
- [ ] Create launch file to start all nodes
- [ ] Integration testing

### Phase 5: Documentation & Polish
- [ ] Add usage examples
- [ ] Document custom messages/services
- [ ] Add troubleshooting guide

## Technical Notes

### Hardware Access (Low-Level)

**I2C Communication:**
- Device: `/dev/i2c-1` (I2C bus 1 on Raspberry Pi)
- Use Linux I2C user-space API: `<linux/i2c-dev.h>`, `<i2c/smbus.h>`
- ioctl calls: `I2C_SLAVE`, `I2C_RDWR`
- Direct register read/write operations

**Sense HAT v2 I2C Devices:**
- **LSM9DS1 IMU:** - **Direct to Pi I2C**
  - Accel/Gyro: 0x6A or 0x6B
  - Magnetometer: 0x1C or 0x1E
  - 16-bit sensor data, configurable ranges and ODR
  - **No interrupts available** (interrupt pins not connected to Pi GPIO)
- **HTS221 Humidity/Temp:** 0x5F - **Direct to Pi I2C**
  - Requires calibration coefficient reading
  - 16-bit ADC values
- **LPS25H Pressure:** 0x5C - **Direct to Pi I2C**
  - 24-bit pressure data
  - 16-bit temperature data
- **TCS3400 Color/Light Sensor:** 0x39 - **Direct to Pi I2C**
  - RGB color channels + clear/ambient light
  - Configurable integration time and gain
- **ATTINY88:** 0x46 - **LED matrix and joystick only**
  - LED matrix control (registers 0x00-0xBF)
  - Joystick reading (register 0xF2)
  - Device ID (register 0xF0 = 0x73)
  - Interrupts on GPIO23 (joystick) and GPIO24 (frame sync)

**GPIO Access:**
- GPIO23: KEYS_INT from ATTINY88 (joystick button change interrupt)
- GPIO24: FRAME_INT from ATTINY88 (LED frame complete interrupt)
- Access via `/sys/class/gpio` or gpiod library for edge detection
- Optional: enables event-driven joystick and synchronized LED updates

**LED Matrix:**
- Direct I2C communication with ATTINY88 at address 0x46
- Write to registers 0x00-0xBF (192 bytes for 64 RGB pixels)
- Format: 5-bit per channel (0-31), row-major order
- Optional: Monitor GPIO24 for frame sync to prevent tearing

**Joystick:**
- Direct I2C communication with ATTINY88 at address 0x46
- Read register 0xF2 for 5-bit button state
- Optional: Monitor GPIO23 interrupt for event-driven reading
- Alternative: Use kernel input event driver at `/dev/input/eventX`

**Sensors (IMU, Environmental, Color):**
- **All sensors connect directly to Pi I2C bus** - no ATTINY88 involvement
- **No sensor interrupts available** - must use polling approach
- **No sensor GPIO pins** - no reset, trigger, or interrupt pins connected to Pi
- **I2C register access only** - limited to standard I2C read/write operations
- Standard I2C register-based communication
- Each sensor has its own I2C address on bus 1

**No External Libraries:**
- No RTIMULib - write our own I2C sensor drivers
- No sense-hat library
- Direct Linux kernel interfaces only
- Standard C++ and POSIX APIs
- Raw I2C register programming for all sensors
- Direct ATTINY88 communication (see docs/ATTINY88_PROTOCOL.md)

### Alternative Approaches

**Option 1: Direct ATTINY88 I2C (Current Plan)**
- Full control over LED and joystick
- Requires understanding ATTINY88 protocol
- More code but educational

**Option 2: Kernel Drivers**
- Use `/dev/fb1` framebuffer for LEDs (RGB565 format)
- Use `/dev/input/eventX` for joystick
- Less code, kernel handles ATTINY88 communication
- Still requires direct I2C for sensors

## Testing Strategy

1. **Unit tests** - hardware interface layer
2. **Node tests** - each component in isolation
3. **Integration tests** - main application with all components
4. **Hardware-in-loop** - on actual Raspberry Pi with Sense HAT v2

## References

- [Sense HAT v2 Schematics](datasheets/sense-hat-v2-schematics_page-0002.jpg) - Hardware schematic
- [ATTINY88 Firmware](https://github.com/raspberrypi/rpi-sense) - Official firmware source
- [Sensor Datasheets](datasheets/) - Complete register documentation
  - ST LSM9DS1 - 9-axis IMU
  - ST HTS221 - Humidity/Temperature
  - ST LPS25H - Pressure
  - AMS TCS3400 - Color/Light

### Dependencies
- `rclcpp` - ROS2 C++ client library
- `sensor_msgs` - standard sensor message types
- `std_msgs` - standard message types
- `std_srvs` - standard service types
- `libgpiod` - modern Linux GPIO interface
- Linux kernel headers: `<linux/i2c-dev.h>`, `<linux/i2c.h>`
- Standard POSIX: `<fcntl.h>`, `<unistd.h>`, `<sys/ioctl.h>`

### Build System
- CMake-based with colcon
- Component-based loading (rclcpp_components)
- Each node can run standalone or composed

## References

- [Sense HAT v2 Schematics](datasheets/sense-hat-v2-schematics_page-0002.jpg) - Hardware schematic
- [ATTINY88 Firmware](https://github.com/raspberrypi/rpi-sense) - Official firmware source
- [Sensor Datasheets](datasheets/) - Complete register documentation
  - ST LSM9DS1 - 9-axis IMU
  - ST HTS221 - Humidity/Temperature
  - ST LPS25H - Pressure
  - AMS TCS3400 - Color/Light
