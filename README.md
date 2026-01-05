# ROS2 Pi Sense HAT

ROS2 package for interfacing with the Raspberry Pi Sense HAT on Ubuntu 24.04 and ROS2 Kilted.

## Hardware Requirements

- Raspberry Pi 4
- Sense HAT v2

## Software Requirements

- Ubuntu 24.04
- ROS2 Kilted
- C++17 or later

## Installation

```bash
# Ensure I2C is enabled and set to 400kHz for optimal performance
sudo raspi-config
# Enable I2C in Interface Options
# Add to /boot/firmware/config.txt:
# dtparam=i2c_arm_baudrate=400000
# Reboot if needed

# Verify I2C devices are detected
i2cdetect -y 1
# Should show devices at:
#   0x46 - ATTINY88 (LED matrix and joystick)
#   0x1C/0x1E - LSM9DS1 magnetometer
#   0x5C - LPS25H pressure sensor
#   0x5F - HTS221 humidity/temperature sensor
#   0x6A/0x6B - LSM9DS1 accelerometer/gyroscope

# Clone this repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone <repository-url> ros2-pi-sense-hat

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select ros2_pi_sense_hat
source install/setup.bash
```

## Usage

```bash
# Launch the LED matrix node
./run_node.sh

# Test individual pixels (batched updates)
./test_pixel.sh

# Test full image patterns
python3 test_pattern.py cross    # Red X pattern
python3 test_pattern.py red      # All red
python3 test_pattern.py green    # All green
python3 test_pattern.py blue     # All blue
python3 test_pattern.py white    # All white
```

## Features - LED Matrix (COMPLETED)

- ✅ Direct I2C register-level ATTINY88 access (no external libraries)
- ✅ Control 8x8 RGB LED matrix via direct ATTINY88 I2C
- ✅ Service-based pixel control for efficient batched updates
- ✅ Full image pattern support via ROS2 Image messages
- ✅ Component-based ROS2 architecture
- ✅ Optimized I2C performance (400kHz)

### LED Matrix Services

- `/sense_hat/led_matrix/clear` - Clear display
- `/sense_hat/led_matrix/set_pixel` - Set individual pixel (immediate update)

### LED Matrix Topics

- `/sense_hat/led_matrix/image` - Full 8x8 RGB8 image updates

## Features - Sensors (TODO)

- Read IMU data (accelerometer, gyroscope, magnetometer)
- Read environmental data (temperature, humidity, pressure)
- Read color and ambient light sensor
- Read 5-way joystick input via ATTINY88 I2C

## Documentation

- [Implementation Plan](IMPLEMENTATION_PLAN.md) - Detailed development roadmap
- [ATTINY88 Protocol](ATTINY88_PROTOCOL.md) - LED matrix and joystick I2C protocol
- [Kernel Driver Disable](KERNEL_DRIVER_DISABLE.md) - How to disable/enable kernel drivers
- [Datasheets](datasheets/) - Sensor datasheets and schematics
  - LSM9DS1 (IMU)
  - HTS221 (Humidity/Temperature)
  - LPS25H (Pressure)
  - TCS3400 (Color/Light)
  - Sense HAT v2 schematics

## License

TBD
