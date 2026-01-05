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
# Ensure I2C is enabled
sudo raspi-config
# Enable I2C in Interface Options
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
colcon build --packages-select ros2-pi-sense-hat
source install/setup.bash
```

## Usage

```bash
# Launch the Sense HAT node
ros2 run ros2-pi-sense-hat sense_hat_node
```

## Features

- Direct I2C register-level sensor access (no external libraries)
- Read IMU data (accelerometer, gyroscope, magnetometer)
- Read environmental data (temperature, humidity, pressure)
- Read color and ambient light sensor
- Control 8x8 RGB LED matrix via direct ATTINY88 I2C
- Read 5-way joystick input via ATTINY88 I2C
- Component-based ROS2 architecture
- Configurable via ROS2 parameters

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
