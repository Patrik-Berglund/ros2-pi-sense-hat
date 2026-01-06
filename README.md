# ROS2 Pi Sense HAT

ROS2 package for interfacing with the Raspberry Pi Sense HAT on Ubuntu 24.04 and ROS2 Kilted.

## Hardware Requirements

- Raspberry Pi 4
- Sense HAT v2

## Software Requirements

- Ubuntu 24.04
- ROS2 Kilted
- C++17 or later
- Python 3.12+ (for demo scripts)
- Pillow (PIL) for image display: `pip3 install Pillow`

## Installation

```bash
# Ensure I2C is enabled and set to 400kHz for optimal performance
sudo raspi-config
# Enable I2C in Interface Options
# Add to /boot/firmware/config.txt:
# dtparam=i2c_arm_baudrate=400000
# Reboot if needed

# Disable kernel drivers to allow direct I2C access
sudo tee /etc/modprobe.d/blacklist-sense-hat.conf << EOF
# Blacklist ST sensor kernel drivers for direct ROS2 I2C access
# LSM9DS1 magnetometer
blacklist st_magn_i2c
blacklist st_magn_spi
blacklist st_magn

# LPS25H pressure sensor
blacklist st_pressure_i2c
blacklist st_pressure_spi
blacklist st_pressure

# ST sensor framework
blacklist st_sensors_i2c
blacklist st_sensors_spi
blacklist st_sensors
EOF
# Reboot to apply blacklist

# Verify I2C devices are detected
i2cdetect -y 1
# Should show devices at:
#   0x46 - ATTINY88 (LED matrix and joystick only)
#   0x1C - LSM9DS1 magnetometer (direct to Pi)
#   0x5C - LPS25H pressure sensor (direct to Pi)
#   0x5F - HTS221 humidity/temperature sensor (direct to Pi)
#   0x6A - LSM9DS1 accelerometer/gyroscope (direct to Pi)
#   0x29 - VL53L0X distance sensor (direct to Pi)

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

# Test individual pixels
./test_pixel.sh

# Test full image patterns
python3 test_pattern.py cross    # Red X pattern
python3 test_pattern.py red      # All red
python3 test_pattern.py green    # All green
python3 test_pattern.py blue     # All blue
python3 test_pattern.py white    # All white

# Animated demos
python3 demo_patterns.py rainbow    # Colorful spiral animation
python3 demo_patterns.py heart      # Pulsing pink heart
python3 demo_patterns.py fire       # Fire effect animation
python3 demo_patterns.py matrix     # Matrix-style falling code

# Fast pixel testing (Python ROS2 client)
python3 test_pixels_fast.py         # Fast corner + center pixels

# Fun demos
python3 pixel_demo_loop.py          # Spinning wheel, bouncing ball, sparkles
python3 countdown_demo.py           # Countdown from 9-0 with colored digits
python3 emoji_demo.py               # Cycling colorful emojis

# Display image files
pip3 install Pillow                 # Install PIL first
python3 image_display.py image.png  # Display any image file (resized to 8x8)
```

## Features - LED Matrix (COMPLETED)

- ✅ Direct I2C register-level ATTINY88 access (no external libraries)
- ✅ Control 8x8 RGB LED matrix via direct ATTINY88 I2C
- ✅ Frame-synchronized updates using GPIO24 (FRAME_INT) via libgpiod
- ✅ Bulk I2C writes (193 bytes per frame) for optimal performance
- ✅ Full image pattern support via ROS2 Image messages
- ✅ Component-based ROS2 architecture
- ✅ Optimized I2C performance (400kHz)
- ✅ Fast Python ROS2 clients for interactive demos
- ✅ Image file display support (PNG, JPG, etc.)

### LED Matrix Services

- `/sense_hat/led_matrix/clear` - Clear display (immediate update)
- `/sense_hat/led_matrix/set_pixel` - Set individual pixel (immediate update)

### LED Matrix Topics

- `/sense_hat/led_matrix/image` - Full 8x8 RGB8 image updates (immediate update)

### Demo Scripts

- `test_pixels_fast.py` - Fast pixel testing using Python ROS2 client
- `demo_patterns.py` - Animated patterns (rainbow, heart, fire, matrix)
- `pixel_demo_loop.py` - Interactive demos (spinning wheel, bouncing ball, sparkles)
- `countdown_demo.py` - Countdown from 9-0 with colored digits
- `emoji_demo.py` - Colorful emoji display (smiley, heart, star, fire, rainbow, sun)
- `image_display.py` - Display any image file on the LED matrix

## Features - IMU (PARTIAL - Accelerometer/Gyroscope Only)

- ✅ Direct I2C LSM9DS1 accelerometer and gyroscope access
- ✅ Optimized 14-byte burst read for gyro + temperature + accel data
- ✅ ROS2 standard sensor_msgs/Imu and sensor_msgs/Temperature messages
- ✅ Configurable full-scale ranges (accel: 2/4/8/16g, gyro: 245/500/2000 dps)
- ❌ **Magnetometer not responding** (hardware issue on this specific Sense HAT)

### IMU Topics

- `/sense_hat/imu/data_raw` - Raw IMU data (accel + gyro, no magnetometer)
- `/sense_hat/temperature/imu` - Temperature from IMU sensor

### IMU Services

- `/sense_hat/imu/calibrate` - Calibrate IMU (placeholder for future implementation)

**Note**: Magnetometer portion of LSM9DS1 not responding at expected I2C addresses (0x1C/0x1E). Accelerometer and gyroscope working correctly at 0x6A.

## Features - Sensors (TODO)

- Read IMU data (accelerometer, gyroscope, magnetometer) - **Direct I2C, polling-based**
- Read environmental data (temperature, humidity, pressure) - **Direct I2C**
- Read color and ambient light sensor - **Direct I2C**

**Note**: All sensors connect directly to Pi I2C bus. ATTINY88 only handles LED matrix and joystick.

**Sensor I/O Constraint**: No sensor GPIO pins (interrupts, reset, etc.) are connected to Pi - I2C access only.

## Documentation

- [Implementation Plan](specs/IMPLEMENTATION_PLAN.md) - Detailed development roadmap
- [ATTINY88 Protocol](docs/ATTINY88_PROTOCOL.md) - LED matrix and joystick I2C protocol
- [Kernel Driver Disable](docs/KERNEL_DRIVER_DISABLE.md) - How to disable/enable kernel drivers
- [Datasheets](docs/datasheets/) - Sensor datasheets and schematics
  - LSM9DS1 (IMU)
  - HTS221 (Humidity/Temperature)
  - LPS25H (Pressure)
  - TCS3400 (Color/Light)
  - Sense HAT v2 schematics

## License

TBD
