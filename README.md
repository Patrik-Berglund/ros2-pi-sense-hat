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

## Features - Sensors (TODO)

- Read IMU data (accelerometer, gyroscope, magnetometer)
- Read environmental data (temperature, humidity, pressure)
- Read color and ambient light sensor
- Read 5-way joystick input via ATTINY88 I2C (register 0xF2)
- Event-driven joystick via GPIO23 (KEYS_INT) interrupt

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
