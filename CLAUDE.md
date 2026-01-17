# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 package for Raspberry Pi Sense HAT v2 on Ubuntu 24.04 with ROS2 Kilted.
All production code is C++17. Python is only for demos and calibration scripts.

## Build Commands

```bash
# Source ROS2 and build
source /opt/ros/kilted/setup.bash
colcon build --packages-select ros2_pi_sense_hat
source install/setup.bash

# Build with verbose output
colcon build --packages-select ros2_pi_sense_hat --event-handlers console_direct+

# Clean build
colcon build --packages-select ros2_pi_sense_hat --cmake-clean-cache

# Run tests
colcon test --packages-select ros2_pi_sense_hat
colcon test-result --verbose
```

## Running Nodes

```bash
# Launch all sensor nodes
ros2 launch ros2_pi_sense_hat sense_hat.launch.py

# Run individual nodes
ros2 run ros2_pi_sense_hat led_matrix_node
ros2 run ros2_pi_sense_hat joystick_node
ros2 run ros2_pi_sense_hat imu_node
ros2 run ros2_pi_sense_hat environmental_node
ros2 run ros2_pi_sense_hat color_node
```

## Debugging ROS2

```bash
# IMPORTANT: Use timeout for commands that don't exit
timeout 10 ros2 topic echo /topic_name
timeout 5 ros2 topic echo /topic_name --once

ros2 topic list
ros2 topic info /topic_name
ros2 node list
ros2 node info /node_name
ros2 param describe /node_name param_name
```

## Architecture

### Driver Layer Pattern
All hardware access uses direct I2C register-level communication (no kernel drivers):
- `I2CDevice` (`i2c_device.hpp/cpp`) - Base I2C operations, used by all drivers
- Sensor drivers inherit I2C functionality and implement sensor-specific register protocols:
  - `ATTINY88Driver` - LED matrix (8x8 RGB) and joystick via GPIO24 frame sync
  - `LSM9DS1Driver` - 9-DOF IMU (accel/gyro at 0x6A, mag at 0x1C)
  - `HTS221Driver` - Humidity/temperature sensor
  - `LPS25HDriver` - Pressure sensor
  - `TCS3400Driver` - Color/light sensor (actually TCS34725 at 0x29)

### Node Pattern
Each node follows the same structure:
1. Declare parameters with `ParameterDescriptor` for validation and self-documentation
2. Initialize driver with I2C bus and address
3. Create publishers for sensor data using standard `sensor_msgs` types
4. Timer-based publishing at configurable rate
5. Graceful shutdown via signal handler

### Sensor Fusion Pipeline
```
LSM9DS1 → IMU Node (calibration) → Madgwick AHRS → EKF (robot_localization) → /odometry/filtered
```

Key files:
- `imu_calibration.hpp/cpp` - Runtime calibration coefficient application
- `scripts/calibrate_imu.py` - Interactive calibration wizard
- `config/madgwick.yaml`, `config/ekf_minimal.yaml` - Fusion parameters

### Custom Services
Defined in `srv/`:
- `SetPixel.srv` - Set individual LED matrix pixel
- `ReadJoystick.srv` - Read joystick state

## Development Environment

**Edit**: WSL (Windows Subsystem for Linux)
**Build/Test**: Raspberry Pi 4 via SSH (Ubuntu 24.04, no GUI)
**Hardware**: Sense HAT v2 with I2C at 400kHz

Kernel drivers must be blacklisted for direct I2C access (see README.md installation).

## Code Style

**ALL production code MUST be C++. Python is ONLY for demos and test scripts.**

Follow [ROS2 Kilted style guide](https://docs.ros.org/en/kilted/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html) with [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) as base.

### C++ Rules (C++17)
- **Line length**: 100 characters max
- **Files**: `.hpp` headers, `.cpp` implementation
- **Indentation**: 2 spaces, no tabs
- **Braces**: Same line for functions/classes/control structures, always use braces even for single statements
- **Pointer syntax**: `char * c` (spaces around `*`) - handles `char * c, * d, * e` correctly
- **Naming**:
  - `CamelCase` for classes/structs
  - `snake_case` for functions/methods (ROS2 convention, differs from Google)
  - `snake_case` for variables
  - `g_snake_case` for globals
  - `member_` trailing underscore for class members
- **Comments**: `///` and `/** */` for documentation (Doxygen), `//` for code comments
- **Exceptions**: Allowed in user-facing APIs, avoid in destructors and C-wrapped code
- **No Boost** unless absolutely required

```cpp
class SensorDriver {
public:
  SensorDriver();

  /// Read sensor data from device
  bool read_data(uint8_t * buffer, size_t length);

private:
  int fd_;
  uint8_t address_;
};

void process_data() {
  if (condition) {
    do_something();
  } else {
    do_other_thing();
  }
}
```

### Architectural Guidelines

**Class Design:**
- Favor composition over inheritance (use inheritance only for true "is-a" relationships)
- Single responsibility - classes should have clear purposes with well-defined invariants
- Make data members `private` unless constants; use accessors when needed
- Mark methods `const` when they don't modify state
- Use `explicit` on single-argument constructors to prevent implicit conversions

**Copy/Move Semantics:**
- Explicitly declare or delete copy/move operations - never leave implicit
- Copyable: declare copy constructor and assignment
- Move-only: declare move ops, delete copy ops
- Non-copyable: delete both

**Ownership & Memory:**
- Prefer single, fixed owners for dynamically allocated objects
- Use `std::unique_ptr` for exclusive ownership
- Use `std::shared_ptr` for shared ownership (sparingly)
- RAII for all resource management - no manual cleanup

**Inheritance:**
- All inheritance must be `public`
- Always use `override` or `final` on virtual function overrides
- Avoid multiple implementation inheritance
- Never call virtual functions in constructors/destructors

**Headers:**
- Include order: related header, C system, C++ stdlib, third-party, project headers
- Separate groups with blank lines
- Avoid forward declarations - include what you need
- Inline functions only when <10 lines

**Namespaces:**
- Use namespaces with unique project-based names
- Never use `using namespace` directives
- Use `internal` namespace for implementation details

**Error Handling:**
- Exceptions allowed in user-facing APIs (ROS2 convention)
- Never throw from destructors
- Consider avoiding exceptions in C-wrapped APIs

### Python Rules (demos/tests only)
- Python 3, PEP8 style
- 100 character line length
- Single quotes preferred

### Linting
```bash
# Fix C++ formatting
ament_clang_format --reformat src/ include/
ament_uncrustify --reformat src/ include/

# Check style
colcon test --packages-select ros2_pi_sense_hat
colcon test-result --verbose
```

## Development Workflow

1. **Specification**: Create spec in `specs/` folder
2. **Investigation**: Read existing code patterns, verify dependencies
3. **Planning**: Draft implementation plan in `specs/`
4. **Review**: Iterate until plan is solid
5. **Implementation**: Write code

When planning, use web search for ROS2 APIs, hardware specs, or unfamiliar concepts.
Never assume function signatures, message types, or register addresses - verify first.

## Communication Style

Be direct about technical issues. Evaluate proposals on technical merit.
Skip flattery. State problems clearly and steer toward better solutions.
User is learning ROS2/C++ - include rationale for technical decisions.
