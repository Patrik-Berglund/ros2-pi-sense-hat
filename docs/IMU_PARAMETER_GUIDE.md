# LSM9DS1 IMU Parameter Configuration Guide

## Overview

The LSM9DS1 IMU driver now supports comprehensive parameter configuration for educational and application-specific tuning. This guide explains the available parameters and their effects.

## Key Hardware Constraints

### Critical Constraint: Shared ODR
**When both accelerometer and gyroscope are active, they MUST use the same Output Data Rate (ODR).** This is controlled by `CTRL_REG1_G` and is a hardware limitation of the LSM9DS1.

- ✅ **Supported**: Both sensors active at same ODR
- ❌ **Not supported**: Different ODRs for accel/gyro
- ❌ **Not supported**: Accelerometer-only mode

### Independent Magnetometer
The magnetometer is a separate I2C device and operates completely independently from the accelerometer/gyroscope.

## Parameter Categories

### 1. IMU Parameters (Accelerometer + Gyroscope)

#### `imu_odr` (Hz)
**Shared output data rate for both accelerometer and gyroscope**
- Valid values: `0, 14, 15, 59, 60, 119, 238, 476, 952`
- Default: `119`
- `0` = Power-down mode
- Higher ODR = More responsive but higher power consumption

#### `accel_range` (g)
**Accelerometer full-scale range**
- Valid values: `2, 4, 8, 16`
- Default: `2`
- Trade-off: Larger range = Lower resolution but can measure higher accelerations

#### `gyro_range` (dps)
**Gyroscope full-scale range**
- Valid values: `245, 500, 2000`
- Default: `245`
- Trade-off: Larger range = Lower resolution but can measure faster rotations

#### `accel_bandwidth_auto` (boolean)
**Use automatic bandwidth selection based on ODR**
- Default: `true`
- `true` = Bandwidth determined by ODR (recommended)
- `false` = Use manual `accel_bandwidth` setting

#### `accel_bandwidth` (0-3)
**Manual accelerometer bandwidth selection (when auto=false)**
- Valid values: `0, 1, 2, 3`
- Default: `1`
- `0` = 408 Hz (maximum bandwidth, most noise)
- `1` = 211 Hz (good balance)
- `2` = 105 Hz (good noise reduction)
- `3` = 50 Hz (maximum noise reduction, slower response)

### 2. Magnetometer Parameters

#### `mag_odr` (Hz)
**Magnetometer output data rate**
- Valid range: `1-80` (fractional values supported)
- Default: `10`
- Common values: `1, 2, 5, 10, 20, 40, 80`

#### `mag_range` (gauss)
**Magnetometer full-scale range**
- Valid values: `4, 8, 12, 16`
- Default: `4`
- Earth's magnetic field ≈ 0.5 gauss, so 4 gauss is usually sufficient

#### `mag_performance_mode` (0-3)
**Magnetometer performance vs power trade-off**
- Valid values: `0, 1, 2, 3`
- Default: `2`
- `0` = Low-power mode
- `1` = Medium-performance mode
- `2` = High-performance mode (recommended)
- `3` = Ultra-high performance mode

#### `mag_temp_compensation` (boolean)
**Enable temperature compensation for magnetometer**
- Default: `true`
- Improves accuracy across temperature variations

### 3. Standard ROS2 Parameters

#### `publish_rate` (Hz)
**ROS2 message publishing frequency**
- Default: `10`
- Should be ≤ `imu_odr` for best results

#### `frame_id` (string)
**TF frame for sensor data**
- Default: `"imu_link"`

#### `enable_magnetometer` (boolean)
**Enable/disable magnetometer**
- Default: `true`

## Usage Examples

### Basic Usage
```bash
# Use default parameters
ros2 run ros2_pi_sense_hat imu_node

# Use parameter file
ros2 launch ros2_pi_sense_hat imu_configured.launch.py
```

### Custom Parameters
```bash
# High-speed motion capture
ros2 run ros2_pi_sense_hat imu_node --ros-args \
  -p imu_odr:=952 \
  -p accel_range:=16 \
  -p gyro_range:=2000 \
  -p publish_rate:=100

# Low-power operation
ros2 run ros2_pi_sense_hat imu_node --ros-args \
  -p imu_odr:=59 \
  -p accel_range:=2 \
  -p gyro_range:=245 \
  -p mag_odr:=5 \
  -p mag_performance_mode:=0 \
  -p publish_rate:=5

# Precision measurement with noise reduction
ros2 run ros2_pi_sense_hat imu_node --ros-args \
  -p imu_odr:=119 \
  -p accel_bandwidth_auto:=false \
  -p accel_bandwidth:=3 \
  -p mag_performance_mode:=3
```

### Parameter File Configuration
Edit `config/imu_params.yaml` and use:
```bash
ros2 launch ros2_pi_sense_hat imu_configured.launch.py config_file:=path/to/your/params.yaml
```

## Learning Exercises

### 1. ODR Constraint Exploration
Try different ODR values and observe the effect on data rates:
```bash
# Test different ODRs
ros2 run ros2_pi_sense_hat imu_node --ros-args -p imu_odr:=14
ros2 run ros2_pi_sense_hat imu_node --ros-args -p imu_odr:=952
```

### 2. Bandwidth vs Noise Trade-off
Compare noise levels with different bandwidth settings:
```bash
# Maximum bandwidth (more noise)
ros2 run ros2_pi_sense_hat imu_node --ros-args \
  -p accel_bandwidth_auto:=false -p accel_bandwidth:=0

# Minimum bandwidth (less noise, slower response)
ros2 run ros2_pi_sense_hat imu_node --ros-args \
  -p accel_bandwidth_auto:=false -p accel_bandwidth:=3
```

### 3. Range vs Resolution
Test different measurement ranges:
```bash
# High resolution, low range
ros2 run ros2_pi_sense_hat imu_node --ros-args \
  -p accel_range:=2 -p gyro_range:=245

# Low resolution, high range  
ros2 run ros2_pi_sense_hat imu_node --ros-args \
  -p accel_range:=16 -p gyro_range:=2000
```

## Troubleshooting

### Invalid Parameter Errors
The node validates all parameters at startup. Common errors:
- `Invalid imu_odr`: Use only supported ODR values
- `Invalid accel_range`: Must be 2, 4, 8, or 16
- `Invalid gyro_range`: Must be 245, 500, or 2000

### Magnetometer Issues
If magnetometer fails to initialize:
- Check I2C connections
- Verify device address (should be 0x1C)
- Set `enable_magnetometer:=false` to continue without it

### Performance Issues
- High ODR + high publish rate = high CPU usage
- Consider reducing `publish_rate` if ODR is high
- Use power-down mode (`imu_odr:=0`) when not needed

## Technical Details

### Register Configuration
The driver handles complex register dependencies:
1. `CTRL_REG1_G` is configured first (master ODR control)
2. `CTRL_REG6_XL` ODR bits are ignored when gyro is active
3. Magnetometer registers are independent

### Data Flow
1. Sensor data read via optimized burst I2C transactions
2. Raw data converted using sensitivity values
3. Published as standard ROS2 sensor messages

### Validation
All parameters are validated at startup with clear error messages for invalid configurations.
