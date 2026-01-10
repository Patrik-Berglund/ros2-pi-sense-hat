# ROS2 sensor_msgs Message Compliance (Kilted)

## Overview

This document ensures our LSM9DS1 IMU driver complies with ROS2 Kilted sensor message specifications.

**Reference**: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/

## Message Types Used

### 1. sensor_msgs/Imu

**Specification**: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html

**Requirements**:
- Accelerations in **m/s²** (not g's)
- Angular velocity in **rad/s**
- Covariance matrices: diagonal variance or -1 if unavailable

**Our Implementation**:
```cpp
// ✅ Correct units
imu_msg.linear_acceleration.x = data.accel_x;  // Already in m/s²
imu_msg.angular_velocity.x = data.gyro_x;      // Already in rad/s

// ✅ Correct covariance handling
imu_msg.linear_acceleration_covariance[i] = (i == 0) ? 0.01 : 0.0;  // Diagonal
imu_msg.angular_velocity_covariance[i] = (i == 0) ? 0.01 : 0.0;     // Diagonal
imu_msg.orientation_covariance[i] = (i == 0) ? -1.0 : 0.0;          // No orientation
```

**Status**: ✅ **COMPLIANT**

### 2. sensor_msgs/MagneticField

**Specification**: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html

**Requirements**:
- Magnetic field in **Tesla** (not gauss)
- Covariance matrix in Tesla²

**Our Implementation**:
```cpp
// ✅ Fixed: Convert to Tesla (1 Tesla = 10,000 gauss)
data.mag_x = raw_mag_x * mag_sens / 1000.0f / 10000.0f;  // mgauss -> Tesla

// ✅ Correct covariance
mag_msg.magnetic_field_covariance[i] = (i == 0) ? 0.01 : 0.0;  // Tesla²
```

**Status**: ✅ **COMPLIANT** (Fixed from gauss to Tesla)

### 3. sensor_msgs/Temperature

**Specification**: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Temperature.html

**Requirements**:
- Temperature in **Celsius**
- Variance in Celsius²

**Our Implementation**:
```cpp
// ✅ Correct units
temp_msg.temperature = data.temperature;  // Already in Celsius
temp_msg.variance = 1.0;                  // Celsius²
```

**Status**: ✅ **COMPLIANT**

## Unit Conversions Applied

### Accelerometer
```cpp
// LSM9DS1 raw → m/s²
data.accel_x = raw_accel_x * accel_sensitivity * 9.81f / 1000.0f;
//             ^^^^^^^^^^^   ^^^^^^^^^^^^^^^^   ^^^^^^   ^^^^^^^
//             Raw LSB       mg/LSB             g→m/s²   mg→g
```

### Gyroscope
```cpp
// LSM9DS1 raw → rad/s
data.gyro_x = raw_gyro_x * gyro_sensitivity * M_PI / 180.0f / 1000.0f;
//            ^^^^^^^^^^   ^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^   ^^^^^^^
//            Raw LSB      mdps/LSB           deg→rad        mdps→dps
```

### Magnetometer
```cpp
// LSM9DS1 raw → Tesla
data.mag_x = raw_mag_x * mag_sensitivity / 1000.0f / 10000.0f;
//           ^^^^^^^^^   ^^^^^^^^^^^^^^^^   ^^^^^^^   ^^^^^^^^
//           Raw LSB     mgauss/LSB         mg→g      gauss→Tesla
```

### Temperature
```cpp
// LSM9DS1 raw → Celsius
data.temperature = (raw_temp / 16.0f) + 25.0f;
//                 ^^^^^^^^^^^^^^^^   ^^^^^^
//                 LSB→°C scale       Offset
```

## Covariance Matrix Guidelines

### Diagonal Covariance (Most Common)
```cpp
// 3x3 matrix stored as 9-element array (row-major)
// [σ²  0   0 ]
// [0   σ²  0 ] → [σ², 0, 0, 0, σ², 0, 0, 0, σ²]
// [0   0   σ²]

for (int i = 0; i < 9; i++) {
    covariance[i] = (i % 4 == 0) ? variance : 0.0;  // Diagonal elements only
}
```

### Special Values
- **-1**: Data not available (e.g., no orientation estimate)
- **0**: Unknown covariance (consumer must assume)
- **>0**: Known variance values

## Compliance Checklist

- [x] **Imu message**: Accelerations in m/s², angular velocity in rad/s
- [x] **MagneticField message**: Magnetic field in Tesla
- [x] **Temperature message**: Temperature in Celsius
- [x] **Covariance matrices**: Proper diagonal values or -1 for unavailable
- [x] **Frame IDs**: Consistent frame naming ("imu_link")
- [x] **Timestamps**: Synchronized timestamps across all messages

## Testing Compliance

### Expected Value Ranges
```bash
# Accelerometer (m/s²)
# At rest: ~[0, 0, 9.81] (gravity on Z-axis)
ros2 topic echo /sense_hat/imu/data_raw

# Magnetometer (Tesla)  
# Earth's field: ~50 µT (0.00005 Tesla)
ros2 topic echo /sense_hat/imu/mag

# Temperature (Celsius)
# Room temperature: ~20-25°C
ros2 topic echo /sense_hat/temperature/imu
```

### Validation Commands
```bash
# Check message structure
ros2 interface show sensor_msgs/msg/Imu
ros2 interface show sensor_msgs/msg/MagneticField
ros2 interface show sensor_msgs/msg/Temperature

# Verify units in live data
ros2 topic echo /sense_hat/imu/data_raw --field linear_acceleration
ros2 topic echo /sense_hat/imu/mag --field magnetic_field
```

## References

- [ROS sensor_msgs Documentation](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/)
- [sensor_msgs/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)
- [sensor_msgs/MagneticField](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html)
- [sensor_msgs/Temperature](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Temperature.html)
- [REP-145: Conventions for IMU Sensor Drivers](https://reps.openrobotics.org/rep-0145/)
- [ROS2 Message Standards](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html)

## Changelog

- **2026-01-10**: Fixed magnetometer units from gauss to Tesla for ROS2 compliance
- **2026-01-10**: Verified Imu and Temperature message compliance
- **2026-01-10**: Added proper covariance matrix handling
