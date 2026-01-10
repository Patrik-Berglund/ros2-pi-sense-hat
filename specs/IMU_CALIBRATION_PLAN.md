# LSM9DS1 IMU Calibration Implementation Plan

## Overview

Calibration is essential for accurate IMU measurements. The LSM9DS1 requires different calibration approaches for each sensor type due to their different error characteristics and physical principles.

## Calibration Types Required

### 1. **Gyroscope Bias Calibration** (Essential)
**Problem**: Gyroscopes have a constant bias (offset) that causes drift over time.
**Solution**: Measure bias when stationary and subtract from all readings.

**Method**: 
- Keep IMU completely stationary
- Collect 1000+ samples over 10-30 seconds
- Calculate mean of each axis → bias values
- Store and subtract from future readings

### 2. **Accelerometer Offset Calibration** (Important)
**Problem**: Manufacturing tolerances cause offset from true zero-g.
**Solution**: Use Earth's gravity (1g) as reference.

**Method**:
- Place IMU in 6 orientations (±X, ±Y, ±Z facing down)
- Measure gravity vector in each position
- Calculate offset and scale factors
- Alternative: Simple stationary calibration assuming current orientation

### 3. **Magnetometer Calibration** (Critical for heading)
**Problem**: Hard-iron (constant magnetic fields) and soft-iron (field distortion) effects.
**Solution**: Ellipse fitting to compensate for magnetic distortions.

**Method**:
- Rotate IMU through full 3D sphere of orientations
- Collect magnetometer data points
- Fit ellipsoid to data → center = hard-iron offset
- Apply transformation matrix for soft-iron correction

## Implementation Architecture

### Phase 1: Basic Calibration Framework

```cpp
class IMUCalibration {
public:
    struct CalibrationData {
        // Gyroscope bias (rad/s)
        float gyro_bias_x, gyro_bias_y, gyro_bias_z;
        
        // Accelerometer offset and scale
        float accel_offset_x, accel_offset_y, accel_offset_z;
        float accel_scale_x, accel_scale_y, accel_scale_z;
        
        // Magnetometer hard-iron offset
        float mag_offset_x, mag_offset_y, mag_offset_z;
        
        // Magnetometer soft-iron transformation matrix
        float mag_transform[3][3];
        
        // Calibration validity flags
        bool gyro_calibrated;
        bool accel_calibrated;
        bool mag_calibrated;
        
        // Calibration timestamp
        uint64_t timestamp;
    };
    
    // Calibration methods
    bool calibrateGyroscope(int sample_count = 1000);
    bool calibrateAccelerometer(bool six_point = false);
    bool calibrateMagnetometer(int sample_count = 500);
    
    // Data correction
    void correctGyroscope(float& gx, float& gy, float& gz);
    void correctAccelerometer(float& ax, float& ay, float& az);
    void correctMagnetometer(float& mx, float& my, float& mz);
    
    // Persistence
    bool saveCalibration(const std::string& filename);
    bool loadCalibration(const std::string& filename);
    
private:
    CalibrationData cal_data_;
    LSM9DS1Driver* driver_;
};
```

### Phase 2: ROS2 Integration

```cpp
// Enhanced IMU Node with calibration
class IMUNode : public rclcpp::Node {
private:
    IMUCalibration calibration_;
    
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_gyro_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_accel_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_mag_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_calibration_srv_;
    
    // Parameters
    std::string calibration_file_;
    bool auto_load_calibration_;
    bool apply_calibration_;
};
```

## Detailed Implementation Steps

### Step 1: Gyroscope Bias Calibration (Simplest)

**Requirements**: IMU must be completely stationary
**Duration**: 10-30 seconds
**Accuracy**: Critical for orientation integration

```cpp
bool IMUCalibration::calibrateGyroscope(int sample_count) {
    RCLCPP_INFO(logger_, "Starting gyroscope calibration - keep IMU stationary!");
    
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < sample_count; i++) {
        IMUData data;
        if (driver_->readAllSensors(data)) {
            sum_x += data.gyro_x;
            sum_y += data.gyro_y;
            sum_z += data.gyro_z;
            valid_samples++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (valid_samples < sample_count * 0.9) {
        return false; // Too many failed readings
    }
    
    cal_data_.gyro_bias_x = sum_x / valid_samples;
    cal_data_.gyro_bias_y = sum_y / valid_samples;
    cal_data_.gyro_bias_z = sum_z / valid_samples;
    cal_data_.gyro_calibrated = true;
    
    RCLCPP_INFO(logger_, "Gyro bias: [%.6f, %.6f, %.6f] rad/s", 
                cal_data_.gyro_bias_x, cal_data_.gyro_bias_y, cal_data_.gyro_bias_z);
    return true;
}
```

### Step 2: Simple Accelerometer Calibration

**Requirements**: IMU stationary in known orientation
**Assumption**: One axis aligned with gravity

```cpp
bool IMUCalibration::calibrateAccelerometer(bool six_point) {
    if (six_point) {
        // TODO: Implement 6-point calibration (user guidance required)
        return false;
    }
    
    // Simple single-point calibration
    RCLCPP_INFO(logger_, "Starting accelerometer calibration - keep IMU level!");
    
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < 500; i++) {
        IMUData data;
        if (driver_->readAllSensors(data)) {
            sum_x += data.accel_x;
            sum_y += data.accel_y;
            sum_z += data.accel_z;
            valid_samples++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    float avg_x = sum_x / valid_samples;
    float avg_y = sum_y / valid_samples;
    float avg_z = sum_z / valid_samples;
    
    // Assume Z-axis is pointing up (against gravity)
    cal_data_.accel_offset_x = avg_x;
    cal_data_.accel_offset_y = avg_y;
    cal_data_.accel_offset_z = avg_z - 9.81f; // Remove gravity
    
    // Simple unity scale factors (can be improved with 6-point calibration)
    cal_data_.accel_scale_x = cal_data_.accel_scale_y = cal_data_.accel_scale_z = 1.0f;
    cal_data_.accel_calibrated = true;
    
    return true;
}
```

### Step 3: Magnetometer Calibration (Most Complex)

**Requirements**: Rotate IMU through full 3D sphere
**Duration**: 30-60 seconds of continuous rotation

```cpp
bool IMUCalibration::calibrateMagnetometer(int sample_count) {
    RCLCPP_INFO(logger_, "Starting magnetometer calibration - rotate IMU in all directions!");
    
    std::vector<float> mag_x_data, mag_y_data, mag_z_data;
    mag_x_data.reserve(sample_count);
    mag_y_data.reserve(sample_count);
    mag_z_data.reserve(sample_count);
    
    for (int i = 0; i < sample_count; i++) {
        IMUData data;
        if (driver_->readAllSensors(data)) {
            mag_x_data.push_back(data.mag_x);
            mag_y_data.push_back(data.mag_y);
            mag_z_data.push_back(data.mag_z);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Simple hard-iron calibration (center of ellipse)
    auto minmax_x = std::minmax_element(mag_x_data.begin(), mag_x_data.end());
    auto minmax_y = std::minmax_element(mag_y_data.begin(), mag_y_data.end());
    auto minmax_z = std::minmax_element(mag_z_data.begin(), mag_z_data.end());
    
    cal_data_.mag_offset_x = (*minmax_x.first + *minmax_x.second) / 2.0f;
    cal_data_.mag_offset_y = (*minmax_y.first + *minmax_y.second) / 2.0f;
    cal_data_.mag_offset_z = (*minmax_z.first + *minmax_z.second) / 2.0f;
    
    // Initialize soft-iron matrix as identity (can be improved with ellipse fitting)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cal_data_.mag_transform[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    cal_data_.mag_calibrated = true;
    return true;
}
```

## ROS2 Service Interface

### Services to Implement

```yaml
# Calibration services
/sense_hat/imu/calibrate_gyro     # std_srvs/Trigger - Gyro bias calibration
/sense_hat/imu/calibrate_accel    # std_srvs/Trigger - Accel offset calibration  
/sense_hat/imu/calibrate_mag      # std_srvs/Trigger - Mag hard-iron calibration
/sense_hat/imu/calibrate_all      # std_srvs/Trigger - Full calibration sequence
/sense_hat/imu/save_calibration   # std_srvs/Trigger - Save to file
/sense_hat/imu/load_calibration   # std_srvs/Trigger - Load from file
/sense_hat/imu/reset_calibration  # std_srvs/Trigger - Clear calibration data
```

### Parameters

```yaml
# Calibration parameters
calibration_file: "~/.ros2_imu_calibration.yaml"
auto_load_calibration: true
apply_calibration: true
gyro_calibration_samples: 1000
accel_calibration_samples: 500
mag_calibration_samples: 500
```

## User Interface & Workflow

### Calibration Sequence

1. **Preparation**
   ```bash
   # Start IMU node
   ros2 run ros2_pi_sense_hat imu_node
   
   # Check current calibration status
   ros2 service call /sense_hat/imu/get_calibration_status std_srvs/srv/Trigger
   ```

2. **Gyroscope Calibration** (30 seconds, stationary)
   ```bash
   ros2 service call /sense_hat/imu/calibrate_gyro std_srvs/srv/Trigger
   ```

3. **Accelerometer Calibration** (10 seconds, level)
   ```bash
   ros2 service call /sense_hat/imu/calibrate_accel std_srvs/srv/Trigger
   ```

4. **Magnetometer Calibration** (60 seconds, rotating)
   ```bash
   ros2 service call /sense_hat/imu/calibrate_mag std_srvs/srv/Trigger
   ```

5. **Save Calibration**
   ```bash
   ros2 service call /sense_hat/imu/save_calibration std_srvs/srv/Trigger
   ```

### Python Calibration Helper

```python
#!/usr/bin/env python3
# demo/calibrate_imu.py - Interactive calibration tool

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class IMUCalibrationHelper(Node):
    def __init__(self):
        super().__init__('imu_calibration_helper')
        
        # Service clients
        self.gyro_cal = self.create_client(Trigger, '/sense_hat/imu/calibrate_gyro')
        self.accel_cal = self.create_client(Trigger, '/sense_hat/imu/calibrate_accel')
        self.mag_cal = self.create_client(Trigger, '/sense_hat/imu/calibrate_mag')
        self.save_cal = self.create_client(Trigger, '/sense_hat/imu/save_calibration')
    
    def run_full_calibration(self):
        print("🎯 IMU Calibration Wizard")
        print("=" * 40)
        
        # Step 1: Gyroscope
        input("📍 Step 1: Place IMU on stable surface and press Enter...")
        print("⏳ Calibrating gyroscope (30 seconds)...")
        self.call_service(self.gyro_cal)
        
        # Step 2: Accelerometer  
        input("📍 Step 2: Keep IMU level and press Enter...")
        print("⏳ Calibrating accelerometer (10 seconds)...")
        self.call_service(self.accel_cal)
        
        # Step 3: Magnetometer
        input("📍 Step 3: Prepare to rotate IMU in all directions and press Enter...")
        print("⏳ Calibrating magnetometer (60 seconds) - ROTATE NOW!")
        self.call_service(self.mag_cal)
        
        # Step 4: Save
        print("💾 Saving calibration...")
        self.call_service(self.save_cal)
        
        print("✅ Calibration complete!")
```

## Implementation Priority

### Phase 1: Essential (Week 1)
- [x] Parameter system (completed)
- [ ] Gyroscope bias calibration
- [ ] Basic data correction
- [ ] **ROS2-native calibration persistence** (ament_index_cpp)

### ROS2 File Storage Integration

**File Storage Strategy using ROS2 APIs:**
```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>

// Priority-based file search
std::vector<std::string> getCalibrationSearchPaths(const std::string& filename) {
    std::vector<std::string> paths;
    
    // 1. User-specified file (highest priority)
    if (!filename.empty()) {
        paths.push_back(filename);
    }
    
    // 2. User home directory (persistent, user-specific)
    const char* home = getenv("HOME");
    if (home) {
        paths.push_back(std::string(home) + "/.ros2_imu_calibration.yaml");
    }
    
    // 3. Package config directory (default/fallback)
    try {
        std::string package_share = ament_index_cpp::get_package_share_directory("ros2_pi_sense_hat");
        paths.push_back(package_share + "/config/calibration_default.yaml");
    } catch (const std::exception& e) {
        // Package not found in development mode
    }
    
    // 4. Temporary fallback
    paths.push_back("/tmp/ros2_imu_calibration.yaml");
    
    return paths;
}
```

**CMakeLists.txt Integration:**
```cmake
# Add ament_index_cpp dependency
find_package(ament_index_cpp REQUIRED)

# Link calibration library
target_link_libraries(imu_calibration
  ${rclcpp_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

# Install default calibration file
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  FILES_MATCHING PATTERN "*.yaml"
)
```

**Storage Locations (Priority Order):**
1. **User-specified**: `--calibration-file /path/to/file.yaml`
2. **User home**: `~/.ros2_imu_calibration.yaml` (persistent)
3. **Package default**: `<package_share>/config/calibration_default.yaml`
4. **Fallback**: `/tmp/ros2_imu_calibration.yaml`

### Phase 2: Important (Week 2)  
- [ ] Accelerometer offset calibration
- [ ] ROS2 service interface
- [ ] Python calibration helper
- [ ] Calibration status reporting

### Phase 3: Advanced (Week 3)
- [ ] Magnetometer hard-iron calibration
- [ ] 6-point accelerometer calibration
- [ ] Soft-iron magnetometer correction
- [ ] Calibration quality metrics

### Phase 4: Polish (Week 4)
- [ ] Interactive calibration GUI
- [ ] Automatic calibration validation
- [ ] Temperature compensation
- [ ] Advanced ellipse fitting algorithms

## Expected Improvements

**Before Calibration:**
- Gyroscope drift: ~1-5°/minute
- Accelerometer offset: ±0.1g
- Magnetometer heading error: ±10-30°

**After Calibration:**
- Gyroscope drift: ~0.1-0.5°/minute  
- Accelerometer accuracy: ±0.01g
- Magnetometer heading error: ±2-5°

This calibration system will provide significant accuracy improvements for educational and practical applications!
