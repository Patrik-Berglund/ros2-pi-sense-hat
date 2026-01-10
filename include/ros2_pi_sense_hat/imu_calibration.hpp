#pragma once

#include <string>
#include <cstdint>

struct CalibrationData {
    // Gyroscope bias (rad/s)
    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;
    
    // Accelerometer offset and scale
    float accel_offset_x = 0.0f;
    float accel_offset_y = 0.0f;
    float accel_offset_z = 0.0f;
    float accel_scale_x = 1.0f;
    float accel_scale_y = 1.0f;
    float accel_scale_z = 1.0f;
    
    // Magnetometer hard-iron offset
    float mag_offset_x = 0.0f;
    float mag_offset_y = 0.0f;
    float mag_offset_z = 0.0f;
    
    // Calibration status
    bool gyro_calibrated = false;
    bool accel_calibrated = false;
    bool mag_calibrated = false;
    
    // Timestamp
    uint64_t timestamp = 0;
};

class IMUCalibration {
public:
    IMUCalibration();
    
    // Core functionality - load/apply calibration
    void correctIMUData(struct IMUData& data) const;
    bool loadCalibration(const std::string& filename);
    bool saveCalibration(const std::string& filename) const;
    
    // Status
    const CalibrationData& getCalibrationData() const { return cal_data_; }
    bool isCalibrated() const { return cal_data_.gyro_calibrated; }
    
    // Backward compatibility - deprecated interactive methods
    [[deprecated("Use Python calibration service instead")]]
    bool calibrateGyroscope(class LSM9DS1Driver*, int = 1000) { return false; }
    
    [[deprecated("Use Python calibration service instead")]]
    bool calibrateAccelerometer(class LSM9DS1Driver*, int = 500) { return false; }
    
    [[deprecated("Use Python calibration service instead")]]
    bool calibrateMagnetometer(class LSM9DS1Driver*, int = 500) { return false; }
    
private:
    CalibrationData cal_data_;
};
