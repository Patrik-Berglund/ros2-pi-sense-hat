#pragma once

#include <string>
#include <cstdint>

struct CalibrationData {
    // Gyroscope bias (rad/s)
    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;
    bool gyro_calibrated = false;
    
    // Accelerometer offset and transformation matrix
    float accel_offset_x = 0.0f;
    float accel_offset_y = 0.0f;
    float accel_offset_z = 0.0f;
    float accel_matrix[9] = {1.0f, 0.0f, 0.0f,
                             0.0f, 1.0f, 0.0f,
                             0.0f, 0.0f, 1.0f};
    bool accel_calibrated = false;
    
    // Magnetometer offset and transformation matrix
    float mag_offset_x = 0.0f;
    float mag_offset_y = 0.0f;
    float mag_offset_z = 0.0f;
    float mag_matrix[9] = {1.0f, 0.0f, 0.0f,
                           0.0f, 1.0f, 0.0f,
                           0.0f, 0.0f, 1.0f};
    bool mag_calibrated = false;
    
    uint64_t timestamp = 0;
};

class IMUCalibration {
public:
    IMUCalibration();
    
    void correctIMUData(struct IMUData& data) const;
    bool loadCalibration(const std::string& filename);
    
    const CalibrationData& getCalibrationData() const { return cal_data_; }
    bool isCalibrated() const { return cal_data_.gyro_calibrated; }
    
private:
    CalibrationData cal_data_;
};
