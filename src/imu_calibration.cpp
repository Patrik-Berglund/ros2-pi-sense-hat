#include "ros2_pi_sense_hat/imu_calibration.hpp"
#include "ros2_pi_sense_hat/lsm9ds1_driver.hpp"
#include <fstream>
#include <iostream>
#include <chrono>

IMUCalibration::IMUCalibration() {
    // Initialize with current timestamp
}

void IMUCalibration::correctIMUData(IMUData& data) const {
    // Apply gyroscope bias correction
    if (cal_data_.gyro_calibrated) {
        data.gyro_x -= cal_data_.gyro_bias_x;
        data.gyro_y -= cal_data_.gyro_bias_y;
        data.gyro_z -= cal_data_.gyro_bias_z;
    }
    
    // Apply accelerometer calibration
    if (cal_data_.accel_calibrated) {
        data.accel_x = (data.accel_x - cal_data_.accel_offset_x) * cal_data_.accel_scale_x;
        data.accel_y = (data.accel_y - cal_data_.accel_offset_y) * cal_data_.accel_scale_y;
        data.accel_z = (data.accel_z - cal_data_.accel_offset_z) * cal_data_.accel_scale_z;
    }
    
    // Apply magnetometer calibration
    if (cal_data_.mag_calibrated) {
        data.mag_x -= cal_data_.mag_offset_x;
        data.mag_y -= cal_data_.mag_offset_y;
        data.mag_z -= cal_data_.mag_offset_z;
    }
}

bool IMUCalibration::loadCalibration(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("gyro_bias_x:") != std::string::npos) {
            cal_data_.gyro_bias_x = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("gyro_bias_y:") != std::string::npos) {
            cal_data_.gyro_bias_y = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("gyro_bias_z:") != std::string::npos) {
            cal_data_.gyro_bias_z = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("accel_offset_x:") != std::string::npos) {
            cal_data_.accel_offset_x = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("accel_offset_y:") != std::string::npos) {
            cal_data_.accel_offset_y = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("accel_offset_z:") != std::string::npos) {
            cal_data_.accel_offset_z = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("accel_scale_x:") != std::string::npos) {
            cal_data_.accel_scale_x = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("accel_scale_y:") != std::string::npos) {
            cal_data_.accel_scale_y = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("accel_scale_z:") != std::string::npos) {
            cal_data_.accel_scale_z = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("gyro_calibrated:") != std::string::npos) {
            cal_data_.gyro_calibrated = (line.find("true") != std::string::npos);
        } else if (line.find("accel_calibrated:") != std::string::npos) {
            cal_data_.accel_calibrated = (line.find("true") != std::string::npos);
        }
    }
    
    return true;
}

bool IMUCalibration::saveCalibration(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    file << "# IMU Calibration Data\n";
    file << "gyro_bias_x: " << cal_data_.gyro_bias_x << "\n";
    file << "gyro_bias_y: " << cal_data_.gyro_bias_y << "\n";
    file << "gyro_bias_z: " << cal_data_.gyro_bias_z << "\n";
    file << "accel_offset_x: " << cal_data_.accel_offset_x << "\n";
    file << "accel_offset_y: " << cal_data_.accel_offset_y << "\n";
    file << "accel_offset_z: " << cal_data_.accel_offset_z << "\n";
    file << "accel_scale_x: " << cal_data_.accel_scale_x << "\n";
    file << "accel_scale_y: " << cal_data_.accel_scale_y << "\n";
    file << "accel_scale_z: " << cal_data_.accel_scale_z << "\n";
    file << "gyro_calibrated: " << (cal_data_.gyro_calibrated ? "true" : "false") << "\n";
    file << "accel_calibrated: " << (cal_data_.accel_calibrated ? "true" : "false") << "\n";
    
    return true;
}
