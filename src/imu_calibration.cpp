#include "ros2_pi_sense_hat/imu_calibration.hpp"
#include "ros2_pi_sense_hat/lsm9ds1_driver.hpp"
#include <fstream>
#include <sstream>
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
    
    // Apply accelerometer calibration (matrix correction)
    if (cal_data_.accel_calibrated) {
        float ax = data.accel_x - cal_data_.accel_offset_x;
        float ay = data.accel_y - cal_data_.accel_offset_y;
        float az = data.accel_z - cal_data_.accel_offset_z;
        
        const float* M = cal_data_.accel_matrix;
        data.accel_x = M[0]*ax + M[1]*ay + M[2]*az;
        data.accel_y = M[3]*ax + M[4]*ay + M[5]*az;
        data.accel_z = M[6]*ax + M[7]*ay + M[8]*az;
    }
    
    // Apply magnetometer calibration (matrix correction)
    if (cal_data_.mag_calibrated) {
        float mx = data.mag_x - cal_data_.mag_offset_x;
        float my = data.mag_y - cal_data_.mag_offset_y;
        float mz = data.mag_z - cal_data_.mag_offset_z;
        
        const float* M = cal_data_.mag_matrix;
        data.mag_x = M[0]*mx + M[1]*my + M[2]*mz;
        data.mag_y = M[3]*mx + M[4]*my + M[5]*mz;
        data.mag_z = M[6]*mx + M[7]*my + M[8]*mz;
    }
}

bool IMUCalibration::loadCalibration(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        // Gyroscope
        if (line.find("gyro_bias_x:") != std::string::npos) {
            cal_data_.gyro_bias_x = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("gyro_bias_y:") != std::string::npos) {
            cal_data_.gyro_bias_y = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("gyro_bias_z:") != std::string::npos) {
            cal_data_.gyro_bias_z = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("gyro_calibrated:") != std::string::npos) {
            cal_data_.gyro_calibrated = (line.find("true") != std::string::npos);
        }
        // Accelerometer offset
        else if (line.find("accel_offset_x:") != std::string::npos) {
            cal_data_.accel_offset_x = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("accel_offset_y:") != std::string::npos) {
            cal_data_.accel_offset_y = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("accel_offset_z:") != std::string::npos) {
            cal_data_.accel_offset_z = std::stof(line.substr(line.find(":") + 1));
        }
        // Accelerometer matrix
        else if (line.find("accel_matrix:") != std::string::npos) {
            size_t start = line.find("[");
            size_t end = line.find("]");
            if (start != std::string::npos && end != std::string::npos) {
                std::string matrix_str = line.substr(start + 1, end - start - 1);
                std::stringstream ss(matrix_str);
                std::string value;
                int i = 0;
                while (std::getline(ss, value, ',') && i < 9) {
                    cal_data_.accel_matrix[i++] = std::stof(value);
                }
            }
        } else if (line.find("accel_calibrated:") != std::string::npos) {
            cal_data_.accel_calibrated = (line.find("true") != std::string::npos);
        }
        // Magnetometer offset
        else if (line.find("mag_offset_x:") != std::string::npos) {
            cal_data_.mag_offset_x = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("mag_offset_y:") != std::string::npos) {
            cal_data_.mag_offset_y = std::stof(line.substr(line.find(":") + 1));
        } else if (line.find("mag_offset_z:") != std::string::npos) {
            cal_data_.mag_offset_z = std::stof(line.substr(line.find(":") + 1));
        }
        // Magnetometer matrix
        else if (line.find("mag_matrix:") != std::string::npos) {
            size_t start = line.find("[");
            size_t end = line.find("]");
            if (start != std::string::npos && end != std::string::npos) {
                std::string matrix_str = line.substr(start + 1, end - start - 1);
                std::stringstream ss(matrix_str);
                std::string value;
                int i = 0;
                while (std::getline(ss, value, ',') && i < 9) {
                    cal_data_.mag_matrix[i++] = std::stof(value);
                }
            }
        } else if (line.find("mag_calibrated:") != std::string::npos) {
            cal_data_.mag_calibrated = (line.find("true") != std::string::npos);
        }
    }
    
    return true;
}

