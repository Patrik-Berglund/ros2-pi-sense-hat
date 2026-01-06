#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <iostream>

int main() {
    std::cout << "Testing I2C device class..." << std::endl;
    
    I2CDevice mag("/dev/i2c-1", 0x1C);
    
    if (!mag.open()) {
        std::cout << "Failed to open magnetometer device" << std::endl;
        return 1;
    }
    
    std::cout << "Magnetometer device opened successfully" << std::endl;
    
    // Test WHO_AM_I
    uint8_t who_am_i;
    if (mag.readReg(0x0F, who_am_i)) {
        std::cout << "WHO_AM_I: 0x" << std::hex << (int)who_am_i << std::endl;
    } else {
        std::cout << "Failed to read WHO_AM_I" << std::endl;
    }
    
    // Test individual register reads
    uint8_t xl, xh, yl, yh, zl, zh;
    if (mag.readReg(0x28, xl) && mag.readReg(0x29, xh) &&
        mag.readReg(0x2A, yl) && mag.readReg(0x2B, yh) &&
        mag.readReg(0x2C, zl) && mag.readReg(0x2D, zh)) {
        
        int16_t raw_x = (int16_t)(xl | (xh << 8));
        int16_t raw_y = (int16_t)(yl | (yh << 8));
        int16_t raw_z = (int16_t)(zl | (zh << 8));
        
        std::cout << "Raw magnetometer data: X=" << raw_x << ", Y=" << raw_y << ", Z=" << raw_z << std::endl;
        
        float sensitivity = 0.14f / 1000.0f;
        std::cout << "Gauss: X=" << raw_x * sensitivity << ", Y=" << raw_y * sensitivity << ", Z=" << raw_z * sensitivity << std::endl;
    } else {
        std::cout << "Failed to read magnetometer registers" << std::endl;
    }
    
    mag.close();
    return 0;
}
