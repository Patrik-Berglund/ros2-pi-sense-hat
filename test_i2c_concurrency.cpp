#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

void test_i2c_access(int device_addr, int iterations) {
    for (int i = 0; i < iterations; i++) {
        int fd = open("/dev/i2c-1", O_RDWR);
        if (fd < 0) {
            std::cerr << "Failed to open I2C device for addr 0x" << std::hex << device_addr << std::endl;
            continue;
        }
        
        if (ioctl(fd, I2C_SLAVE, device_addr) < 0) {
            std::cerr << "Failed to set I2C slave address 0x" << std::hex << device_addr << std::endl;
            close(fd);
            continue;
        }
        
        // Try to read WHO_AM_I or similar register
        uint8_t reg = (device_addr == 0x6A) ? 0x0F : 0x00;  // WHO_AM_I for LSM9DS1, or reg 0 for others
        uint8_t value;
        
        if (write(fd, &reg, 1) != 1) {
            std::cerr << "Write failed for addr 0x" << std::hex << device_addr << std::endl;
        } else if (read(fd, &value, 1) != 1) {
            std::cerr << "Read failed for addr 0x" << std::hex << device_addr << std::endl;
        } else {
            std::cout << "Thread " << device_addr << " iteration " << i << ": reg 0x" << std::hex << (int)reg << " = 0x" << (int)value << std::endl;
        }
        
        close(fd);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    std::cout << "Testing concurrent I2C access..." << std::endl;
    
    // Start multiple threads accessing different I2C devices simultaneously
    std::thread t1(test_i2c_access, 0x6A, 20);  // LSM9DS1 accel/gyro
    std::thread t2(test_i2c_access, 0x46, 20);  // ATTINY88
    std::thread t3(test_i2c_access, 0x5F, 20);  // HTS221
    
    t1.join();
    t2.join();
    t3.join();
    
    std::cout << "Test completed." << std::endl;
    return 0;
}
