#!/usr/bin/env python3

import subprocess
import time

def read_i2c_reg(addr, reg):
    """Read a register via i2cget"""
    try:
        result = subprocess.run(['i2cget', '-y', '1', f'0x{addr:02x}', f'0x{reg:02x}'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            return int(result.stdout.strip(), 16)
        return None
    except:
        return None

def read_i2c_reg16(addr, reg_low):
    """Read 16-bit register (little endian)"""
    low = read_i2c_reg(addr, reg_low)
    high = read_i2c_reg(addr, reg_low + 1)
    if low is not None and high is not None:
        return (high << 8) | low
    return None

print("=== LSM9DS1 Debug Test ===")

# Check WHO_AM_I registers
print(f"Accel/Gyro WHO_AM_I (0x6A, 0x0F): 0x{read_i2c_reg(0x6A, 0x0F):02x} (expect 0x68)")
print(f"Magnetometer WHO_AM_I (0x1C, 0x0F): 0x{read_i2c_reg(0x1C, 0x0F):02x} (expect 0x3D)")

# Check magnetometer control registers
print(f"\nMagnetometer Control Registers:")
print(f"CTRL_REG1_M (0x20): 0x{read_i2c_reg(0x1C, 0x20):02x}")
print(f"CTRL_REG2_M (0x21): 0x{read_i2c_reg(0x1C, 0x21):02x}")
print(f"CTRL_REG3_M (0x22): 0x{read_i2c_reg(0x1C, 0x22):02x}")

# Check magnetometer status
status = read_i2c_reg(0x1C, 0x27)
print(f"STATUS_REG_M (0x27): 0x{status:02x}")
if status:
    print(f"  ZYXDA (new data): {'Yes' if status & 0x08 else 'No'}")
    print(f"  XDA: {'Yes' if status & 0x01 else 'No'}")
    print(f"  YDA: {'Yes' if status & 0x02 else 'No'}")
    print(f"  ZDA: {'Yes' if status & 0x04 else 'No'}")

# Read magnetometer data
print(f"\nMagnetometer Raw Data:")
mag_x = read_i2c_reg16(0x1C, 0x28)
mag_y = read_i2c_reg16(0x1C, 0x2A)
mag_z = read_i2c_reg16(0x1C, 0x2C)

if mag_x is not None:
    # Convert to signed 16-bit
    if mag_x > 32767: mag_x -= 65536
    if mag_y > 32767: mag_y -= 65536
    if mag_z > 32767: mag_z -= 65536
    
    print(f"Raw X: {mag_x}, Y: {mag_y}, Z: {mag_z}")
    
    # Convert to gauss (±4 gauss range, 0.14 mgauss/LSB)
    sensitivity = 0.14 / 1000.0  # gauss/LSB
    print(f"Gauss X: {mag_x * sensitivity:.3f}, Y: {mag_y * sensitivity:.3f}, Z: {mag_z * sensitivity:.3f}")

# Read temperature
print(f"\nTemperature:")
temp_raw = read_i2c_reg16(0x6A, 0x15)
if temp_raw is not None:
    if temp_raw > 32767: temp_raw -= 65536
    temp_c = (temp_raw / 16.0) + 25.0
    print(f"Raw temp: {temp_raw}, Calculated: {temp_c:.1f}°C")
