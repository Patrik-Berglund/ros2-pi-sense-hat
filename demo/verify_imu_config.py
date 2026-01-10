#!/usr/bin/env python3

import smbus
import time
import sys

# I2C addresses
LSM9DS1_AG_ADDR = 0x6A  # Accel/Gyro
LSM9DS1_M_ADDR = 0x1C   # Magnetometer

# Key registers
CTRL_REG1_G = 0x10
CTRL_REG6_XL = 0x20
CTRL_REG1_M = 0x20
CTRL_REG2_M = 0x21
CTRL_REG3_M = 0x22

def decode_odr_g(value):
    odr_bits = (value >> 5) & 0x07
    odr_map = {0: "Power-down", 1: "14.9 Hz", 2: "59.5 Hz", 3: "119 Hz", 
               4: "238 Hz", 5: "476 Hz", 6: "952 Hz", 7: "n.a."}
    return odr_map.get(odr_bits, "Unknown")

def decode_fs_g(value):
    fs_bits = (value >> 3) & 0x03
    fs_map = {0: "±245 dps", 1: "±500 dps", 2: "Not Available", 3: "±2000 dps"}
    return fs_map.get(fs_bits, "Unknown")

def decode_fs_xl(value):
    fs_bits = (value >> 3) & 0x03
    fs_map = {0: "±2g", 1: "±16g", 2: "±4g", 3: "±8g"}
    return fs_map.get(fs_bits, "Unknown")

def decode_bw_xl(value):
    bw_scal_odr = (value >> 2) & 0x01
    bw_xl = value & 0x03
    if bw_scal_odr == 0:
        return "Auto (ODR-dependent)"
    else:
        bw_map = {0: "408 Hz", 1: "211 Hz", 2: "105 Hz", 3: "50 Hz"}
        return f"Manual: {bw_map.get(bw_xl, 'Unknown')}"

def decode_mag_odr(value):
    do_bits = (value >> 2) & 0x07
    odr_map = {0: "0.625 Hz", 1: "1.25 Hz", 2: "2.5 Hz", 3: "5 Hz",
               4: "10 Hz", 5: "20 Hz", 6: "40 Hz", 7: "80 Hz"}
    return odr_map.get(do_bits, "Unknown")

def decode_mag_fs(value):
    fs_bits = (value >> 5) & 0x03
    fs_map = {0: "±4 gauss", 1: "±8 gauss", 2: "±12 gauss", 3: "±16 gauss"}
    return fs_map.get(fs_bits, "Unknown")

def decode_mag_om(value):
    om_bits = (value >> 5) & 0x03
    om_map = {0: "Low-power", 1: "Medium-performance", 2: "High-performance", 3: "Ultra-high performance"}
    return om_map.get(om_bits, "Unknown")

def main():
    try:
        bus = smbus.SMBus(1)
        
        print("=== LSM9DS1 Register Configuration Verification ===\n")
        
        # Read accelerometer/gyroscope registers
        print("📊 Accelerometer/Gyroscope (0x6A):")
        try:
            ctrl_reg1_g = bus.read_byte_data(LSM9DS1_AG_ADDR, CTRL_REG1_G)
            ctrl_reg6_xl = bus.read_byte_data(LSM9DS1_AG_ADDR, CTRL_REG6_XL)
            
            print(f"  CTRL_REG1_G  (0x10): 0x{ctrl_reg1_g:02X}")
            print(f"    ODR_G:  {decode_odr_g(ctrl_reg1_g)}")
            print(f"    FS_G:   {decode_fs_g(ctrl_reg1_g)}")
            print(f"    BW_G:   {ctrl_reg1_g & 0x03} (bandwidth bits)")
            
            print(f"  CTRL_REG6_XL (0x20): 0x{ctrl_reg6_xl:02X}")
            print(f"    ODR_XL: {decode_odr_g(ctrl_reg6_xl)} (ignored when gyro active)")
            print(f"    FS_XL:  {decode_fs_xl(ctrl_reg6_xl)}")
            print(f"    BW_XL:  {decode_bw_xl(ctrl_reg6_xl)}")
            
        except Exception as e:
            print(f"  ❌ Error reading accel/gyro: {e}")
        
        print()
        
        # Read magnetometer registers
        print("🧭 Magnetometer (0x1C):")
        try:
            ctrl_reg1_m = bus.read_byte_data(LSM9DS1_M_ADDR, CTRL_REG1_M)
            ctrl_reg2_m = bus.read_byte_data(LSM9DS1_M_ADDR, CTRL_REG2_M)
            ctrl_reg3_m = bus.read_byte_data(LSM9DS1_M_ADDR, CTRL_REG3_M)
            
            print(f"  CTRL_REG1_M (0x20): 0x{ctrl_reg1_m:02X}")
            print(f"    TEMP_COMP: {'Enabled' if ctrl_reg1_m & 0x80 else 'Disabled'}")
            print(f"    OM:        {decode_mag_om(ctrl_reg1_m)}")
            print(f"    DO:        {decode_mag_odr(ctrl_reg1_m)}")
            print(f"    ST:        {'Enabled' if ctrl_reg1_m & 0x01 else 'Disabled'}")
            
            print(f"  CTRL_REG2_M (0x21): 0x{ctrl_reg2_m:02X}")
            print(f"    FS:        {decode_mag_fs(ctrl_reg2_m)}")
            
            print(f"  CTRL_REG3_M (0x22): 0x{ctrl_reg3_m:02X}")
            md_bits = ctrl_reg3_m & 0x03
            md_map = {0: "Continuous", 1: "Single", 2: "Power-down", 3: "Power-down"}
            print(f"    MD:        {md_map.get(md_bits, 'Unknown')} mode")
            
        except Exception as e:
            print(f"  ❌ Error reading magnetometer: {e}")
        
        print("\n=== Configuration Test ===")
        print("To test configuration changes:")
        print("1. Run IMU node with different parameters")
        print("2. Run this script again to see register changes")
        print("\nExample:")
        print("ros2 run ros2_pi_sense_hat imu_node --ros-args -p imu_odr:=238 -p accel_range:=8")
        
    except Exception as e:
        print(f"❌ Error accessing I2C: {e}")
        print("Make sure:")
        print("- I2C is enabled")
        print("- LSM9DS1 is connected")
        print("- Run with sudo if needed")

if __name__ == "__main__":
    main()
