#!/usr/bin/env python3

"""
Debug script to check LSM9DS1 I2C communication
"""

import subprocess
import sys

def check_i2c_devices():
    """Check which I2C devices are present"""
    print("=== I2C Device Scan ===")
    try:
        result = subprocess.run(['i2cdetect', '-y', '1'], 
                              capture_output=True, text=True, check=True)
        print(result.stdout)
        
        # Check for LSM9DS1 addresses
        output = result.stdout.lower()
        
        # Possible accelerometer/gyroscope addresses
        ag_addresses = ['6a', '6b']
        found_ag = [addr for addr in ag_addresses if addr in output]
        
        # Possible magnetometer addresses  
        mag_addresses = ['1c', '1e']
        found_mag = [addr for addr in mag_addresses if addr in output]
        
        print(f"Accelerometer/Gyroscope found at: {found_ag}")
        print(f"Magnetometer found at: {found_mag}")
        
        return found_ag, found_mag
        
    except Exception as e:
        print(f"Error scanning I2C: {e}")
        return [], []

def test_who_am_i():
    """Test WHO_AM_I register reads"""
    print("\n=== WHO_AM_I Register Test ===")
    
    # Test accelerometer/gyroscope
    try:
        result = subprocess.run(['i2cget', '-y', '1', '0x6a', '0x0f'], 
                              capture_output=True, text=True, check=True)
        who_am_i_ag = result.stdout.strip()
        print(f"Accel/Gyro WHO_AM_I (0x6A): {who_am_i_ag} (expected: 0x68)")
    except Exception as e:
        print(f"Failed to read accel/gyro WHO_AM_I: {e}")
    
    # Test magnetometer at 0x1C
    try:
        result = subprocess.run(['i2cget', '-y', '1', '0x1c', '0x0f'], 
                              capture_output=True, text=True, check=True)
        who_am_i_m = result.stdout.strip()
        print(f"Magnetometer WHO_AM_I (0x1C): {who_am_i_m} (expected: 0x3d)")
    except Exception as e:
        print(f"Failed to read magnetometer WHO_AM_I at 0x1C: {e}")
        
    # Test magnetometer at 0x1E
    try:
        result = subprocess.run(['i2cget', '-y', '1', '0x1e', '0x0f'], 
                              capture_output=True, text=True, check=True)
        who_am_i_m = result.stdout.strip()
        print(f"Magnetometer WHO_AM_I (0x1E): {who_am_i_m} (expected: 0x3d)")
    except Exception as e:
        print(f"Failed to read magnetometer WHO_AM_I at 0x1E: {e}")

def main():
    print("=== LSM9DS1 Debug Tool ===")
    
    # Check I2C devices
    ag_found, mag_found = check_i2c_devices()
    
    # Test WHO_AM_I registers
    test_who_am_i()
    
    print("\n=== Analysis ===")
    if not ag_found:
        print("❌ No accelerometer/gyroscope found!")
    else:
        print(f"✅ Accelerometer/gyroscope at: 0x{ag_found[0].upper()}")
        
    if not mag_found:
        print("❌ No magnetometer found!")
    else:
        print(f"✅ Magnetometer at: 0x{mag_found[0].upper()}")

if __name__ == '__main__':
    main()
