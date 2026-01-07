#!/usr/bin/env python3

import subprocess
import time

def test_magnetometer_burst_read():
    """Test 6-byte burst read from magnetometer"""
    print("=== Testing Magnetometer Burst Read ===")
    
    # Test with auto-increment (0x28 | 0x80 = 0xA8)
    try:
        result = subprocess.run(['i2cget', '-y', '1', '0x1C', '0xA8', 'i', '6'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            # Parse the output: "0x38 0xfe 0xa9 0x03 0xff 0xee"
            values = [int(x, 16) for x in result.stdout.strip().split()]
            print(f"Raw bytes: {[hex(x) for x in values]}")
            
            # Convert to signed 16-bit values (little endian)
            raw_x = values[0] | (values[1] << 8)
            raw_y = values[2] | (values[3] << 8)  
            raw_z = values[4] | (values[5] << 8)
            
            # Convert to signed
            if raw_x > 32767: raw_x -= 65536
            if raw_y > 32767: raw_y -= 65536
            if raw_z > 32767: raw_z -= 65536
            
            print(f"Raw values: X={raw_x}, Y={raw_y}, Z={raw_z}")
            
            # Convert to gauss (±4 gauss range, 0.14 mgauss/LSB)
            sensitivity = 0.14 / 1000.0
            gauss_x = raw_x * sensitivity
            gauss_y = raw_y * sensitivity
            gauss_z = raw_z * sensitivity
            
            print(f"Gauss values: X={gauss_x:.3f}, Y={gauss_y:.3f}, Z={gauss_z:.3f}")
            return True
        else:
            print(f"Error: {result.stderr}")
            return False
    except Exception as e:
        print(f"Exception: {e}")
        return False

if __name__ == "__main__":
    test_magnetometer_burst_read()
