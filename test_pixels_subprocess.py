#!/usr/bin/env python3
import subprocess
import time

def set_pixel(x, y, r, g, b):
    cmd = f'bash -c "source /opt/ros/kilted/setup.bash && source install/setup.bash && ros2 service call /sense_hat/led_matrix/set_pixel ros2_pi_sense_hat/srv/SetPixel \\"{{x: {x}, y: {y}, r: {r}, g: {g}, b: {b}}}\\"" '
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error: {result.stderr}")
    return result.returncode == 0

def main():
    print("Setting pixels quickly...")
    start = time.time()
    
    # Test first pixel with output
    print("Testing first pixel...")
    success = set_pixel(0, 0, 31, 0, 0)
    print(f"First pixel success: {success}")
    
    if success:
        # Continue with rest
        set_pixel(7, 0, 0, 31, 0)    # Green top-right
        set_pixel(0, 7, 0, 0, 31)    # Blue bottom-left
        set_pixel(7, 7, 31, 31, 0)   # Yellow bottom-right
        
        # Center cross
        set_pixel(3, 3, 31, 31, 31)  # White center
        set_pixel(4, 3, 31, 31, 31)
        set_pixel(3, 4, 31, 31, 31)
        set_pixel(4, 4, 31, 31, 31)
    
    elapsed = time.time() - start
    print(f"Completed in {elapsed:.3f} seconds")

if __name__ == '__main__':
    main()
