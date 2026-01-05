#!/usr/bin/env python3
import sys
from sensor_msgs.msg import Image

# Create a simple test pattern
img = Image()
img.width = 8
img.height = 8
img.encoding = "rgb8"
img.step = 24  # 8 pixels * 3 bytes

# Create pattern based on argument
pattern = sys.argv[1] if len(sys.argv) > 1 else "red"

data = []
for y in range(8):
    for x in range(8):
        if pattern == "red":
            data.extend([255, 0, 0])  # Red
        elif pattern == "green":
            data.extend([0, 255, 0])  # Green
        elif pattern == "blue":
            data.extend([0, 0, 255])  # Blue
        elif pattern == "white":
            data.extend([255, 255, 255])  # White
        elif pattern == "cross":
            if x == y or x == 7-y:
                data.extend([255, 0, 0])  # Red cross
            else:
                data.extend([0, 0, 0])  # Black
        else:
            data.extend([0, 0, 0])  # Black

img.data = data
print(img)
