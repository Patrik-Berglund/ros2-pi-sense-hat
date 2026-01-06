#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/home/patrik/ros2-pi-sense-hat/install/ros2_pi_sense_hat/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import numpy as np

class ImageDisplayer(Node):
    def __init__(self):
        super().__init__('image_displayer')
        self.publisher = self.create_publisher(Image, '/sense_hat/led_matrix/image', 10)

    def load_and_display_image(self, image_path):
        try:
            # Load image and resize to 8x8
            pil_img = PILImage.open(image_path)
            
            # Handle transparency - create black background
            if pil_img.mode in ('RGBA', 'LA'):
                background = PILImage.new('RGB', pil_img.size, (0, 0, 0))  # Black background
                background.paste(pil_img, mask=pil_img.split()[-1])  # Use alpha channel as mask
                pil_img = background
            else:
                pil_img = pil_img.convert('RGB')
                
            pil_img = pil_img.resize((8, 8), PILImage.NEAREST)
            
            # Convert to numpy array
            img_array = np.array(pil_img)
            
            # Create ROS2 Image message
            ros_img = Image()
            ros_img.width = 8
            ros_img.height = 8
            ros_img.encoding = "rgb8"
            ros_img.step = 24  # 8 pixels * 3 bytes
            
            # Flatten the array and convert to list
            ros_img.data = img_array.flatten().tolist()
            
            # Publish the image
            self.publisher.publish(ros_img)
            self.get_logger().info(f'Published image: {image_path}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load image {image_path}: {e}')
            return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 image_display.py <image_file>")
        print("Example: python3 image_display.py my_image.png")
        return
    
    rclpy.init()
    node = ImageDisplayer()
    
    image_path = sys.argv[1]
    success = node.load_and_display_image(image_path)
    
    if success:
        print(f"Image {image_path} displayed on LED matrix!")
    else:
        print(f"Failed to display image {image_path}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
