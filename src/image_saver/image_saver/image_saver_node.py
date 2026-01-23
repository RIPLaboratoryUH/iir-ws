import os
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # Create save directory
        self.save_dir = Path.home() / 'image_saves'
        self.save_dir.mkdir(exist_ok=True)
        self.get_logger().info(f'Image save directory: {self.save_dir}')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Current image storage
        self.current_image = None
        self.current_timestamp = None
        self.image_lock = False
        
        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        
        # Create timer to save image every 3 seconds
        self.timer = self.create_timer(3.0, self.save_current_image)
        
        self.get_logger().info('Image Saver Node initialized. Saving snapshots every 3 seconds.')
    
    def image_callback(self, msg: CompressedImage):
        """Callback function for compressed image subscription"""
        try:
            # Convert compressed ROS image to OpenCV format
            self.current_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Store the ROS timestamp
            self.current_timestamp = msg.header.stamp
            self.image_lock = False
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def save_current_image(self):
        """Save the current image with timestamp"""
        self.get_logger().info('Saving current image...')
        if self.current_image is None:
            self.get_logger().warn('No image available to save yet.')
            return
        
        if self.image_lock:
            self.get_logger().warn('Already saving image, please wait...')
            return
        
        self.image_lock = True
        
        try:
            # Create filename with ROS timestamp
            ros_sec = self.current_timestamp.sec
            ros_nanosec = self.current_timestamp.nanosec
            timestamp_str = f'{ros_sec}_{ros_nanosec:09d}'
            
            # Also add human-readable timestamp
            now = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            filename = f'image_{now}_{timestamp_str}.png'
            filepath = self.save_dir / filename
            
            # Save image
            cv2.imwrite(str(filepath), self.current_image)
            
            self.get_logger().info(
                f'Image saved: {filename}\n'
                f'  ROS timestamp: {ros_sec}.{ros_nanosec:09d}s'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')
        finally:
            self.image_lock = False


def main(args=None):
    rclpy.init(args=args)
    
    node = ImageSaverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
