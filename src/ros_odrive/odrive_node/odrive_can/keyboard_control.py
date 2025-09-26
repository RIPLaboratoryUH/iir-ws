#!/usr/bin/env python3
#modify so that it publishes to a seperate topic for each motor. the format is thus:
#ros2 topic pub /odrive_axis16/control_message odrive_can/msg/ControlMessage "{control_mode: 2, input_mode: 1, input_pos: 0.0, input_vel: 1.0, input_torque: 0.0}"
#ros2 topic pub /odrive_axis19/control_message odrive_can/msg/ControlMessage "{control_mode: 2, input_mode: 1, input_pos: 0.0, input_vel: 1.0, input_torque: 0.0}"

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage
from std_msgs.msg import Empty
import termios
import tty
import sys
from threading import Lock
import signal

msg = """
ODrive Keyboard Control
---------------------------
Moving around:
   w    i    e
   a    s    d
   z    x    c

w/x : increase/decrease velocity for both motors
a/d : turn left/right
i/s : stop movement
e/c : increase/decrease turning speed
q : quit
"""

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        # Publishers for each motor
        self.control_pub_16 = self.create_publisher(ControlMessage, '/odrive_axis16/control_message', 10)
        self.control_pub_19 = self.create_publisher(ControlMessage, '/odrive_axis19/control_message', 10)
        
        # Control parameters
        self.velocity = 0.0
        self.turn_rate = 0.5  # Scale factor for turning
        self.max_velocity = 10.0  # Maximum velocity limit
        self.control_mode = 2  # Velocity control mode
        self.input_mode = 1    # Velocity input mode
        
        # Thread safety
        self.lock = Lock()
        
        self.get_logger().info(msg)
        
    def publish_control(self, left_vel, right_vel):
        # Create message for left motor (axis 16)
        msg_left = ControlMessage()
        msg_left.control_mode = self.control_mode  # Velocity control
        msg_left.input_mode = self.input_mode      # Velocity input
        msg_left.input_vel = left_vel   # Set velocity
        msg_left.input_pos = 0.0  # Not used in velocity mode
        msg_left.input_torque = 0.0  # No torque feedforward
        
        # Create message for right motor (axis 19)
        msg_right = ControlMessage()
        msg_right.control_mode = self.control_mode
        msg_right.input_mode = self.input_mode
        msg_right.input_vel = right_vel
        msg_right.input_pos = 0.0
        msg_right.input_torque = 0.0
        
        with self.lock:
            self.control_pub_16.publish(msg_left)
            self.control_pub_19.publish(msg_right)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = KeyboardControl()
    
    try:
        while True:
            key = get_key(settings)
            
            if key == 'w':
                node.velocity = min(node.velocity + 0.5, node.max_velocity)
                node.publish_control(node.velocity, -node.velocity)
            elif key == 'x':
                node.velocity = max(node.velocity - 0.5, -node.max_velocity)
                node.publish_control(node.velocity, -node.velocity)
            elif key == 'a':
                left_vel = node.velocity * (1 - node.turn_rate)
                right_vel = node.velocity * (1 + node.turn_rate)
                node.publish_control(left_vel, -right_vel)
            elif key == 'd':
                left_vel = node.velocity * (1 + node.turn_rate)
                right_vel = node.velocity * (1 - node.turn_rate)
                node.publish_control(left_vel, -right_vel)
            elif key == 'i' or key == 's':
                node.velocity = 0.0
                node.publish_control(0.0, 0.0)
            elif key == 'e':
                node.turn_rate = min(node.turn_rate + 0.1, 1.0)
                node.get_logger().info(f'Turn rate: {node.turn_rate:.1f}')
            elif key == 'c':
                node.turn_rate = max(node.turn_rate - 0.1, 0.0)
                node.get_logger().info(f'Turn rate: {node.turn_rate:.1f}')
            elif key == 'q':
                node.publish_control(0.0, 0.0)  # Stop before quitting
                break

    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    
    finally:
        # Reset terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()