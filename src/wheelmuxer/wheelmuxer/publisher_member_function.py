# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import Float32


# class WheelPositionSubscriber(Node):
#     def __init__(self):
#         super().__init__('wheel_position_subscriber')
#         self.subscription_left = self.create_subscription(
#             Float32,
#             'left_wheel_pos',
#             self.listener_callback_left,
#             10)
#         self.subscription_right = self.create_subscription(
#             Float32,
#             'right_wheel_pos',
#             self.listener_callback_right,
#             10)
#         self.subscription_left  # prevent unused variable warning
#         self.subscription_right  # prevent unused variable warning

#     def listener_callback_left(self, msg):
#         self.get_logger().info('Left wheel position: "%s"' % msg.data)


#     def listener_callback_right(self, msg):
#         self.get_logger().info('Right wheel position: "%s"' % msg.data)
class WheelPositionPublisher(Node):
    def __init__(self):
        super().__init__('wheel_position_publisher')
        self.publisher_ = self.create_publisher(JointState, 'wheelmux', 10)
        self.subscription_left = self.create_subscription(
            Float32,
            'left_wheel_pos',
            self.listener_callback_left,
            10)
        
        self.subscription_right = self.create_subscription(
            Float32,
            'right_wheel_pos',
            self.listener_callback_right,
            10)
        self.subscription_left  # prevent unused variable warning
        self.subscription_right  # prevent unused variable warning
        self.dataWheel = [0,0]
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback_left(self, msg):
        self.get_logger().info('Left wheel position: "%s"' % msg.data)
        self.dataWheel[0] = msg.data

    def listener_callback_right(self, msg):
        self.get_logger().info('Right wheel position: "%s"' % msg.data)
        self.dataWheel[1] = msg.data


    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']  
        msg.position = [self.dataWheel[0], self.dataWheel[1]]  
        # msg.position = [self.i, self.i]
        msg.velocity = [0,0] # Optionally add velocity data
        msg.effort = [0.0, 0.0]   # Optionally add effort data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    wheel_position_publisher = WheelPositionPublisher()
    rclpy.spin(wheel_position_publisher)
    wheel_position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()