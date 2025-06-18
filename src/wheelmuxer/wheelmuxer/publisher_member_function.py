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
from geometry_msgs.msg import Vector3
from   rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

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
        self.leftprev = 0
        self.rightprev = 0
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(JointState, 'wheelmux', 100)
        self.subscription = self.create_subscription(
            Vector3,
            'wheel_poses',
            self.listener_callback,
            qos_profile)

        self.dataWheel = [0,0]
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg):
        self.dataWheel[0] = msg.y
        if abs((self.leftprev - msg.y)) > 100:
            self.get_logger().info('Left wheel position: "%s"' % msg.y)
            self.get_logger().info('if you see this, something is probably wrong. wheelmuxer detected a large jump in wheel position')
        self.leftprev = msg.y
        self.dataWheel[1] = msg.z
        if abs((self.rightprev - msg.z)) > 100:
          self.get_logger().info('Right wheel position: "%s"' % msg.z)
          self.get_logger().info('if you see this, something is probably wrong. wheelmuxer detected a large jump in wheel position')

        self.rightprev = msg.z


    def timer_callback(self):
        if self.i == 0:
            self.get_logger().info('WheelPositionPublisher is running')
            self.get_logger().info('Publishing wheel positions to "/wheelmux" topic, as a JointState message')
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']  
        msg.position = [self.dataWheel[0], self.dataWheel[1]]  
        # msg.position = [self.i, self.i]
        msg.velocity = [0,0] # Optionally add velocity data
        msg.effort = [0.0, 0.0]   # Optionally add effort data
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.position)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    wheel_position_publisher = WheelPositionPublisher()
    rclpy.spin(wheel_position_publisher)
    wheel_position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()