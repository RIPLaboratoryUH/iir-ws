#!/usr/bin/env python3
import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class FramePublisher(Node):

    def __init__(self):
        super().__init__('iir_tf2_frame_publisher')

      
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a iir odom topic and call handle_iir_odom
        # callback function on each message
        self.subscription = self.create_subscription(
            Odometry,
            f'diff_drive_controller/odom',
            self.handle_iir_odom,
            1)
        self.subscription  # prevent unused variable warning

    def handle_iir_odom(self,msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link' #try odom?
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x =msg.pose.pose.orientation.x
        t.transform.rotation.y =msg.pose.pose.orientation.y
        t.transform.rotation.z =msg.pose.pose.orientation.z
        t.transform.rotation.w =msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    print("init")
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()