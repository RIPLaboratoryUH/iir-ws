#!/usr/bin/env python3
# Navigate the robot back to the odom origin (0, 0) — i.e. where it started.

import argparse
import math

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Send a Nav2 goal in map frame (defaults to origin).'
    )
    parser.add_argument('--x', type=float, default=0.0, help='Goal x position (m)')
    parser.add_argument('--y', type=float, default=0.0, help='Goal y position (m)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw (rad)')
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    navigator = BasicNavigator()

    print('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = args.x
    goal.pose.position.y = args.y
    goal.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal.pose.orientation.w = math.cos(args.yaw / 2.0)

    print(f'Sending goal: x={args.x:.3f}, y={args.y:.3f}, yaw={args.yaw:.3f} rad')
    navigator.goToPose(goal)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            print(
                f'Distance remaining: {feedback.distance_remaining:.2f} m  |  '
                f'ETA: {feedback.estimated_time_remaining}'
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Arrived home!')
    elif result == TaskResult.CANCELED:
        print('Navigation was canceled.')
    elif result == TaskResult.FAILED:
        error_code, error_msg = navigator.getTaskError()
        print(f'Navigation failed — {error_code}: {error_msg}')
    else:
        print('Navigation returned an unknown status.')

    exit(0)


if __name__ == '__main__':
    main()
