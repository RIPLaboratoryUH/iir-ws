#! /usr/bin/env python3
# Script to spin the robot 360 degrees N times

import argparse
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Spin the robot 360 degrees N times.'
    )
    parser.add_argument(
        '--count',
        type=int,
        default=1,
        help='Number of 360-degree spins to perform',
    )
    return parser.parse_args()

def main() -> None:
    args = parse_args()
    rclpy.init()

    navigator = BasicNavigator()

    # Wait for navigation to fully activate
    print('Waiting for Nav2 to activate...')
    # navigator.waitUntilNav2Active()
    print('Nav2 is active!')

    spins = args.count

    print(f'Starting {spins} 360-degree spins.')

    try:
        # Number of segments to break one 360 rotation into
        segments = 4
        
        for i in range(spins):
            print(f'Executing spin {i + 1}/{spins} using a path of poses')
            #  # 2 * pi radians is one full 360-degree spin
            # spin_dist = 2.0 * math.pi
            # navigator.spin(spin_dist=spin_dist, time_allowance=800)

            for j in range(1, segments + 1):
                # Calculate yaw for this segment (e.g. 90, 180, 270, 360 degrees)
                yaw = (2.0 * math.pi / segments) * j
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = navigator.get_clock().now().to_msg()
                pose.pose.position.x = 0.0
                pose.pose.position.y = 0.0
                pose.pose.position.z = 0.0
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                print(f"  -> Going to segment {j} (yaw: {math.degrees(yaw):.1f} deg)")
                navigator.goToPose(pose)

                # Wait for segment completion
                while not navigator.isTaskComplete():
                    pass

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'  ✓ Spin {i + 1} succeeded!')
            elif result == TaskResult.CANCELED:
                print(f'  ✗ Spin {i + 1} was canceled!')
                break
            elif result == TaskResult.FAILED:
                # BasicNavigator in older versions might just return FAILED, while newer returns (code, msg) via getTaskError()
                print(f'  ✗ Spin {i + 1} failed!')
                break
            else:
                print(f'  ✗ Spin {i + 1} has an invalid return status!')
                break
                
    finally:
        print('Finished spinning.')
        navigator.lifecycleShutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
