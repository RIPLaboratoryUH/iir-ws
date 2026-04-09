#! /usr/bin/env python3
# Lawn mower pattern using driveOnHeading + spin primitives.

import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run a lawn mower pattern using driveOnHeading and spin.'
    )
    parser.add_argument('--rows', type=int, default=5, help='Number of rows to mow')
    parser.add_argument(
        '--length',
        type=float,
        default=3.0,
        help='Length of each row in meters',
    )
    parser.add_argument(
        '--spacing',
        type=float,
        default=0.1,
        help='Spacing between rows in meters',
    )
    parser.add_argument(
        '--speed',
        type=float,
        default=0.10,
        help='Linear speed for driveOnHeading in m/s',
    )
    parser.add_argument(
        '--turn-angle-deg',
        type=float,
        default=90.0,
        help='Turn angle at corners in degrees',
    )
    parser.add_argument(
        '--turn-dir',
        choices=['left', 'right'],
        default='left',
        help='Initial corner turn direction',
    )
    parser.add_argument(
        '--segment-timeout',
        type=float,
        default=30.0,
        help='Timeout for each drive segment in seconds',
    )
    parser.add_argument(
        '--turn-timeout',
        type=float,
        default=15.0,
        help='Timeout for each turn in seconds',
    )
    parser.add_argument(
        '--settle-time',
        type=float,
        default=0.20,
        help='Pause between segments in seconds',
    )
    return parser.parse_args()


def wait_for_current_task(navigator: BasicNavigator, task_name: str) -> bool:
    """Wait for the active navigator task and report status."""
    print(f'Executing: {task_name}')

    while not navigator.isTaskComplete():
        time.sleep(0.05)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'  [OK] {task_name} succeeded')
        return True
    if result == TaskResult.CANCELED:
        print(f'  [FAIL] {task_name} canceled')
        return False
    if result == TaskResult.FAILED:
        error_code, error_msg = navigator.getTaskError()
        print(f'  [FAIL] {task_name} failed: {error_code}: {error_msg}')
        return False

    print(f'  [FAIL] {task_name} returned unknown status')
    return False


def set_initial_pose(navigator: BasicNavigator) -> None:
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)


def main() -> None:
    args = parse_args()

    num_rows = args.rows
    row_length = args.length
    row_spacing = args.spacing

    drive_speed = args.speed
    segment_timeout = args.segment_timeout
    turn_timeout = args.turn_timeout
    settle_time = args.settle_time

    rclpy.init()
    navigator = BasicNavigator()

    try:
        print('Waiting for Nav2 to activate...')
        navigator.waitUntilNav2Active()
        print('Nav2 is active!')

        set_initial_pose(navigator)
        print('Initial pose set at origin')

        print('\nStarting lawn mower pattern (driveOnHeading/spin):')
        print(f'  Rows: {num_rows}')
        print(f'  Row length: {row_length}m')
        print(f'  Row spacing: {row_spacing}m')
        print(f'  Drive speed: {drive_speed}m/s')

        # Alternate corner direction each row pair to form a serpentine path.
        turn_sign = 1.0 if args.turn_dir == 'left' else -1.0

        for row in range(num_rows):
            print(f'\n=== Row {row + 1}/{num_rows} ===')

            navigator.driveOnHeading(
                dist=row_length,
                speed=drive_speed,
                time_allowance=segment_timeout,
            )
            if not wait_for_current_task(navigator, f'Drive {row_length}m'):
                print('Navigation failed, aborting lawn mower pattern')
                return
            time.sleep(settle_time)

            if row == num_rows - 1:
                break

            turn_angle = turn_sign * math.radians(args.turn_angle_deg)

            navigator.spin(spin_dist=turn_angle, time_allowance=turn_timeout)
            if not wait_for_current_task(navigator, 'First 90-degree turn'):
                print('Navigation failed, aborting lawn mower pattern')
                return
            time.sleep(settle_time)

            navigator.driveOnHeading(
                dist=row_spacing,
                speed=drive_speed,
                time_allowance=segment_timeout,
            )
            if not wait_for_current_task(navigator, f'Shift {row_spacing}m'):
                print('Navigation failed, aborting lawn mower pattern')
                return
            time.sleep(settle_time)

            navigator.spin(spin_dist=turn_angle, time_allowance=turn_timeout)
            if not wait_for_current_task(navigator, 'Second 90-degree turn'):
                print('Navigation failed, aborting lawn mower pattern')
                return
            time.sleep(settle_time)

            turn_sign *= -1.0

        print('\n' + '=' * 50)
        print('Lawn mower pattern completed!')
        print('=' * 50)

    finally:
        navigator.lifecycleShutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
