#! /usr/bin/env python3
# Lawn mower pattern using deterministic behavior actions:
# drive straight for row length, turn 90 deg in place, shift by spacing, turn 90 deg.

import argparse
import math
import time

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run a precise lawn-mower pattern with straight drives and 90-deg turns.'
    )
    parser.add_argument('--rows', type=int, default=5, help='Number of long strips')
    parser.add_argument('--length', type=float, default=2.0, help='Long strip length (m)')
    parser.add_argument('--spacing', type=float, default=0.1, help='Strip spacing (m)')
    parser.add_argument('--speed', type=float, default=0.10, help='Drive speed (m/s)')
    parser.add_argument(
        '--turn-angle-deg',
        type=float,
        default=90.0,
        help='In-place turn angle in degrees for each corner',
    )
    parser.add_argument(
        '--turn-bias-deg',
        type=float,
        default=0.0,
        help='Bias added to each commanded turn angle to compensate for under/overshoot',
    )
    parser.add_argument(
        '--turn-dir',
        choices=['right', 'left'],
        default='right',
        help='Initial 90-deg turn direction after first long strip',
    )
    parser.add_argument(
        '--segment-timeout',
        type=float,
        default=30.0,
        help='Time allowance per drive segment in seconds',
    )
    parser.add_argument(
        '--turn-timeout',
        type=float,
        default=15.0,
        help='Time allowance per 90-deg turn in seconds',
    )
    parser.add_argument(
        '--settle-time',
        type=float,
        default=0.5,
        help='Pause after each turn to let the robot settle before the next segment',
    )
    return parser.parse_args()


def wait_for_task(navigator: BasicNavigator, label: str) -> bool:
    while not navigator.isTaskComplete():
        time.sleep(0.05)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'{label}: succeeded')
        return True
    if result == TaskResult.CANCELED:
        print(f'{label}: canceled')
        return False
    if result == TaskResult.FAILED:
        error_code, error_msg = navigator.getTaskError()
        print(f'{label}: failed ({error_code}: {error_msg})')
        return False

    print(f'{label}: unknown status')
    return False


def execute_sequence(navigator: BasicNavigator, args: argparse.Namespace) -> bool:
    base_sign = -1.0 if args.turn_dir == 'right' else 1.0
    turn_angle_rad = math.radians(args.turn_angle_deg + args.turn_bias_deg)
    segment_timeout_sec = max(1, math.ceil(args.segment_timeout))
    turn_timeout_sec = max(1, math.ceil(args.turn_timeout))

    for row in range(args.rows):
        long_label = f'row {row + 1}/{args.rows} drive {args.length:.2f}m'
        navigator.driveOnHeading(
            dist=args.length,
            speed=args.speed,
            time_allowance=segment_timeout_sec,
        )
        if not wait_for_task(navigator, long_label):
            return False

        if row == args.rows - 1:
            break

        # Alternate left/right pairs every row to create serpentine motion.
        turn_sign = base_sign if row % 2 == 0 else -base_sign
        turn_amount = turn_sign * turn_angle_rad

        navigator.spin(spin_dist=turn_amount, time_allowance=turn_timeout_sec)
        if not wait_for_task(navigator, f'row {row + 1} first 90-deg turn'):
            return False
        time.sleep(args.settle_time)

        navigator.driveOnHeading(
            dist=args.spacing,
            speed=args.speed,
            time_allowance=segment_timeout_sec,
        )
        if not wait_for_task(navigator, f'row {row + 1} spacing shift {args.spacing:.2f}m'):
            return False

        navigator.spin(spin_dist=turn_amount, time_allowance=turn_timeout_sec)
        if not wait_for_task(navigator, f'row {row + 1} second 90-deg turn'):
            return False
        time.sleep(args.settle_time)

    return True


def main() -> None:
    args = parse_args()
    rclpy.init()
    navigator = BasicNavigator()

    print('Waiting for Nav2 to activate...')
    # Odometry-only setup: no AMCL lifecycle check.
    time.sleep(3)
    print('Starting lawn mower pattern...')

    print(
        f'Starting precise lawn_mower: rows={args.rows}, length={args.length:.2f}, '
        f'spacing={args.spacing:.2f}, speed={args.speed:.2f}, '
        f'turn_angle_deg={args.turn_angle_deg:.1f}, turn_bias_deg={args.turn_bias_deg:.1f}, '
        f'settle_time={args.settle_time:.2f}, turn_dir={args.turn_dir}'
    )

    try:
        ok = execute_sequence(navigator, args)
        if ok:
            print('Lawn mower mission succeeded.')
        else:
            print('Lawn mower mission failed.')
    except KeyboardInterrupt:
        print('Ctrl-C detected, canceling active task...')
        try:
            navigator.cancelTask()
        except Exception as exc:
            print(f'Cancel request failed: {exc}')
        print('Stop requested; exiting.')
    finally:
        rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()
