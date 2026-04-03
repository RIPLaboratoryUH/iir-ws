
# Rectangular path:
# 2m forward -> turn 90 -> 1m forward -> turn 90 -> 2m forward -> turn 90 -> 1m forward -> turn 90
# Repeat N times.

import argparse
import math
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Run a rectangular path repeatedly.')
    parser.add_argument('--repeats', type=int, default=3, help='Number of rectangle loops')
    parser.add_argument('--long-side', type=float, default=2.0, help='Long side length (m)')
    parser.add_argument('--short-side', type=float, default=1.0, help='Short side length (m)')
    parser.add_argument('--speed', type=float, default=0.10, help='Drive speed (m/s)')
    parser.add_argument(
        '--turn-angle-deg',
        type=float,
        default=90.0,
        help='Turn angle in degrees at each corner',
    )
    parser.add_argument(
        '--turn-dir',
        choices=['right', 'left'],
        default='right',
        help='Corner turn direction',
    )
    parser.add_argument('--segment-timeout', type=float, default=30.0, help='Drive timeout (s)')
    parser.add_argument('--turn-timeout', type=float, default=15.0, help='Turn timeout (s)')
    parser.add_argument('--settle-time', type=float, default=0.3, help='Pause after turns (s)')
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


def execute_rectangle(navigator: BasicNavigator, args: argparse.Namespace) -> bool:
    turn_sign = -1.0 if args.turn_dir == 'right' else 1.0
    turn_amount = turn_sign * math.radians(args.turn_angle_deg)
    segment_timeout_sec = max(1, math.ceil(args.segment_timeout))
    turn_timeout_sec = max(1, math.ceil(args.turn_timeout))

    for lap in range(args.repeats):
        print(f'Starting rectangle {lap + 1}/{args.repeats}')

        # Side 1: long
        navigator.driveOnHeading(args.long_side, args.speed, segment_timeout_sec)
        if not wait_for_task(navigator, f'lap {lap + 1} side 1 ({args.long_side:.2f}m)'):
            return False

        navigator.spin(turn_amount, turn_timeout_sec)
        if not wait_for_task(navigator, f'lap {lap + 1} turn 1'):
            return False
        time.sleep(args.settle_time)

        # Side 2: short
        navigator.driveOnHeading(args.short_side, args.speed, segment_timeout_sec)
        if not wait_for_task(navigator, f'lap {lap + 1} side 2 ({args.short_side:.2f}m)'):
            return False

        navigator.spin(turn_amount, turn_timeout_sec)
        if not wait_for_task(navigator, f'lap {lap + 1} turn 2'):
            return False
        time.sleep(args.settle_time)

        # Side 3: long
        navigator.driveOnHeading(args.long_side, args.speed, segment_timeout_sec)
        if not wait_for_task(navigator, f'lap {lap + 1} side 3 ({args.long_side:.2f}m)'):
            return False

        navigator.spin(turn_amount, turn_timeout_sec)
        if not wait_for_task(navigator, f'lap {lap + 1} turn 3'):
            return False
        time.sleep(args.settle_time)

        # Side 4: short
        navigator.driveOnHeading(args.short_side, args.speed, segment_timeout_sec)
        if not wait_for_task(navigator, f'lap {lap + 1} side 4 ({args.short_side:.2f}m)'):
            return False

        # Final corner turn so next lap starts with same heading
        navigator.spin(turn_amount, turn_timeout_sec)
        if not wait_for_task(navigator, f'lap {lap + 1} turn 4'):
            return False
        time.sleep(args.settle_time)

    return True


def main() -> None:
    args = parse_args()
    rclpy.init()
    navigator = BasicNavigator()

    print('Waiting for Nav2 to activate...')
    time.sleep(3)

    try:
        ok = execute_rectangle(navigator, args)
        print('Rectangular mission succeeded.' if ok else 'Rectangular mission failed.')
    except KeyboardInterrupt:
        print('Ctrl-C detected, canceling active task...')
        try:
            navigator.cancelTask()
        except Exception as exc:
            print(f'Cancel request failed: {exc}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()