#! /usr/bin/env python3
# Lawn mower pattern navigation script
# Drives in a back-and-forth pattern to cover an area

import argparse
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run a lawn mower coverage pattern with goToPose goals.'
    )
    parser.add_argument('--rows', type=int, default=10, help='Number of rows to mow')
    parser.add_argument(
        '--length',
        type=float,
        default=0.13,
        help='Length of each row in meters',
    )
    parser.add_argument(
        '--spacing',
        type=float,
        default=0.13,
        help='Spacing between rows in meters',
    )
    return parser.parse_args()

def create_pose(navigator, x, y, yaw):
    """
    Create a PoseStamped message with the given position and orientation.
    
    Args:
        navigator: BasicNavigator instance
        x: X position in meters
        y: Y position in meters
        yaw: Orientation in radians
    
    Returns:
        PoseStamped message
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    # Convert yaw to quaternion
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    return pose


def execute_goal(navigator, goal_pose, goal_name):
    """
    Execute a navigation goal and wait for completion.
    
    Args:
        navigator: BasicNavigator instance
        goal_pose: PoseStamped goal
        goal_name: String description of the goal
    
    Returns:
        True if succeeded, False otherwise
    """
    print(f'Executing: {goal_name}')
    navigator.goToPose(goal_pose)
    
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            print(f'  Distance remaining: {feedback.distance_remaining:.2f}m')
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'  ✓ {goal_name} succeeded!')
        return True
    elif result == TaskResult.CANCELED:
        print(f'  ✗ {goal_name} was canceled!')
        return False
    elif result == TaskResult.FAILED:
        error_code, error_msg = navigator.getTaskError()
        print(f'  ✗ {goal_name} failed! {error_code}: {error_msg}')
        return False
    else:
        print(f'  ✗ {goal_name} has an invalid return status!')
        return False


def main() -> None:
    args = parse_args()

    rclpy.init()

    navigator = BasicNavigator()
    
    # Wait for navigation to fully activate
    print('Waiting for Nav2 to activate...')
    # navigator.waitUntilNav2Active()
    print('Nav2 is active!')

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    navigator.setInitialPose(initial_pose)
    print('Initial pose set at origin')

    # Lawn mower parameters
    row_length = args.length
    row_spacing = args.spacing
    num_rows = args.rows
    
    print(f'\nStarting lawn mower pattern:')
    print(f'  Row length: {row_length}m')
    print(f'  Row spacing: {row_spacing}m')
    print(f'  Number of rows: {num_rows}')
    print()

    # Starting position and orientation
    current_x = 0.0
    current_y = 0.0
    current_yaw = 0.0  # Facing in +X direction (0 radians)
    
    # Direction toggle (1 for forward, -1 for backward along X)
    direction = 1
    
    for row in range(num_rows):
        print(f'\n=== Row {row + 1}/{num_rows} ===')
        
        # Plan the heading at the end of the row so drive + turn happen in one goal.
        turn_sign = -1 if direction == 1 else 1
        row_end_yaw = current_yaw
        if row < num_rows - 1:
            row_end_yaw += turn_sign * (math.pi / 2)

        current_x += direction * row_length
        goal_pose = create_pose(navigator, current_x, current_y, row_end_yaw)
        if not execute_goal(navigator, goal_pose, f'Row {row + 1}: Drive {row_length}m + turn'):
            print('Navigation failed, aborting lawn mower pattern')
            break

        current_yaw = row_end_yaw

        # Don't do the shift on the last row.
        if row < num_rows - 1:
            # Shift by one row spacing and include the second 90-degree turn.
            current_y -= row_spacing
            current_yaw += turn_sign * (math.pi / 2)
            goal_pose = create_pose(navigator, current_x, current_y, current_yaw)

            if not execute_goal(navigator, goal_pose, f'Shift {row_spacing}m to next row'):
                print('Navigation failed, aborting lawn mower pattern')
                break

            # Toggle direction for next row (creates back-and-forth pattern)
            direction *= -1
    
    print('\n' + '='*50)
    print('Lawn mower pattern completed!')
    print('='*50)
    
    # Optionally return to origin
    # print('\nReturning to origin...')
    # return_pose = create_pose(navigator, 0.0, 0.0, 0.0)
    # execute_goal(navigator, return_pose, 'Return to origin')
    
    exit(0)


if __name__ == '__main__':
    main()
