#! /usr/bin/env python3
# Square pattern navigation script
# Drive forward and on the door do a square pattern

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math

def create_pose(navigator, x, y, yaw):
    """
    Create a PoseStamped message with the given position and orientation

    Args:
        navigator: BasicNavigator instance
        x: X position in meters
        y: Y position in meters
        yaw: Orientation in radians

    Returns:
        PoseStamped message
    """

    pose = PoseStamped()
    pose.header.frame_id = 'odom'
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
    Execute a navigation goal and wait for completion

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
    
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'odom'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(initial_pose)
    print('Initial pose set at origin')

    forward_distance = 2.0
    x_square = 1.0
    y_square = 0.7

    current_x = 0.0
    current_y = 0.0

    current_x += forward_distance
    goal_pose = create_pose(navigator, current_x, current_y, 0.0)
    if not execute_goal(navigator, goal_pose, 'Move forward'):
        print('Navigation failed')
        rclpy.shutdown()
        return

    starting_x = current_x
    starting_y = current_y

    square_goals = [
        (
            starting_x + x_square,
            starting_y,
            math.pi / 2.0,
            'Drive forward and face left'
        ),

        (
            starting_x + x_square,
            starting_y + y_square,
            math.pi,
            'Drive left and face backward'
        ),

        (
            starting_x,
            starting_y + y_square,
            -math.pi / 2.0,
            'Drive backward and face right'
        ),

        (
            starting_x,
            starting_y,
            0.0,
            'Drive right and face forward'
        ),
    ]

    for x, y, yaw, name in square_goals:
        goal_pose = create_pose(navigator, x, y, yaw)

        if not execute_goal(navigator, goal_pose, name):
            print('Navigation failed, aborting square pattern.')
            rclpy.shutdown()
            return

    print('\nSquare pattern completed!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
