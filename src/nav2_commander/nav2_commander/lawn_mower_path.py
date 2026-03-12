#! /usr/bin/env python3
# Lawn mower pattern navigation script
# Drives in a back-and-forth pattern to cover an area

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
import time

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
    

def iter_lawn_mower_waypoints(navigator, num_rows, row_length, row_spacing):
    # Start at origin facing +X (yaw = 0).
    x = 0.0
    y = 0.0
    yaw = 0.0
    turn_sign = -1.0  # first corner is a right turn

    for row in range(num_rows):
        # 1) Drive forward along current heading by row_length.
        x += row_length * math.cos(yaw)
        y += row_length * math.sin(yaw)
        yield create_pose(navigator, x, y, yaw), f'row {row + 1} long strip'

        # No corner maneuver after the last long strip.
        if row == num_rows - 1:
            break

        # 2) Turn 90 deg in place.
        yaw += turn_sign * (math.pi / 2.0)
        yield create_pose(navigator, x, y, yaw), f'row {row + 1} first corner turn'

        # 3) Drive forward by row_spacing AND turn 90 deg to align with next long strip (combined move+turn).
        x += row_spacing * math.cos(yaw)
        y += row_spacing * math.sin(yaw)
        yaw += turn_sign * (math.pi / 2.0)
        yield create_pose(navigator, x, y, yaw), f'row {row + 1} spacing shift and turn'

        # Alternate turn direction each strip pair to make a serpentine path.
        turn_sign *= -1.0

def main() -> None:
    num_rows = 5
    row_length = 2.0       # Drive forward 2 meters per row
    row_spacing = 0.1      # 10 cm spacing between rows
    post_goal_delay = 0.3  # Brief settle time after each completed segment
    
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

    # Send each waypoint individually so Nav2 must complete each segment in order.
    for pose, goal_name in iter_lawn_mower_waypoints(navigator, num_rows, row_length, row_spacing):
        if not execute_goal(navigator, pose, goal_name):
            print('Mission aborted due to failed segment.')
            return
        time.sleep(post_goal_delay)

    print('Lawn mower mission complete.')


if __name__ == '__main__':
    main()