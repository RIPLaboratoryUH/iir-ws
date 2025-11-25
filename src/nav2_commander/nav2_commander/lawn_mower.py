#! /usr/bin/env python3
# Lawn mower pattern navigation script
# Drives in a back-and-forth pattern to cover an area

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math

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
    

def create_poses(navigator, num_rows, row_length, row_spacing):
    poses = []
    for i in range(num_rows):
        if(i %2 == 0):
            poses.append(create_pose(navigator,row_spacing*i,row_length,0.0))
            poses.append(create_pose(navigator,row_spacing*(i+1),row_length,0.0))
        else:
            poses.append(create_pose(navigator,row_spacing*i,0.0,0.0))
            poses.append(create_pose(navigator,row_spacing*(i+1),0.0,0.0)) 
    return poses

def main() -> None:
    num_rows = 5
    row_length = 3.0       # Drive forward 3 meters per row
    row_spacing = 0.1      # 10 cm spacing between rows
    
    rclpy.init()
    navigator = BasicNavigator()
    poses = create_poses(navigator, num_rows,row_length,row_spacing)
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

    navigator.goThroughPoses(poses)
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            print(f'  Distance remaining: {feedback.distance_remaining:.2f}m')
    exit(0)


if __name__ == '__main__':
    main()
