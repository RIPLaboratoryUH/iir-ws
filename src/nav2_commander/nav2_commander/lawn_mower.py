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


def main() -> None:
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
    row_length = 3.0       # Drive forward 1 meter per row
    row_spacing = 0.1      # 10 cm spacing between rows
    num_rows = 5           # Number of rows to mow
    
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
        
        # 1. Drive forward 1 meter in current direction
        current_x += direction * row_length
        goal_pose = create_pose(navigator, current_x, current_y, current_yaw)
        
        if not execute_goal(navigator, goal_pose, f'Row {row + 1}: Drive {row_length}m'):
            print('Navigation failed, aborting lawn mower pattern')
            break
        
        # Don't do the turn and shift on the last row
        if row < num_rows - 1:
            # 2. Turn 90 degrees (left if going forward, right if going backward)
            if direction == 1:
                current_yaw += math.pi / 2  # Turn left (counterclockwise)
            else:
                current_yaw -= math.pi / 2  # Turn right (clockwise)
            
            goal_pose = create_pose(navigator, current_x, current_y, current_yaw)
            
            if not execute_goal(navigator, goal_pose, f'Turn 90° (yaw={math.degrees(current_yaw):.0f}°)'):
                print('Navigation failed, aborting lawn mower pattern')
                break
            
            # 3. Drive forward the row spacing distance
            current_y += row_spacing
            goal_pose = create_pose(navigator, current_x, current_y, current_yaw)
            
            if not execute_goal(navigator, goal_pose, f'Shift {row_spacing}m to next row'):
                print('Navigation failed, aborting lawn mower pattern')
                break
            
            # 4. Turn 90 degrees again (same direction as before)
            if direction == 1:
                current_yaw += math.pi / 2  # Turn left again
            else:
                current_yaw -= math.pi / 2  # Turn right again
            
            goal_pose = create_pose(navigator, current_x, current_y, current_yaw)
            
            if not execute_goal(navigator, goal_pose, f'Turn 90° (yaw={math.degrees(current_yaw):.0f}°)'):
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
