import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
        DeclareLaunchArgument,
        ExecuteProcess,
        IncludeLaunchDescription,
)

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
        LaunchConfiguration,
        PathJoinSubstitution,
        Command,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch Directories
    
    sim_dir = get_package_share_directory('iir_sim')
    desc_dir = get_package_share_directory('iir_base')
    
    launch_dir = os.path.join(sim_dir, 'launch')

    # Launch Configs

    nav2_config_file = LaunchConfiguration('nav2_config_file')
    robot_sdf = LaunchConfiguration('robot_sdf')
    
    declare_nav2_config_file_cmd = DeclareLaunchArgument(
            'nav2_config_file',
            default_value=os.path.join(desc_dir, 'config', 'nav2_default_view.rviz'),
            description='Full path to the Rviz2 config for Nav2'
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(desc_dir, 'urdf', 'iirbot.urdf.xacro'),
            description='Full path to the robot urdf/sdf file'
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('iir_base'),
            'config',
            'diffbot_controllers.yaml',
        ]
    )

    # Get URDF via xacro w/ mock hardware

    robot_description = Command([
        'xacro ', robot_sdf,
        ' mock_hardware:=true',
    ])


    start_controller_manager_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': ParameterValue(robot_description, value_type=str),
            },
            robot_controllers
        ]
    )



    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': ParameterValue(robot_description, value_type=str)
            }
        ]
    )

    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', nav2_config_file],
            parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_nav2_config_file_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_controller_manager_cmd)
    ld.add_action(rviz_cmd)

    return ld
