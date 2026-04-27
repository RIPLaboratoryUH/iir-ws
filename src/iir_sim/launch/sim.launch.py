import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

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
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    pose = {
        'x': LaunchConfiguration('x_pose', default='0.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    ekf_params = PathJoinSubstitution(
            [
                FindPackageShare('iir_base'),
                'config',
                'ekf.yaml'
            ]
    )


    # Get URDF via xacro w/ mock hardware

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("iir_base"), "urdf", "iirbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            'True',
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{'use_sim_time': True}],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    gazebo_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': '-r -s'}.items()
    )

    gazebo_client = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('roz_gz_sim'),
                    'launch',
                    'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-v4 -g'}.items(),
    )

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[
            {
                'config_file': os.path.join(
                    desc_dir,
                    'config',
                    'gz_ros_bridge.yaml'
                ),
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    spawn_model_cmd = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arugments=[
                '-name', 'iir_robot',
                '-topic', 'robot_description',
                '-x', pose['x'],
                '-y', pose['y'],
                '-z', pose['z'],
                '-R', pose['R'],
                '-P', pose['P'],
                '-Y', pose['Y'],
            ],
            parameters=[{'use_sim_time': True}]
    )

    timeout = 10
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"],
        parameters=[{'controller_manager_timeout': timeout}],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--param-file', robot_controllers],
        parameters=[{'controller_manager_timeout': timeout}],
    )

    robot_localization_cmd = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': True}]
    )

    static_transfrom_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0', 'map', 'odom'],
            output='screen'
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

    # ld.add_action(robot_state_pub_node)
    ld.add_action(control_node)
    ld.add_action(robot_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_cmd)

    return ld
