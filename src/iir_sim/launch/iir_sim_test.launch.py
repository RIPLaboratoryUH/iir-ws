import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression
)

from launch_ros.actions import Node

def generate_launch_description():

    # Launch Directory

    sim_dir = get_package_share_directory('iir_sim')
    desc_dir = get_package_share_directory('iir_base')
    launch_dir = os.path.join(sim_dir, 'launch')

    # Create Launch Config Variables

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Config for SIM

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    robot_sdf = LaunchConfiguration('robot_sdf')
    # nav2_params_file = LaunchConfiguration('nav2_params_file')

    pose = {
        'x': LaunchConfiguration('x_pose', default='-8.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00')
    }
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='If true use Gazebo time'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value='/home/riplab/GIT/iir-ws/src/IIR_base/config/nav2_default_view.rviz',
        # default_value=os.path.join('IIR_base', 'config', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ2 config'
    )

    # declare_nav2_params_file_cmd = DeclareLaunchArgument(
    #     'nav2_config_file',
    #     default_value=os.path.join(desc_dir, 'config', )
    # )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='To start robot state publisher'
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value='/home/riplab/GIT/iir-ws/src/IIR_base/urdf/iirbot.urdf.xacro',
        # default_value=os.path.join(desc_dir, 'urdf', 'iirbot.urdf.xacro'),
        description='Full path to the robot urdf/sdf file'
    )

    robot_description_with_mock_hardware = Command([
        'xacro', ' ', robot_sdf,
        ' ', 'mock_hardware:=true',
    ])

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description_with_mock_hardware, value_type=str)
            }
        ]
    )

    # start_control_node_cmd = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         {
    #             'use_sim_time': use_sim_time,
    #             'robot_description': ParameterValue(robot_description_with_mock_hardware, value_type=str)
    #             'robot_controllers '
    #         }
    #     ]
    # )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
            ),
        launch_arguments={'gz_args': '-r -s'}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-v4 -g ']}.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[
            {
                'config_file': os.path.join(
                    desc_dir, 'config', 'gz_ros_bridge.yaml'
                ),
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
    )

    gz_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'iir',
            '-topic', 'robot_description',
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    # gz_robot = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, '')
    #     )
    # )

    # Create Launch Description with Population

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    ld.add_action(bridge)
    ld.add_action(gz_robot)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)


    return ld
