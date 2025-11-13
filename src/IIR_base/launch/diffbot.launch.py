# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="use simulation clock, set true if simulating w/gz"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_keyboard_twist",
            default_value="false",
            description="publish cmd_vel from keyboard"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joy_twist",
            default_value="false",
            description="publish cmd vel from joystick"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
            description="activates URG node from lidar_launch, adds lidar to URDF"
        )
    )
    
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_keyboard_twist = LaunchConfiguration("use_keyboard_twist")
    use_joy_twist = LaunchConfiguration("use_joy_twist")

    # joy = LaunchConfiguration
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("iir_base"), "urdf", "iirbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("iir_base"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("iir_base"), "config", "iirbot_view.rviz"]
    )
    nav_rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("iir_base"), "config", "nav2_default_view.rviz"]
    )

    bridge_params = PathJoinSubstitution(
        [
    FindPackageShare('iir_base'),
    'config',
    'gz_ros_bridge.yaml'
])
    
    ekf_params = PathJoinSubstitution(
        [
    FindPackageShare('iir_base'),
    'config',
    'ekf.yaml'
])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",

        parameters=[{'robot_description': robot_description}, robot_controllers,{'use_sim_time': use_sim_time}],
        condition=(IfCondition(use_mock_hardware))
            

    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{'use_sim_time': use_sim_time}],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(gui),
    )
    joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    parameters=[{'use_sim_time': use_sim_time}]
    
)
    timeout = 10
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{'controller_manager_timeout' : timeout}],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", robot_controllers],
        parameters=[{'controller_manager_timeout' : timeout}],

    )

    joystick_node = Node(
        #ros2 run teleop_twist_joy teleop_node --ros-args -r cmd_vel:=diff_drive_controller/cmd_vel -p stamped:=true

        package='teleop_twist_joy',
        executable='teleop_node',
        arguments=['--remap', 'cmd_vel:=cmd_vel_user'],
        condition=IfCondition(use_joy_twist)
    )
    keyboard_node = Node(
        #ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=diff_drive_controller/cmd_vel -p stamped:=true

    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    arguments=['--remap', 'cmd_vel:=cmd_vel_user'],
    condition=IfCondition(use_keyboard_twist),
    output='screen'
)


    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="link_broad",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'

    )
    odom_to_tf = Node(
        package="odom_to_tf_ros2",
        executable="odom_to_tf",
        arguments=['--ros-args', '-p' ,'odom_topic:=diff_drive_controller/odom']
    )
    my_tf_publisher = Node(
        package="iir_base",
        executable="tf2_publish.py"
    )
    micro_ros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0'],
        output='screen'
    )
    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_params},{'use_sim_time': use_sim_time}],
            output='screen'
        )
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_params, {'use_sim_time' : use_sim_time}]
       )
    wheelmuxer = Node(
        package='wheelmuxer',
        executable='talker',
        output='screen'
    )

    picolistener16 = Node(
        package='picolistener',
        executable='listener16',
        output='screen'
    )
    picolistener19 = Node(
                package='picolistener',
        executable='listener19',
        output='screen'
    )

    lidar = Node(
            package="urg_node",
            executable="urg_node_driver",
            name="urg_node_driver",
            output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidar"))
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        # static_transform_publisher,
    #    robot_localization_node,
        control_node, #make it so this is on when using 'mock hardware' and not on when using gz
        robot_state_pub_node,

       # wheelmuxer,
        picolistener16,
        picolistener19,
#         odom_to_tf,
        lidar,
        # micro_ros_node,
        # twist_stamper,
        # joystick_node,
        # keyboard_node,
        # bridge,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        #rviz_node,
        joint_state_publisher,
        # my_tf_publisher,
        
    ]

    return LaunchDescription(declared_arguments + nodes)
