# Copyright 2020 ros2_control Development Team
#
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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
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

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

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

        parameters=[{'robot_description': robot_description}, robot_controllers,{'use_sim_time': True}],
        condition=(IfCondition(use_mock_hardware))
            

    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{'use_sim_time': True}],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", nav_rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(gui),
    )
    joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    parameters=[{'use_sim_time': True}]
    
)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", robot_controllers],
    )

    #static transform publisher - this should be somewhre else but im just testing
    #apparently, this doesnt work here, but if we launch it in a terminal elsewhere it actually does publish the map->odom transform +map frame
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="link_broad",
        arguments=['0', '0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'

    )
    my_tf_publisher = Node(
        package="iir_base",
        executable="tf2_publish.py"
    )
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_params},{'use_sim_time': True}],
            output='screen'
        )
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_params, {'use_sim_time' : True}]
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
        # robot_localization_node,
        control_node, #make it so this is on when using 'mock hardware' and not on when using gz
        robot_state_pub_node,
        # bridge,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        rviz_node,
        joint_state_publisher,
        # my_tf_publisher,
        
    ]

    return LaunchDescription(declared_arguments + nodes)
