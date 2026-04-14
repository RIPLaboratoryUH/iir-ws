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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


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
            "use_lidar",
            default_value="false",
            description="activates URG node from lidar_launch, adds lidar to URDF"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_display_reader",
            default_value="true",
            description="activates display_reader node for number recognition"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "display_reader_exposure",
            default_value="-3.0",
            description="camera exposure for display reader (-1 = auto)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_arducam_tof",
            default_value="false",
            description="activates Arducam TOF camera pointcloud node"
        )
    )
    declared_arguments.append( #run on groundstation but not on robot
        DeclareLaunchArgument(
            "use_display_marker",
            default_value="false",
            description="activates display_marker node for visualizing readings on map"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_on_wall",
            default_value="false",
            description="sets the is_on_wall argument for the URDF, which changes the location of the IMU"
        )
    )
    
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    is_on_wall = LaunchConfiguration("is_on_wall")
    
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
            " ",
            "is_on_wall:=",
            is_on_wall,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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

        parameters=[robot_description, robot_controllers,{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_mock_hardware)
            

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

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="link_broad",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'

    )
    micro_ros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0'],
        output='screen'
    )
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_params, {'use_sim_time' : use_sim_time}]
       )
    lidar = Node(
            package="urg_node",
            executable="urg_node_driver",
            name="urg_node_driver",
            output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidar"))
    )

    display_reader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("display_reader"),
                "launch",
                "display_reader.launch.py"
            ])
        ),
        launch_arguments={
            'exposure': LaunchConfiguration('display_reader_exposure')
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_display_reader"))
    )

    arducam_tof = Node(
        package="arducam_rclpy_tof_pointcloud",
        executable="tof_pointcloud",
        name="arducam_tof_pointcloud",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_arducam_tof"))
    )

    display_marker = Node(
        package="display_marker",
        executable="display_marker_node",
        name="display_marker_node",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_display_marker"))
    )

    #ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -p min_height:=0.0 -p max_height:=5.0  -r  cloud_in:=/point_cloud
    
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        output="screen",
        parameters=[{'min_height': 0.0}, {'max_height': 1.0}],
        remappings=[('cloud_in', '/point_cloud')]
    )

    nodes = [
        static_transform_publisher,
        control_node,
        robot_state_pub_node,
        robot_localization_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        joint_state_publisher,
        display_reader,
        arducam_tof,
        display_marker,
        # pointcloud_to_laserscan
    ]

    return LaunchDescription(declared_arguments + nodes)
