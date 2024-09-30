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
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


    bridge_params = PathJoinSubstitution(
        [
    FindPackageShare('iir_base'),
    'config',
    'gz_ros_bridge.yaml'
])


    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_params}],
            output='screen'
        )
    gz_node = IncludeLaunchDescription( #launches gazebo with ros stuff
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros_gz_sim'),'launch'),
         '/gz_sim.launch.py']),
         launch_arguments={'gz_args':['empty.sdf']}.items()
    )
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            
            '-topic', 'robot_description',
        ]
    )



    nodes = [
        gz_node,
        bridge, 
        spawner
    ]

    return LaunchDescription(declared_arguments + nodes)
