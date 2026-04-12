from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command,FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Declare Launch Arguments
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Start RViz2',
    )

    # declare_use_mock_hardware = DeclareLaunchArgument(
    #     'use_mock_hardware',
    #     default_value='false',
    #     descroption='Start the robot with mock hardware'
    # )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true' 
    )

    # Create Launch Config 

    gui = LaunchConfiguration('gui')
    # use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("iir_base"), "config", "iirbot_view.rviz"]
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_gui_cmd)
    # ld.add_action(declare_use_mock_hardware)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_rviz_cmd)

    return ld