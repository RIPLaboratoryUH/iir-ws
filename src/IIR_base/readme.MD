IIR_base is used for the general launch and upkeep of the robot's state. 

The primary file to examine is launch/diffbot.launch.py. This file is the typical launch file for teleoperation-control. It can be launched using this command: `ros2 launch iir_base diffbot.launch.py` (if looking for a teleop control guide, review the README at the top of this directory.)

## Nodes launched by diffbot.launch.py

| Node | Package | Description |
|------|---------|-------------|
| `static_transform_publisher` | `tf2_ros` | Publishes a static identity transform from `map` → `odom`. Acts as a placeholder so the TF tree is complete, so that NAV2 works properly. NAV2 requires a TF tree with map->odom->base_link transforms |
| `control_node` (ros2_control_node) | `controller_manager` | Manages hardware interfaces and controllers via ROS 2 Control. This relates to the ros2control.xacro file in /urdf. The actual hardware interface can be found in the package at src/ros_odrive |
| `robot_state_pub_node` (robot_state_publisher) | `robot_state_publisher` | Reads the URDF and joint states to publish the full robot TF tree. |
| `robot_localization_node` (ekf_node) | `robot_localization` | Runs an Extended Kalman Filter fusing wheel odometry and IMU data to produce a smooth, drift-corrected pose estimate on the `odom` frame. See config/ekf.yaml for the configuration of this node. |
| `robot_controller_spawner` | `controller_manager` | Spawns the `diff_drive_controller`, which converts `/cmd_vel` Twist messages into individual wheel velocity commands and publishes odometry. The robot will accept Twist messages from the topic `diff_drive_controller/cmd_vel` |
| `joint_state_broadcaster_spawner` | `controller_manager` | Spawns the `joint_broad` (joint state broadcaster), which reads hardware joint positions/velocities and publishes them on `/joint_states`. |
| `joint_state_publisher` | `joint_state_publisher` | Publishes joint states for joints not driven by hardware (e.g. caster). Required for a complete TF tree when not using the GUI variant. |
| `display_reader` | `display_reader` | Launches the display reader subsystem (via its own launch file) for recognising numbers on displays using a camera. Enabled by `use_display_reader:=true`. Camera exposure is set via `display_reader_exposure`. Review the `number-recognizer` package for more information. |
| `arducam_tof` | `arducam_rclpy_tof_pointcloud` | Publishes a point cloud from the Arducam TOF depth camera. Enabled by `use_arducam_tof:=true`. |
| `display_marker` | `display_marker` | Visualises display reader readings as markers on the map in RViz. Enabled by `use_display_marker:=true`. |
| `pointcloud_to_laserscan` | `pointcloud_to_laserscan` | Converts the TOF camera point cloud (`/point_cloud`) into a 2D `LaserScan` message for use with navigation or obstacle avoidance. Height range filtered to 0.0–1.0 m. |




