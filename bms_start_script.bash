#!/bin/bash

tmux new-session -d bash
tmux split-window -h bash
tmux split-window -v bash
tmux split-window -v bash

#sends keys to first second and third terminals
tmux send -t 0:0.0 "source install/local_setup.bash && ros2 launch iir_base diffbot.launch.py" C-m

tmux send -t 0:0.1 "source install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0" C-m
# tmux send -t 0:0.2 "source install/local_setup.bash && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/riplab/mapper_params_online_async.yaml " C-m

#tmux -2 attach-session -d


tmux send -t 0:0.2 "source install/local_setup.bash && ros2 run daly_bms_can daly_bms_pub" C-m
# tmux send -t 0:0.3 "source install/local_setup.bash && ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p framerate:=30.0 -p image_width:=640 -p image_height:=480" C-m

tmux -2 attach-session -d


#"export ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/128.171.62.185:7447\"];transport/shared_memory/enabled=true' && ros2 run rmw_zenoh_cpp rmw_zenohd" C-m this launches the zenoh router