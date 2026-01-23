#!/bin/bash

tmux new-session -d bash
tmux split-window -h bash
tmux split-window -v bash
tmux split-window -v bash

#sends keys to first second and third terminals
tmux send -t 0:0.0 "source install/local_setup.bash && ros2 launch iir_base diffbot.launch.py use_mock_hardware:=true use_lidar:=true" C-m

tmux send -t 0:0.1 "export ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/128.171.62.185:7447\"];transport/shared_memory/enabled=true' && ros2 run rmw_zenoh_cpp rmw_zenohd" C-m
tmux send -t 0:0.2 "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/riplab/mapper_params_online_async.yaml " C-m

#tmux -2 attach-session -d









tmux send -t 0:0.3 "source install/local_setup.bash && ros2 run daly_bms_can daly_bms_pub" C-m

tmux -2 attach-session -d
