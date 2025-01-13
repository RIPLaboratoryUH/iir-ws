#!/bin/bash

tmux new-session -d bash
tmux split-window -h bash
tmux split-window -v bash

#sends keys to first and second terminals
tmux send -t 0:0.0 "ros2 launch iir_base diffbot.launch.py use_mock_hardware:=true" C-m
tmux send -t 0:0.1 "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0" C-m
tmux send -t 0:0.2 "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=diff_drive_controller/cmd_vel -p stamped:=true" C-m

tmux -2 attach-session -d
