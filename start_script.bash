#!/bin/bash


tmux new-session -d bash
tmux split-window -h bash
tmux split-window -v bash

#sends keys to first second and third terminals
tmux send -t 0:0.0 "source install/local_setup.bash && ros2 launch iir_base diffbot.launch.py use_mock_hardware:=true display_reader_exposure:=15.0" C-m

tmux send -t 0:0.1 " ros2 run micro_ros_agent micro_ros_agent serial --dev \"/dev/ttyACM0\"" C-m


tmux -2 attach-session -d
