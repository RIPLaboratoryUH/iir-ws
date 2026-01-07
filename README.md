# iir-ws
This package contains everything needed to run the lab's Infrastructure Inspection Robot
cloning this package, then doing ```colcon build``` then ```ros2 launch iir_base diffbot.launch.py``` will launch RSP, and ros2control among other things. There is also a sim package ```iir_sim``` and a nav package ```iir_nav``` which have launch files ```iir_sim.launch.py``` and ```navigation.launch.py```


## IIR Simple Driving Tutorial

Note: Run each of these commands one at a time. You do not need ROS2 installed to follow this, you simply need a terminal with SSH capabilites.

1. Plug riplab router in
2. connect laptop to riplab network
3. turn robot on The PI will automatically connect to the riplab network

`ssh riplab@192.168.1.22`



Then run these commands to configure the pi (must be done every time robot is reset). Run these commands one at a time.

```sudo ip link set can0 down
$ sudo ip link set can0 down
$ sudo ip link set can0 type can bitrate 1000000
$ sudo ip link set can0 up
$ expz
$ source install/local_setup.bash
$ cd iir-ws
$ ./lidar_script
```

This will open 3 windows, the 1st is the base control system, the second is a zenoh router, the third is the map node. 

To start the teleop controller, hit CTRL-B and then % to open a new tmux window. In the new window, run this command:

```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=diff_drive_controller/cmd_vel -p stamped:=true
```

this final command will open up a window that will allow you to drive the robot around.

Controls:

`q`/`z` will increase/decrease the speed of the motors

`i` /`,`will drive forward/backward

`j`/`l` will turn left/right

`k` will stop the robot



### Resetting 

CTRL-C any of the tmux windows, then run

`pkill tmux`

This will force quit all tmux windows.

Then redo the `./lidar_script` , open a new tmux pane, and rerun the teleop twist command.



### Troubleshoot Connection Issues

Check http://router.asus.com/Main_Login.asp

user: `admin`

pw: you know this one

Click on clients to see all devices connected. If the PI is not showing up here then you should try shutting down and rebooting the PI.  If this does not fix it, then replace the microSD card with one of the others on my desk.

