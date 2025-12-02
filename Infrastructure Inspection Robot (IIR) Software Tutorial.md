# Infrastructure Inspection Robot (IIR) Software Tutorial

## Overview

------

## System Start-up

### Local Machine Setup

#### 1. Start the Zenoh Router

```bash
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/128.171.62.185:7447\"];transport/shared_memory/enabled=true' && ros2 run rmw_zenoh_cpp rmw_zenohd
```
```
#OR, if alias is configured in ~/.bashrc
router
```

Expected Output

![image-20251103151449354](Infrastructure Inspection Robot (IIR) Software Tutorial.assets/image-20251103151449354.png)

#### 2. Clone and Navigate to Repository

```bash
# Open a new terminal window
cd GIT/iir-ws
# OR if cloning for the first time:
git clone 
cd iir-ws
```

#### 3. Build the Workspace

```bash
colcon build
```

**If build fails**, install dependencies:

```bash
rosdep update
#then
rosdep install --from-paths src --ignore-src -r -y
```

Expected Output

![image-20251103151627122](Infrastructure Inspection Robot (IIR) Software Tutorial.assets/image-20251103151627122.png)



#### 4. Source the Workspace

```bash
source install/local_setup.bash
```

#### 5. Set Zenoh Environment Variable

```bash
#if alias is enabled
expz

#else
export RMW_IMPLEMENTATION=rmw_zenoh_cpp 
```

#### 6. Launch IIR Navigation

**After doing the steps on the RPI5**, run:

```bash
ros2 launch iir_nav2 bringup_launch.py
```

[Expected output here]

#### 7. Start RViz2

Open another terminal and run:

```bash
#set zenoh variables
expz
#start visualization software
rviz2
```

------

### Raspberry Pi 5 (RPI5) Setup

#### 1. SSH into RPI5

Choose one of the following methods:

```bash
# Option 1: Tailscale IP (aliased)
ssh rpi5

# Option 2: Static IP from riplab router
ssh 192.168.1.22

#or whatever other ssh method
```

#### 2. Navigate to Workspace

```bash
cd iir-ws
```

#### 3. Start Hardware Services

Run the following aliased commands:

```bash
# Start LiDAR
lidarup

# Start CAN networking
canup

#actual command for lidarup
sudo ip link set dev eth0 up && sudo ifconfig eth0 192.168.0.15 netmask 255.255.255.0

#actual command for canup
sudo ip link set can0 up type can bitrate 1000000
```

These start the lidar and CAN-bus networking drivers. Allows for ROS2 to detect these hardware devices. 

#### 4. Set Zenoh Variables

```bash
expz
```

#### 5. Source the Workspace

```bash
source install/local_setup.bash
```

#### 6. Run LiDAR Script

```bash
./lidar_script
```

This command will open up 3 tmux panes, and run the following commands in each window. 

```bash
source install/local_setup.bash && ros2 launch iir_base diffbot.launch.py use_mock_hardware:=true use_lidar:=true


export ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/128.171.62.185:7447\"];transport/shared_memory/enabled=true' && ros2 run rmw_zenoh_cpp rmw_zenohd


ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/riplab/mapper_params_online_async.yaml

```

#### Expected Output

When working correctly, you should see 3 panes.

------

## Using NAV2

### 1. Load Configuration

In rviz2, Click File > recent configs > nav2_default_view

### 2. Set Initial Pose

1. Click the **2D Pose Estimate** tool
2. Click on the robot's location on the map
3. Drag the arrow to point in the direction the robot is facing (positive x-body direction)

[Screenshot or diagram showing proper pose estimation]

### 3. Set Navigation Goal

1. Select the **NAV2 Goal** tool
2. Click on the desired destination on the map
3. The robot will automatically plan and execute a path

------

## Troubleshooting

### Common Issues

**Issue**: LiDAR Disconnecting

**Symptoms**:  This will appear on the RPI terminal, left pane

```bash 
[urg_node_driver-5] [WARN] [1761958586.833969884] [urg_node_driver]: Could not grab single echo scan.
[urg_node_driver-5] [ERROR] [1761958598.158216361] [urg_node_driver]: Error connecting to Hokuyo: Could not open network Hokuyo: [urg_node_driver-5] 192.168.0.10:10940
[urg_node_driver-5] could not open ethernet port.
```
Other messages indicating scans are failing will appear in the slam_toolbox window. 

**Solution:**  Usually this issue can be fixed by re-rerunning `lidarup` / the lidar networking configuration command. If not, go through the reset procedure.



**Issue**: System Needs Reset

**Symptoms**: You can just tell…

**Solution**:

- Stop all nodes on the RPI by opening a new tmux pane (CTRL-B, then %), and running `pkill tmux`.
- Re-run `lidarup` and `canup` 
- At this point, "CTRL-C" the rviz2 and nav2 terminals as well
- Now re-run `./lidar_script.bash` on the RPI.
- Re-run the nav2 and rviz2 commands on the local.

------

## Additional Resources

- https://github.com/RIPLaboratoryUH/iir-ws
- [Link to related resources]

------

## Appendix

### Common Command Reference

| Alias     | Command   | Purpose   |
| --------- | --------- | --------- |
| `lidarup` | [command] | [purpose] |
| `canup`   | [command] | [purpose] |
| `expz`    | [command] | [purpose] |

### Environment Variables

[List important environment variables and their values]

------

## Version History

| Version | Date    | Changes |
| ------- | ------- | ------- |
| Initial | 12/1/25 | N/A     |




