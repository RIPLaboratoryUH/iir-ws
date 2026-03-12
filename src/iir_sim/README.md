# iir_sim

This package launches a Gazebo simulation environment for the IIR robot.

## Launch Instructions

Run the base package first with simulation flags enabled:

```bash
ros2 launch iir_base diffbot.launch.py use_sim_time:=true use_mock_hardware:=true
```

Then, in a separate terminal, launch the simulation (Gazebo + ROS-GZ bridge + robot spawner):

```bash
ros2 launch iir_sim iir_sim.launch.py
```

this may or may not work, I rarely use this.