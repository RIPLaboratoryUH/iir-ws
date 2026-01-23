# Image Saver

A ROS2 package that listens to the `/image_raw` topic and saves images when the space key is pressed.

## Features

- Subscribes to `/image_raw` topic (sensor_msgs/Image)
- Listens for space key press
- Saves images in PNG format with ROS timestamps
- Stores images in `~/image_saves/` directory
- Filename format: `image_YYYYMMDD_HHMMSS_<ROS_SEC>_<ROS_NANOSEC>.png`

## Installation

This package requires:
- `rclpy`
- `sensor_msgs`
- `cv-bridge`
- `opencv-python`
- `pynput`

Build with colcon:
```bash
cd ~/iir-ws
colcon build --packages-select image_saver
source install/setup.bash
```

## Usage

Run the node:
```bash
ros2 run image_saver image_saver_node
```

The node will:
1. Start subscribing to `/image_raw`
2. Wait for images to arrive
3. Save images to `~/image_saves/` when you press the SPACE key

## Notes

- Images are saved in BGR format (OpenCV default)
- The ROS timestamp is preserved in both the filename and the node's logging
- If no image has been received yet, pressing space will log a warning
- The save operation is thread-safe with image locking
