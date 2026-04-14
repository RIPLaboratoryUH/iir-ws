# nav2_commander

ROS 2 Python nodes built on top of the Nav2 Simple Commander API.

This package currently focuses on:
- single-goal navigation (`go_home`)
- lawn-mower coverage with waypoint goals (`lawn_mower`)
- lawn-mower coverage with motion primitives (`lawn_mower_heading`)
- rectangular waypoint loop demo (`waypoint_path`)

Nav2 must be running before these nodes are launched.

## Executables

### go_home
Send a single pose goal in `map` frame (defaults to origin).

```bash
ros2 run nav2_commander go_home --x 0.0 --y 0.0 --yaw 0.0
```

Arguments:
- `--x` (default `0.0`): goal x in meters
- `--y` (default `0.0`): goal y in meters
- `--yaw` (default `0.0`): goal yaw in radians

### lawn_mower
Serpentine coverage using sequential `goToPose` goals.

```bash
ros2 run nav2_commander lawn_mower --rows 5 --length 3.0 --spacing 0.1
```

Arguments:
- `--rows` (default `5`): number of rows
- `--length` (default `3.0`): row length in meters
- `--spacing` (default `0.1`): spacing between rows in meters

Pattern:
- drive row and finish with corner heading
- shift to next row and turn to row heading
- alternate direction each row

### lawn_mower_heading
Serpentine coverage using `driveOnHeading` + `spin` primitives.

```bash
ros2 run nav2_commander lawn_mower_heading --rows 5 --length 3.0 --spacing 0.1 --speed 0.10
```

Arguments:
- `--rows` (default `5`): number of rows
- `--length` (default `0.13`): row length in meters
- `--spacing` (default `0.13`): spacing between rows in meters
- `--speed` (default `0.10`): linear speed in m/s
- `--turn-angle-deg` (default `90.0`): corner turn angle in degrees
- `--turn-dir` (default `right`): initial turn direction (`left` or `right`)
- `--segment-timeout` (default `30`): drive timeout in seconds, must be INT
- `--turn-timeout` (default `15`): spin timeout in seconds, must be INT
- `--settle-time` (default `1`): pause between segments in seconds, must be INT

### waypoint_path
Runs a repeated rectangular waypoint path with `followWaypoints`.

```bash
ros2 run nav2_commander waypoint_path --repeats 3 --long-side 2.0 --short-side 1.0
```

Arguments:
- `--repeats` (default `3`): number of rectangle laps
- `--long-side` (default `2.0`): long side length in meters
- `--short-side` (default `1.0`): short side length in meters
- `--speed` (default `0.10`): drive speed in m/s
- `--turn-angle-deg` (default `90.0`): corner turn angle in degrees
- `--turn-dir` (default `right`): corner turn direction (`right` or `left`)
- `--segment-timeout` (default `30.0`): drive timeout in seconds
- `--turn-timeout` (default `15.0`): turn timeout in seconds
- `--settle-time` (default `0.3`): pause after turns in seconds

## Dependencies

- `nav2_simple_commander`
- `rclpy`
- `geometry_msgs`

