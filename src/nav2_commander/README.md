# nav2_commander

A ROS 2 Python package providing autonomous navigation behaviors built on top of the [Nav2 Simple Commander API](https://navigation.ros.org/commander_api/index.html). Includes utilities for returning to a home position and executing lawn-mower coverage patterns. Nav2 must be running before executing any of these nodes.

---

## Executables

### `go_home`
Navigates the robot to a target pose (default: origin). Accepts position and heading as CLI arguments.

```bash
ros2 run nav2_commander go_home --x 0.0 --y 0.0 --yaw 0.0
```

| Argument | Default | Description |
|----------|---------|-------------|
| `--x` | `0.0` | Target X position (meters) |
| `--y` | `0.0` | Target Y position (meters) |
| `--yaw` | `0.0` | Target heading (radians) |

Prints distance remaining and ETA while navigating.

---

### `lawn_mower`
Executes a serpentine coverage pattern using `driveOnHeading` and `spin` primitives — no costmap planning between rows.

```bash
ros2 run nav2_commander lawn_mower --rows 5 --length 2.0 --spacing 0.1 --speed 0.10
```

| Argument | Default | Description |
|----------|---------|-------------|
| `--rows` | `5` | Number of passes |
| `--length` | `2.0` | Length of each pass (meters) |
| `--spacing` | `0.1` | Lateral distance between passes (meters) |
| `--speed` | `0.10` | Drive speed (m/s) |
| `--turn-angle-deg` | `90.0` | In-place turn angle (degrees) |
| `--turn-bias-deg` | `0.0` | Bias to correct turn over/undershoot |
| `--turn-dir` | `right` | Initial turn direction (`right` or `left`) |
| `--segment-timeout` | `30.0` | Timeout per drive segment (seconds) |
| `--turn-timeout` | `15.0` | Timeout per turn (seconds) |
| `--settle-time` | `0.5` | Pause after each turn (seconds) |

**Pattern**: drive → turn 90° → shift by spacing → turn 90° → repeat (alternating turn directions).

---

### `lawn_mower_path`
Pre-computes all goal poses upfront and executes them in a single `goThroughPoses` call. More efficient than sequential `goToPose` calls but relies on the planner to navigate between waypoints.

```bash
ros2 run nav2_commander lawn_mower_path
```

Parameters are currently hardcoded (`num_rows=5`, `row_length=2.0m`, `row_spacing=0.1m`).

---

## Other Scripts

| File | Description |
|------|-------------|
| `commander.py` | Minimal demo/template showing basic Nav2 navigation with `goToPose`. |
| `lawn_mower_old.py` | Earlier implementation using `goToPose` for each waypoint including turns. Superseded by `lawn_mower.py`. |

---

## Dependencies

- `nav2_simple_commander` — Nav2 Python commander API (`BasicNavigator`, `TaskResult`)
- `rclpy`
- `geometry_msgs`

