# ROS Testing

Waypoint navigation testing packages for ROS1 and ROS2 robots.

## Packages

| Package | ROS Version | Robot | Language |
|---------|-------------|-------|----------|
| [fastbot_waypoints](fastbot_waypoints/) | ROS2 (Galactic/Humble) | FastBot | C++ |
| [tortoisebot_waypoints](tortoisebot_waypoints/) | ROS1 (Noetic) | TortoiseBot | Python |

## Overview

Both packages provide:
- **Waypoint Action Server** - Navigates robot to target X, Y coordinates
- **Integration Tests** - Verify robot reaches target position and orientation
- **Pass/Fail Modes** - Easily switch between passing and failing test conditions

## Quick Start

### FastBot (ROS2)

```bash
# Terminal 1: Gazebo simulation
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py

# Terminal 2: Action server
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server

# Terminal 3: Run tests
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

### TortoiseBot (ROS1)

```bash
# Terminal 1: Gazebo simulation
source /opt/ros/noetic/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch

# Terminal 2: Action server
source ~/catkin_ws/devel/setup.bash
rosrun tortoisebot_waypoints waypoint_action_server.py

# Terminal 3: Run tests
cd ~/catkin_ws && catkin_make && source devel/setup.bash
rostest tortoisebot_waypoints waypoint_test.test --reuse-master
```

## Switching Between Pass/Fail Conditions

### FastBot (C++)

Edit `fastbot_waypoints/test/test_waypoint_reached.cpp`:

```cpp
// Passing: reachable targets
constexpr Waypoint2D TARGET_1 = {1.0, 1.0};
constexpr Waypoint2D TARGET_2 = {1.5, 1.5};

// Failing: unreachable targets
constexpr Waypoint2D TARGET_1 = {-5.0, -5.0};
constexpr Waypoint2D TARGET_2 = {-5.0, -5.0};
```

### TortoiseBot (Python)

Edit `tortoisebot_waypoints/test/waypoint_test.test`:

```xml
<!-- Passing -->
<arg name="target_x" default="0.5" />
<arg name="target_y" default="0.5" />

<!-- Failing -->
<arg name="target_x" default="2.0" />
<arg name="target_y" default="2.0" />
```

## Expected Results

| Condition | FastBot (ROS2) | TortoiseBot (ROS1) |
|-----------|----------------|---------------------|
| **Passing** | `3 tests, 0 errors, 0 failures` | `2 tests, 0 errors, 0 failures` |
| **Failing** | `2 tests, 1 errors, 1 failures` | `1 tests, 1 errors, 0 failures` |

## See Also

- [fastbot_waypoints/README.md](fastbot_waypoints/README.md) - Detailed FastBot instructions
- [tortoisebot_waypoints/README.md](tortoisebot_waypoints/README.md) - Detailed TortoiseBot instructions

