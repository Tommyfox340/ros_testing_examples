# FastBot Waypoints

ROS2 package for waypoint navigation testing of the FastBot.

## Prerequisites

1. **Terminal 1** - Launch the ROS2 simulation for the FastBot:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
```

> **Note:** Abort and re-launch Gazebo if there are any issues. Use `kill -9 <gazebo_pid>`.

2. **Terminal 2** - Launch the Waypoints Action Server:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

## Running Tests

### Passing Conditions (2 tests pass)

Ensure `test/test_waypoint_reached.cpp` uses reachable target coordinates:

```cpp
// ============================================================
// CHANGE TARGETS TO TEST FAILING CONDITIONS
// Set: -5.0, -5.0
// ============================================================
// Passing: Correct target coordinates
constexpr Waypoint2D TARGET_1 = {1.0, 1.0};
constexpr Waypoint2D TARGET_2 = {1.5, 1.5};
```

Then run in **Terminal 3**:

```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

**Expected Output:**

```
Summary: 3 tests, 0 errors, 0 failures, 0 skipped
```

### Failing Conditions (1 error, 1 skipped)

Change target to unreachable position in `test/test_waypoint_reached.cpp`:

```cpp
// ============================================================
// CHANGE TARGETS TO TEST FAILING CONDITIONS
// Set: -5.0, -5.0
// ============================================================
// Failing: Unreachable target coordinates
constexpr Waypoint2D TARGET_1 = {-5.0, -5.0};
constexpr Waypoint2D TARGET_2 = {-5.0, -5.0};
```

Then run in **Terminal 3**:

```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

**Expected Output:**

```
Summary: 2 tests, 1 errors, 1 failures, 0 skipped
```

## Test Cases

### test_waypoint_reached.cpp

1. **TestEndPosition** - Verifies robot reaches target X, Y within 10cm tolerance
2. **TestEndYaw** - Verifies robot yaw matches direction to target within ~20 degrees

## How It Works

- The test uses `CRASH_IF_FATAL` macro to abort if the action fails (unreachable target)
- When the first test crashes (`std::abort()`), CTest records it as an **error**
- The second test is automatically **skipped** because the process terminated
