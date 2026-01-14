# TortoiseBot Waypoints

ROS1 package for waypoint navigation testing of the TortoiseBot.

## Prerequisites

Launch the ROS1 simulation for the TortoiseBot before running tests.

```
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch
```

## Running Tests

### Passing Conditions (2 tests pass)

Ensure `test/waypoint_test.test` uses the default reachable target (0.5, 0.5):

```xml
<arg name="target_x" default="0.5" />
<arg name="target_y" default="0.5" />
<test test-name="test_waypoint_reached" pkg="tortoise_waypoints" type="test_waypoint_reached.py" />
```

Then run:

```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
```

**Expected Output:**

```
SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```

### Failing Conditions (1 error)

**Option A:** Change test file to use `test_waypoint_fail.py` in `test/waypoint_test.test`:

```xml
<test test-name="test_waypoint_reached" pkg="tortoise_waypoints" type="test_waypoint_fail.py" />
```

**Option B:** Change target to unreachable position in `test/waypoint_test.test`:

```xml
<arg name="target_x" default="2.0" />
<arg name="target_y" default="2.0" />
```

Then run:

```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
```

**Expected Output:**

```
SUMMARY
 * RESULT: FAIL
 * TESTS: 1
 * ERRORS: 1
 * FAILURES: 0
```

## Test Cases

### test_waypoint_reached.py (passing)

1. **test_end_position** - Verifies robot reaches target X, Y within 10cm tolerance
2. **test_end_orientation** - Verifies robot yaw matches direction to target within 18 degrees

### test_waypoint_fail.py (failing)

1. **test_unreachable_goal** - Attempts to reach unreachable position (2.0, 2.0), times out
