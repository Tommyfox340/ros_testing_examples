import os
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit


@pytest.mark.launch_test
def generate_test_description():
    # Path to gtest executable is injected from CMake via ENV (no install needed)
    gtest_exe = os.environ.get("GTEST_EXE")
    if not gtest_exe:
        raise RuntimeError("GTEST_EXE env var not set. Set it from CMake add_launch_test(ENV ...).")

    action_server = launch_ros.actions.Node(
        package="fastbot_waypoints",
        executable="fastbot_action_server",
        name="fastbot_action_server",
        output="screen",
    )

    # Reset Gazebo world. (Gazebo must already be running!)
    # Common service name for Gazebo Classic is /gazebo/reset_world. :contentReference[oaicite:1]{index=1}
    reset_world = ExecuteProcess(
        cmd=["ros2", "service", "call", "/gazebo/reset_world", "std_srvs/srv/Empty", "{}"],
        name="reset_world",
        output="screen",
    )

    # Run the gtest executable
    cpp_gtest = ExecuteProcess(
        cmd=[gtest_exe],
        name="cpp_gtest",
        output="screen",
    )

    # Sequence:
    # - start action server
    # - after a short delay, reset world
    # - when reset finishes, start gtest
    start_reset = TimerAction(period=2.0, actions=[reset_world])

    start_gtest_after_reset = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_world,
            on_exit=[TimerAction(period=2.0, actions=[cpp_gtest])]
        )
    )

    # Shutdown the whole launch when gtest ends
    shutdown_on_gtest_exit = RegisterEventHandler(
        OnProcessExit(target_action=cpp_gtest, on_exit=[Shutdown()])
    )

    return (
        launch.LaunchDescription([
            action_server,
            start_reset,
            start_gtest_after_reset,
            shutdown_on_gtest_exit,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            "action_server": action_server,
            "reset_world": reset_world,
            "cpp_gtest": cpp_gtest,
        }
    )


@launch_testing.post_shutdown_test()
class TestExitCodes(unittest.TestCase):
    def test_gtest_exit_code(self, proc_info, cpp_gtest):
        # gtest must succeed
        launch_testing.asserts.assertExitCodes(proc_info, process=cpp_gtest, allowable_exit_codes=[0])  # :contentReference[oaicite:2]{index=2}

    def test_reset_exit_code(self, proc_info, reset_world):
        # reset call must succeed (Gazebo must be running and offering service)
        launch_testing.asserts.assertExitCodes(proc_info, process=reset_world, allowable_exit_codes=[0])  # :contentReference[oaicite:3]{index=3}

    def test_action_server_exit_ok(self, proc_info, action_server):
        # action server usually gets SIGINT on shutdown => allow 0 and common signal exits
        launch_testing.asserts.assertExitCodes(proc_info, process=action_server, allowable_exit_codes=[0, -2, -15])  # :contentReference[oaicite:4]{index=4}
