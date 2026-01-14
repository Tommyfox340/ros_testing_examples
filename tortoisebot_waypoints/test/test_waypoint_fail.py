#!/usr/bin/env python

import rospy
import unittest
import rostest
import actionlib
from geometry_msgs.msg import Point
from tortoise_waypoints.msg import WaypointActionAction, WaypointActionGoal

PKG = "tortoise_waypoints"
NAME = "test_waypoint_fail"

# Unreachable goal - robot cannot get there in time
TARGET_X = 2.0
TARGET_Y = 2.0


class TestWaypointFail(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_waypoint_fail', anonymous=True)
        cls.goal_error = None
        cls.send_goal(TARGET_X, TARGET_Y)

    @classmethod
    def send_goal(cls, x, y):
        client = actionlib.SimpleActionClient("tortoisebot_as", WaypointActionAction)
        if not client.wait_for_server(rospy.Duration(10.0)):
            cls.goal_error = "Action server not available"
            return

        goal = WaypointActionGoal()
        goal.position = Point(x, y, 0.0)
        client.send_goal(goal)
        
        # Short timeout - will fail because goal is too far
        if not client.wait_for_result(rospy.Duration(15.0)):
            cls.goal_error = "Action timed out"
            return

        if not client.get_result().success:
            cls.goal_error = "Action failed"
            return

    def test_unreachable_goal(self):
        """This test should error - goal is unreachable in time"""
        if self.goal_error:
            raise RuntimeError(self.goal_error)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointFail)

