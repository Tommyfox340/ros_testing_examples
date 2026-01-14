#!/usr/bin/env python

import rospy
import unittest
import rostest
import actionlib
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tortoise_waypoints.msg import WaypointActionAction, WaypointActionGoal
from tf.transformations import euler_from_quaternion

PKG = "tortoise_waypoints"
NAME = "test_waypoint_reached"

# Tolerances
POSITION_TOLERANCE = 0.1  # 10 cm
YAW_TOLERANCE = math.pi / 10  # ~18 degrees


class TestWaypointReached(unittest.TestCase):

    odom_msg = None
    target_x = None
    target_y = None

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_waypoint_reached', anonymous=True)

        # Get target from params (default 0.5, 0.5)
        cls.target_x = rospy.get_param('/target_x', 0.5)
        cls.target_y = rospy.get_param('/target_y', 0.5)

        rospy.Subscriber('/odom', Odometry, cls.odom_callback)

        # Wait for odom
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while cls.odom_msg is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        # Send goal and wait for result
        cls.send_goal(cls.target_x, cls.target_y)

    @classmethod
    def odom_callback(cls, msg):
        cls.odom_msg = msg

    @classmethod
    def send_goal(cls, x, y):
        client = actionlib.SimpleActionClient("tortoisebot_as", WaypointActionAction)
        assert client.wait_for_server(rospy.Duration(10.0)), "Action server not available"

        goal = WaypointActionGoal()
        goal.position = Point(x, y, 0.0)
        client.send_goal(goal)
        assert client.wait_for_result(rospy.Duration(15.0)), "Action timed out"
        assert client.get_result().success, "Action failed"

    def test_end_position(self):
        """Check end position [X, Y] is correct within tolerance"""
        pos = self.odom_msg.pose.pose.position
        self.assertAlmostEqual(pos.x, self.target_x, delta=POSITION_TOLERANCE,
                               msg="X position: %.3f, expected: %.3f" % (pos.x, self.target_x))
        self.assertAlmostEqual(pos.y, self.target_y, delta=POSITION_TOLERANCE,
                               msg="Y position: %.3f, expected: %.3f" % (pos.y, self.target_y))

    def test_end_orientation(self):
        """Check end rotation [Yaw] is correct within tolerance"""
        pos = self.odom_msg.pose.pose.position
        orient = self.odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

        expected_yaw = math.atan2(self.target_y - pos.y, self.target_x - pos.x)
        yaw_error = abs(yaw - expected_yaw)
        if yaw_error > math.pi:
            yaw_error = 2 * math.pi - yaw_error

        self.assertLess(yaw_error, YAW_TOLERANCE,
                        msg="Yaw: %.3f, expected: %.3f" % (yaw, expected_yaw))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointReached)