#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyRequest


rospy.init_node('reset_world_client')
rospy.wait_for_service('/gazebo/reset_world')
reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
srv_msg = EmptyRequest()
result = reset_world_service(srv_msg)