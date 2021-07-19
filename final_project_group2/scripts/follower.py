#!/usr/bin/env python

from scripts import leader
import os
import rospy
import leader.Leader
import actionlib
import sys
import tf
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from bot_controller.util import compute_distance
from tf.transformations import euler_from_quaternion
from math import radians, degrees, atan2, pi
from sensor_msgs.msg import LaserScan

import src.move_base_example
import src.spawn_marker_on_robot

class Follower(leader.Leader):
    def __init__(self, rate=4):
        super().__init__(self)
        self._safe_dist = 0.5


if __name__ == '__main__':
    try:
        Follower = Follower()
        Follower.handle_inputs()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
    rospy.init_node('node_name')