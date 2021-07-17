#!/usr/bin/env python


import rospy
import actionlib
import sys
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import radians, degrees, atan2, pi


class Follower(object):
    pass


if __name__ == '__main__':
    rospy.init_node('movebase_client_leader')