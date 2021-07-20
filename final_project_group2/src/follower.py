#!/usr/bin/env python

# from scripts import leader
import os
import rospy
# import leader.Leader
import actionlib
import sys
import tf
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from bot_controller.util import compute_distance
from tf.transformations import euler_from_quaternion
from math import radians, degrees, atan2, pi
from sensor_msgs.msg import LaserScan

# import src.move_base_example
# import src.spawn_marker_on_robot

class Follower():
    def __init__(self, rate=4):
        self._safe_dist = 0.5
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        
        # parent frame for the listener
        self._parent_frame = 'map'
        # child frame for the listener
        self._child_frame = 'fiducial_marker'
        self._tf_listener = tf.TransformListener()

    def get_follower_goal(self):
        """Get the current pose of the robot from the /odom topic

        Return
        ----------
        The position (x, y, z) and the yaw of the robot.

        """
        try:
            now = rospy.Time.now()
            self._tf_listener.waitForTransform(self._parent_frame,
                                               self._child_frame,
                                               now,
                                               rospy.Duration(5))
            (trans, rot) = self._tf_listener.lookupTransform(self._parent_frame, self._child_frame, now)
            self._current_x_pos = trans[0]
            self._current_y_pos = trans[1]
            # self.current_orientation = rot
            rospy.loginfo(
                "odom: ({},{}), {}".format(self._current_x_pos, self._current_y_pos, self._current_orientation[2]))
            # return Point(*trans), rot[2]
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logfatal("TF Exception")
        goal = [self._current_x_pos, self._current_y_pos]
        return goal

    def rotate(self, relative_angle):
        """

        Args:
            relative_angle:

        Returns:

        """
        if relative_angle > 0:
            angular_velocity = 0.1
        elif relative_angle < 0:
            angular_velocity = -0.1
        else:
            angular_velocity = 0.0
        rospy.sleep(5.0)
        t0 = rospy.Time.now().to_sec()
        # keep rotating the robot until the desired relative rotation is reached
        while not rospy.is_shutdown():
            self.run(0, angular_velocity)
            self._rate.sleep()
            t1 = rospy.Time.now().to_sec()

            rospy.loginfo("t0: {t}".format(t=t0))
            rospy.loginfo("t1: {t}".format(t=t1))
            current_angle = (t1 - t0) * angular_velocity
            rospy.loginfo("current angle: {}".format(current_angle))
            if abs(current_angle) >= radians(abs(relative_angle)):
                rospy.loginfo("reached")
                self.run(0, 0)
                break


if __name__ == '__main__':
    try:
        Follower = Follower()
        Follower.handle_inputs()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
    rospy.init_node('node_name')