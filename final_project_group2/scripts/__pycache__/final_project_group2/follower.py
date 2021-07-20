#!/usr/bin/env python

from numpy import broadcast
import leader
import os
import rospy
import actionlib
import sys
import tf
from nav_msgs.msg import Odometry
from bot_controller.util import compute_distance
from tf.transformations import euler_from_quaternion
from math import radians, degrees, atan2, pi
from sensor_msgs.msg import LaserScan
import fiducial_msgs
import geometry_msgs.msg
import rospy
import tf2_ros
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

class Follower():
    def __init__(self, rate=4):
        super().__init__(self)
        self._safe_dist = 0.5
        self._tag_topic = fiducial_msgs() #.fiducial_trandforms
        self._tag_topic.translation
        self._brodcast = tf.TransformBroadcaster()

    def track_leader(self):
        # if fiducial_transforms.transforms != 0: # If transform is detected...
        if fiducial_transforms.fiducial_id == 0: # ID detected, follow the tag using translation and rotation
            
        
            #transform from camera to marker - robot to camera - map to robot
            self._brodcast.sendTransform(('x,y,z coordinates from fiducial transform'), 
                                            tf.transformations.quaternion_from_euler('3 rotational angles from fiducial transform'), 
                                            rospy.Time.now(), 'marker', 
                                            'follower_tf/camera_rgb_optical_frame')

        
        goal_x = ''
        goal_y = ''
        self.go_to_goal(goal_x, goal_y)


if __name__ == '__main__':
    try:
        Follower = Follower()
        Follower.handle_inputs()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
    rospy.init_node('node_name')