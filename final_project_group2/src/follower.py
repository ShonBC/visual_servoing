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
# from bot_controller.util import compute_distance
from tf.transformations import euler_from_quaternion
from math import radians, degrees, atan2, pi
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
import rospy
import tf2_ros
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

# import src.move_base_example
# import src.spawn_marker_on_robot

class Follower():
    def __init__(self, rate=4):
        self._safe_dist = 0.5
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        self._velocity_publisher = rospy.Publisher('follower/cmd_vel', Twist, queue_size=10)
        self._rate = rospy.Rate(rate)
        
        # parent frame for the listener
        self._parent_frame = 'map'
        # child frame for the listener
        self._child_frame = 'fiducial_marker'
        self._tf_listener = tf.TransformListener()
        self._tf_buffer = tf2_ros.Buffer()

        # rospy.init_node('node_name', anonymous=True)
        listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.Subscriber('/fiducial_transforms',
                         FiducialTransformArray,
                         self.fiducial_transforms_callback,
                         queue_size=1)
    
    def fiducial_transforms_callback(self, msg):
        """fiducial_transforms topic callback function that reads the position
           of the marker on the leader and broadcast a new transform named fiducial_marker
           which is the tf transform between the marker and the follower camera rgb optical frame
        """
        broadcaster = tf2_ros.TransformBroadcaster()
        # Creating transform frame to be broadcast
        t_frame = geometry_msgs.msg.TransformStamped()
        t_frame.header.stamp = rospy.Time.now()
        t_frame.header.frame_id = "follower_tf/camera_rgb_optical_frame"
        t_frame.child_frame_id = self._child_frame

        # Adding pose data to transform from fiducial_tranform topic and broadcasting new transform
        rate = rospy.Rate(10.0)
        for m in msg.transforms:
            trans = m.transform.translation
            rot = m.transform.rotation
            t_frame.transform.translation.x = trans.x
            t_frame.transform.translation.y = trans.y
            t_frame.transform.translation.z = trans.z
            t_frame.transform.rotation.x = rot.x
            t_frame.transform.rotation.y = rot.y
            t_frame.transform.rotation.z = rot.z
            t_frame.transform.rotation.w = rot.w

            broadcaster.sendTransform(t_frame)
            rate.sleep()
    
    # def get_follower_goal(self):
    #     try:
    #         now = rospy.Time.now()
    #         transform_obj = self._tf_buffer.lookup_transform(self._parent_frame, self._child_frame, now, rospy.Duration(3))
    #         rospy.loginfo(transform_obj)
    #         return transform_obj[0]
    #     except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #         rospy.logwarn("Nothing")

        



    def get_follower_goal(self):
        """Get the current pose of the robot from the /odom topic

        Return
        ----------
        The position (x, y, z) and the yaw of the robot.

        """
        
        # marker_map_tf = self._tf_listener.lookupTransform('fiducial_marker', 'map', now)
        
        try:
            now = rospy.Time.now()
            self._tf_listener.waitForTransform(self._parent_frame,
                                               self._child_frame,
                                               now,
                                               rospy.Duration(5))
            (trans, rot) = self._tf_listener.lookupTransform(self._parent_frame, self._child_frame, now)
            print(trans)
            print(rot)
            self._current_x_pos = trans[0]
            self._current_y_pos = trans[1]
            self.current_orientation = rot
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
        rospy.sleep(1)
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
    def run(self, linear, angular):
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)

if __name__ == '__main__':
    try:
        Follower = Follower()
        Follower.handle_inputs()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
    rospy.init_node('node_name')