#!/usr/bin/env python

# from final_project_group2.scripts
import os
import read_waypts_yaml
import rospy
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



class Leader(object):
    def __init__(self, rate=4):
        self._waypoints = read_waypts_yaml.create_commands()
        rospy.init_node('turtlebot_controller')
        rospy.loginfo('Press Ctrl c to exit')
        self._velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("scan", LaserScan, self.sensor_callback)

        self._rate = rospy.Rate(rate)
        self._robot_name = 'waffle'
        self._velocity_msg = Twist()
        self._kp_linear = 0.1
        self._kp_angular = 0.5
        self._velocity_msg.linear.x = 0.1
        self._velocity_msg.angular.z = 0.1
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        self._initial_orientation = None
        self._yaw = None
        # start position of the robot
        self._initial_x_pos = None
        self._initial_y_pos = None

        # set up a tf listener to retrieve transform between the robot and the world
        self._tf_listener = tf.TransformListener()
        # parent frame for the listener
        self._parent_frame = 'odom'
        # child frame for the listener
        self._child_frame = 'base_footprint'

    def read_waypoints(self):
        self._waypoints = read_waypts_yaml.create_commands()

    @property
    def current_x_pos(self):
        return self._current_x_pos

    @current_x_pos.setter
    def current_x_pos(self, x):
        self._current_x_pos = x

    @property
    def current_y_pos(self):
        return self._current_y_pos

    @current_y_pos.setter
    def current_y_pos(self, y):
        self._current_y_pos = y

    @property
    def current_orientation(self):
        return self._current_orientation

    @current_orientation.setter
    def current_orientation(self, orientation):
        self._current_orientation = orientation

    def odom_callback(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.current_x_pos = msg.pose.pose.position.x
        self.current_y_pos = msg.pose.pose.position.y
        self.current_orientation = euler_from_quaternion(quaternion)

    def get_odom_data(self):
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
            self.current_x_pos = trans[0]
            self.current_y_pos = trans[1]
            self.current_orientation = rot
            rospy.loginfo(
                "odom: ({},{}), {}".format(self.current_x_pos, self.current_y_pos, self.current_orientation[2]))
            # return Point(*trans), rot[2]
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logfatal("TF Exception")

    def go_straight(self, distance_to_drive):
        self.get_odom_data()
        self._initial_x_pos = self.current_x_pos
        self._initial_y_pos = self.current_y_pos

        # compute the driven distance
        driven_distance = compute_distance(
            self._initial_x_pos,
            self._initial_y_pos,
            self.current_x_pos,
            self.current_y_pos)

        # keep moving the robot until the distance is reached
        while not rospy.is_shutdown():
            if driven_distance <= distance_to_drive:
                driven_distance = compute_distance(
                    self._initial_x_pos,
                    self._initial_y_pos,
                    self.current_x_pos,
                    self.current_y_pos)
                rospy.loginfo("Distance driven: {}".format(driven_distance))
                self.run(0.1, 0)
            else:
                self.run(0, 0)
                break
            self._rate.sleep()

    def go_straight_v2(self, distance_to_drive):
        # original time
        rospy.sleep(2)
        t_0 = rospy.Time.now().to_sec()

        # keep moving the robot until the distance is reached
        while not rospy.is_shutdown():
            # current time
            t_1 = rospy.Time.now().to_sec()
            driven_distance = (t_1 - t_0) * abs(self._velocity_msg.linear.x)
            rospy.loginfo("Distance driven: {}".format(driven_distance))
            if driven_distance <= distance_to_drive:
                self.run(0.1, 0)
            else:
                # self._velocity_publisher.publish(Twist())
                self.run(0, 0)
                break
            self._rate.sleep()

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

    def go_to_goal(self, goal_x, goal_y):
        """
        @param goal_x: x position of the goal to reach
        @type goal_x: float
        @param goal_y: y position of the goal to reach
        @type goal_y: float
        """
        rospy.loginfo("Go to goal ({}, {})".format(goal_x, goal_y))
        # get position and yaw from transform
        self.get_odom_data()

        distance_to_goal = compute_distance(self.current_x_pos, self.current_y_pos, goal_x, goal_y)

        while not rospy.is_shutdown():
            move_cmd = Twist()
            if distance_to_goal > 0.05:
                distance_to_goal = compute_distance(self.current_x_pos,
                                                    self.current_y_pos, goal_x, goal_y)
                angle_to_goal = atan2(goal_y - self.current_y_pos, goal_x - self.current_x_pos)
                rospy.loginfo("Distance to goal: {}".format(distance_to_goal))
                rospy.loginfo("Angle to goal: {}".format(angle_to_goal))

                if angle_to_goal < 0:
                    angle_to_goal = 2 * pi + angle_to_goal

                w = angle_to_goal - self.current_orientation[2]
                if w > pi:
                    w = w - 2 * pi
                # proportional control
                move_cmd.angular.z = self._kp_angular * w

                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)
                # proportional control
                move_cmd.linear.x = min(self._kp_linear * distance_to_goal, 0.5)

                self._velocity_publisher.publish(move_cmd)
                self._rate.sleep()
            else:
                rospy.loginfo("Goal reached")
                break

    def run(self, linear, angular):
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)

    def sensor_callback(self, msg):
        """Callback function to deal with messages on the /scan topic
        """
        front = msg.ranges[0]
        left = msg.ranges[90]
        right = msg.ranges[270]

        rospy.loginfo("Distance from obstacle (front): {f}".format(f=front))
        rospy.loginfo("Distance from obstacle (left): {l}".format(l=left))
        rospy.loginfo("Distance from obstacle (right): {r}".format(r=right))
        rospy.loginfo("--" * 20)

    def handle_inputs(self):
        """

        Returns:

        """
        rospy.myargv(argv=sys.argv)
        if len(sys.argv) == 3:
            if sys.argv[1] == '-sv1':
                self.go_straight(float(sys.argv[2]))
            elif sys.argv[1] == '-sv2':
                self.go_straight_v2(float(sys.argv[2]))
            elif sys.argv[1] == '-r':
                self.rotate(float(sys.argv[2]))
            else:
                print_usage()
        elif len(sys.argv) == 4:
            if sys.argv[1] == '-g':
                self.go_to_goal(float(sys.argv[2]), float(sys.argv[3]))
            else:
                print_usage()
        elif len(sys.argv) == 1:
            rospy.loginfo('LIDAR')
        else:
            print_usage()
        return


def print_usage():
    usage = """
            Usage:
            \trosrun turtlebot_controller controller -[s [n] | r [n] | g [x y]]
            Examples:
            \trosrun turtlebot_controller controller -s 5
            \trosrun turtlebot_controller controller -r 45
            \trosrun turtlebot_controller controller -g 2 3
            \n
            s: Task the robot to move in a straight line.
            \t n: distance to drive in meter.
            r: Task the robot to rotate an angle (deg) from its current orientation.
            \t n: angle to rotate.
            g: Task the robot to reach a goal pose
            \t x y (meter): pose in 2D of the goal to reach.
            """
    print(usage)


if __name__ == '__main__':
    try:
        Leader = Leader()
        Leader.handle_inputs()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
    rospy.init_node('node_name')