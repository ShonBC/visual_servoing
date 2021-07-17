#!/usr/bin/env python


# import rospy
# import actionlib
# import sys
# from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
# from math import radians, degrees, atan2, pi
# from final_project_group2.scripts import read_waypts_yaml



class Leader(object):
    def __init__(self, rate=4):
        self._waypoints = read_waypts_yaml.create_commands()

    def read_waypoints(self):
        self._waypoints = read_waypts_yaml.create_commands()



if __name__ == '__main__':
    # rospy.init_node('node_name')

    ex = {('1', 'living'): {'pos': {{'x': 123}, {'y': 123}, {'z': 123}}}}
    print(list(ex.keys())[0][0])