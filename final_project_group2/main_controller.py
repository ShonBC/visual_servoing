"""
This will be the main program where the nodes will be created and the robot will be controlled.
"""

from yaml import read_waypts_yaml as waypts
from src import broadcaster_example, move_base_example, spawn_marker_on_robot as move
from src import *
import rospy

robot_commands = waypts.create_commands()
print(robot_commands)

# Task follower to go through each location using parameter server.
rospy.init_node('movebase_client_leader')
move.handle_inputs()

# Get the marker in the follower camera frame through topic fiducial_transforms.
rospy.init_node('fiducial_broadcaster')
rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_transforms_callback, queue_size=1)
rospy.spin()
move.fiducial_transforms_callback(msg)

# In the callback function for fiducial_transforms, frame broadcast for the marker using data from fiducial_transforms.


# If follower’s camera detects the marker: Do transform between and /map and marker_tf and use result as the goal for
# follower to reach.

# If follower’s camera cannot detect the marker, go to "exploration" mode.

