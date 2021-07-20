#!/usr/bin/env python

import rospy
import actionlib
import leader
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def handle_inputs():
    rospy.myargv(argv=sys.argv)
    if len(sys.argv) == 2:
        movebase_client(sys.argv[1])
    else:
        print("Not enough arguments")
    return


def movebase_client(location):
    location_x = location[0]
    location_y = location[1]

    client = actionlib.SimpleActionClient('/leader/move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = location_x
    goal.target_pose.pose.position.y = location_y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    goal = True
    print('Leader arrived at goal')
    return goal


if __name__ == '__main__':
    rospy.init_node('movebase_client_leader')
    leader = leader.Leader()
    wypts = leader.waypoints
    keys_ = list(wypts.keys())
    for i in range(1, 5):
        goal_reached = False
        for num in range(0, 4):
            if i in keys_[num]:
                location_next = [float(wypts[keys_[num]]['position']['x']), float(wypts[keys_[num]]['position']['y'])]
                room = keys_[num]
                print('Leader new goal', room[1])
                while goal_reached is False:
                    goal_reached = movebase_client(location_next)
                continue
