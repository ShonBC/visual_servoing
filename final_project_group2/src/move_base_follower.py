#!/usr/bin/env python

from genpy import message
import rospy
import actionlib
import follower
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


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
    wait = client.wait_for_result()
    goal = True
    print('Leader arrived at goal')
    return goal

def follower_callback(msg):
    x_pose = msg.pose.position.x
    y_pose = msg.pose.position.y
   

if __name__ == '__main__':
    rospy.init_node('movebase_client_follower')
    follow = follower.Follower()
    leader_goal = rospy.Subscriber('/leader/move_base_simple/goal', PoseStamped, follower_callback, queue_size=1)
    
    
    while True:
        print('begin')
        wypts = follow.get_follower_goal()
        print(wypts)
        goal_reached = False
        
        if wypts: # If marker is detected, follow
            print('marker detected')
            
            while goal_reached is False:
                goal_reached = movebase_client(wypts)

        else: # If no marker, rotate 360, 
            print('rotate')
            angle = 0
            while angle < 360:
                angle += 5
                follow.rotate(angle)
                wypts = follower.get_follower_goal()

                if wypts: # if marker is found follow
                    print('marker detected 2')
                    while goal_reached is False:
                        goal_reached = movebase_client(wypts)
                    
                    break

            else: #Look to parameter server to go to leaders goal
                while goal_reached is False:# use subscriber to read leader goal and call movebase_client(leader_goal)
                    print('param server')
                    goal_reached = movebase_client(leader_goal)
                    # Print "cant find marker, going to leader goal"
    rospy.spin()     
