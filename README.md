ENPM809E - Python Applications for Robotics

Group 2: Souvik Pramanik, Brenda Scheufele, Shon Cortes

July 21, 2021

# visual_servoing
The final project consists of two waffle turtlebots, named leader and follower. The goal of this assignment is to make follower follow leader using visual servoing. Visual servoing, also known as vision-based robot control,is a technique which uses feedback information extracted from a vision sensor (visual feedback) to control the motion of a robot. Visual servoing is possible with the waffle model since it comes with a depth camera which also publishes RGB images. We can observe the live stream from the camera in the bottom left corner of rviz.We will use the follower camera to perceive the leader robot in the environment, and then try to follow it. To make it easier, a marker has been attached to leader. Follower has to use its camera to find this marker and follow the marker by keeping a safe distance.

1. Place the visual_servoing package in the src folder within a catkin workspace. 
2. Use "catkin build" to build the workspace. 
3. 