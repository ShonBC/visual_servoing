#!/usr/bin/env python

import read_waypts_yaml


class Leader:
    """ This class initializes the robot, and a set of waypoints that will be visited in order. """

    def __init__(self, name='waffle'):
        """ Initialize class attributes

        Args:
            name (str): Name of robot.  Default is "waffle"
            waypoints (dict): dictionary of rooms to visit, their priority and their coordinates.
        """

        self.waypoints = read_waypts_yaml.create_commands()
        self.name = name

    def read_waypoints(self):
        """Calls the function to read the .yaml file where waypoints are stored"""

        self.waypoints = read_waypts_yaml.create_commands()


if __name__ == '__main__':
    leader = Leader()  # Initialize leader
    waypoints = leader.waypoints
    keys_ = list(waypoints.keys())  
    # Stores a new location based on waypoints.
    location = []
    for i in range(0, 4):
        number = keys_[i]
        print(number)
        if i+1 == number[0]:
            location = [float(waypoints[keys_[i]]['position']['x']), float(waypoints[keys_[i]]['position']['y'])]

