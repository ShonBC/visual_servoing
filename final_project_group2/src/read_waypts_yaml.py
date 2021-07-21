#!/usr/bin/env python

"""
This Python file will take the information from "waypoints.yml" and convert the information to a data structure which
will be further used for ROS commands.
"""
import os

robot_waypoints = {}    # Dict created which will store the robot coordinates for each room.


def file_path():
    """Find file path for yaml file to read

    Returns:
        [str]: file path to yaml file
    """

    abs_path = os.path.abspath(__file__)
    file_dir = os.path.dirname(abs_path)
    parent_dir = os.path.dirname(file_dir)
    new_path = os.path.join(parent_dir, 'yaml')
    return new_path


def read_file(path):
    """
    Open a file and store its content in a list
    :param path: Absolute path for the file to open
    :type path: str
    :return: List of each line of the file
    :rtype: List
    """

    with open(path, 'r') as opened_file:
        state_list = []
        lines = opened_file.readlines()

        for line in lines:
            state_list.append(line)

        return state_list


def create_commands():
    """
    This function will return a dictionary containing all of the commands from waypoints.yaml.
    
    :return: dictionary of robot_waypoints
    
    """
    path = file_path()
    # Read 'waypoints.yaml' and return a list called initial_list.
    initial_list = read_file(path + '/' + 'waypoints.yaml')
    count = 0       # This variable will be used to order the rooms.
    section = ""    # This variable will be used to keep track of the rooms.
    sub_section = ""    # This ill be used to keep track of room's orientation or position.
    for elt in initial_list:
        new_elt = (" ".join(elt.split())).replace(":", "")  # This new variable deletes ":" and "\n" in elt and stores it in new_elt.
        # Check if elt is either "orientation", "position", "x", "y", "z" or "w'.
        if "orientation" in elt or "w" in elt or "x" in elt or "y" in elt or "z" in elt or "position" in elt:
            # If currrent elt contains orientation or position, create new empty dict in either "orientation" or "position" key inside room
            if new_elt == "orientation" or new_elt == "position":
                robot_waypoints[section][new_elt] = {}
                sub_section = new_elt   # Keep track of key being used. 
            else:
                # Store coordinate position for the particular sub_section(orientation or position) for room.
                robot_waypoints[section][sub_section][new_elt[0]] = new_elt[2:]     

        else:
            count += 1      # Increment count to indicate room order.
            section = (count, new_elt)  # Store room order and room name in tuple key.
            robot_waypoints[section] = {}   # Create new dict in value located in section.

    return robot_waypoints


# This code can be tested manually.
if __name__ == "__main__":
    robot_goals = create_commands()
