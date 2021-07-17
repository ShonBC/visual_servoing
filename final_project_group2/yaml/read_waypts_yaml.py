"""
This Python file will take the information from "waypoints.yml" and convert the information to a data structure which
will be further used for ROS commands.

"""

robot_waypts = {}


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
            # print(line)

        return state_list


def create_commands():
    """
    This function will return a dictionary containing all of the commands from waypoints.yaml.
    :return: dictionary robot_waypts
    """
    initial_list = read_file('waypoints.yaml')
    section = ""
    sub_section = ""
    for elt in initial_list:
        new_elt = (" ".join(elt.split())).replace(":", "")
        print(new_elt)
        if "orientation" in elt or "w" in elt or "x" in elt or "y" in elt or "z" in elt or "position" in elt:
            if new_elt == "orientation" or new_elt == "position":
                robot_waypts[section][new_elt] = {}
                sub_section = new_elt
            else:
                # if "w" in new_elt:
                #     robot_waypts[section]["orientation"]["w"] = new_elt[2:]
                # else:
                robot_waypts[section][sub_section][new_elt[0]] = new_elt[2:]

        else:
            robot_waypts[new_elt] = {}
            section = new_elt

        return robot_waypts


if __name__ == "__main__":
    robot_cmds = create_commands()
    print(robot_cmds)



