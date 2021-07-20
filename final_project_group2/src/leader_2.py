#!/usr/bin/env python

import read_waypts_yaml


class Leader:
    def __init__(self, name='waffle'):
        self.waypoints = read_waypts_yaml.create_commands()
        self.name = name

    def read_waypoints(self):
        self.waypoints = read_waypts_yaml.create_commands()


if __name__ == '__main__':
    # rospy.init_node('node_name')
    leader = Leader()
    wypts = leader.waypoints

    print(wypts)
    keys_ = list(wypts.keys())
    print('keys', keys_)
    for i in range(0, 4):
        number = keys_[i]
        if number[0] == i+1:
            location = [float(wypts[keys_[i]]['position']['x']), float(wypts[keys_[i]]['position']['y'])]
            print(location)
