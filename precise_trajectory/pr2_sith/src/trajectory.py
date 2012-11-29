#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_sith.lightsaber import RobotController
import sys
import yaml
import random

if __name__ == '__main__':
    rospy.init_node('sith')

    scripts = []
    movements = yaml.load( open(sys.argv[1], 'r'))
    controller = RobotController()

    for arm in ['l', 'r']:
        if arm not in movements[0]:
            continue
        goal = controller.make_trajectory_long(movements, arm)
        controller.start_trajectory(goal, arm)
    rospy.spin()

