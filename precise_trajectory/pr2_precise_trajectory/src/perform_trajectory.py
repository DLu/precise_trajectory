#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_precise_trajectory.full_arm_controller import FullArmController
import sys
import yaml

if __name__ == '__main__':
    rospy.init_node('perform_trajectory')

    movements = yaml.load( open(sys.argv[1], 'r'))
    controller = FullArmController()

    controller.start_action(movements)
    rospy.spin()

