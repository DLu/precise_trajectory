#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_precise_trajectory.full_arm_controller import FullArmController
import sys
import yaml

if __name__ == '__main__':
    rospy.init_node('sith')

    scripts = []
    scripts.append(yaml.load( open("scripts/start%s.yaml"% sys.argv[1], 'r')))
    scripts.append(yaml.load( open("scripts/attack%s.yaml"%sys.argv[2], 'r')))
    scripts.append( scripts[0] ) 

    controller = FullArmController(['r'])
    controller.do_action(scripts)

