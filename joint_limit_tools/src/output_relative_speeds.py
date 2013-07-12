#!/usr/bin/env python
import roslib; roslib.load_manifest('joint_limit_tools')
import rospy
from pr2_precise_trajectory import *
from pr2_precise_trajectory.converter import load_trajectory
from joint_limit_tools import calculate_relative_speed
import sys

rospy.init_node('relative_speeds')
trajectory = load_trajectory(sys.argv[1])
limits = calculate_relative_speed(trajectory, [RIGHT])
for m in limits:
    for k,v in m.iteritems():
        print '%s: %f'%(k,v),
    print

