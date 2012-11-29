#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_tools import load_trajectory
from pr2_trajectory import *
import sys

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    trajectory = load_trajectory( open(sys.argv[1]))
    tmap = kinect_to_pr2(trajectory)
    j = JointActor()
    j.actions(tmap)

