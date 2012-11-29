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

    for arm in BOTH_ARMS:
        tolerance = JointTolerance()
        tolerance.name = '%s_shoulder_lift_joint'%arm
        tolerance.position = .10
        j.follow(arm, tmap[arm], [tolerance])

    print j.follow_result(BOTH_ARMS)      

