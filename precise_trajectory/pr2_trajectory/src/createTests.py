#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from trajectory_tools import write_trajectory
from pr2_trajectory import *
import sys

def np(pts, pos, time):
    pt = JointTrajectoryPoint()
    pt.positions = [pos]

    if len(pts)==0:
        st = rospy.Duration(0)
    else:
        st = pts[-1].time_from_start
    pt.velocities = [0]
    pt.time_from_start = st + rospy.Duration(time)
    pts.append(pt)

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')

    for (name, (low, hi)) in TEST_RANGES.items():
        trajectory = JointTrajectory()
        trajectory.joint_names = [name]
        pts = trajectory.points

        np(pts, hi, 0)
        for t in [4, 3, 2, 1, 0.5]:
            np(pts, low, t)
            np(pts, hi, t)
            np(pts, hi, 2)

        write_trajectory( trajectory, open("tests/%s.traj"%name, 'w') )

