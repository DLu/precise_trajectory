#!/usr/bin/python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_tools import load_trajectory, write_trajectory
from pr2_trajectory import *
import sys

Q = 0.0008
R = 0.0025
P = 0.001

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    if len(sys.argv)!=3:
        print "Usage: %s input output"
        exit(1)

    trajectory = load_trajectory( open(sys.argv[1]))
    graph_trajectory(trajectory, 'o', prefix_filter='r')

    for (i, name) in enumerate(trajectory.joint_names):
        q = Q
        r = R
        p = P
        x = trajectory.points[0].positions[i]

        for point in trajectory.points[1:]:
            a = point.positions[i]
            p = p + q
            k = p / (p+r)
            x = x + k * (a - x)
            p = (1 - k) * p
            point.positions[i] = x

    graph_trajectory(trajectory, '-', prefix_filter='r')
    if True:
        show_graph()

    write_trajectory( trajectory, open(sys.argv[2], 'w') )


