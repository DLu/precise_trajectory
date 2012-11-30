#!/usr/bin/env python
import roslib; roslib.load_manifest('modify_trajectory')
import rospy
import trajectory_msgs.msg
from trajectory_tools import load_trajectory, write_trajectory
import arm_navigation_msgs.srv
from pr2_trajectory import *
import sys

if __name__ == '__main__':
    rospy.init_node('filterer')
    infile = sys.argv[1]
    outfile = sys.argv[2]

    print "Converting %s to %s"%(infile, outfile)
    NAME = 'trajectory_filter1'
    rospy.wait_for_service('%s/filter_trajectory'%NAME)
    filter_trajectory = rospy.ServiceProxy('%s/filter_trajectory'%NAME, arm_navigation_msgs.srv.FilterJointTrajectory)

    trajectory = load_trajectory( open(infile))

    filtered_response = filter_trajectory(trajectory, None, [], rospy.Duration(1.0))
    filtered = filtered_response.trajectory

    if True:
        graph_trajectory(trajectory, 'o', prefix_filter='r_')
        graph_trajectory(filtered, '-', prefix_filter='r_')

        show_graph()

    write_trajectory( filtered, open(outfile, 'w') )






