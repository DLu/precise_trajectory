#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_tools import load_trajectory
from pr2_trajectory import *
import sys
import os.path

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')

    args = sys.argv[1:]
    args.reverse()
    
    for arg in args:
        trajectory = load_trajectory( open(arg) )
        if arg == args[-1]:
            width = 2
            ms = 8
        else:
            width = 1
            ms =4 
        fn = os.path.basename(arg).split('.')[0]
        graph_trajectory(trajectory, 'o-', prefix_filter='r_',  label_prefix=fn, linewidth=width, markersize=ms)

    show_graph(loc=9)

