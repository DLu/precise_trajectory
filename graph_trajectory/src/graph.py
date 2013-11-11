#!/usr/bin/env python
import roslib; roslib.load_manifest('graph_trajectory')
import rospy
from pr2_precise_trajectory.converter import load_trajectory
from graph_trajectory import *
import sys
import os.path

if __name__ == '__main__':
    args = sys.argv[1:]
    args.reverse()
    
    for arg in args:
        trajectory = load_trajectory( arg )
        if arg == args[-1]:
            width = 2
            ms = 8
        else:
            width = 1
            ms =4 
        fn = os.path.basename(arg).split('.')[0]
        graph_trajectory(trajectory, 'o-', prefix_filter='r_',  label_prefix=fn, linewidth=width, markersize=ms)

    show_graph(True)
    raw_input()

