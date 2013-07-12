#!/usr/bin/env python
import roslib; roslib.load_manifest('graph_trajectory')
import rospy
from pr2_precise_trajectory.converter import load_trajectory
from graph_trajectory import *
import sys
import os.path

if __name__ == '__main__':
    arg = sys.argv[1]
    trajectory = load_trajectory( arg )
    g = Grapher(keys=trajectory[0].keys())
    g.graph(trajectory)
    g.show(True)

