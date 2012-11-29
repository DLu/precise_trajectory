#!/usr/bin/python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_tools import load_trajectory, write_trajectory
from pr2_trajectory import *
import sys

def moving_average( data, W=1, B=2 ):
    avg = []

    for (i,x) in enumerate(data):
        start = max(0, i-W)
        end = min(i+W+1, len(data))
        total = 0.0
        divisor = 0.0
        
        for j in range(start, end):
            dist = abs(j-i)
            f = pow(B, -(dist))
            total += data[j] * f
            divisor += f

        x = total / divisor
        avg.append(x)
    return avg

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    if len(sys.argv)!=3:
        print "Usage: %s input output"
        exit(1)

    trajectory = load_trajectory( open(sys.argv[1]))    
    graph_trajectory(trajectory, 'o', prefix_filter='r')

    for (i, name) in enumerate(trajectory.joint_names):
        x = trajectory.points[0].positions[i]
        original = [x]

        for point in trajectory.points[1:]:
            a = point.positions[i]
            original.append(a)
        filtered = moving_average( original )
        for (pt, v) in zip(trajectory.points, filtered):
            pt.positions[i] = v

    graph_trajectory(trajectory, '-', prefix_filter='r')
    if True:
        show_graph()

    write_trajectory( trajectory, open(sys.argv[2], 'w') )


