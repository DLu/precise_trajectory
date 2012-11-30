#!/usr/bin/python
import roslib; roslib.load_manifest('modify_trajectory')
import rospy
from trajectory_tools import load_trajectory, write_trajectory
from pr2_trajectory import *
from mocap_analysis.pca import *
import sys

def trajectory_to_data(trajectory):
    data = []
    for pt in trajectory.points:
        data.append( pt.positions )
    return data

def data_to_trajectory(data, trajectory):
    for (pt, row) in zip(trajectory.points, data):
        pt.positions = row
    return trajectory


if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    if len(sys.argv)>3:
        comps = []
        for arg in sys.argv[3:]:
            if ":" in arg:
                i = arg.index(":")
                a1 = int(arg[:i])
                a2 = int(arg[i+1:])
                comps+= range(a1, a2)
            elif arg=='m':
                break
            else:
                comps.append(int(arg))

    else:
        comps = None


    trajectory = load_trajectory( open(sys.argv[1]))    
    graph_trajectory(trajectory, 'o', prefix_filter='r')

    data = trajectory_to_data(trajectory)
    (c,v,m) = pca_data(data)

    
    data2 = reconstruct(c,v,m, comps)
    trajectory = load_trajectory( open(sys.argv[1]))    
    traj2 = data_to_trajectory(data2, trajectory)
    graph_trajectory(traj2, '-', prefix_filter='r')
    if True:
        show_graph()

    write_trajectory( traj2, open(sys.argv[2], 'w') )


