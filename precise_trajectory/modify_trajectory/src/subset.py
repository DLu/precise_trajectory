#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
import trajectory_msgs.msg
from trajectory_tools import load_trajectory, write_trajectory
from pr2_trajectory import kinect_to_pr2, traj_to_joint_state
import sys

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    if len(sys.argv)!=3:
        print "Usage: %s input output"
        exit(1)

    trajectory = load_trajectory( open(sys.argv[1]))
    #subset = [0,  17,  21,  27,  32,  39,  47,  56,  67,  78,  96] # wave
    #subset = [0, 7, 19, 26, 30, 42, 50, 55, 60, 68, 84] #beckon 
    #subset = [0, 31, 45,60, 69, 82, 111, 132]#look
    #subset = [0, 29, 68, 91]# shrug
    subset = [0, 19, 28, 38, 47, 57, 69, 79, 92, 99]#refuse
    #subset = [1559, 1691]

    if len(subset)==2:
        trajectory.points = trajectory.points[subset[0] : subset[1]+1]
    else:
        all_points = trajectory.points
        trajectory.points = []
        for idx in subset:
            trajectory.points.append( all_points[idx] )

    start_time = trajectory.points[0].time_from_start
    for point in trajectory.points:
        point.time_from_start = point.time_from_start - start_time

    write_trajectory( trajectory, open(sys.argv[2], 'w') )


    
