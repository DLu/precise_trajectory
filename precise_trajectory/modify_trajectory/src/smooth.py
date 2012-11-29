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

    for (i, point) in enumerate(trajectory.points):
        if i==0 or i==len(trajectory.points)-1:
            point.velocities = [0.0] * len( trajectory.joint_names )
        else:
            past = trajectory.points[i-1]
            future = trajectory.points[i+1]
            t = future.time_from_start - past.time_from_start
            for (j, name) in enumerate(point.joint_names):
                x = future.positions[j] - past.positions[j]
                point.velocities.append(x)

    write_trajectory( trajectory, open(sys.argv[2], 'w') )


    
