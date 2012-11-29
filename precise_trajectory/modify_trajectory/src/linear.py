#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
import trajectory_msgs.msg
from trajectory_tools import load_trajectory, write_trajectory
from pr2_trajectory import BOTH_ARMS, ARM_JOINTS
import sys, math
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    if len(sys.argv)!=3:
        print "Usage: %s input output"
        exit(1)

    resolution = .1

    trajectory = load_trajectory( open(sys.argv[1]))
    opoints = trajectory.points
    last_pt = opoints[0]
    trajectory.points = [last_pt]

    for pt in opoints[1:]:
        st = last_pt.time_from_start.to_sec()
        tm =      pt.time_from_start.to_sec()
        df = tm - st
        nx = int(math.floor(df / resolution))

        if nx>0:
            for a in range(nx):
                ntd = (a+1)*resolution
                new_t = st + ntd
                frac = ntd / df
                np = JointTrajectoryPoint()
                np.positions = [x+(y-x)*frac for (x,y) in zip(last_pt.positions, pt.positions)]
                np.time_from_start = rospy.Duration(new_t)
                trajectory.points.append(np)
        trajectory.points.append(pt)
        last_pt = pt

    write_trajectory( trajectory, open(sys.argv[2], 'w') )


    
