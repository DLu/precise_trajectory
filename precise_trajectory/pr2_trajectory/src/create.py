#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from trajectory_tools import write_trajectory
import sys

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')

    trajectory = JointTrajectory()
    trajectory.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint"]
    trajectory.points = [JointTrajectoryPoint(), JointTrajectoryPoint(), JointTrajectoryPoint()]
    trajectory.points[0].positions = [ 0.0,  0.0]
    trajectory.points[0].velocities = [0.0, 0.0]
    trajectory.points[0].time_from_start = rospy.Duration(0)
    trajectory.points[1].positions = [ 0.5,  0.2]
    trajectory.points[1].velocities = [0.0, 0.0]
    trajectory.points[1].time_from_start = rospy.Duration(1)
    trajectory.points[2].positions = [-1.5, -0.5]
    trajectory.points[2].velocities = [0.0, 0.0]
    trajectory.points[2].time_from_start = rospy.Duration(2)

    write_trajectory( trajectory, open(sys.argv[1], 'w') )


    
