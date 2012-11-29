#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_sith.lightsaber import RobotController, ARM_JOINTS
import sys
import yaml
import random

from pr2_controllers_msgs.msg import *
from pr2_gripper_sensor_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('sith')
    angle_set = [                                                                                                            
[-0.48482740922451023, -0.1068084172004516,   0.55686040914859314, -0.059769434915311281, -0.63266340164037371, 0.0096497613233154089, -9.7171674477394649],
[-0.2229254049937921,   0.31887323586116434,  0.55686040914859314, -0.012139870870130798, -0.38669758792659065, 0.0096497613233154089, -9.7171674477394649],
[ 0.35236361284633211, -0.074746941343863268, 0.55285153848934598, -0.058032186378708994, -0.38733390588304684, 0.0095627434831161429, -9.7172544655796642],
[-0.13529311044745829, -0.3808959680588847,   0.55285153848934598, -0.1249162550378986,   -0.38733390588304684, 0.0095627434831161429, -9.7172544655796642]]

    N = int(sys.argv[1])
    v = float(sys.argv[2])
    duration = float(sys.argv[3])

    velocities = [ [0, v], [v,0], [0,-v], [-v,0] ]
    controller = RobotController()
    arm = 'r'

    # our goal variable
    goal = JointTrajectoryGoal()
    # Start the trajectory immediately
    goal.trajectory.header.stamp = rospy.Time.now()

    # First, the joint names, which apply to all waypoints
    names = ['%s_%s'%(arm, a) for a in ARM_JOINTS]
    goal.trajectory.joint_names = names

    # We will have N waypoints in this goal trajectory
    goal.trajectory.points = []
    for i in range(N):
        pt = trajectory_msgs.msg.JointTrajectoryPoint()
        ii = i % len(angle_set)
        pt.positions = angle_set[ii]
        if v>=0.0:
            pt.velocities = velocities[ii] + [0.0]*5
        pt.time_from_start = rospy.Duration(duration*(i+1))
        goal.trajectory.points.append(pt)
    print goal

    controller.start_trajectory(goal, arm)
    rospy.spin()
