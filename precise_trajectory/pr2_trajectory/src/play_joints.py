#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from sensor_msgs.msg import JointState
import trajectory_msgs.msg
from trajectory_tools import load_trajectory
from pr2_trajectory import kinect_to_pr2, traj_to_joint_state
import sys

pub = rospy.Publisher('/arm_angles', JointState)

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    trajectory = load_trajectory( open(sys.argv[1]))
    trajs = kinect_to_pr2(trajectory).values()
    last = rospy.Duration(0.0)
    for (t, point) in enumerate(trajectory.points):
        wait = point.time_from_start - last
        last = point.time_from_start
        rospy.sleep(wait)
        state = traj_to_joint_state(trajs, t)
        pub.publish(state)

        if rospy.is_shutdown():
            break

