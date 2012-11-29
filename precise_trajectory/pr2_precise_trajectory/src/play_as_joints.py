#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_precise_trajectory.converter import simple_to_joint_states
import sys
import yaml

if __name__ == '__main__':
    movements = yaml.load( open(sys.argv[1], 'r'))
    rospy.init_node('play_as_joints')
    pub = rospy.Publisher('/arm_angles', JointState)

    for state in simple_to_joint_states(movements):
        wait = (state.header.stamp - rospy.Time.now()).to_sec()
        if wait > 0:
            rospy.sleep(wait)
        pub.publish(state)

        if rospy.is_shutdown():
            break


