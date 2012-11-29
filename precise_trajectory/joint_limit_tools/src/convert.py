#!/usr/bin/python
import roslib; roslib.load_manifest('joint_limit_tools')
import rospy
from urdf_python.urdf import *
from arm_navigation_msgs.msg import JointLimits
import joint_limit_tools

rospy.init_node('convert')

BOTH_ARMS = ['l', 'r']
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

joints = []

for arm in BOTH_ARMS:
    for joint in ARM_JOINTS:
        joints.append("%s_%s"%(arm, joint))

limits = joint_limit_tools.get_joint_limits(joints)

for joint in joints:
    print limits[joint]
