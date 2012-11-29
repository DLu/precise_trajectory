#!/usr/bin/python
import roslib; roslib.load_manifest('joint_limit_tools')
import rospy
from urdf_python.urdf import *
from arm_navigation_msgs.msg import JointLimits

def get_urdf():
    return URDF().parse(rospy.get_param('robot_description'))

def get_joint_limit(name, urdf=None):
    if urdf is None:
        urdf = get_urdf()

    j = JointLimits()
    j.joint_name = name
    jt = urdf.joints[name]

    if jt.joint_type != 'continuous':
        j.has_position_limits = True
        j.min_position = jt.limits.lower
        j.max_position = jt.limits.upper
    j.has_velocity_limits = True
    j.max_velocity = jt.limits.velocity
    return j

def get_joint_limits(joints):
    urdf = get_urdf()
    x = {}
    for joint in joints:
        x[joint] = get_joint_limit(joint, urdf)
    return x


