#!/usr/bin/python
import roslib; roslib.load_manifest('joint_limit_tools')
import rospy
from urdf_python.urdf import *
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
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

def calculate_relative_speed(trajectory, arms=['l', 'r']):
    names = {}
    all_names = []
    for arm in arms:
        names[arm] = get_arm_joint_names(arm)
        all_names += names[arm]
    limits = get_joint_limits(all_names)    

    prev = None
    array = []
    for move in trajectory:
        if prev is None:
            prev = move
            continue
        m = {}
        t = move.get('time', 3.0)
        for arm in arms:
            for name, p1, p2 in zip(names[arm], prev[arm], move[arm]):
                limit = limits[name]
                vlimit = limit.max_velocity
                v = abs(p2-p1)/t
                m[name] = v / vlimit
        array.append(m)
    return array

