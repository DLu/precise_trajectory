#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_joint_recorder')
import rospy
from joy_listener import JoyListener, PS3
from pr2_precise_trajectory import *
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
from sensor_msgs.msg import JointState

last = {}

def joint_cb(msg):
    for (name, pos) in zip(msg.name, msg.position):
        last[name] = pos

def print_joints(side):
    curr = dict(last)
    arr = []
    for name in get_arm_joint_names(side):
        arr.append(curr[name])
    print side + ': '  + repr(arr)

def print_right():
    print_joints(RIGHT)
def print_left():
    print_joints(LEFT)
def newline():
    print

if __name__ == '__main__':
    rospy.init_node('recorder')
    joy = JoyListener()
    joy[ PS3('right') ] = print_right
    joy[ PS3('left') ] = print_left
    joy[ PS3('start') ] = newline
    sub1 = rospy.Subscriber('/joint_states', JointState, joint_cb)
    rospy.spin()

