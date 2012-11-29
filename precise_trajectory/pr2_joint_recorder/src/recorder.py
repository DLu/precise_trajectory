#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_joint_recorder')
import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

last = {}
current = []

def joint_cb(msg):
    for (name, pos) in zip(msg.name, msg.position):
        last[name] = pos

START = 3
RIGHT = 5
LEFT  = 7
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
SECONDS = 2

time = None

def print_joints(side):
    curr = dict(last)
    arr = []
    for j in ARM_JOINTS:
        arr.append(curr['%s_%s' % (side, j)])
    print side + ': '  + repr(arr)

def joy_cb(msg):
    buttons = msg.buttons
    global time
    if (time is None or rospy.get_rostime() - time).to_sec() < SECONDS:
        return

    if buttons[RIGHT]:
        print_joints('r')
        time = rospy.get_rostime()
    elif buttons[LEFT]:
        print_joints('l')
        time = rospy.get_rostime()
    elif buttons[START]:
        print
        time = rospy.get_rostime()

if __name__ == '__main__':
    rospy.init_node('recorder')
    time = rospy.get_rostime()
    sub1 = rospy.Subscriber('/joint_states', JointState, joint_cb)
    sub2 = rospy.Subscriber('/joy', Joy, joy_cb)
    rospy.spin()

