#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from sensor_msgs.msg import JointState
import trajectory_msgs.msg
from trajectory_tools import write_trajectory
from pr2_trajectory import *
import sys

data = []
names = []
indexes = []
times = []

def joint_cb(msg):
    if len(indexes)==0:
        for name in names:
            indexes.append( msg.name.index(name) )

    array = []
    for index in indexes:
        array.append( msg.position[index] )

    data.append(array)
    times.append(msg.header.stamp)

def diff(a,b):
    t = 0
    for (x,y) in zip(a,b):
        t += abs(x-y)
    return t

LIMIT = 0.001

if __name__ == '__main__':
    rospy.init_node('recorder')
    pub = rospy.Subscriber('/joint_states', JointState, joint_cb)
    for arm in BOTH_ARMS:
        for angle in ARM_JOINTS:
            names.append("%s_%s"%(arm, angle))


    while not rospy.is_shutdown():
        try:
            rospy.sleep(1.0)
        except:
            break

    t = JointTrajectory()
    t.joint_names = names
    firstTime = None
    lastAngles = None

    for (time, angles) in zip(times, data):
        if firstTime is None:
            if diff(angles, data[0])>LIMIT:
                firstTime = time
        elif diff(angles, lastAngles)>LIMIT:
            pt = JointTrajectoryPoint()
            pt.positions = angles
            pt.time_from_start = time - firstTime
            t.points.append(pt)
        lastAngles = angles
    t.points = t.points[:-1]

    trajectory = write_trajectory( t, open(sys.argv[1], 'w'))


