#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
import trajectory_msgs.msg
from trajectory_tools import load_trajectory, write_trajectory
from pr2_trajectory import BOTH_ARMS, ARM_JOINTS
import sys
import joint_limit_tools

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    if len(sys.argv)!=3:
        print "Usage: %s input output"
        exit(1)

    trajectory = load_trajectory( open(sys.argv[1]))
    joints = []

    for arm in BOTH_ARMS:
        for joint in ARM_JOINTS:
            joints.append("%s_%s"%(arm, joint))

    limits = {'r_elbow_flex_joint': 0.800652, 'r_shoulder_lift_joint': 0.197003, 'r_upper_arm_roll_joint': 2.334443, 'r_wrist_roll_joint': 2.426422, 'r_shoulder_pan_joint': 0.200658, 'r_forearm_roll_joint': 1.607750, 'r_wrist_flex_joint': 2.157186, 'l_elbow_flex_joint': 0.800652, 'l_shoulder_lift_joint': 0.197003, 'l_upper_arm_roll_joint': 2.334443, 'l_wrist_roll_joint': 2.426422, 'l_shoulder_pan_joint': 0.200658, 'l_forearm_roll_joint': 1.607750, 'l_wrist_flex_joint': 2.157186}
    worst_ratio = 1
    times = [0]

    for (i, point) in enumerate(trajectory.points):
        if i==0:
            continue

        past = trajectory.points[i-1]
        t = (point.time_from_start - past.time_from_start).to_sec()
        times.append(t)

        for (j, name) in enumerate(trajectory.joint_names):
            x = point.positions[j] - past.positions[j]
            v = abs(x/t)
            lim = limits[name]
            ratio = v / lim
            if ratio > worst_ratio:
                print name, ratio, i
                worst_ratio = ratio
    print worst_ratio

    for (i, point) in enumerate(trajectory.points):
        if i==0:
            continue

        past = trajectory.points[i-1]
        point.time_from_start = past.time_from_start + rospy.Duration( times[i] * worst_ratio )

    write_trajectory( trajectory, open(sys.argv[2], 'w') )


    
