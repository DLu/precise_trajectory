#!/usr/bin/env python
import roslib; roslib.load_manifest('joint_limit_tools')
import rospy
from pr2_precise_trajectory import *
from pr2_precise_trajectory.converter import load_trajectory, save_trajectory, tprint
import sys
from joint_limit_tools import calculate_relative_speed

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    if len(sys.argv)!=3:
        print "Usage: %s input output"
        exit(1)

    trajectory = load_trajectory( sys.argv[1] )
    limit_factors = calculate_relative_speed(trajectory, [RIGHT])
    worst_ratio = max( [max(m.values()) for m in limit_factors] )


    for move in trajectory:
        t = get_time( move )
        move[TIME] = t * worst_ratio
    limit_factors = calculate_relative_speed(trajectory, [RIGHT])
    worst_ratio2 = max( [max(m.values()) for m in limit_factors] )
    print "%.2f => %.2f"%(worst_ratio, worst_ratio2)
    
    save_trajectory( trajectory, sys.argv[2] )


    
