#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from trajectory_tools import load_trajectory
from pr2_trajectory import *
import sys

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    trajectory = load_trajectory( open(sys.argv[1]))
    #for pt in trajectory.points:
    #    pt.time_from_start = rospy.Duration( pt.time_from_start.to_sec() * 5 )

    j = JointActor()
    rec = Recorder()

    tmap = kinect_to_pr2(trajectory)

    j.presets(tmap, 3.0)
    rospy.sleep(1.0)
    rec.start()
    j.actions(tmap)
    rec.stop()

    graph_trajectory(trajectory, 'o', 'r_')
    graph_joints(rec.data, '-', rec.start_time, trajectory.joint_names)

    show_graph()
    
