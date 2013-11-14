#!/usr/bin/python
import roslib; roslib.load_manifest('precise_sound')
import rospy
import actionlib
from precise_sound import Sound
from precise_sound.msg import *

import sys
rospy.init_node('SoundClient')
a = actionlib.SimpleActionClient('/precise_sound/play', PlaySoundsAction)
a.wait_for_server()
goal = PlaySoundsGoal()
goal.filenames.append(sys.argv[1])
goal.header.stamp = rospy.Time.now()
goal.times.append(0.0)

a.send_goal(goal)


rospy.sleep(5)
