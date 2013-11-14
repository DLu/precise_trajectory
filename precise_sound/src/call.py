#!/usr/bin/python
import roslib; roslib.load_manifest('precise_sound')
import rospy
import actionlib
from precise_sound import Sound
from precise_sound.msg import *

import sys
rospy.init_node('SoundClient')
a = actionlib.SimpleActionClient('/precise_sound/play', PlaySoundAction)
a.wait_for_server()
goal = PlaySoundGoal()
goal.filename = sys.argv[1]
goal.header.stamp = rospy.Time.now()

a.send_goal(goal)


rospy.sleep(5)
