#!/usr/bin/python
import roslib; roslib.load_manifest('precise_sound')
import rospy
import actionlib
from precise_sound import Sound
from precise_sound.msg import *

ACTION_NAME = '/precise_sound/play'

class SoundServer:
    def __init__(self):
        rospy.init_node('SoundServer')
        self.server = actionlib.SimpleActionServer(ACTION_NAME, PlaySoundsAction, self.execute, False) 
        self.server.start()

    def execute(self, goal):
        r = rospy.Rate(100)

        for filename, time in zip(goal.filenames, goal.times):
            sound = Sound(filename)

            # wait to start
            while rospy.Time.now() < goal.header.stamp + rospy.Duration(time):
                r.sleep()

            sound.play()

        self.server.set_succeeded()

if __name__=='__main__':
    pm = SoundServer()
    rospy.spin()
