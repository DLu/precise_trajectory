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
        self.server = actionlib.SimpleActionServer(ACTION_NAME, PlaySoundAction, self.execute, False) 
        self.server.start()

    def execute(self, goal):
        r = rospy.Rate(100)
        sound = Sound(goal.filename)

        # wait to start
        while rospy.Time.now() < goal.header.stamp:
            r.sleep()
        sound.play()

if __name__=='__main__':
    pm = SoundServer()
    rospy.spin()
