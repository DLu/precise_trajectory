#!/usr/bin/env python
import rospy

from std_srvs.srv import Empty
from pr2_precise_trajectory import *
from pr2_precise_trajectory.converter import *
import argparse

from pr2_score_creator import *

BUTTON_LAG = 1.0
ALL = 'all'

class MotionPlayback:
    def __init__(self, filename, directory=None):
        rospy.init_node('motion_playback')
        self.time = None
        self.mi = 0
        
        self.score = Score(filename, directory)
        self.interface = Interface(self.score, None)

        #rospy.wait_for_service('/proceed')
        #self.proxy = rospy.ServiceProxy('/proceed', Empty)

        #if self.score.has_data():
        #    self.goto(0)
        #else:
        #    self.mode_switcher.mannequin_mode()

    def save_as_current(self):
        self.score.save(self.mi, self.get_physical_state())        

    def save_as_next(self):
        self.mi = self.score.insert(self.mi, self.get_physical_state())

    def get_physical_state(self):
        mode = self.keys[self.key_i]
        if mode==ALL:
            mode = self.keys[1:]
        else:
            mode = [mode]
        m = {}
        for key in mode:
            m[key] = self.controller.joint_watcher.get_positions( key )
        return m

    def delete(self):
        new_i = self.score.delete(self.mi)
        if new_i is None:
            return
        self.mi = new_i
        self.goto(0)

    def set_property(self):
        while not rospy.is_shutdown():
            print "(l)abel change | (s)ervice transition | (i)mpact transition | (r)emove transition | change (t)ime | e(x)it"
            selection = raw_input()
            if selection == 'l':
                field = 'label'
                print self.score.get_property(self.mi, field)
                value = raw_input()
                break
            elif selection == 't':
                field = 't'
                value = float(raw_input())
                break
            elif selection=='s':
                field = 'transition'
                value = 'service'
                break
            elif selection=='i':
                field = 'transition'
                value = 'impact'
                break
            elif selection=='r':
                field = 'transition'
                value = 'wait'
                break
            elif selection=='x':
                return
        self.score.set_property(self.mi, field, value)

    def change_time(self, factor, shift=True):
        self.score.scale_time(self.mi, factor, shift)

    def start_action(self, movements):
        self.mode_switcher.position_mode()
        self.controller.do_action(movements)
        self.mode_switcher.mannequin_mode()

    def play(self, starti=None):
        if starti is None:
            starti = self.mi

        if self.interface:
            self.interface.start(starti)
        self.start_action( self.score.get_subset(starti) )
        if self.interface:
            self.interface.done()
        self.mi = self.score.num_keyframes() - 1

    def goto(self, delta):
        new_i = self.mi + delta
        #m = self.score.get_state(new_i)
        m = self.score.get_keyframe(new_i)
        m2 = {}
        m2.update(m)
        if 'transition' in m2:
            del m2['transition']
        if 'label' in m2:
            rospy.loginfo('State: %s'%m2['label'])
        self.start_action( [m2] )
        self.mi = new_i

    def change_mode(self, delta):
        self.key_i += delta 
        self.key_i = self.key_i % len(self.keys)
        rospy.loginfo("Current Mode: %s"%self.keys[self.key_i])

    def toggle_teleop(self):
        if self.joy.axes_cb is None:
            rospy.loginfo("Teleop ON")
            self.joy.axes_cb = self.drive
        else:
            rospy.loginfo("Teleop OFF")
            self.joy.axes_cb = None

    def drive(self, axes):
        z = axes[0]
        x = axes[3]
        y = axes[2]
        self.controller.base.publish_command(.6*x, .6*y, .8*z)


    def spin(self):
        r = rospy.Rate(4)
        while not self.score.has_data() and not rospy.is_shutdown():
            r.sleep()
        print 'a'    

        while not rospy.is_shutdown():
            print 'b'    
            if self.interface:
                self.interface.cycle(self.mi)
            print 'c'
            r.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Motion Viewer')
    parser.add_argument('filename')
    parser.add_argument('directory', nargs='?')
    args = parser.parse_args()

    mp = MotionPlayback(args.filename, args.directory)
    mp.spin()

