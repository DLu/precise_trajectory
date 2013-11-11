#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_score_creator')
import rospy

from joy_listener import JoyListener, PS3
#from sensor_msgs.msg import JointState
from pr2_precise_trajectory import *
from pr2_precise_trajectory.full_controller import FullPr2Controller
from pr2_precise_trajectory.converter import *
import argparse

from pr2_score_creator import *

BUTTON_LAG = 1.0
ALL = 'all'

class InteractiveRecorder:
    def __init__(self, keys, filename, impact=False):
        rospy.init_node('interactive_recorder')
        self.time = None
        self.keys = [ALL] + keys
        self.key_i = 0
        self.change_mode(0)
        self.mi = 0
        
        self.score = Score(filename)
        self.mode_switcher = ModeSwitcher(keys)
        self.controller = FullPr2Controller(keys=keys, impact=impact)

        self.joy = JoyListener(BUTTON_LAG)
        self.joy[ PS3('x') ] = self.save_as_current
        self.joy[ PS3('circle') ] = self.save_as_next 
        self.joy[ PS3('triangle') ] = self.delete
        self.joy[ PS3('square') ] = self.set_property
        self.joy[ PS3('right') ] = lambda: self.goto(1)
        self.joy[ PS3('left') ] = lambda: self.goto(-1)
        self.joy[ PS3('up') ] = lambda: self.change_mode(-1)
        self.joy[ PS3('down') ] = lambda: self.change_mode(1)
        self.joy[ PS3('select') ] = self.play # play from here
        self.joy[ PS3('start') ] = lambda: self.play(0) # play from start
        self.joy[ PS3('ps3') ] = self.to_file
        self.joy[ PS3('r1') ] = lambda: self.change_time(1.1) 
        self.joy[ PS3('r2') ] = lambda: self.change_time(1.5)
        self.joy[ PS3('l1') ] = lambda: self.change_time(.9)
        self.joy[ PS3('l2') ] = lambda: self.change_time(.5)
        self.joy[ PS3('left_joy') ] = self.toggle_teleop

        if self.score.has_data()
            self.goto(0)
        else:
            self.mode_switcher.mannequin_mode()

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
        field = raw_input("Field: ")
        value = raw_input("Value: ")
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

        # TODO: start interface
        self.start_action( self.score.get_subset(starti) )
        self.mi = self.score.num_keyframes()

    def goto(self, delta):
        new_i = self.mi + delta
        m = self.score.get_keyframe(new_i)
        self.start_action( [m] )
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
        while not self.score.has_data():
            r.sleep()

        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Interactive Motion Recorder')
    parser.add_argument('filename')
    parser.add_argument('-l', dest='keys', action='append_const', const=LEFT)
    parser.add_argument('-r', dest='keys', action='append_const', const=RIGHT)
    parser.add_argument('-b', dest='keys', action='append_const', const=BASE)
    parser.add_argument('-p', '--head', dest='keys', action='append_const', const=HEAD)
    parser.add_argument('-lh', dest='keys', action='append_const', const=LEFT_HAND)
    parser.add_argument('-rh', dest='keys', action='append_const', const=RIGHT_HAND)
    parser.add_argument('-a', '--all', action='store_true', dest='all')
    parser.add_argument('-i', '--impact', action='store_true', dest='impact')

    args = parser.parse_args()
    if args.all:
        args.keys = [LEFT, RIGHT, BASE, HEAD, LEFT_HAND, RIGHT_HAND]

    if args.keys is None or len(args.keys)==0:
        print "Must specify at least one part"
        exit(1)

    ir = InteractiveRecorder(args.keys, args.filename, args.impact)
    ir.spin()

