#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_joint_recorder')
import rospy
import sys
import yaml
import os.path
from joy_listener import JoyListener, PS3
from sensor_msgs.msg import JointState
from pr2_precise_trajectory import *
from pr2_precise_trajectory.full_controller import FullPr2Controller
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
from pr2_precise_trajectory.joint_watcher import JointWatcher
from pr2_precise_trajectory.converter import *
from pr2_mechanism_msgs.srv import SwitchController
from graph_trajectory import *

MANNEQUIN_CONTROLLER = '%s_arm_controller_loose'
POSITION_CONTROLLER = '%s_arm_controller'

BUTTON_LAG = 1.0

class InteractiveRecorder:
    def __init__(self, arms, filename):
        rospy.init_node('interactive_recorder')
        self.running = True
        self.time = None
        self.filename = filename
        self.arms = arms
        self.mi = 0
        self.recorded = []
        self.grapher = Grapher()
        
        self.controllers = {}
        for arm in self.arms:
            self.controllers[arm] = POSITION_CONTROLLER % arm

        if os.path.exists(self.filename):
            self.movements = load_trajectory(self.filename)
        else:
            self.movements = []

        self.controller = FullPr2Controller(keys=arms, impact=False)
        rospy.loginfo("Waiting for control manager")
        rospy.wait_for_service('pr2_controller_manager/switch_controller')
        rospy.loginfo("Got control manager!")
        self.switcher = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)

        self.switch_to(MANNEQUIN_CONTROLLER)

        self.joy = JoyListener(BUTTON_LAG)
        self.joy[ PS3('x') ] = self.save_as_current
        self.joy[ PS3('circle') ] = self.save_as_next 
        self.joy[ PS3('triangle') ] = self.delete
        self.joy[ PS3('square') ] = self.set_impact
        self.joy[ PS3('right') ] = lambda: self.goto(1)
        self.joy[ PS3('left') ] = lambda: self.goto(-1)
        self.joy[ PS3('select') ] = self.play # play from here
        self.joy[ PS3('start') ] = lambda: self.play(0) # play from start
        self.joy[ PS3('ps3') ] = self.to_file
        self.joy[ PS3('r1') ] = lambda: self.change_time(1.1) 
        self.joy[ PS3('r2') ] = lambda: self.change_time(1.5)
        self.joy[ PS3('l1') ] = lambda: self.change_time(.9)
        self.joy[ PS3('l2') ] = lambda: self.change_time(.5)

        if len(self.movements) > 0:
            self.goto(0)

    def save_as_current(self):
        self.save(self.mi)

    def save_as_next(self):
        self.save(self.mi + 1, insert=True)

    def save(self, index, insert=False):
        if len(self.movements)==0:
            insert = True
        if not insert:
            m = self.movements[index]
        else:
            m = {}

        for arm in self.arms:
            m[arm] = self.jwatcher.get_positions( get_arm_joint_names(arm) )

        if insert and index < len(self.movements):
            t = get_time( self.movements[index] )
            m[TIME] = t / 2
            self.movements[index][TIME] = t / 2

        if insert:
            if index > len(self.movements):
                index = len(self.movements)
            self.movements = self.movements[:index] + [m] + self.movements[index:]
        else:
            self.movements[index] = m
        self.mi = index

    def delete(self):
        if self.mi >= len(self.movements):
            return
        elif self.mi == len(self.movements)-1:
            self.movements = self.movements[:self.mi]
            self.mi -= 1
        else:
            t =  get_time( self.movements[self.mi]   )
            t2 = get_time( self.movements[self.mi+1] )
            self.movements[self.mi+1][TIME] = t + t2
            self.movements = self.movements[:self.mi] + self.movements[self.mi+1:]
        self.goto(0)

    def set_impact(self):
        if self.mi >= len(self.movements):
            return

        m = self.movements[self.mi]
        if 'transition' not in m or m['transition'] == 'wait':
            m['transition'] = 'impact'
        else:
            m['transition'] = 'wait'

        print m['transition']

    def change_time(self, factor):
        if self.mi >= len(self.movements):
            return
        m = self.movements[self.mi]
        t = get_time( m )
        nt = t * factor
        print "%f ==> %f"%(t, nt)
        m[TIME] = nt

    def start_action(self, movements):
        self.switch_to(POSITION_CONTROLLER)
        self.controller.do_action(movements)
        self.switch_to(MANNEQUIN_CONTROLLER)

    def play(self, starti=None):
        if starti is None:
            starti = self.mi
        self.recorded = None
        self.controller.joint_watcher.record()
        t_off = self.total_time(starti-1)
        print t_off, starti
        self.start_action( self.movements[starti:] )
        self.recorded = self.controller.joint_watcher.stop(t_off)
        self.mi = len(self.movements)-1

    def goto(self, delta):
        ni = self.mi + delta
        if ni < 0 or ni >= len(self.movements):
            return

        m = self.movements[ni]
        m2 = {}
        for arm in self.arms:
            m2[arm] = m[arm]
        self.start_action( [m2] )
        self.mi = ni
        print self.mi, self.movements[self.mi]

    def to_file(self):
        x = yaml.dump(self.movements)
        print x
        print
        f = open(self.filename, 'w')
        f.write(x)
        f.close()

    def switch_to(self, newname):
        start = []
        stop = []
        for arm in self.arms:
            cname = newname % arm
            rospy.loginfo("Switching to %s"%cname)
            stop.append( self.controllers[arm] ) 
            start.append( cname ) 
            self.controllers[arm] = cname
        self.switcher(start, stop, 1)

    def total_time(self, i):
        return sum(get_time(m) for m in self.movements[:i+1])

    def spin(self):
        r = rospy.Rate(4)
        while len(self.movements)==0:
            r.sleep()
        self.grapher.graph(self.movements, 'o-')
        self.grapher.show(block=False)
        t0 = None
        hilite = None
        while self.running and not rospy.is_shutdown():
            self.grapher.graph(self.movements)

            t = self.total_time(self.mi)
            if t != t0:
                if hilite is not None:
                    hilite.remove()
                hilite = self.grapher.ax.axvspan(t-0.5, t+0.5, color='red', alpha=0.5)
            if self.recorded is not None and len(self.recorded)>0:
                self.grapher.graph(self.recorded, "-", label_prefix="REC", linewidth=5, alpha=0.5)
            self.grapher.show(block=False)
            r.sleep()

if __name__ == '__main__':
    if len(sys.argv)==1:
        print "Need to specify filename"
        exit(1)
    filename = sys.argv[1]

    arms = []
    if '-l' in sys.argv:
        arms.append(LEFT)
    if '-r' in sys.argv:
        arms.append(RIGHT)
    if len(arms)==0:
        print "Must specify at least one arm"
        exit(1)

    ir = InteractiveRecorder(arms, filename)
    ir.spin()

