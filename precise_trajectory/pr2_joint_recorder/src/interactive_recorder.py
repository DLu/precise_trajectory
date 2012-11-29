#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_joint_recorder')
import rospy
import sys
import yaml
import os.path
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from pr2_sith.lightsaber import RobotController
from pr2_mechanism_msgs.srv import SwitchController

SAVE_AS_CURRENT = 14 # X
SAVE_AS_NEXT    = 13 # O
DELETE          = 12 # TRIANGLE
SET_IMPACT      = 15 # SQUARE

GO_FORWARD      = 5  # RIGHT
GO_BACK         = 7  # LEFT
PLAY_FROM_HERE  = 0  # SELECT
PLAY_FROM_START = 3  # START
TO_FILE         = 16 # PS3 

PLUS_TIME       = 11 # R1
PLUS_TIME_BIG   = 9  # R2
MINUS_TIME      = 10 # L1
MINUS_TIME_BIG  = 8  # L2

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

MANNEQUIN_CONTROLLER = '%s_arm_controller_loose'
POSITION_CONTROLLER = '%s_arm_controller'

BUTTON_LAG = 1.0
DEFAULT_TIME = 3.0

class InteractiveRecorder:
    def __init__(self, arms, filename):
        rospy.init_node('interactive_recorder')
        self.time = None
        self.positions = {}
        self.filename = filename
        self.arms = arms
        self.mi = 0
        
        self.controllers = {}
        for arm in self.arms:
            self.controllers[arm] = POSITION_CONTROLLER % arm

        if os.path.exists(self.filename):
            self.movements = yaml.load( open(self.filename, 'r'))
        else:
            self.movements = []
        
        self.sub1 = rospy.Subscriber('/joint_states', JointState, self.joint_cb)
        self.sub2 = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.controller = RobotController()
        rospy.loginfo("Waiting for control manager")
        rospy.wait_for_service('pr2_controller_manager/switch_controller')
        rospy.loginfo("Got control manager!")
        self.switcher = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)

        self.switch_to(MANNEQUIN_CONTROLLER)


    def joint_cb(self, msg):
        for (name, pos) in zip(msg.name, msg.position):
            self.positions[name] = pos

    def joy_cb(self, msg):
        buttons = msg.buttons
        if self.time is not None and (rospy.get_rostime() - self.time).to_sec() < BUTTON_LAG:
            return

        if buttons[SAVE_AS_CURRENT]:
            self.save(self.mi)
        elif buttons[SAVE_AS_NEXT]:
            self.save(self.mi + 1, insert=True)
        elif buttons[DELETE]:
            self.delete()
        elif buttons[SET_IMPACT]:
            self.set_impact()
        elif buttons[GO_FORWARD]:
            self.goto(self.mi+1)
        elif buttons[GO_BACK]:
            self.goto(self.mi-1)
        elif buttons[PLAY_FROM_HERE]:
            self.play(self.mi)
        elif buttons[PLAY_FROM_START]:
            self.play(0)
        elif buttons[PLUS_TIME]:
            self.change_time(1.1)
        elif buttons[PLUS_TIME_BIG]:
            self.change_time(1.5)
        elif buttons[MINUS_TIME]:
            self.change_time(.9)
        elif buttons[MINUS_TIME_BIG]:
            self.change_time(.5)
        elif buttons[TO_FILE]:
            self.to_file()
        else:
            return

        self.time = rospy.get_rostime()

    def save(self, index, insert=False):
        if not insert:
            m = self.movements[index]
        else:
            m = {}

        for arm in self.arms:
            arr = []
            for j in ARM_JOINTS:
                arr.append( self.positions['%s_%s' % (arm, j)] )
            m[arm] = arr

        if insert and index < len(self.movements):
            t = self.movements[index].get('time', DEFAULT_TIME)
            m['time'] = t / 2
            self.movements[index]['time'] = t / 2

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
            t = self.movements[self.mi].get('time', DEFAULT_TIME)
            t2 = self.movements[self.mi+1].get('time', DEFAULT_TIME)
            self.movements[self.mi+1]['time'] = t + t2
            self.movements = self.movements[:self.mi] + self.movements[self.mi+1:]
        self.goto(self.mi)

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
        t = m.get('time', DEFAULT_TIME)
        nt = t * factor
        print "%f ==> %f"%(t, nt)
        m['time'] = nt

    def start_action(self, movements):
        self.switch_to(POSITION_CONTROLLER)
        for arm in self.arms:
            goal = self.controller.make_trajectory_long(movements, arm)
            self.controller.start_trajectory(goal, arm)
        time = sum([m.get('time', DEFAULT_TIME) for m in movements])
        rospy.sleep(time / self.controller.speedup)
        self.mi = len(self.movements)-1
        self.switch_to(MANNEQUIN_CONTROLLER)


    def play(self, starti):
        self.start_action( self.movements[starti:] )

    def goto(self, ni):
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
        rospy.loginfo("Switching to %s"%newname);
        start = []
        stop = []
        for arm in self.arms:
            stop.append( self.controllers[arm] ) 
            start.append( newname%arm ) 
            self.controllers[arm] = newname%arm
        self.switcher(start, stop, 1)

    def spin(self):
        s = ''
        while s != 'quit':
            s = raw_input()
            if len(s)==0:
                continue

            try:
                t = float(s)
                self.movements[self.mi]['time'] = t
            except ValueError:
                self.movements[self.mi]['label'] = s
                

if __name__ == '__main__':
    if len(sys.argv)==1:
        print "Need to specify filename"
        exit(1)
    filename = sys.argv[1]

    arms = []
    if '-l' in sys.argv:
        arms.append('l')
    if '-r' in sys.argv:
        arms.append('r')
    if len(arms)==0:
        print "Must specify at least one arm"
        exit(1)

    ir = InteractiveRecorder(arms, filename)
    ir.spin()

