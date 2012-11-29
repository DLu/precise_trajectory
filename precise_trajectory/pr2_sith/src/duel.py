#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_sith.lightsaber import RobotController, both_arms, OPEN, CLOSED
import sys
import yaml
import random
import sys
from resource_retriever.retriever import get

from dynamic_reconfigure.server import Server
from pr2_sith.cfg import TheForceConfig

class Duel:
    def __init__(self):
        self.forward_time = 1.9
        self.back_time = 0.8
        self.controller = RobotController()
        self.srv = Server(TheForceConfig, self.callback)

    def move_arm(self, movements):
        for (n, move) in enumerate(movements):
            time = move.get('time', self.forward_time)
            arms = []
            a = 'r'
            arms.append(a)
            goal = self.controller.make_trajectory(move[a], time, a)
            self.controller.start_trajectory(goal, a)
            transition = move.get('transition', 'wait')
            if transition=='wait':
                rospy.sleep(time)
            elif transition=='impact':
                rospy.sleep(.1)
                self.controller.wait_for_impact(arms)
                goal = self.controller.make_trajectory(self.controller.right_arm(), self.back_time, 'r')
                self.controller.start_trajectory(goal, 'r')
                rospy.sleep(self.back_time)

    def setup(self):
        self.move_arm( starts[0] )
        self.controller.gripper(['r'], OPEN)
        raw_input('Press enter to close grip\n')
        self.controller.gripper(['r'], CLOSED)

    def callback(self, config, level):
        self.forward_time = config.forward_time
        self.back_time = config.back_time
        self.controller.trigger = config.trigger_conditions
        self.controller.acceleration_trigger = config.acceleration_trigger
        self.controller.slip_trigger = config.slip_trigger
        return config

if __name__ == '__main__':
    rospy.init_node('sith')

    starts = []
    attacks = []

    for i in range(1, 7):
        for (move_type, scripts) in zip(['start', 'attack'], [starts, attacks]):
            movements = yaml.load( get( 'package://pr2_sith/scripts/%s%d.yaml'%(move_type, i)))
            scripts.append(movements)
    transitions = [range(1,7), [3,4,5], [1,4,5], [2,3,5,6], range(2,7), [1,2]]

    duel = Duel()

    if len(sys.argv)>1:
        if sys.argv[1]=='--setup':
            duel.setup()

    while not rospy.is_shutdown():
        start = random.choice(range(1,7))
        if start==4:
            continue
        attack = random.choice( transitions[start-1] )
        rospy.loginfo("Start %d"%start)
        duel.move_arm( starts[start-1] )
        rospy.loginfo("Attack %d"%attack)
        duel.move_arm( attacks[attack-1] )

