#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_precise_trajectory import *
from pr2_precise_trajectory.full_controller import FullPr2Controller, OPEN, CLOSED
import sys
import yaml
import random
from resource_retriever.retriever import get

from dynamic_reconfigure.server import Server
from pr2_sith.cfg import TheForceConfig

class Duel:
    def __init__(self):
        self.forward_time = 1.9
        self.back_time = 0.8
        self.controller = FullPr2Controller([RIGHT, RIGHT_HAND])
        self.srv = Server(TheForceConfig, self.callback)

    def move_arm(self, move):
        move[0][TIME] = self.forward_time
        move[1][TIME] = self.back_time
        self.controller.do_action(move)

    def setup(self, start):
        self.controller.do_action(start)
        self.controller.hands[RIGHT_HAND].change_position(OPEN)
        raw_input('Press enter to close grip\n')
        self.controller.hands[RIGHT_HAND].change_position(CLOSED)

    def callback(self, config, level):
        self.forward_time = config.forward_time
        self.back_time = config.back_time
        self.controller.trigger = config.trigger_conditions
        self.controller.acceleration_trigger = config.acceleration_trigger
        self.controller.slip_trigger = config.slip_trigger
        return config

if __name__ == '__main__':
    starts = []
    attacks = []

    for i in range(1, 7):
        for (move_type, scripts) in zip(['start', 'attack'], [starts, attacks]):
            movements = yaml.load( get( 'package://pr2_sith/scripts/%s%d.yaml'%(move_type, i)))
            scripts.append(movements)
    transitions = [range(1,7), [3,4,5], [1,4,5], [2,3,5,6], range(2,7), [1,2]]

    moves = []
    for start, attack_indexes in zip(starts, transitions):
        if start==starts[3]: #never works
            continue
        for attack in [ attacks[i-1] for i in attack_indexes ]:
            moves.append( start + attack )

    rospy.init_node('sith')
    duel = Duel()

    if len(sys.argv)>1:
        if sys.argv[1]=='--setup':
            duel.setup(starts[0])

    while not rospy.is_shutdown():
        move = random.choice(moves)
        duel.move_arm( move )

