#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_sith.lightsaber import RobotController, both_arms
import sys
import yaml

if __name__ == '__main__':
    rospy.init_node('sith')

    scripts = []
    scripts.append(yaml.load( open("scripts/start%s.yaml"% sys.argv[1], 'r')))
    scripts.append(yaml.load( open("scripts/attack%s.yaml"%sys.argv[2], 'r')))
    scripts.append( scripts[0] ) 

    controller = RobotController()

    for movements in scripts:
        for (n, move) in enumerate(movements):
            print "Move %d"%n
            time = move.get('time', 2.5)
            arms = []
            for a in both_arms:
                if a in move:
                    arms.append(a)
                    goal = controller.make_trajectory(move[a], time, a)
                    controller.start_trajectory(goal, a)
            transition = move.get('transition', 'wait')
            if transition=='wait':
                rospy.sleep(time)
            elif transition=='impact':
                # now start looking for a slap during the move
                rospy.sleep(.1)
                controller.wait_for_impact(arms)
                x = 1.0
                goal = controller.make_trajectory(controller.right_arm(), x, 'r')
                controller.start_trajectory(goal, 'r')
                rospy.sleep(x)


