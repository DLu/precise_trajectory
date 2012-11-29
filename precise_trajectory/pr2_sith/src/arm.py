#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_sith.lightsaber import RobotController, both_arms
import sys
import yaml
import random

if __name__ == '__main__':
    rospy.init_node('sith')

    scripts = []
    loop = True

    if len(sys.argv)>1:
        for arg in sys.argv[1:]:
            if arg == '-once':
                loop = False
            else:
                movements = yaml.load( open(arg, 'r'))
                scripts.append(movements)
    else:
        print "No movement specified"
        exit(1)

    controller = RobotController()

    while not rospy.is_shutdown():
        movements = random.choice(scripts)
        for (n, move) in enumerate(movements):
            print "Move %d"%n
            time = move.get('time', 1) * 3
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
                rospy.sleep(.1)
                controller.wait_for_impact(arms)
                x = 1.0
                goal = controller.make_trajectory(controller.right_arm(), x, 'r')
                controller.start_trajectory(goal, 'r')
                rospy.sleep(x)
        if not loop:
            break

