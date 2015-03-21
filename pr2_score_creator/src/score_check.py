#!/usr/bin/python

import roslib; roslib.load_manifest('pr2_score_creator')
import rospy

from pr2_precise_trajectory import *
from pr2_precise_trajectory.full_controller import FullPr2Controller, transition_split
from pr2_precise_trajectory.converter import *
from pr2_score_creator import *

import sys

def get_velocities(A, B, key):
    a = A[key]
    b = B[key]
    t = get_time(B)
    if key=='b':
        z = hypot(a[0]-b[0], a[1]-b[1])
        return z/t, (b[2]-a[2])/t
    else:
        return [(p1-p0)/t for p0, p1 in zip(a,b)]

score = Score(sys.argv[1], None)

for key in [LEFT, RIGHT, BASE, HEAD, LEFT_HAND, RIGHT_HAND]:
    moves = precise_subset(score.movements, key, True)
    if len(moves)<=1:
        continue
    print "=============%s====================="%key
    for i in range(1, len(moves)):
        l1 = moves[i-1].get('label', '%d'%i)
        l2 = moves[i].get('label', '%d'%i)
        vs = get_velocities( moves[i-1], moves[i], key)
        v = ' '.join(['%+2f'%x for x in vs])
        print "%10s -> %-10s %s"%(l1, l2, v)
#    print moves
