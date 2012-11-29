from pylab import *
import math
import collections
from pr2_precise_trajectory.arm_controller import get_arm_joint_names

def normalize(angle):
    return angle - 2 * math.pi * math.floor((angle+math.pi)/(2*math.pi))

def graph_trajectory(trajectory, gtype="o-", prefix_filter=None, label_prefix=None, **keywords):
    times = collections.defaultdict(list)
    positions = collections.defaultdict(list)
    
    t0 = 0.0
    for move in trajectory:
        t0 += move.get('time', 1.0)
        for arm in ['l', 'r']:
            if arm in move:
                times[arm].append(t0)
                positions[arm].append( move[arm] )

    for arm, t in times.iteritems():
        names = get_arm_joint_names(arm)
        pos = positions[arm]
        for (i, name) in enumerate(names):
            if prefix_filter is not None:
                if name.find(prefix_filter)!=0:
                    continue
            y = []
            for pt in pos:
                y.append( normalize( pt[i] ))
            if label_prefix is None:
                label = name
            else:
                label = "%s %s"%(label_prefix, name)
            plot(t,y, gtype, label=label, **keywords)

def show_graph(loc=None):
    if loc is not None:
        legend(loc=loc)
    grid(True)
    title('Trajectories')
    show()

