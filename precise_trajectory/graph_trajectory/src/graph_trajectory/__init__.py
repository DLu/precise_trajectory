from pylab import *
import math
import collections
from pr2_precise_trajectory import *
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
ion()
def normalize(angle):
    return angle - 2 * math.pi * math.floor((angle+math.pi)/(2*math.pi))

def get_graph_data(trajectory):
    data = []

    times = collections.defaultdict(list)
    positions = collections.defaultdict(list)
    
    t0 = 0.0
    for move in trajectory:
        t0 += get_time( move )
        for arm in [LEFT, RIGHT]:
            if arm in move:
                times[arm].append(t0)
                positions[arm].append( move[arm] )

    for arm, t in times.iteritems():
        names = get_arm_joint_names(arm)
        pos = positions[arm]
        for (i, name) in enumerate(names):
            y = []
            for pt in pos:
                y.append( normalize( pt[i] ))
            data.append((name, t, y))
    return data

def graph_trajectory(trajectory, gtype="o-", prefix_filter=None, label_prefix=None, **keywords):
    fig = figure(1)
    ax = fig.add_subplot(111)
    plots = {}
    data = get_graph_data(trajectory)
    for (name, t, y) in data:
        if prefix_filter is not None:
            if name.find(prefix_filter)!=0:
                continue
        if label_prefix is None:
            label = name
        else:
            label = "%s %s"%(label_prefix, name)
        p, = ax.plot(t,y, gtype, label=label, **keywords)
        plots[name] = p
    return ax, plots

def update_graph(trajectory, ax, plots):
    data = get_graph_data(trajectory)
    min_t = 1E9
    max_t = -1
    for (name, t, y) in data:
        if name not in plots:
            return
        plot = plots[name]
        plot.set_xdata(t)
        plot.set_ydata(y)
        min_t = min(min_t, t[0])
        max_t = max(max_t, t[-1])

    ax.set_xlim([min_t*.9, max_t*1.1])
    draw()

def show_graph(loc=None, block=True):
    if loc is not None:
        legend(loc=loc)
    grid(True)
    title('Trajectories')
    if block:
        show()
    else:
        draw()

