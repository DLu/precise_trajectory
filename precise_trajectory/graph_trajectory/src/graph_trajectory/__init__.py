from pylab import *
import math
import collections
from pr2_precise_trajectory import *
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
ion()
def normalize(angle):
    return angle - 2 * math.pi * math.floor((angle+math.pi)/(2*math.pi))

THE_GRAPH = None

class Grapher:
    def __init__(self):
        self.plots = {}
        self.fig = figure(1)
        self.ax = self.fig.add_subplot(111)
        box = self.ax.get_position()
        self.ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])
        draw()

    def graph(self, trajectory, gtype="o-", prefix_filter=None, label_prefix=None, **keywords):
        data = get_graph_data(trajectory)
        for (name, t, y) in data:
            if prefix_filter is not None:
                if name.find(prefix_filter)!=0:
                    continue
            if label_prefix is None:
                label = name
            else:
                label = "%s %s"%(label_prefix, name)

            if label in self.plots:
                plot = self.plots[label]
                plot.set_xdata(t)
                plot.set_ydata(y)
            else:
                p, = self.ax.plot(t,y, gtype, label=label, **keywords)
                self.plots[label] = p

    def show(self, block):
        self.ax.relim()
        self.ax.autoscale_view(False,True,True)
        legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        grid(True)
        title('Trajectories')
        if block:
            show()
        else:
            draw()

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
    global THE_GRAPH
    if THE_GRAPH is None:
        THE_GRAPH = Grapher()
    THE_GRAPH.graph(trajectory, **keywords)

def show_graph(block=True):
    global THE_GRAPH
    THE_GRAPH.show(block)

