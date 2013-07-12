from pylab import *
import math
import collections
from pr2_precise_trajectory import *
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
ion()
def normalize(angle):
    return angle - 2 * math.pi * math.floor((angle+math.pi)/(2*math.pi))

THE_GRAPH = None
FIELDS = [LEFT, RIGHT, BASE, LEFT_HAND, RIGHT_HAND, HEAD]

class Grapher:
    def __init__(self, name=None, keys=[None]):
        self.plots = {}
        self.fig = figure(1, figsize=(18, 16))
        self.fig.subplots_adjust(hspace=0)
        if name is None:
            name = "Trajectory Plotter"
        self.fig.canvas.set_window_title(name) 

        self.subplots = {}
        n = len(keys)
        for i, key in enumerate(sorted(keys)):
            subplot = self.fig.add_subplot(n, 1, i+1)
            box = subplot.get_position()
            subplot.set_position([box.x0, box.y0, box.width * 0.8, box.height])
            self.subplots[key] = subplot
                
        draw()

    def get_subplot(self, key):
        if key in self.subplots:
            return self.subplots[key]
        else:
            return self.subplots[None]

    def graph(self, trajectory, gtype="o-", prefix_filter=None, label_prefix=None, **keywords):
        for key, data in get_graph_data(trajectory).iteritems():
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
                    p, = self.get_subplot(key).plot(t,y, gtype, label=label, **keywords)
                    self.plots[label] = p

    def show(self, block):
        for subplot in self.subplots.values():
            subplot.relim()
            subplot.autoscale_view(False,True,True)
            subplot.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
            subplot.grid(True)
        if block:
            show()
        else:
            draw()

def get_graph_data(trajectory):
    times = collections.defaultdict(list)
    positions = collections.defaultdict(list)
    
    t0 = 0.0
    for move in trajectory:
        t0 += get_time( move )
        for key in FIELDS:
            if key in move:
                times[key].append(t0)
                positions[key].append( move[key] )

    m = {}
    for key, t in times.iteritems():
        if key in [LEFT, RIGHT]:
            names = get_arm_joint_names(key)
        elif key==BASE:
            names = ['x', 'y', 'theta']
        elif key==HEAD:
            names = ['pan', 'tilt']
        else:
            names = [key]
        pos = positions[key]
        data = []
        for (i, name) in enumerate(names):
            if name in ['x', 'y']:
                y = [pt[i] for pt in pos]
            else:
                y = []
                for pt in pos:
                    y.append( normalize( pt[i] ))
            data.append((name, t, y))
        m[key] = data
    return m

def graph_trajectory(trajectory, gtype="o-", prefix_filter=None, label_prefix=None, **keywords):
    global THE_GRAPH
    if THE_GRAPH is None:
        THE_GRAPH = Grapher()
    THE_GRAPH.graph(trajectory, **keywords)

def show_graph(block=True):
    global THE_GRAPH
    THE_GRAPH.show(block)

