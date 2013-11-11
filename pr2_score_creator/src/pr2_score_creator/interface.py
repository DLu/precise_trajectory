from graph_trajectory import *

class Interface:
    def __init__(self, jw):
        self.grapher = Grapher()
        self.joint_watcher = jw
        self.recorded = []
        self.grapher.graph(self.movements, 'o-')
        self.grapher.show(block=False)


        t0 = None

    def start(self):
        self.recorded = None
        self.controller.joint_watcher.record()
        t_off = self.total_time(starti-1)
        self.recorded = self.controller.joint_watcher.stop(t_off)

    def cycle(self):
        self.grapher.graph(self.movements)
        t = self.total_time(self.mi)
        if t != t0:
            self.grapher.hilite
        if self.recorded is not None and len(self.recorded)>0:
            self.grapher.graph(self.recorded, "-", label_prefix="REC", linewidth=5, alpha=0.5)
        self.grapher.show(block=False)
