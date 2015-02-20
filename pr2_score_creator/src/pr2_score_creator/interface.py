from graph_trajectory import *

class Interface:
    def __init__(self, score, jw):
        self.score = score
        self.grapher = Grapher(scale=0.5)
        self.joint_watcher = jw
        self.recorded = []
        self.grapher.graph(self.score.to_full(), 'o-')
        self.grapher.show(block=False)

    def start(self, start_i):
        self.recorded = None
        if self.joint_watcher:
            self.joint_watcher.record()
        self.t_off = self.score.total_time(start_i-1)

    def done(self):
        if self.joint_watcher:
            self.recorded = self.joint_watcher.stop(self.t_off)

    def cycle(self, index):
        self.grapher.graph(self.score.to_full())
        t = self.score.total_time(index)
        self.grapher.hilite(t)
        if self.recorded is not None and len(self.recorded)>0:
            self.grapher.graph(self.recorded, "-", label_prefix="REC", linewidth=5, alpha=0.5)
        self.grapher.show(block=False)
