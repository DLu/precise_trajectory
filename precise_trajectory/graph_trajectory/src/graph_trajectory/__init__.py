from pylab import *
import math
def normalize(angle):
    return angle - 2 * math.pi * math.floor((angle+math.pi)/(2*math.pi))

def graph_trajectory(trajectory, gtype="o-", prefix_filter=None, label_prefix=None, **keywords):
    t = []
    for pt in trajectory.points:
        t.append(pt.time_from_start.to_sec())
    for (i, name) in enumerate(trajectory.joint_names):
        if prefix_filter is not None:
            if name.find(prefix_filter)!=0:
                continue
        y = []
        for pt in trajectory.points:
            y.append( normalize( pt.positions[i] ))
        if label_prefix is None:
            label = name
        else:
            label = "%s %s"%(label_prefix, name)
        plot(t,y, gtype, label=label, **keywords)

def graph_joints(data, gtype="o-", start_time=None, names_filter=None, label_prefix=None, **keywords):
    t = []
    if start_time is None:
        start_time = data[0].header.stamp

    for msg in data:
        t.append((msg.header.stamp - start_time).to_sec() )

    if names_filter is None:
        indexes = range( len(data[0].name) )
    else:
        indexes = []
        for name in names_filter:
            indexes.append( data[0].name.index(name) )

    for ind in indexes:
        y = []
        for msg in data:
            y.append( normalize( msg.position[ind] ))
        name = data[0].name[ind]
        if label_prefix is None:
            label = name
        else:
            label = "%s %s"%(label_prefix, name)
        plot(t,y, gtype, label=label, **keywords)
        plot(t,y,gtype, **keywords)

def show_graph(loc=None):
    if loc is not None:
        legend(loc=loc)
    grid(True)
    title('Trajectories')
    show()

