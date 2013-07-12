#!/usr/bin/env python
import roslib; roslib.load_manifest('modify_trajectory')
import rospy
from pr2_precise_trajectory.converter import load_trajectory, save_trajectory, tprint
from modify_trajectory import *
from graph_trajectory import *
import argparse

if __name__ == '__main__':
    G = globals().keys()[:]
    MAGIC = 'modify_'
    functions = [v[len(MAGIC):] for v in G if MAGIC in v]

    parser = argparse.ArgumentParser(description='Modify a trajectory')
    parser.add_argument('source_trajectory')
    parser.add_argument('modifier', choices=functions)
    parser.add_argument('target_filename', default=None, nargs="?")
    parser.add_argument('-p', '--print', action='store_true', dest='should_print')
    parser.add_argument('-g', '--graph', action='store_true', dest='should_graph')

    args = parser.parse_args()

    trajectory = load_trajectory( args.source_trajectory )
    modifier = globals()[args.modifier]
    new_trajectory = modifier(trajectory)
    if args.should_print:
        tprint(new_trajectory)
    if args.target_filename is not None:
        save_trajectory( new_trajectory, args.target_filename)
    if args.should_graph:
        graph_trajectory(trajectory, 'o')
        graph_trajectory(new_trajectory,  '-')
        show_graph()

