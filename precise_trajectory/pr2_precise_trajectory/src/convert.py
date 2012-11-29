#!/usr/bin/python
import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
import sys
from pr2_precise_trajectory.converter import *
import pickle
import yaml

if __name__=='__main__':
    infile = sys.argv[1]
    outfile = None if len(sys.argv)<3 else sys.argv[2]
    if 'traj' in infile:
        trajectory = pickle.load(open(infile))
        result = trajectory_to_simple( trajectory )
        if outfile is None:
            tprint(result )
        else:
            f = open(outfile, 'w')
            f.write(yaml.dump(result))
            f.close()


        
