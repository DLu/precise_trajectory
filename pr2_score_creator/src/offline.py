#!/usr/bin/python
import rospy
from pr2_score_creator import *
from sensor_msgs.msg import JointState
from graph_trajectory import get_graph_data
import tf

class Offline:
    def __init__(self, filename, directory=None):
        rospy.init_node('offline')
        self.pub = rospy.Publisher('/source_joints', JointState)
        self.tf = tf.TransformBroadcaster()
        self.score = Score(filename, directory)
        import pprint
        pprint.pprint( get_graph_data(self.score.movements) )
        
        self.mi = 0
        self.t = 0
        
        #self.goto(0)
        

    def spin(self):
        while not rospy.is_shutdown():
            m0 = self.score.movements[self.mi]
            
            
            self.tf.sendTransform((5, 5, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 1.51),
                     rospy.Time.now(),
                     '/base_footprint',
                     "/map")


import sys
o = Offline(sys.argv[1])
o.spin()
