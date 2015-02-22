#!/usr/bin/python
import rospy
from pr2_score_creator import *
from sensor_msgs.msg import JointState
from graph_trajectory import get_graph_data
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
from pr2_precise_trajectory.head_controller import HEAD_JOINTS
from std_srvs.srv import Empty, EmptyResponse
import tf

JNAMES = {'h': HEAD_JOINTS, 'l': get_arm_joint_names('l'), 'r': get_arm_joint_names('r')}

class Offline:
    def __init__(self, filename, directory=None):
        rospy.init_node('offline')
        self.pub = rospy.Publisher('/source_joints', JointState)
        self.service = rospy.Service('/proceed', Empty, self.service_call)
        self.service_flag = False
        self.tf = tf.TransformBroadcaster()
        self.score = Score(filename, directory)
        
        self.mi = 0
        self.t = 0
        self.marker = None
        
    def service_call(self, req):
        self.service_flag = True
        self.marker = rospy.Time.now()
        return EmptyResponse()
            

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            transition = self.score.movements[self.mi].get('transition', 'wait')
            if transition == 'service' and self.service_flag==False:
                None
            elif self.marker is not None:
                ellapsed = rospy.Time.now() - self.marker
                es = ellapsed.to_sec()
                t = get_time(self.score.movements[self.mi])
                if es > t:
                    self.mi += 1
                    self.t = 0.0
                    if self.mi >= len(self.score.movements):
                        self.marker = None
                    elif self.score.movements[self.mi].get('transition', 'wait')=='service':
                        self.service_flag = False
                        self.marker = None
                        print 'new service'    
                    else:     
                        self.marker = rospy.Time.now()
                else:       
                    self.t = es
                    
                    
            
            state = self.score.get_state(self.mi, self.t, ['b', 'l', 'r', 'h'])
            
            if 'b' in state:
                bs = state['b']
                self.tf.sendTransform((bs[0], bs[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, bs[2]),
                         rospy.Time.now(),
                         '/base_footprint',
                         '/map')
            js = JointState()
            for key in ['l', 'r', 'h']:
                names = JNAMES[key]
                if key not in state:
                    continue
                for name, value in zip(names, state[key]):
                    js.name.append(name)
                    js.position.append(value)            
            self.pub.publish(js)        
            r.sleep()    

import sys
o = Offline(sys.argv[1])
o.spin()
