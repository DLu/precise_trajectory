import roslib; roslib.load_manifest('pr2_precise_trajectory')
from pr2_precise_trajectory import *
import rospy
from sensor_msgs.msg import JointState

class JointWatcher:
    def __init__(self, name_map):
        self.name_map = name_map
        self.key_map = {}
        for key, names in name_map.iteritems():
            for name in names:
                self.key_map[name] = key

        self.state = {}
        self.data = []
        self.start_time = None
        self.done = False

        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_cb)        

    def joint_cb(self, msg):
        self.state = {}
        for key, names in self.name_map.iteritems():
            pos = []
            for name in names:
                i = msg.name.index(name) 
                pos.append( msg.position[i] )
            self.state[key] = pos
        self.state[TIME] = msg.header.stamp

        if self.start_time is not None and not self.done:
            self.data.append(self.state)

    def get_state(self):
        return self.state

    def record(self):
        self.start_time = rospy.Time.now()
        self.data = []
        self.done = False
    
    def stop(self, delay=0.0):
        self.done = True
        last = None
        for move in self.data:
            if last is None:
                last = move[TIME]
                move[TIME] = delay+ 0.0
            else:
                temp = move[TIME]
                move[TIME] = (temp - last).to_sec()
                last = temp
        return self.data

