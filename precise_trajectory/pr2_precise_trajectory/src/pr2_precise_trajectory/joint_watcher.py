import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from sensor_msgs.msg import JointState

class JointWatcher:
    def __init__(self, names):
        self.name_list = names
        self.name_set = set(names)
        self.data = []
        self.start_time = None
        self.done = False

        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_cb)
        self.joint_pos = {}
        

    def joint_cb(self, msg):
        pos = []
        for i, name in enumerate(msg.names):
            if name in name_set:
                self.joint_pos[name] = msg.position[i]
                pos.append(msg.position[i])
        if self.start_time is not None and not self.done:
            self.data.append(pos)

    def get_positions(self, names=None):
        if names is None:
            names = self.name_list
        return [ self.joint_pos[name] for name in names ]

    def record(self):
        self.start_time = rospy.Time.now()
        self.done = False
    
    def stop(self):
        self.done = True

