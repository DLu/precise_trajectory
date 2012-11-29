import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from sensor_msgs.msg import JointState

class JointWatcher:
    def __init__(self, names):
        self.name_list = names
        self.name_set = set(names)

        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_cb)
        self.joint_pos = {}

    def joint_cb(self, msg):
        for i, name in enumerate(msg.names):
            if name in name_set:
                self.joint_pos[name] = msg.position[i]

    def get_positions(self, names=None):
        if names is None:
            names = self.name_list
        return [ self.joint_pos[name] for name in names ]

