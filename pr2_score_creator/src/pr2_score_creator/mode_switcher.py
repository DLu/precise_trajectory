import rospy
from pr2_precise_trajectory import *
from pr2_mechanism_msgs.srv import SwitchController

MANNEQUIN_CONTROLLERS = {LEFT: 'l_arm_controller_loose', RIGHT: 'r_arm_controller_loose', HEAD: 'head_traj_controller_loose'}
POSITION_CONTROLLERS = {LEFT: 'l_arm_controller', RIGHT: 'r_arm_controller', HEAD: 'head_traj_controller'}

class ModeSwitcher:
    def __init__(self, keys, start_in_mannequin=True, quiet=True):
        self.quiet = quiet
        self.controllers = {}
        for key in keys:
            if key in POSITION_CONTROLLERS:
                self.controllers[key] = POSITION_CONTROLLERS[key]

        rospy.loginfo("Waiting for control manager")
        rospy.wait_for_service('pr2_controller_manager/switch_controller')
        rospy.loginfo("Got control manager!")
        self.switcher = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)

        if start_in_mannequin:
            self.mannequin_mode()
        else:
            self.position_mode()

    def switch_to(self, controller_map):
        start = []
        stop = []
        for key in self.controllers:
            cname = controller_map[key]
            if not self.quiet:
                rospy.loginfo("Switching to %s"%cname)
            stop.append( self.controllers[key] ) 
            start.append( cname ) 
            self.controllers[key] = cname
        self.switcher(start, stop, 1)

    def mannequin_mode(self):
        self.switch_to(MANNEQUIN_CONTROLLERS)

    def position_mode(self):
        self.switch_to(POSITION_CONTROLLERS)


