import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_controllers_msgs.msg import *
from actionlib import SimpleActionClient

OPEN = 0.09
CLOSED = 0.002

class GripperController:
    def __init__(self, arms=['l', 'r']):
        self.effort = -1.0
        self.grippers = {}
        for arm in arms:
            self.grippers[arm] = SimpleActionClient("%s_gripper_sensor_controller/gripper_action"%arm, Pr2GripperCommandAction)
            #wait for the action servers to come up 
            rospy.loginfo("[GRIPPER] Waiting for %s controllers"%arm)
            self.grippers[arm].wait_for_server()        
            rospy.loginfo("[GRIPPER] Got %s controllers"%arm)

    def change_position(self, arms, position, should_wait=True):
        gg = Pr2GripperCommandGoal()
        gg.command.position = position
        gg.command.max_effort = self.effort

        for arm in arms:
            self.grippers[arm].send_goal(gg)

        if should_wait:
            for arm in arms:
                self.grippers[arm].wait_for_result()

