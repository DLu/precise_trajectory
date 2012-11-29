import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from actionlib import SimpleActionClient, SimpleGoalState
import trajectory_msgs.msg

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

def get_arm_joint_names(side):
    return [ '%s_%s' % (side, name) for name in ARM_JOINTS ]

class ArmController:
    def __init__(self, name):
        self.arm = SimpleActionClient("%s/joint_trajectory_action"%name, JointTrajectoryAction)
        #wait for the action servers to come up 
        rospy.loginfo("[ARM] Waiting for %s"%name)
        self.arms.wait_for_server()
        rospy.loginfo("[ARM] Got %s"%name)

    def start_trajectory(self, trajectory, set_time_stamp=True, wait=True):
        """Creates an action from the trajectory and sends it to the server"""
        goal = JointTrajectoryGoal()
        goal.trajectory = trajectory
        if set_time_stamp:
            goal.trajectory.header.stamp = rospy.Time.now()
        self.arm.send_goal(goal)

        if wait:
            self.wait()

    def wait(self):
        self.arm.wait_for_result()

    def is_done(self):
        return self.arm.get_state() > SimpleGoalState.ACTIVE

