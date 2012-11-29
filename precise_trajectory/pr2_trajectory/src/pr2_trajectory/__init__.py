import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from pylab import *
from control_msgs.msg import *

ERRORS = [ 'SUCCESSFUL', 'ABORTED', 'GOAL_TOLERANCE_VIOLATED', 'PATH_TOLERANCE_VIOLATED', 'OLD_HEADER_TIMESTAMP', 'INVALID_JOINTS', 'INVALID_GOAL']

TEST_RANGES = {"r_shoulder_pan_joint": (-.7, .16), 
          "r_shoulder_lift_joint": (-.3, .3),
          "r_upper_arm_roll_joint": (-.6, .6),
          "r_elbow_flex_joint": (-1.5, -.2), 
          "r_forearm_roll_joint": (-1.5, 1.5), 
          "r_wrist_flex_joint": (-1.5, -.2), 
          "r_wrist_roll_joint": (-1.5, 1.5)}

class JointActor:
    def __init__(self):
        self.follow_clients = {}
        for arm in BOTH_ARMS:
            self.follow_clients[arm] = SimpleActionClient("%s_arm_controller/follow_joint_trajectory"%arm, FollowJointTrajectoryAction)
            self.follow_clients[arm].wait_for_server()

    def follow(self, arm, trajectory, tolerances):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.path_tolerance = tolerances
        self.follow_clients[arm].send_goal(goal)

    def follow_result(self, arms):
        rmap = {}
        for arm in arms:
            self.follow_clients[arm].wait_for_result()
            result = self.follow_clients[arm].get_result()
            if result is None:
                ec = -5
            else:
                ec = result.error_code
            rmap[arm] = (ec, ERRORS[ec])
        if len(rmap)==1:
            return rmap[arm]
        else:
            return rmap

