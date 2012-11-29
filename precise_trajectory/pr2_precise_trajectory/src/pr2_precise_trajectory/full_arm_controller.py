import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_precise_trajectory.arm_controller import *
from pr2_precise_trajectory.gripper_controller import *
from pr2_precise_trajectory.impact_watcher import *
from pr2_precise_trajectory.joint_watcher import *
from trajectory_converter import simple_to_message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FullArmController:
    def __init__(self, arms=['l', 'r']):
        self.arms = {}

        joint_names = []
        for arm in arms:
            self.arms[arm] = ArmController(arm)
            joint_names += get_arm_joint_names(arm)
        self.grippers = GripperController(arms)
        self.impacts = ImpactWatcher(['%s_gripper_sensor_controller'%arm for arm in arms])
        self.joint_watcher = JointWatcher(joint_names)

    def start_action(self, movements, arms):
        for arm in arms:
            trajectory = simple_to_message(movements, arm)
            self.arms[arm].start_trajectory(trajectory, wait=False)

    def stop_arm(self, arms, time=0.8):
        for arm in arms:
            self.do_trajectory_short(self.joint_watcher.get_positions(get_arm_joint_names(arm)), time, arm, wait=False)
        rospy.sleep(self.back_time)


