import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_precise_trajectory.arm_controller import *
from pr2_precise_trajectory.gripper_controller import *
from pr2_precise_trajectory.base_controller import *
from pr2_precise_trajectory.impact_watcher import *
from pr2_precise_trajectory.joint_watcher import *
from pr2_precise_trajectory.converter import simple_to_message, simple_to_message_single
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FullPr2Controller:
    def __init__(self, arms=['l', 'r']):
        self.arms = {}

        joint_names = []
        for arm in arms:
            self.arms[arm] = ArmController(arm)
            joint_names += get_arm_joint_names(arm)
        self.grippers = GripperController(arms)
        self.base = BaseController()
        self.head = HeadController()
        self.impacts = ImpactWatcher(['%s_gripper_sensor_controller'%arm for arm in arms])
        self.joint_watcher = JointWatcher(joint_names)

    def do_action(self, movements): #TODO Add Base, Head, Gripper
        if len(movements)==0:
            return
        arms = self.arms.keys()

        first = movements[0]
        chunks = [[first]]
        last_transition = first.get('transition', 'wait')
        
        for move in movements[1:]:
            transition = move.get('transition', 'wait')
            if transition==last_transition and transition=='wait':
                chunks[-1].append(move)
            else:
                last_transition = transition
                chunks.append( [move] )

        for ms in chunks:
            for arm in arms:
                trajectory = simple_to_message(ms, arm)
                self.arms[arm].start_trajectory(trajectory, wait=False)
            transition = ms[0].get('transition', 'wait')
            if transition=='wait':
                for arm in arms:
                    self.arms[arm].wait()
            elif transition=='impact':
                rospy.sleep(.1)
                self.impacts.wait_for_impact()
                self.stop_arm(arms)

    def stop_arm(self, arms, time=0.1):
        for arm in arms:
            trajectory = simple_to_message_single(self.joint_watcher.get_positions(get_arm_joint_names(arm)), time, arm)
            self.arms[arm].start_trajectory(trajectory, wait=False)
        rospy.sleep(time)


