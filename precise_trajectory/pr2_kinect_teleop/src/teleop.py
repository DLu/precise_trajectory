#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_kinect_teleop')
import rospy
from pr2_precise_trajectory.arm_controller import ArmController, get_arm_joint_names
from pr2_precise_trajectory.converter import simple_to_message_single
from sensor_msgs.msg import JointState

#For head motion
from actionlib import SimpleActionClient, SimpleGoalState
from trajectory_msgs.msg import JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal

VELOCITY_WINDOW = 5

class ArmJoint:
    def __init__(self, arms=['l', 'r']):
        rospy.init_node('pr2_kinect_teleop')
        self.arms = {}        
        self.joint_histories = {}
        for arm in arms:
            self.arms[arm] = ArmController(arm) 
            self.joint_histories[arm] = []

        head = SimpleActionClient("/head_traj_controller/joint_trajectory_action", JointTrajectoryAction)
        head.wait_for_server()
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        goal.trajectory.points = [ JointTrajectoryPoint() ]
        goal.trajectory.points[0].positions = [0,0]
        goal.trajectory.points[0].time_from_start = rospy.Duration(2.0)
        head.send_goal(goal)

        self.joint_sub = rospy.Subscriber('/kinect_angles', JointState, self.joint_cb)

    def joint_cb(self, msg):
        for arm in self.arms:
            position = []
            for name in get_arm_joint_names(arm):
                if name not in msg.name:
                    position.append(0.0)
                else:
                    i = msg.name.index(name)
                    position.append(msg.position[i])
            self.joint_histories[arm].append(position)
            if len(self.joint_histories[arm]) > VELOCITY_WINDOW:
                self.joint_histories[arm].pop(0)

    def get_positions(self, arm):
        hist = self.joint_histories[arm]
        if len(hist) > 0:
            return hist[-1]
        elif arm=='l':
            return [1.51, 0, 0, -1.51, 0, 0, 0]
        elif arm=='r':
            return  [-1.51, 0, 0, -1.51, 0, 0, 0]
        else:
            return [0.0] * len(get_arm_joint_names(arm))

    def get_velocities(self, arm):
        v = []
        for (i, joint) in enumerate(get_arm_joint_names(arm)):
            pos = [a[i] for a in self.joint_histories[arm]]
            if len(pos)!=0:
                v.append( sum(pos)/len(pos) )
            else:
                v.append( 0.0 )
        return v

    def spin(self):
        hz = 4.0
        r = rospy.Rate(hz)

        while not rospy.is_shutdown():
            for arm in self.arms:
                trajectory = simple_to_message_single(self.get_positions(arm), 1.0 / hz, arm)
                self.arms[arm].start_trajectory(trajectory, wait=False)
            r.sleep()

if __name__ == '__main__':
    a = ArmJoint()
    a.spin()


