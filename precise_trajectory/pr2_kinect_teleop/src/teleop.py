#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_kinect_teleop')
import rospy
from pr2_trajectory import BOTH_ARMS, ARM_JOINTS
from pr2_controllers_msgs.msg import *
from pr2_gripper_sensor_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib import SimpleActionClient, SimpleGoalState
from sensor_msgs.msg import JointState
import trajectory_msgs.msg

VELOCITY_WINDOW = 5

class ArmJoint:
    def __init__(self):
        self.arms = {}
        for arm in BOTH_ARMS:
            self.arms[arm] = SimpleActionClient("%s_arm_controller/joint_trajectory_action"%arm, JointTrajectoryAction)
            #wait for the action servers to come up 
            self.arms[arm].wait_for_server()
        head = SimpleActionClient("/head_traj_controller/joint_trajectory_action", JointTrajectoryAction)
        head.wait_for_server()
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        goal.trajectory.points = [ JointTrajectoryPoint() ]
        goal.trajectory.points[0].positions = [0,0]
        goal.trajectory.points[0].time_from_start = rospy.Duration(2.0)
        head.send_goal(goal)

        self.joint_sub = rospy.Subscriber('/kinect_angles', JointState, self.joint_cb)
        self.joint_histories = {'l': [], 'r': []}
        self.names = {'l': ['l_%s'%a for a in ARM_JOINTS],
                      'r': ['r_%s'%a for a in ARM_JOINTS]}


    def joint_cb(self, msg):
        for arm in BOTH_ARMS:
            position = []
            for joint in ARM_JOINTS:
                name = "%s_%s"%(arm, joint)
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
        else:
            return [0.0] * len(ARM_JOINTS)

    def get_velocities(self, arm):
        v = []
        for (i, joint) in enumerate(ARM_JOINTS):
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
            for arm in ['l', 'r']:
                # our goal variable
                goal = JointTrajectoryGoal()
                # Start the trajectory immediately
                goal.trajectory.header.stamp = rospy.Time.now()

                # First, the joint names, which apply to all waypoints
                goal.trajectory.joint_names = self.names[arm]

                # We will have N waypoints in this goal trajectory
                goal.trajectory.points = [trajectory_msgs.msg.JointTrajectoryPoint()]

                # First trajectory point
                goal.trajectory.points[0].positions = self.get_positions(arm)
                # goal.trajectory.points[0].velocities = self.get_velocities(arm)
                # set time we want this trajectory to be reached at
                goal.trajectory.points[0].time_from_start = rospy.Duration(1.0/hz)
                self.arms[arm].send_goal(goal)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('pr2_kinect_teleop')
    a = ArmJoint()
    a.spin()


