#!/usr/bin/python

import roslib; roslib.load_manifest('pr2_pose_morph')
from pr2_pose_morph.msg import *
import rospy
import tf
import yaml
import sys
import collections
from  geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pr2_controllers_msgs.msg import *
from actionlib import SimpleActionClient, GoalStatus

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
LEFTY = '/l_arm_controller/joint_trajectory_action'
RIGHTY = '/r_arm_controller/joint_trajectory_action'
HEAD = '/head_traj_controller/joint_trajectory_action'
CONDITIONS = ['Pending', 'Active', 'Preempted', 'Succeeded', 'Aborted', 'Rejected', 'Preempting', 'Recalling', 'Recalled', 'Lost']

class PoseMorph:
    def __init__(self):
        rospy.init_node('pr2_pose_morph')
        self.joint_map = {}
        self.joint_map[LEFTY] = ['l_%s'%a for a in ARM_JOINTS]
        self.joint_map[RIGHTY] = ['r_%s'%a for a in ARM_JOINTS]
        self.joint_map[HEAD] = ['head_pan_joint', 'head_tilt_joint']
        self.joint_servers = {}

        for action_name in self.joint_map:
            server = SimpleActionClient(action_name, JointTrajectoryAction)
            self.joint_servers[action_name] = server

        self.nav_server = SimpleActionClient("/base_controller/move_sequence", MoveSequenceAction)

        for server in [self.nav_server] + self.joint_servers.values():
            rospy.loginfo("Waiting for %s..."%server.action_client.ns)
            server.wait_for_server()
        rospy.loginfo("Ready")
    
    def execute(self, poses):
        goals = {}
        values = {}

        for name, joint_set in self.joint_map.iteritems():
            goals[name] = JointTrajectoryGoal()
            goals[name].trajectory.joint_names = joint_set
        nav_goal = MoveSequenceGoal()
        nav_goal.header.frame_id = '/map'
        J = 0
        for pose in poses:
            t = pose['t']
            for name, joint_set in self.joint_map.iteritems():
                current = {}
                for joint in joint_set:
                    if joint in pose:
                        current[joint] = pose[joint]
                        values[joint] = pose[joint]

                if len(current)>0:
                    point = trajectory_msgs.msg.JointTrajectoryPoint()
                    for joint in joint_set:
                        point.positions.append( values.get(joint, 0.0) )
                    #point.velocities = [0.0]*7
                    point.time_from_start = rospy.Duration(t)
                    goals[name].trajectory.points.append(point)
            if 'x' in pose or 'y' in pose or 'theta' in pose:
                nav_goal.times.append(t-J)
                p = Pose()
                p.position.x = pose.get('x', 0.0)
                p.position.y = pose.get('y', 0.0)
                q = quaternion_from_euler(0, 0, pose.get('theta', 0.0))
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
                nav_goal.poses.append(p)

        servers = []
        now = rospy.Time.now()
        for name, joint_set in self.joint_map.iteritems():
            server = self.joint_servers[name]
            goals[name].trajectory.header.stamp = now
            if len(goals[name].trajectory.points)==0:
                rospy.loginfo("%-50s No points"%name)
                continue
            servers.append(server)
            server.send_goal(goals[name])

        if len(nav_goal.poses)>0:
            print nav_goal
            nav_goal.header.stamp = now + rospy.Duration(J)
            self.nav_server.send_goal(nav_goal)
            servers.append(self.nav_server)
        else:
            rospy.loginfo("%-50s No points"%self.nav_server.action_client.ns)

        rate = rospy.Rate(1)
        while len(servers) > 0 and not rospy.is_shutdown():
            for server in servers:
                if server.get_state() > 1:
                    rospy.loginfo("%-50s %-10s %s"%(server.action_client.ns, CONDITIONS[server.get_state()], server.get_goal_status_text()))
                    servers.remove(server)
            rate.sleep()

        for server in servers:
            rospy.loginfo("-%50s %-10s %s"%(server.action_client.ns, CONDITIONS[server.get_state()], server.get_goal_status_text()))
            
                    

if __name__=='__main__':
    if len(sys.argv) < 2:
        print "Usage: %s [yaml-file]"%sys.argv[0]
        exit(1)
    poses = yaml.load(open(sys.argv[1], 'r'))
    pm = PoseMorph()
    pm.execute(poses)

