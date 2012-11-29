#!/usr/bin/python

import roslib; roslib.load_manifest('pr2_pose_morph')
import rospy
import tf
import actionlib
from  geometry_msgs.msg import Pose, Twist, TransformStamped, PoseStamped
from    sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pr2_pose_morph.msg import *

class PoseMorph:
    def __init__(self):
        rospy.init_node('pr2_move_sequence_server')
        self.cmd_pub = rospy.Publisher('/base_controller/command', Twist)
        self.tf = tf.TransformListener()
        self.server = actionlib.SimpleActionServer('/base_controller/move_sequence', MoveSequenceAction, self.execute, False) # check false
        self.server.start()
        rospy.loginfo("Ready!")

    def execute(self, goal):
        r = rospy.Rate(100)

        while rospy.Time.now() < goal.header.stamp:
            r.sleep()
        rospy.loginfo("And go!")

        feedback = MoveSequenceFeedback()
        for pose, time in zip(goal.poses, goal.times):
            goal_time = goal.header.stamp + rospy.Duration(time)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = goal.header.frame_id
            goal_pose.pose = pose

            while rospy.Time.now() <= goal_time:
                relative = self.tf.transformPose('/base_footprint', goal_pose)
                dx = relative.pose.position.x
                dy = relative.pose.position.y
                rot = euler_from_quaternion([relative.pose.orientation.x, relative.pose.orientation.y, relative.pose.orientation.z, relative.pose.orientation.w])
                dz = rot[2]

                t = (goal_time - rospy.Time.now()).to_sec()

                cmd = Twist()
                if t < .1:
                    t = 1.0
                cmd.linear.x = dx/t
                cmd.linear.y = dy/t
                cmd.angular.z = dz/t

                tlim = 1.2
                alim = 1.4

                if abs(cmd.linear.x) > tlim:
                    cmd.linear.x *= tlim / abs(cmd.linear.x)
                if abs(cmd.linear.y) > tlim:
                    cmd.linear.y *= tlim / abs(cmd.linear.y)
                if abs(cmd.angular.z) > alim:
                    cmd.angular.z *= alim / abs(cmd.angular.z)

                self.cmd_pub.publish(cmd)
                feedback.percent_complete = t / time
                self.server.publish_feedback(feedback)
                r.sleep()
            feedback.pose_index += 1

        self.server.set_succeeded(MoveSequenceResult())



if __name__=='__main__':
    pm = PoseMorph()
    rospy.spin()
