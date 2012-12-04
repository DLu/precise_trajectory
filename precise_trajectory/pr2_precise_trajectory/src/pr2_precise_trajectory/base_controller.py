import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
import tf
import actionlib
from  geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from pr2_pose_morph.msg import *
from math import copysign

def orientation_to_euler(orientation):
    return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

ACTION_NAME = '/base_controller/move_sequence'

class BaseController:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/base_controller/command', Twist)
        self.tf = tf.TransformListener()
        self.server = actionlib.SimpleActionServer(ACTION_NAME, MoveSequenceAction, self.execute, False) 
        self.server.start()

        self.client = actionlib.SimpleActionClient(ACTION_NAME, MoveSequenceAction)
        rospy.loginfo("[BASE] Waiting for %s..."%ACTION_NAME)
        self.client.wait_for_server()

        self.frame = rospy.get_param('frame_id', '/base_footprint')
        self.tlim = rospy.get_param('translation_speed_limit', 1.2)
        self.rlim = rospy.get_param('rotation_speed_limit', 1.4)

        rospy.loginfo("[BASE] Ready!")

    def send_goal(self, goal):
        self.client.send_goal(goal)

    def wait_for_goal(self):
        self.client.wait_for_result()

    def execute(self, goal):
        r = rospy.Rate(100)

        # wait to start
        while rospy.Time.now() < goal.header.stamp:
            r.sleep()

        feedback = MoveSequenceFeedback()
        for pose, time in zip(goal.poses, goal.times):
            goal_time = goal.header.stamp + rospy.Duration(time)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = goal.header.frame_id
            goal_pose.pose = pose

            while rospy.Time.now() <= goal_time:
                relative = self.tf.transformPose(self.footprint, goal_pose)
                dx = relative.pose.position.x
                dy = relative.pose.position.y
                rot = orientation_to_euler(relative.pose.orientation)
                dz = rot[2]

                t = (goal_time - rospy.Time.now()).to_sec()

                cmd = Twist()
                if t < .1:
                    t = 1.0
                cmd.linear.x = dx/t
                cmd.linear.y = dy/t
                cmd.angular.z = dz/t

                alim = 1.4

                if abs(cmd.linear.x) > self.tlim:
                    cmd.linear.x = copysign(self.tlim, cmd.linear.x)
                if abs(cmd.linear.y) > tlim:
                    cmd.linear.y = copysign(self.tlim, cmd.linear.y)
                if abs(cmd.angular.z) > alim:
                    cmd.angular.z = copysign(self.rlim, cmd.angular.z)

                self.cmd_pub.publish(cmd)
                feedback.percent_complete = t / time
                self.server.publish_feedback(feedback)
                r.sleep()
            feedback.pose_index += 1

        self.server.set_succeeded(MoveSequenceResult())



if __name__=='__main__':
    pm = PoseMorph()
    rospy.spin()
