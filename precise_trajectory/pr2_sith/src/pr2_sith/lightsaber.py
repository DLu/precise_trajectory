#!/usr/bin/env python
"""********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  \author Joe Romano
 *********************************************************************/
//@author  Joe Romano
//@email   joeromano@gmail.com
//@brief   high_five.cpp - pr2 gives you props yo
"""

import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_controllers_msgs.msg import *
from pr2_gripper_sensor_msgs.msg import *
from actionlib import SimpleActionClient, SimpleGoalState
from sensor_msgs.msg import JointState
import trajectory_msgs.msg

both_arms = ['l', 'r']
OPEN = 0.09
CLOSED = 0.002

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

class RobotController:
    def __init__(self):
        self.effort = -1.0
        self.accel = rospy.get_param('acceleration_trigger', 6.0)
        self.slip = rospy.get_param('slip_trigger', 0.008)
        self.trigger = PR2GripperEventDetectorCommand.ACC  # use just acceleration as our contact signal
        self.speedup = 2.5

        self.arms = {}
        #Initialize the client for the Action interface to the gripper controller
        #and tell the action client that we want to spin a thread by default
        self.grippers = {}
        self.placers = {}
        for arm in both_arms:
            self.arms[arm] = SimpleActionClient("%s_arm_controller/joint_trajectory_action"%arm, JointTrajectoryAction)
            self.grippers[arm] = SimpleActionClient("%s_gripper_sensor_controller/gripper_action"%arm, Pr2GripperCommandAction)
            self.placers[arm]  = SimpleActionClient("%s_gripper_sensor_controller/event_detector"%arm, PR2GripperEventDetectorAction)
            #wait for the action servers to come up 
            rospy.loginfo("Waiting for %s controllers"%arm)
            self.arms[arm].wait_for_server()
            self.grippers[arm].wait_for_server()        
            self.placers[arm].wait_for_server()
            rospy.loginfo("Got %s controllers"%arm)

        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_cb)
        self.joint_pos = {}


    def joint_cb(self, msg):
        for arm in both_arms:
            for joint in ARM_JOINTS:
                name = "%s_%s"%(arm, joint)
                i = msg.name.index(name)
                self.joint_pos[name] = msg.position[i]

    def start_trajectory(self, goal, arm):
        """ Sends the command to start a given trajectory """
        self.arms[arm].send_goal(goal)

    def make_trajectory(self, angles, duration, arm):
        """ Generates a simple trajectory from a single waypoint"""

        # our goal variable
        goal = JointTrajectoryGoal()
        # Start the trajectory immediately
        goal.trajectory.header.stamp = rospy.Time.now()

        # First, the joint names, which apply to all waypoints
        names = ['%s_%s'%(arm, a) for a in ARM_JOINTS]
        goal.trajectory.joint_names = names

        # We will have N waypoints in this goal trajectory
        goal.trajectory.points = [trajectory_msgs.msg.JointTrajectoryPoint()]

        # First trajectory point
        goal.trajectory.points[0].positions = angles
        goal.trajectory.points[0].velocities = [0.0]*7
        # set time we want this trajectory to be reached at
        goal.trajectory.points[0].time_from_start = rospy.Duration(duration)

        return goal

    def make_trajectory_long(self, movements, arm):
        # our goal variable
        goal = JointTrajectoryGoal()
        # Start the trajectory immediately
        goal.trajectory.header.stamp = rospy.Time.now()

        # First, the joint names, which apply to all waypoints
        names = ['%s_%s'%(arm, a) for a in ARM_JOINTS]
        goal.trajectory.joint_names = names

        # We will have N waypoints in this goal trajectory
        goal.trajectory.points = []
        t=0
        for move in movements:
            pt = trajectory_msgs.msg.JointTrajectoryPoint()
            pt.positions = move[arm]
            t+= move.get('time', 3)/self.speedup
            pt.time_from_start = rospy.Duration(t)
            #pt.velocities = [0.0]*7
            goal.trajectory.points.append(pt)
        return goal

    def gripper(self, arms, position, should_wait=True):
        gg = Pr2GripperCommandGoal()
        gg.command.position = position
        gg.command.max_effort = self.effort

        for arm in arms:
            self.grippers[arm].send_goal(gg)

        if should_wait:
            for arm in arms:
                self.grippers[arm].wait_for_result()
        
    def wait_for_impact(self, arms):
        #move into place mode to drop an object
        place_goal = PR2GripperEventDetectorGoal()
        place_goal.command.trigger_conditions = self.trigger
        place_goal.command.acceleration_trigger_magnitude = self.accel  # m/^2
        place_goal.command.slip_trigger_magnitude = self.slip        # slip gain

        for arm in arms:
            self.placers[arm].send_goal(place_goal)

        #wait for a slap
        while(not self.slapDone(arms) and not rospy.is_shutdown()):
            rospy.sleep(0.005)

    def is_done(self, client):
        return client.get_state() > SimpleGoalState.ACTIVE

    def slapDone(self, arms):
        #! Returns the current state of the action
        done = True
        for arm in arms:
            done = done and self.is_done(self.placers[arm])
        return done

    def right_arm(self):
        arr = []
        for joint in ARM_JOINTS:
            arr.append( self.joint_pos["r_%s"%joint] )
        return arr

