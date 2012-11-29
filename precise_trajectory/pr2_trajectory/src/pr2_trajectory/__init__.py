import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from pylab import *
from control_msgs.msg import *

BOTH_ARMS = ['l', 'r']
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
ERRORS = [ 'SUCCESSFUL', 'ABORTED', 'GOAL_TOLERANCE_VIOLATED', 'PATH_TOLERANCE_VIOLATED', 'OLD_HEADER_TIMESTAMP', 'INVALID_JOINTS', 'INVALID_GOAL']

TEST_RANGES = {"r_shoulder_pan_joint": (-.7, .16), 
          "r_shoulder_lift_joint": (-.3, .3),
          "r_upper_arm_roll_joint": (-.6, .6),
          "r_elbow_flex_joint": (-1.5, -.2), 
          "r_forearm_roll_joint": (-1.5, 1.5), 
          "r_wrist_flex_joint": (-1.5, -.2), 
          "r_wrist_roll_joint": (-1.5, 1.5)}

def kinect_to_pr2(ktraj):
    ltraj = JointTrajectory()
    rtraj = JointTrajectory()
    tmap = {'l': ltraj, 'r': rtraj}

    for arm in BOTH_ARMS:
        traj = tmap[arm]
        traj.joint_names = ['%s_%s'%(arm, a) for a in ARM_JOINTS]
        for point in ktraj.points:
            pt = JointTrajectoryPoint()
            traj.points.append(pt)
        for name in traj.joint_names:
            if name in ktraj.joint_names:
                idx = ktraj.joint_names.index(name)
            else:
                idx = -1

            for (kpt, pt) in zip(ktraj.points, traj.points):
                if len(kpt.positions)>0:
                    if idx >= 0:
                        value = kpt.positions[idx]
                    else:
                        value = 0.0
                    pt.positions.append(value)
                if len(kpt.velocities)>0:
                    if idx >= 0:
                        value = kpt.velocities[idx]
                    else:
                        value = 0.0
                    pt.velocities.append(value)

                pt.time_from_start = kpt.time_from_start
    return tmap

def traj_to_joint_state(trajs, t):
    state = JointState()
    state.header.stamp = rospy.Time.now()
    for traj in trajs:
        state.name += traj.joint_names
        state.position += traj.points[t].positions
    return state

class JointActor:
    def __init__(self):
        self.action_clients = {}
        self.follow_clients = {}
        for arm in BOTH_ARMS:
            self.action_clients[arm] = SimpleActionClient("%s_arm_controller/joint_trajectory_action"%arm, JointTrajectoryAction)
            self.action_clients[arm].wait_for_server()
            self.follow_clients[arm] = SimpleActionClient("%s_arm_controller/follow_joint_trajectory"%arm, FollowJointTrajectoryAction)
            self.follow_clients[arm].wait_for_server()
        print "Arms ready"

    def actions(self, tmap, wait=True):
        for (arm, trajectory) in tmap.items():
            self.action(arm, trajectory, False)
        if wait:
            for arm in tmap:
                self.action_clients[arm].wait_for_result()

    def action(self, arm, trajectory, wait=True):
        goal = JointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.trajectory.header.stamp = rospy.Time.now()
        self.action_clients[arm].send_goal(goal)
        if wait:
            self.action_clients[arm].wait_for_result()

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

    def presets(self, tmap, action_time=3.0, sleep_time=1.0):
        startmap = {}
        for (arm, otraj) in tmap.items():
            trajectory = JointTrajectory()
            trajectory.joint_names = otraj.joint_names
            pt = JointTrajectoryPoint()
            pt.positions = otraj.points[0].positions
            pt.time_from_start = rospy.Duration(action_time)
            trajectory.points.append(pt)
            startmap[arm] = trajectory
        self.actions(startmap)
        rospy.sleep(sleep_time)

import math
def normalize(angle):
    return angle - 2 * math.pi * math.floor((angle+math.pi)/(2*math.pi))

def graph_trajectory(trajectory, gtype="o-", prefix_filter=None, label_prefix=None, **keywords):
    t = []
    for pt in trajectory.points:
        t.append(pt.time_from_start.to_sec())
    for (i, name) in enumerate(trajectory.joint_names):
        if prefix_filter is not None:
            if name.find(prefix_filter)!=0:
                continue
        y = []
        for pt in trajectory.points:
            y.append( normalize( pt.positions[i] ))
        if label_prefix is None:
            label = name
        else:
            label = "%s %s"%(label_prefix, name)
        plot(t,y, gtype, label=label, **keywords)

def graph_joints(data, gtype="o-", start_time=None, names_filter=None, label_prefix=None, **keywords):
    t = []
    if start_time is None:
        start_time = data[0].header.stamp

    for msg in data:
        t.append((msg.header.stamp - start_time).to_sec() )

    if names_filter is None:
        indexes = range( len(data[0].name) )
    else:
        indexes = []
        for name in names_filter:
            indexes.append( data[0].name.index(name) )

    for ind in indexes:
        y = []
        for msg in data:
            y.append( normalize( msg.position[ind] ))
        name = data[0].name[ind]
        if label_prefix is None:
            label = name
        else:
            label = "%s %s"%(label_prefix, name)
        plot(t,y, gtype, label=label, **keywords)
        plot(t,y,gtype, **keywords)

def show_graph(loc=None):
    if loc is not None:
        legend(loc=loc)
    grid(True)
    title('Trajectories')
    show()

class Recorder:
    def __init__(self):
        self.start_time = None
        self.data = []
        self.done = False
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_cb)

    def joint_cb(self, msg):
        if self.start_time is not None and not self.done:
            self.data.append(msg)

    def start(self):
        self.start_time = rospy.Time.now()
    
    def stop(self):
        self.done = True



