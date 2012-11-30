import roslib; roslib.load_manifest('pr2_precise_trajectory')
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pickle
import yaml
import rospy

def load_trajectory(filename):
    if ".traj" in filename:
        trajectory = pickle.load(open(filename, 'r'))
        return trajectory_to_simple( trajectory )
    elif '.yaml' in filename:
        return yaml.load( open(filename, 'r'))
    else:
        print "Unknown file type"
        return None

def save_trajectory(trajectory, filename):
    f = open(filename, 'w')
    f.write(yaml.dump(trajectory))
    f.close()


def simple_to_message_single(angles, duration, arm):
    movements = {arm: angles, 'time': duration}
    return simple_to_message([movements], arm)

def simple_to_message(movements, arm, default_time=3.0):
    trajectory = JointTrajectory()
    trajectory.joint_names = get_arm_joint_names(arm)
    trajectory.header.stamp = rospy.Time.now()
    t=0
    for move in movements:
        pt = JointTrajectoryPoint()
        pt.positions = move[arm]
        t+= move.get('time', default_time)
        pt.time_from_start = rospy.Duration(t)
        #pt.velocities = [0.0]*7
        trajectory.points.append(pt)
    return trajectory

def trajectory_to_simple(trajectory, fill_missing_with_zeros=True):
    indexes = {}
    for arm in ['l', 'r']:
        idx = []
        found = 0
        missing = 0
        for name in get_arm_joint_names(arm):
            if name not in trajectory.joint_names:
                idx.append(None)
                missing += 1
            else:
                idx.append(trajectory.joint_names.index(name))
                found += 1
            
        if found>0 and (fill_missing_with_zeros or missing == 0):
            indexes[arm] = idx

    if len(indexes)==0:
        print "ERROR: Neither arm defined"
        return []

    arr = []
    last_time = 0.0
    for point in trajectory.points:
        m = {}
        for arm, idx in indexes.iteritems():
            m[arm] = []
            for index in idx: 
                if index==None:
                    m[arm].append(0.0)
                else:
                    m[arm].append( point.positions[index] ) 
            #TODO Velocity
        time = point.time_from_start.to_sec()
        m['time'] = time- last_time
        last_time = time
        arr.append(m)
    return arr
    
def simple_to_joint_states(movements, default_time=3.0, start_time=None):
    arr = []
    if start_time is None:
        start_time = rospy.Time.now()
    for move in movements:
        start_time += rospy.Duration( move.get('time', default_time) )
        state = JointState()
        state.header.stamp = start_time
        for arm in ['l', 'r']:
            if arm not in move:
                continue
            state.name += get_arm_joint_names(arm)
            state.position += move[arm]
        arr.append(state)
    return arr

def tprint(movements):
    for move in movements:
        print "%0.4f"%move['time'] , 
        for arm in ['l', 'r']:
            if arm in move:
                j = ["%.3f"%x for x in move[arm] ]
                print "%s: [%s]"%(arm,",".join(j)),
        for x,a in move.iteritems():
            if x not in ['l', 'r', 'time']:
                print "%s: %s"%(x,str(a)),
        print 
