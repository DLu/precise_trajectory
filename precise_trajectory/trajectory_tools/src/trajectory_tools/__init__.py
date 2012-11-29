import roslib; roslib.load_manifest('trajectory_tools')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import pickle
import yaml

def get_time(obj, dmap):
    obj.secs = dmap.get('secs', 0)
    obj.nsecs = dmap.get('nsecs', 0)

def trajectory_from_text( fd ):
    tmap = yaml.load(fd)
    jt = JointTrajectory()
    header = tmap.get('header', {})
    jt.header.seq = header.get('seq', 0)
    if 'stamp' in header:
        get_time(jt.header.stamp, header['stamp'])
    jt.header.frame_id = header.get('frame_id', '')
    jt.joint_names = tmap.get('joint_names', [])
    for pt in tmap.get('points', []):
        tpt = JointTrajectoryPoint()
        tpt.positions = pt.get('positions', [])
        tpt.velocities = pt.get('velocities', [])
        tpt.accelerations = pt.get('accelerations', [])
        get_time(tpt.time_from_start, pt.get('time_from_start', {}))
        jt.points.append(tpt)
    return jt

def load_trajectory( fd ):
    return pickle.load( fd )

def write_trajectory( trajectory, fd ):
    pickle.dump(trajectory, fd)
