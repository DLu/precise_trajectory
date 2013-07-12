#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from pr2_trajectory import *
import sys
import collections

TRIALS = 2
PCT = 0.5
RESOLUTION = .001
MINV = 0.01
MAXV = 5.0

class Tester:
    def __init__(self):
        self.j = JointActor()
        self.data = collections.defaultdict( dict )

    def test_once(self, name, speed):
        (low, high) = TEST_RANGES[name]
        time = (high - low) / speed

        traj = JointTrajectory()
        traj.points = [JointTrajectoryPoint(), JointTrajectoryPoint(), JointTrajectoryPoint()]
        for jn in ARM_JOINTS:
            jname = 'r_%s'%jn
            traj.joint_names.append(jname)
            if jname==name:
                traj.points[0].positions.append(high)
                traj.points[1].positions.append(low)
                traj.points[2].positions.append(high)
            else:
                (xl, xh) = TEST_RANGES[jname]
                if xh < 0:
                    value = xh
                else:
                    value = 0.0
                traj.points[0].positions.append(value)
                traj.points[1].positions.append(value)
                traj.points[2].positions.append(value)
        traj.points[0].time_from_start = rospy.Duration(0)
        traj.points[1].time_from_start = rospy.Duration(time)
        traj.points[2].time_from_start = rospy.Duration(time*2)

        self.j.presets( {'r': traj}, sleep_time=3.0 )

        tolerance = JointTolerance()
        tolerance.name = name
        tolerance.position = .05

        #print "Trial %f seconds - %f rad/s - "%(time, speed),

        rec = Recorder()
        rec.start()
        self.j.follow('r', traj, [tolerance])
        A = self.j.follow_result(['r'])
        if A is None:
            return False
        (code, error) = A
        rec.stop()

        #print error

        if False:
            figure()
            graph_trajectory(traj, 'o')
            graph_joints(rec.data, '-', rec.start_time, traj.joint_names)
            show_graph()
        if code >= 0:
            return True
        else:
            return False
    
    def test_n(self, name, speed):
        c = 0
        N = TRIALS
        print speed, '\t',
        for i in range(N):
            x = self.test_once(name, speed)
            if x:
                c += 1
                print 'o',
            else:
                print 'x',
            if rospy.is_shutdown():
                print
                return c/(i+1)
        print
        return c/float(N)

    def binary(self, name):
        minV = MINV
        maxV = MAXV
        while maxV - minV > RESOLUTION and not rospy.is_shutdown():
            print '%s [%.4f, %.4f]'%(name, minV, maxV)
            mid = (minV+maxV)/2
            pct = self.test_n(name, mid)
            self.data[name][mid] = pct
            if pct > PCT:
                minV = mid
            else:
                maxV = mid
        return minV

if __name__ == '__main__':
    rospy.init_node('pr2_trajectory')
    tester = Tester()
    values = {}

    for name in TEST_RANGES:
        v = tester.binary(name)
        values[name] = v

    for name in tester.data:
        print "====== %s ===== %f"%(name, values[name])
        d = tester.data[name]
        for v in sorted(d):
            print "%.5f\t%.3f"%(v, d[v])
        
