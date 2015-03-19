#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_score_creator')
import rospy

from  geometry_msgs.msg import Twist, PoseStamped
from joy_listener import JoyListener, PS3
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from math import copysign
BUTTON_LAG = 1.0

class HalfTeleop:
    def __init__(self):
        rospy.init_node('half_teleop')
        
        rospy.wait_for_service('/proceed')
        self.proxy = rospy.ServiceProxy('/proceed', Empty)
        self.mode  = 0
        self.cmd_pub = rospy.Publisher('/base_controller/command', Twist)
        self.tlim = 1.0
        self.rlim = 1.0

        self.mcmd = None
        self.sub1 = rospy.Subscriber('/base_controller/command_bc', Twist, self.cb1)

        
        self.joy = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.cmd = [0,0,0]
        print "ROCK AND ROLL"

    def joy_cb(self, msg):
        self.drive(msg.axes)
        if msg.buttons[ PS3('l1') ]:
            self.mode = 1
        elif msg.buttons[ PS3('r1') ]: 
            self.mode = 2
        else: 
            self.mode = 0

        if msg.buttons[ PS3('select') ]:
            self.proxy()

    def cb1(self, msg):
        self.mcmd = msg
        
    def enable(self, mode):
        print self.mode
        self.mode = mode
        self.t = rospy.Time.now()

    def drive(self, axes):
        self.cmd = [.6 * axes[3], .6 * axes[2], .8 *axes[0]]
        #self.controller.base.publish_command(.6*x, .6*y, .8*z)
        
    def publish_command(self, dx, dy, dz):
        print dx, dy, dz
        cmd = Twist()
        cmd.linear.x = dx
        cmd.linear.y = dy
        cmd.angular.z = dz

        if abs(cmd.linear.x) > self.tlim:
            cmd.linear.x = copysign(self.tlim, cmd.linear.x)
        if abs(cmd.linear.y) > self.tlim:
            cmd.linear.y = copysign(self.tlim, cmd.linear.y)
        if abs(cmd.angular.z) > self.rlim:
            cmd.angular.z = copysign(self.rlim, cmd.angular.z)

        self.cmd_pub.publish(cmd)

    def spin(self):
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.mode==0:
                if self.mcmd:
                    self.cmd_pub.publish(self.mcmd)
                    self.mcmd = None
            elif self.mode==1:
                self.publish_command(self.cmd[0], self.cmd[1], self.cmd[2])
            elif self.mode==2:
                dx, dy, dz = self.cmd
                dy += -dz * 0.8
                self.publish_command(dx, dy, dz)
                        
            r.sleep()


if __name__ == '__main__':
    ir = HalfTeleop()
    ir.spin()

