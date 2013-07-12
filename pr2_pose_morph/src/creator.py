#!/usr/bin/python

import roslib; roslib.load_manifest('pr2_pose_morph')
import rospy
import sys, operator
import wx
import tf
import yaml
import os
from  geometry_msgs.msg import Pose, TransformStamped
from    sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from          threading import Thread
from        urdf_python import URDF
from                      easy_markers.interactive import InteractiveGenerator
from                        visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerFeedback

class Creator:
    def __init__(self, filename=None):
        rospy.init_node('pose_creator')
        self.joint_pub = rospy.Publisher('/jointstates2', JointState)
        self.tf_pub = tf.TransformBroadcaster()
        self.ig = InteractiveGenerator()
        self.marker = self.ig.makeMarker(controls=["move_x", "move_y", "rotate_z"], name="Odom", description="Odom", imode=2, callback=self.callback, frame='/odom_combined')

        self.all_joints = set()
        self.filename = filename
        if filename is not None and os.path.exists(filename):
            f = open(filename)
            self.poses = yaml.load(f)
            f.close()

            for p in self.poses:
                for n in p:
                    if n not in ['x', 'y', 'theta', 't']:
                        self.all_joints.add(n)
        else:
            self.poses = [{'t': 0.0}]
        self.current_i = 0
        self.t = self.poses[0]['t']

        app = wx.App()
        self.gui = CreatorGUI(self)
        self.gui.Show()
        Thread(target=app.MainLoop).start()

    def callback(self, msg):
        q = msg.pose.orientation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.set_values({'x': msg.pose.position.x, 'y': msg.pose.position.y, 'theta': rpy[2]}, False)
        
    def set_value(self, field, value):
        self.set_values({field: value})

    def set_values(self, params, add=True):
        pose = self.poses[self.current_i]
        for k,v in params.iteritems():
            pose[k] = v
            if add:
                self.all_joints.add(k)
        self.poses[self.current_i] = pose

    def spin(self):
        r = rospy.Rate(10)
        while not self.gui.IsActive():
            None

        while not rospy.is_shutdown():
            self.tf_pub.sendTransform((self.get('x'),self.get('y'),0), 
                                        quaternion_from_euler(0, 0, self.get('theta')),
                                        rospy.Time.now(), '/base_footprint', '/odom_combined')
            js = JointState()
            js.header.stamp = rospy.Time.now()

            for joint in self.all_joints:
                js.name.append(joint)
                js.position.append(self.get(joint))

            js.name.append('torso_lift_joint')
            js.position.append(0.3)

            self.joint_pub.publish(js)
            r.sleep()
        print "QUIT"

    def get(self, field):
        before = None
        prior = None
        after = None
        for i,data in enumerate(self.poses):
            if not field in data:
                prior = None
                continue

            t = data['t']
            if t <= self.t:
                before = data
                prior = data
            else:
                after = data
                break

        if before is None:
            return 0.0
        if after is None:
            return before[field]
        if prior is None:
            return before[field]


        frac = (self.t - before['t'])/(after['t'] - before['t'])
        return before[field] + (after[field] - before[field]) * frac


    def add(self):
        if self.current_i + 1 < len(self.poses):
            newt = (self.poses[self.current_i]['t'] + self.poses[self.current_i+1]['t']) / 2
        else:
            newt = self.poses[self.current_i]['t'] + 1.0

        self.poses = self.poses[:self.current_i+1] + [{'t': newt}] + self.poses[self.current_i+1:]
        self.set_index(self.current_i + 1)

    def delete(self):
        self.poses = self.poses[:self.current_i] + self.poses[self.current_i+1:]
        self.set_index(self.current_i)

    def set_index(self, i):
        self.current_i = i
        self.t = self.poses[self.current_i]['t']

        p = Pose()
        p.position.x = self.get('x')
        p.position.y = self.get('y')
        q = quaternion_from_euler(0, 0, self.get('theta'))
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        self.ig.server.setPose('Odom', p)
        self.ig.server.applyChanges()

    def update_time(self, t):
        self.poses[self.current_i]['t'] = t
        self.poses = sorted(self.poses,  key=operator.itemgetter('t'))
        self.t = t

    def play(self):
        print 'here'
        self.t = 0.0
        rate = rospy.Rate(10)
        t_inc = .1

        while self.t < self.poses[-1]['t']:
            rate.sleep()
            self.t += t_inc

        self.set_index(len(self.poses)-1)

    def save(self):
        if self.filename is not None:
            f = open(self.filename, "w")
            yaml.dump(self.poses, f)
            f.close()
        print yaml.dump(self.poses)      
        
            
RANGE = 1000
class CreatorGUI(wx.Frame):
    def __init__(self, creator):
        wx.Frame.__init__(self, None, -1, "Pose Creator", (-1, -1))
        panel = wx.Panel(self, wx.ID_ANY)
        box = wx.BoxSizer(wx.VERTICAL)

        self.creator = creator
        self.buttons = {}
        self.sliders = {}
        self.joints  = {}

        self.addbutton = wx.Button(panel, -1, 'Add Pose')
        self.addbutton.Bind(wx.EVT_BUTTON, self.add)
        box.Add(self.addbutton, 0, wx.EXPAND)

        self.deletebutton = wx.Button(panel, -1, 'Delete')
        self.deletebutton.Bind(wx.EVT_BUTTON, self.delete)
        box.Add(self.deletebutton, -1, wx.EXPAND)  

        self.pose_list = wx.ListBox(panel, -1)
        self.pose_list.Bind(wx.EVT_LISTBOX, self.switch)
        box.Add(self.pose_list, 0, wx.EXPAND)

        self.time_box = wx.TextCtrl(panel, -1, style=wx.TE_PROCESS_ENTER )
        self.time_box.Bind(wx.EVT_TEXT_ENTER, self.time_set)
        box.Add(self.time_box, 0, wx.EXPAND)

        urdf = URDF().parse(rospy.get_param('robot_description'))
        arm_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        sliders = {'Head': ['head_pan_joint', 'head_tilt_joint'],
                   'LeftArm': ['l_%s'%a for a in arm_joints],
                   'RightArm':['r_%s'%a for a in arm_joints]}


        for title, joints in sorted(sliders.items()):
            p2 = wx.Panel(panel)
            sbox = wx.StaticBox(p2, -1, title)
            sizer = wx.StaticBoxSizer(sbox, wx.VERTICAL)
            for j in joints:
                p3 = wx.Panel(p2)
                s2 = wx.BoxSizer(wx.HORIZONTAL)

                self.buttons[j] = wx.Button(p3, -1, j, size=(300, 24))
                self.sliders[j] = wx.Slider(p3, -1, RANGE/2, 0, RANGE, 
                        style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL, size=(300, 24), name=j)

                if urdf.joints[j].joint_type == 'continuous':
                    self.joints[j] = (-3.14, 3.14)
                else:
                    self.joints[j] = (urdf.joints[j].limits.lower, urdf.joints[j].limits.upper)

                s2.Add(self.buttons[j])
                s2.Add(self.sliders[j])
                p3.SetSizer(s2)
                s2.Fit(p2)

                sizer.Add(p3)
            p2.Sizer = sizer
            sizer.Fit(p2)
            box.Add(p2, 0, wx.EXPAND)
        self.Bind(wx.EVT_SLIDER, self.slider_update)

        self.playbutton = wx.Button(panel, -1, 'Play')
        self.playbutton.Bind(wx.EVT_BUTTON, self.play)
        box.Add(self.playbutton, -1, wx.EXPAND)  

        self.savebutton = wx.Button(panel, -1, 'Save')
        self.savebutton.Bind(wx.EVT_BUTTON, self.save)
        box.Add(self.savebutton, -1, wx.EXPAND)        


        panel.SetSizer(box)
        box.Fit(self)        
        self.Update()



    def Update(self):
        self.pose_list.Set([str(data['t']) for data in self.creator.poses])
        self.pose_list.SetSelection(self.creator.current_i)
        self.time_box.SetValue( str(self.creator.t) )

        for j in self.joints:
            (lower, upper) = self.joints[j]
            v = self.creator.get(j)
            pct = (v - lower) / (upper - lower)
            self.sliders[j].SetValue(int(pct*RANGE))

    def switch(self, evt):
        if len(self.pose_list.GetSelections())>0:
            self.creator.set_index( self.pose_list.GetSelections()[0] )
            self.Update()

    def add(self, evt):
        self.creator.add()
        self.Update()

    def delete(self, evt):
        self.creator.delete()
        self.Update()

    def time_set(self, evt):
        self.creator.update_time(float(self.time_box.Value))
        self.Update()

    def slider_update(self, evt):
        joint = evt.GetEventObject().Name
        value = evt.GetEventObject().Value
        pct = value / float(RANGE)
        (lower, upper) = self.joints[joint]
        self.creator.set_value(str(joint), lower+(upper-lower)*pct)
        self.Update()

    def play(self, e):
        self.creator.play()
        self.Update()

    def save(self, e):  
        self.creator.save()

if __name__=='__main__':
    if len(sys.argv)>1:
        c = Creator(sys.argv[1])
    else:
        c = Creator()
    c.spin()
