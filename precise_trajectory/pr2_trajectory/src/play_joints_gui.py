#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_trajectory')
import rospy
from sensor_msgs.msg import JointState
import trajectory_msgs.msg
from trajectory_tools import load_trajectory
from pr2_trajectory import kinect_to_pr2, traj_to_joint_state
import sys
import wx

class MyFrame(wx.Frame):
    def __init__(self, trajectories):
        wx.Frame.__init__(self, None, -1, "Player", wx.DefaultPosition, (300, 150))

        self.pub = rospy.Publisher('/arm_angles', JointState)
        self.trajectories = trajectories
        self.max_t = len(trajectories[0].points)
        panel = wx.Panel(self, -1)

        vbox = wx.BoxSizer(wx.VERTICAL)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        self.sld = wx.Slider(panel, -1, 0, 0, self.max_t-1, wx.DefaultPosition, (250, -1),
                              wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)

        self.Bind(wx.EVT_SCROLL, self.OnAdjust)

        vbox.Add(self.sld, 1, wx.ALIGN_CENTRE)
        vbox.Add(hbox, 0, wx.ALIGN_CENTRE | wx.ALL, 20)
        panel.SetSizer(vbox)

    def OnAdjust(self, event):
        self.play(self.sld.GetValue())

    def play(self, t):
        state = traj_to_joint_state(self.trajectories, t )
        self.pub.publish(state)

class MyApp(wx.App):
    def __init__(self, mocaps):
        self.mocaps = mocaps
        wx.App.__init__(self, 0)

    def OnInit(self):
        frame = MyFrame(self.mocaps)
        frame.Show(True)
        frame.Centre()
        frame.play(0)
        return True

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print "%s trajectory "%sys.argv[0]
            sys.exit(1)

        rospy.init_node('mocap_play')

        trajectory = load_trajectory( open(sys.argv[1]))
        app = MyApp(kinect_to_pr2(trajectory).values())
        app.MainLoop()

    except rospy.ROSInterruptException: pass

