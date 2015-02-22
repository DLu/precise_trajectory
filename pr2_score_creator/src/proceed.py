#!/usr/bin/python

import rospy
from std_srvs.srv import Empty

rospy.init_node('proceed_button')
rospy.wait_for_service('/proceed')
proxy = rospy.ServiceProxy('/proceed', Empty)

print "Ready"

while not rospy.is_shutdown():
    raw_input("Proceed? ")
    proxy()
    
