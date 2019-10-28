#!/usr/bin/env python

import rospy
from  autominy_msgs.msg import Speed

def callback(data):
	rospy.loginfo('Speed is %f', data.value)

rospy.init_node('speedometer')
rospy.Subscriber('/sensors/speed', Speed, callback)

rospy.spin()
