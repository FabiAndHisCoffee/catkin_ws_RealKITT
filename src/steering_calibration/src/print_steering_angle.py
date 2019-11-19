#!/usr/bin/env python

import rospy
from autominy_msgs.msg import SteeringFeedback

steering_angle_power = 0
steering_angle = 0
ANGLE_PER_POWER = 0.200469279054956
FORWARD_STEERING_MEAN = 330


def steering_angle_update(data):
    global steering_angle_power
    global steering_angle
    steering_angle_power = data.value
    steering_angle = (steering_angle_power - FORWARD_STEERING_MEAN) * ANGLE_PER_POWER
    rospy.loginfo('steering_angle is {}'.format(steering_angle))

rospy.sleep(1.0)

rospy.init_node('steering_output')
rospy.loginfo('init_node(')
rospy.sleep(1.0)

rospy.Subscriber('/sensors/arduino/steering_angle', SteeringFeedback, steering_angle_update)
rospy.loginfo('Subscriber(communication')
rospy.sleep(1.0)

rospy.spin()