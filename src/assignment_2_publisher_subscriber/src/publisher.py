#!/usr/bin/env python

import rospy
from autominy_msgs.msg import Speed
from autominy_msgs.msg import NormalizedSteeringCommand

STEERING = 1.0
SPEED	 = 0.3

norm_steering_left = NormalizedSteeringCommand()
norm_steering_left.value = STEERING

move_speed = Speed()
move_speed.value = SPEED

rospy.init_node('left_steering_commander')

rate = rospy.Rate(10)

publisher_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size = 10)
publisher_speed= rospy.Publisher('/actuators/speed', Speed, queue_size = 10)

while not rospy.is_shutdown():
	publisher_steering.publish(norm_steering_left)
	publisher_speed.publish(move_speed)


