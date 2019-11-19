#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

from autominy_msgs.msg import NormalizedSpeedCommand
from autominy_msgs.msg import Speed
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SteeringFeedback
from std_msgs.msg import Float32
from std_msgs.msg import String

CAR_NUMBER = '10'

x_position = 0
y_position = 0
steering_angle = 0

# norm_steering_left = NormalizedSteeringCommand()
# norm_steering_left.value = STEERING
#
# move_speed = Speed()
# move_speed.value = SPEED

last_driving_control_info = ""

def callbackDrivingControl(msg):
    last_driving_control_info = msg.data


def position_update(data):
	global x_position
	global y_position
	x_position = data.pose.pose.position.x
	# rospy.loginfo('Position x is %f', data.pose.pose.position.x)
	y_position = data.pose.pose.position.y
	# rospy.loginfo('Position y is %f', data.pose.pose.position.y)

def steering_angle_update(data):
    global steering_angle
    steering_angle = data.value

rospy.sleep(1.0)

rospy.init_node('steering_calibration')
rospy.loginfo('init_node(')
rospy.sleep(1.0)


rospy.Subscriber('/communication/gps/'+CAR_NUMBER, Odometry, position_update)
rospy.loginfo('Subscriber(communication')
rospy.sleep(1.0)
rospy.Subscriber('/sensors/arduino/steering_angle', SteeringFeedback, steering_angle_update)
rospy.loginfo('Subscriber(communication')
rospy.sleep(1.0)


publisher_steering 	= rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size = 10)
rospy.loginfo('Publisher(actuators/steering_normalized')
rospy.sleep(1.0)


pub_speed = rospy.Publisher('/actuators/speed_normalized', NormalizedSpeedCommand, queue_size = 10)
rospy.loginfo('Publisher(actuators/speed')
rospy.sleep(1.0)


# pub_forward = rospy.Publisher(
#     "simple_drive_control/forward",
#     Float32,
#     queue_size=10)
# rospy.loginfo('Publisher(simple_drive_control/forward')
# rospy.sleep(1.0)
# publisher_steering.publish(norm_steering_left)
# publisher_speed.publish(move_speed)

positions = []
steering_angles = []

positions.append((x_position, y_position))
steering_angles.append(steering_angle)

drive_command = NormalizedSpeedCommand()
drive_command.value = 0.33

stop_command = NormalizedSpeedCommand()
stop_command.value = 0.0

steering_cmd = NormalizedSteeringCommand()
steering_cmd.value = -1.0

# publisher_steering.publish()

publisher_steering.publish(steering_cmd)
rospy.loginfo('publisher_steering.publish(steering_cmd={})'.format(steering_cmd.value))
rospy.sleep(0.5)

# drive for a while


pub_speed.publish(drive_command)
rospy.loginfo('pub_speed.publish({})'.format(drive_command.value))

SLEEP_AT_WHEEL = 0.4
positions.append((x_position, y_position))
steering_angles.append(steering_angle)
rospy.sleep(SLEEP_AT_WHEEL)
positions.append((x_position, y_position))
steering_angles.append(steering_angle)
rospy.sleep(SLEEP_AT_WHEEL)
positions.append((x_position, y_position))
steering_angles.append(steering_angle)
rospy.sleep(SLEEP_AT_WHEEL)

# pub_speed.publish(drive_command)
# rospy.loginfo('pub_speed.publish({})'.format(drive_command.value))

pub_speed.publish(stop_command)
rospy.loginfo('pub_speed.publish({})'.format(stop_command.value))
rospy.sleep(0.5)

positions.append((x_position, y_position))
steering_angles.append(steering_angle)
rospy.sleep(1.0)

# pub_speed.publish(stop_command)
# rospy.loginfo('pub_speed.publish({})'.format(stop_command.value))
# rospy.sleep(1.0)
rospy.loginfo('positions = {}'.format(positions))
rospy.loginfo('steering_angles = {}'.format(steering_angles))

# rospy.spin()