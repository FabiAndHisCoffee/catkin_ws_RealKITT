#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

from autominy_msgs.msg import NormalizedSpeedCommand
from autominy_msgs.msg import Speed
from autominy_msgs.msg import Tick
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SteeringFeedback
from std_msgs.msg import Float32
from std_msgs.msg import String

CAR_NUMBER = '11'

x_position = 0
y_position = 0
cumulative_ticks = 0
last_tick = 0
tick_changed = False
steering_angle_power = 0
steering_angle = 0
ANGLE_PER_POWER = 0.200469279054956
FORWARD_STEERING_MEAN = 330

# norm_steering_left = NormalizedSteeringCommand()
# norm_steering_left.value = STEERING
#
# move_speed = Speed()
# move_speed.value = SPEED

last_driving_control_info = ""


class TickCounter(object):
    def __init__(self):
        self.cumulative_ticks = 0
        self.last_tick = 0

    def callback(self, data):
        # new_tick = data.value
        # tick_has_changed = (bool(new_tick) != bool(self.last_tick)) and bool(new_tick)
        self.cumulative_ticks += data.value
        # last stuff
        # self.last_tick = data.value
        # rospy.loginfo('cumulative_ticks is {}'.format(self.cumulative_ticks))


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
    global steering_angle_power
    global steering_angle
    steering_angle_power = data.value
    steering_angle = (steering_angle_power - FORWARD_STEERING_MEAN) * ANGLE_PER_POWER
    # rospy.loginfo('steering_angle is {}'.format(steering_angle))


tick_counter = TickCounter()

rospy.sleep(1.0)

rospy.init_node('steering_calibration')
rospy.loginfo('init_node(')
rospy.sleep(1.0)

subscriber_tick = rospy.Subscriber('/sensors/arduino/ticks', Tick, tick_counter.callback)
rospy.loginfo('Subscriber(/sensors/arduino/ticks')
rospy.sleep(1.0)
subscriber_position = rospy.Subscriber('/communication/gps/'+CAR_NUMBER, Odometry, position_update)
rospy.loginfo('Subscriber(communication/gps/'+CAR_NUMBER)
rospy.sleep(1.0)
subscriber_steering = rospy.Subscriber('/sensors/arduino/steering_angle', SteeringFeedback, steering_angle_update)
rospy.loginfo('Subscriber(communication')
rospy.sleep(1.0)


publisher_steering 	= rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size = 10)
rospy.loginfo('Publisher(actuators/steering_normalized')
rospy.sleep(1.0)


publisher_speed = rospy.Publisher('/actuators/speed_normalized', NormalizedSpeedCommand, queue_size = 10)
rospy.loginfo('Publisher(actuators/speed_normalized')
rospy.sleep(1.0)


# pub_forward = rospy.Publisher(
#     "simple_drive_control/forward",
#     Float32,
#     queue_size=10)
# rospy.loginfo('Publisher(simple_drive_control/forward')
# rospy.sleep(1.0)
# publisher_steering.publish(norm_steering_left)
# publisher_speed.publish(move_speed)

positions_and_ticks = []
steering_angles = []


steering_angles.append(steering_angle_power)

drive_command = NormalizedSpeedCommand()
drive_command.value = 0.067

stop_command = NormalizedSpeedCommand()
stop_command.value = 0.0

steering_cmd = NormalizedSteeringCommand()
steering_cmd.value = 1.0

# publisher_steering.publish()

publisher_steering.publish(steering_cmd)
rospy.loginfo('publisher_steering.publish(steering_cmd={})'.format(steering_cmd.value))
rospy.sleep(0.5)

# drive for a while

positions_and_ticks.append(((x_position, y_position), tick_counter.cumulative_ticks))
publisher_speed.publish(drive_command)
rospy.loginfo('pub_speed.publish({})'.format(drive_command.value))

rospy.sleep(5.0)

# pub_speed.publish(drive_command)
# rospy.loginfo('pub_speed.publish({})'.format(drive_command.value))

publisher_speed.publish(stop_command)
positions_and_ticks.append(((x_position, y_position),tick_counter.cumulative_ticks))
rospy.loginfo('pub_speed.publish({})'.format(stop_command.value))
rospy.sleep(0.5)

steering_angles.append(steering_angle_power)
rospy.sleep(1.0)

# pub_speed.publish(stop_command)
# rospy.loginfo('pub_speed.publish({})'.format(stop_command.value))
# rospy.sleep(1.0)
rospy.loginfo('positions = {}'.format(positions_and_ticks))
first_pos, first_ticks = positions_and_ticks[0]
x_1 , y_1 = first_pos
last_pos, last_ticks = positions_and_ticks[1]

x_2 , y_2 = last_pos

dx = x_1 - x_2
dy = y_1 - y_2
import math

distance = math.sqrt((dx*dx)+(dy*dy))
rospy.loginfo('distance = {}'.format(distance))
rospy.loginfo('steering_angles = {}'.format(steering_angles))

# rospy.spin()