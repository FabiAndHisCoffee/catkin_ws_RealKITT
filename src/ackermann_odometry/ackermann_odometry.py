#!/usr/bin/env python

# --- imports ---
import rospy
from autominy_msgs.msg import Speed, SteeringAngle
from math import cos,sin,tan
from nav_msgs import Odometry
import random

CAR_AXLE_DISTANCE = 0.27 #meters

class Ackermann(object):
    def __init__(self):
        self.steeringSubscriber = rospy.Subscriber('/sensors/steering', SteeringAngle, self.steering_angle_update)
        self.speedSubscriber = rospy.Subscriber('/sensors/steering', Speed, self.speed_update)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.speed = 0
        self.steering = 0
        self.oldTime = rospy.Time.now()
        self.newTime = rospy.Time.now()

    def steering_angle_update(self,data):
        self.steering = data.value

    def speed_update(self,data):
        self.speed = data.value

    def calculateAckermann(self):
        odometry_msg = Odometry()
        odometry_msg.header.frame_id = "map"
        odometry_msg.child_frame_id = "base_link"
        #TMP SOLUTION
        theta = random.random()
        timeDelta_sec = self.oldTime.to_sec() - rospy.Time.now().to_sec()
        self.oldTime = rospy.Time.now()

        x_point = self.speed * cos(self.steering)
        y_point = self.speed * sin(self.steering)
        theta_point = (self.speed/CAR_AXLE_DISTANCE)*tan(self.steering)
        
        self.x = self.x + timeDelta_sec * x_point
        self.y = self.y + timeDelta_sec * y_point
        self.theta = self.theta + timeDelta_sec * theta_point

        odometry_msg.pose.pose.position.x = self.x
        odometry_msg.pose.pose.position.y = self.y
        odometry_msg.pose.pose.position.z = 0

        odometry_msg.twist.pose.position.x = self.x
        odometry_msg.twist.pose.position.y = self.y
        odometry_msg.twist.pose.position.z = 0

        return odometry_msg

rospy.init_node("ackermann_odometry")
ackermann = Ackermann()
rate = rospy.Rate(100)

ackermann_pub = rospy.Publisher('odometry/ackermann',Odometry,queue_size = 10)

while not rospy.is_shutdown():
    ackermann_pub.publish(ackermann.calculateAckermann())