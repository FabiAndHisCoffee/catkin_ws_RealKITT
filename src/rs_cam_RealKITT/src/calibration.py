#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


# def bridge_opencv():
#     image_pub = rospy.Publisher("quadrotor/videocamera1/camera_info", Image)
#
#     cv2.namedWindow("Image window", 1)
#
#
# image_sub = rospy.Subscriber("quadrotor/videocamera1/image", Image, callback)
#
#
# def callback(data):
#     bridge = CvBridge()
#     try:
#         cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#     except CvBridgeError, e:
#         print
#         e
#     (rows, cols, channels) = cv_image.shape
#     if cols > 60 and rows > 60:
#         cv2.circle(cv_image, (50, 50), 10, 255)
#         x = 5
#         puba.publish(x)
#
#
# cv2.imshow("Image window", cv_image)
# cv2.waitKey(3)

distortion_coefficients = None
intrinsic_parameters = None
# f_x = distortion_coefficients[0]
# f_y = distortion_coefficients[4]
# c_x = distortion_coefficients[2]
# c_y = distortion_coefficients[5]
#
# k_1 = intrinsic_parameters[0]
# k_2 = intrinsic_parameters[1]
# t_1 = intrinsic_parameters[2]
# t_2 = intrinsic_parameters[3]
# k_3 = intrinsic_parameters[4]
grayscale_image = None

def get_camera_calibration(data):
    distortion_coefficients = data.D
    intrinsic_parameters = data.K

    # print(distortion_coefficients)
    # print(intrinsic_parameters)
    f_x = intrinsic_parameters[0]
    f_y = intrinsic_parameters[4]
    c_x = intrinsic_parameters[2]
    c_y = intrinsic_parameters[5]

    k_1 = distortion_coefficients[0]
    k_2 = distortion_coefficients[1]
    t_1 = distortion_coefficients[2]
    t_2 = distortion_coefficients[3]
    k_3 = distortion_coefficients[4]
    # rospy.signal_shotdown()

def get_raw_image_greyscale(image):

    infra1_image = bridge.imgmsg_to_cv2(image, "mono8")
    RECT_COLOR= 0

    (rows, cols) = infra1_image.shape
    threshold = 254
    ret, thresh1 = cv2.threshold(infra1_image, threshold, 255, cv2.THRESH_BINARY)
    cv2.rectangle(thresh1, (0,300), (640, 480), RECT_COLOR, thickness=cv2.FILLED)
    cv2.rectangle(thresh1, (320-50,220), (320+150, 480), RECT_COLOR, thickness=cv2.FILLED)
    cv2.rectangle(thresh1, (0,0), (640, 90), RECT_COLOR, thickness=cv2.FILLED)

    # cv.imwrite("/home/gh/FUB/WS1920/Gray_Image.jpg", cv_image);
    cv2.imshow("infra1_image", infra1_image)
    cv2.imshow("thresh with {}".format(threshold), thresh1)
    cv2.waitKey(3)


bridge = CvBridge()
# cv.namedWindow("Image window", cv.WINDOW_AUTOSIZE)

rospy.init_node('camera_calibration')
rospy.Subscriber('/sensors/camera/infra1/camera_info', CameraInfo, get_camera_calibration)
rospy.Subscriber('/sensors/camera/infra1/image_rect_raw', Image, get_raw_image_greyscale)

rospy.spin()
