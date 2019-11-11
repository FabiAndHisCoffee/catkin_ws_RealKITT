#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

def rosPrint(data):
    rospy.loginfo(data)

# def bridge_opencv():
#     image_pub = rospy.Publisher("quadrotor/videocamera1/camera_info", Image)
#
#     cv2.namedWindow("Image window", 1)
#
#
# image_sub = rospy.Subscriber("quadrotor/videocamera1/image", Image, callback)
#git config --global user.email
#
# def callback(data):
#     bridge = CvBridge()
#     try:n
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


#input: image as greyscale which should be a 2D array
#output: the mean position of all white color 
# (in our task there is only one white cluster)
def get_white_clusters(greyscale_array):
    cnt = 0
    valueX = 0
    valueY = 0
    for y in range(0,greyscale_array.shape[0]-1):
	#rosPrint(x)
        for x in range(0,greyscale_array.shape[1]-1):
	    #rosPrint(y)
	    if(greyscale_array[y,x] == 255):
		valueX += x
		valueY += y
		cnt += 1
    if(cnt != 0):
        valueX /= cnt
        valueY /= cnt
    return [valueX,valueY]


#callback for image subscriber
def get_raw_image_greyscale(image):

    infra1_image = bridge.imgmsg_to_cv2(image, "mono8")
    RECT_COLOR= 0
    BOUNDING_BOX_COLOR = 120

    (rows, cols) = infra1_image.shape
    threshold = 254
    ret, thresh1 = cv2.threshold(infra1_image, threshold, 255, cv2.THRESH_BINARY)
    
    # BLACK RECTANGLES
    cv2.rectangle(thresh1, (0,300), (640, 480), RECT_COLOR, thickness=cv2.FILLED)
    cv2.rectangle(thresh1, (320-50,220), (320+150, 480), RECT_COLOR, thickness=cv2.FILLED)
    cv2.rectangle(thresh1, (0,0), (640, 90), RECT_COLOR, thickness=cv2.FILLED)
	

    # BOUNDING BOXES
    cv2.rectangle(thresh1, (0,0), (320, 140), BOUNDING_BOX_COLOR) #Box1
    cv2.rectangle(thresh1, (322,0), (640, 140), BOUNDING_BOX_COLOR)#Box2
    cv2.rectangle(thresh1, (0,142), (320, 200), BOUNDING_BOX_COLOR)#Box3
    cv2.rectangle(thresh1, (322,142), (640, 200), BOUNDING_BOX_COLOR)#Box4
    cv2.rectangle(thresh1, (0,202), (320, 300), BOUNDING_BOX_COLOR)#Box5
    cv2.rectangle(thresh1, (322,202), (640, 300), BOUNDING_BOX_COLOR)#Box6
    

    offsetX = 320
    offsetY1 = 140
    offsetY2 = 200
    width_heigh = 10

    BB1 = thresh1[0:offsetY1,0:offsetX]
    cluster1 = get_white_clusters(BB1)
    cv2.rectangle(thresh1, (cluster1[0]-width_heigh,cluster1[1]-width_heigh), (cluster1[0]+width_heigh, cluster1[1]+width_heigh), 180)

    BB2 = thresh1[0:offsetY1,offsetX:640]
    cluster2 = get_white_clusters(BB2)
    cv2.rectangle(thresh1, (cluster2[0]-width_heigh+offsetX,cluster2[1]-width_heigh), (cluster2[0]+width_heigh+offsetX, cluster2[1]+width_heigh), 180)

    BB3 = thresh1[offsetY1:offsetY2,0:offsetX]
    cluster3 = get_white_clusters(BB3)
    cv2.rectangle(thresh1, (cluster3[0]-width_heigh,cluster3[1]-width_heigh+offsetY1), (cluster3[0]+width_heigh, cluster3[1]+width_heigh+offsetY1), 180)

    BB4 = thresh1[offsetY1:offsetY2,offsetX+2:640]
    cluster4 = get_white_clusters(BB4)
    cv2.rectangle(thresh1, (cluster4[0]-width_heigh+offsetX,cluster4[1]-width_heigh+offsetY1), (cluster4[0]+width_heigh+offsetX, cluster4[1]+width_heigh+offsetY1), 180)

    BB5 = thresh1[offsetY2:300,0:offsetX]
    cluster5 = get_white_clusters(BB5)
    cv2.rectangle(thresh1, (cluster5[0]-width_heigh,cluster5[1]-width_heigh+offsetY2), (cluster5[0]+width_heigh, cluster5[1]+width_heigh+offsetY2), 180)

    BB6 = thresh1[offsetY2:300,offsetX+2:640]
    cluster6 = get_white_clusters(BB6)
    cv2.rectangle(thresh1, (cluster6[0]-width_heigh+offsetX,cluster6[1]-width_heigh+offsetY2), (cluster6[0]+width_heigh+offsetX, cluster6[1]+width_heigh+offsetY2), 180)

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
