#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

def rosPrint(data):
    rospy.loginfo(data)


distortion_coefficients = np.array([[0],[0],[0],[0],[0]])
intrinsic_parameters = np.array([[0,0,0],[0,0,0],[0,0,1]])
#grayscale_image = None


BOTTOM_LEFT = np.array([0.5,0.2,0])
BOTTOM_RIGHT = np.array([0.5,-0.2,0])
MIDDLE_LEFT = np.array([0.8,0.2,0])
MIDDLE_RIGHT = np.array([0.8,-0.2,0])
TOP_LEFT = np.array([1.1,0.2,0])
TOP_RIGHT = np.array([1.1,-0.2,0])
OBJECT_POINTS = np.array([TOP_LEFT,TOP_RIGHT,MIDDLE_LEFT,MIDDLE_RIGHT,BOTTOM_LEFT,BOTTOM_RIGHT],dtype="float32")

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
# from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
def rotationMatrixToEulerAngles(R) :
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def get_camera_calibration(data):
    global intrinsic_parameters 
    global distortion_coefficients

    f_x = data.K[0]
    f_y = data.K[4]
    c_x = data.K[2]
    c_y = data.K[5]
    
    intrinsic_parameters = np.array([[f_x,0,c_x],[0,f_y,c_y],[0,0,1]],dtype="float32")
    k_1 = data.D[0]
    k_2 = data.D[1]
    t_1 = data.D[2]
    t_2 = data.D[3]
    k_3 = data.D[4]
#    distortion_coefficients = np.array([[k_1],[k_2],[t_1],[t_2],[k_3]])
    distortion_coefficients = np.array([k_1,k_2,t_1,t_2,k_3],dtype="float32")


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
    return np.array([valueX,valueY])


#callback for image subscriber
def get_raw_image_greyscale(image):
    #GLOBALS
    global intrinsic_parameters 
    global distortion_coefficients
    global OBJECT_POINTS
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
    cluster2[0] += offsetX 
    cv2.rectangle(thresh1, (cluster2[0]-width_heigh,cluster2[1]-width_heigh), (cluster2[0]+width_heigh, cluster2[1]+width_heigh), 180)
    
    BB3 = thresh1[offsetY1:offsetY2,0:offsetX]
    cluster3 = get_white_clusters(BB3)
    cluster3[1] += offsetY1
    cv2.rectangle(thresh1, (cluster3[0]-width_heigh,cluster3[1]-width_heigh), (cluster3[0]+width_heigh, cluster3[1]+width_heigh), 180)

    BB4 = thresh1[offsetY1:offsetY2,offsetX+2:640]
    cluster4 = get_white_clusters(BB4)
    cluster4[0] += offsetX
    cluster4[1] += offsetY1
    cv2.rectangle(thresh1, (cluster4[0]-width_heigh,cluster4[1]-width_heigh), (cluster4[0]+width_heigh, cluster4[1]+width_heigh), 180)

    BB5 = thresh1[offsetY2:300,0:offsetX]
    cluster5 = get_white_clusters(BB5)
    cluster5[1] += offsetY2
    cv2.rectangle(thresh1, (cluster5[0]-width_heigh,cluster5[1]-width_heigh), (cluster5[0]+width_heigh, cluster5[1]+width_heigh), 180)

    BB6 = thresh1[offsetY2:300,offsetX+2:640]
    cluster6 = get_white_clusters(BB6)
    cluster6[0] += offsetX
    cluster6[1] += offsetY2
    cv2.rectangle(thresh1, (cluster6[0]-width_heigh,cluster6[1]-width_heigh), (cluster6[0]+width_heigh, cluster6[1]+width_heigh), 180)
    #[TOP_LEFT,TOP_RIGHT,MIDDLE_LEFT,MIDDLE_RIGHT,BOTTOM_LEFT,BOTTOM_RIGHT]
    imagePoints = np.array([cluster1,cluster2,cluster3,cluster4,cluster5,cluster6],dtype="float32")

    #solvePnP


    ret, rvec, tvec = cv2.solvePnP(OBJECT_POINTS,imagePoints,intrinsic_parameters,distortion_coefficients)
    rosPrint("Rotation Vector: ")
    rosPrint(rvec)
    rosPrint("Transpose Vector: ")
    rosPrint(tvec)

    #Rodrigues

    src = rvec

    dst = np.array([[0,0,0],[0,0,0],[0,0,0]],dtype="float32")

    cv2.Rodrigues(src, dst, jacobian=0)

    rosPrint("Rotation Matrix: ")
    rosPrint(dst)
    yaw_pitch_roll = rotationMatrixToEulerAngles(dst)
    
    bottom_row = np.array([0,0,0,1])
    translation_matrix = np.append(dst, tvec, axis=1)
    
    translation_matrix = np.vstack((translation_matrix, bottom_row))

    rosPrint("Translation Matrix: from camera to world ")
    rosPrint(translation_matrix)

    inverse_trans_matrix = np.linalg.inv(translation_matrix)


    rosPrint("yaw_pitch_roll: ")
    rosPrint(yaw_pitch_roll)

    rosPrint("Inverse Translation Matrix: ")
    rosPrint(inverse_trans_matrix)


    rosPrint("inverse_trans_matrix * OBJECT_POINTS[0]")
    rosPrint(np.dot(inverse_trans_matrix,np.append(OBJECT_POINTS[0], [1])))
    rosPrint("-------------------------")
    

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
