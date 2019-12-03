
import rospy
import ransac

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import math


bridge = CvBridge()

#callback for image subscriber
def get_raw_image_greyscale(image):
    infra1_image = bridge.imgmsg_to_cv2(image, "mono8")
    cv2.imshow("infra1_image", infra1_image)
    processed = pre_processing(image_gray)
    infra1_thresh = bridge.cv2_to_imgmsg(image, "mono8")
    publisher_thresh.publish(infra1_thresh)
        
rospy.init_node('RANSAC_preprocessing_RealKITT')
        
rospy.Subscriber('/sensors/camera/infra1/image_rect_raw', Image, get_raw_image_greyscale)
publisher_thresh = rospy.Publisher('/images/infra1_thresh', Image, queue_size = 10)
 
rospy.spin()