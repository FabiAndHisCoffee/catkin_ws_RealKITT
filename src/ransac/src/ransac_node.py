import ransac

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

rospy.init_node('RANSAC_RealKITT')

bridge = CvBridge()
# cv.namedWindow("Image window", cv.WINDOW_AUTOSIZE)
#callback for image subscriber
def get_thresh_image_greyscale(image):
    infra1_image = bridge.imgmsg_to_cv2(image, "mono8")
    lines = find_lines(processed)
    image_rgb = draw_lines_into_image(lines, infra1_image)
    infra1_thresh = bridge.cv2_to_imgmsg(image, "rgb8")
    publisher_lines.publish(image_rgb)
    cv2.imshow("RANSAC_lines", image_rgb)
    cv2.waitKey(3)
        

rospy.init_node('camera_calibration')
rospy.Subscriber('/images/infra1_thresh', Image, get_thresh_image_greyscale)
publisher_lines = rospy.Publisher('/images/infra_lines', Image, queue_size = 10)
 
rospy.spin()