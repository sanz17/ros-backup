#!/usr/bin/env python3

"""
Description: Obtaining TF of a ArUco marker using single sjcam camera
Algorithm: 
	Input: ROS topic for the RGB image
	Process: 
        - Subscribe to the image topic
		- Convert ROS format to OpenCV format
		- Detecting ArUco marker along with its ID
		- Applying perspective projection to calculate focal length
		- Extracting depth for each aurco marker
		- Broadcasting TF of each ArUco marker with naming convention as aruco1 for ID1, aruco2 for ID2 and so on.
	Output: TF of ArUco marker with respect to ebot_base
"""

import cv2 as cv
import numpy as np
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions

cv_image = None

def callback(data):
    # Initializing variables
    
    global cv_image
    focal_length = 476.70308
    center_x = 400.5
    center_y = 400.5
    aruco_dimension = 0.1
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        cv.imshow("output",frame)
        print(frame)
        
        
        
        
        
    except CvBridgeError as e:
        print(e)



def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    # subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
    rospy.Subscriber("/camera/color/image_raw2", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
