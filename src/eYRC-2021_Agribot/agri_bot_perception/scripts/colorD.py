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
    center_x = 320.5
    center_y = 240.5
    tomato_dimension = 0.1
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        # frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        upper_red=np.array([10,255,255])
        lower_red=np.array([0,60,100])

        mask=cv.inRange(hsv,lower_red,upper_red)
        result=cv.bitwise_and(frame,frame,mask=mask)

        gray=cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        edged = cv.Canny(gray, 30, 200)
        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) 
        for i in contours:
            M = cv.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # cv.drawContours(frame, contours, -1, (0, 255, 0), 3)
                cv.drawContours(frame, [i], -1, (0, 255, 0), 2)
                print(cx,cy)
                cv.circle(frame,(cx, cy), 1, (0, 0, 255), -1)
                cv.putText(frame, "center", (cx - 20, cy - 20),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            # print(f"x: {cx} y: {cy}")
        cv.imshow("Frame",frame)
        cv.imshow('Canny Edges After Contouring', edged)
        cv.imshow("masked",result)
        cv.waitKey(1)
    #     gray = cv.cvtColor(mask, cv.COLOR_HSV2BGR)
    #     gray=cv.cvtColor(gray,cv.COLORBGR2RAY)
  
    #     # Find Canny edges
    #     edged = cv.Canny(gray, 30, 200)
    #     cv.waitKey(0)
  
    #     # Finding Contours
    #     # Use a copy of the image e.g. edged.copy()
    #     # since findContours alters the image
    #     contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)  
    #     cv.imshow('Canny Edges After Contouring', edged)
  
    #     print("Number of Contours found = " + str(len(contours)))
  
    # # Draw all contours
    # # -1 signifies drawing all contours
    #     cv.drawContours(frame, contours, -1, (0, 255, 0), 3)
  
    #     cv.imshow('Contours', frame)
        # cv.destroyAllWindows()
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
