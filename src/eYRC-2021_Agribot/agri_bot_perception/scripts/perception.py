#!/usr/bin/env python3
import cv2 
import numpy as np
from numpy.core.defchararray import lower
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from nav_msgs.msg import Odometry

bridge = CvBridge()
depth_image=None

def isTooclose(l):
    threshold=0.05
    j=0
    while j<len(l):
        i=j+1
        while i<len(l):
            print(l[j],l[i],dist(l[j],l[i]))
            if dist(l[j],l[i])<=threshold:
                cx=(l[j][0]+l[i][0])/2
                cy=(l[j][1]+l[i][1])/2
                w=l[j][2]+l[i][2]
                l[j]=[cx,cy,w]
                l.pop(i)
            else:
                i+=1
        j+=1

def get_contour_center(c):
    x,y,w,h = cv2.boundingRect(c)
    M=cv2.moments(c)
    l=[]
    cx=-1
    cy=-1
    if M['m00']!=0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        l.append([cx,cy,w])
    isTooclose(l)
    return cx,cy


def get_distance(c):
    global depth_image
    sum=0
    for i in c:
        # print(depth_image[i[0][0],i[0][1]])
        sum+=depth_image[i[0][1],i[0][0]]
        # print(sum)
    return sum/len(c)

def image_callback(data):
    global bridge
    global count
    rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")
    if 1==1:
        hsv=cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV)
        upper=(10,255,255)
        lower=(0,60,100)

        mask=cv2.inRange(hsv,lower,upper)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        print(len(contours))
        
        count=0
        for c in contours:
            count+=1
            ((x,y),rad)=cv2.minEnclosingCircle(c)
            cx,cy = get_contour_center(c)
            print(x,y)
            print(cx,cy)
            print(get_distance(c))
            print("----------------------------------")
            cv2.drawContours(rgb_image,[c],-1,(255,255,255),1)
            cv2.putText(rgb_image, "obj"+str(count), (cx,cy),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            # cv2.circle(rgb_image,(cx,cy),int(rad),(0,0,0),1)
            cv2.ellipse(rgb_image,cv2.fitEllipse(c),(0,255,0),1)
            
        
    cv2.imshow("RGB Image",rgb_image)
    cv2.waitKey(1)


def depth_callback(data):
    global depth_image
    depth_image=bridge.imgmsg_to_cv2(data,"32FC1")
    pass

def depth_callback2(data):
    frame=bridge.imgmsg_to_cv2(data,'32FC1')
    # print(frame)
    cv2.imshow("grey",frame)
    
    cv2.waitKey(1)


def main(args):
    rospy.init_node('tomato_detector', anonymous=True)
    # rospy.Subscriber('/odom', Odometry, odom_callback)
    depth_sub=rospy.Subscriber("/camera/depth/image_raw2", Image, depth_callback)
    # depth_sub2=rospy.Subscriber("/camera/depth/image_raw2", Image, depth_callback2)
    img_sub=rospy.Subscriber("/camera/color/image_raw2", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
