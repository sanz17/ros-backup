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
focal_length = 554.387
center_x = 320.5
center_y = 240.5
br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()

def dist(a,b):
    sum=0
    for i in range(3):
        sum+=(a[i]-b[i])**2
    return sum**0.5

def get_contour_center(c):
    M=cv2.moments(c)
    cx=-1
    cy=-1
    if M['m00']!=0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx,cy

def get_distance(c):
    global depth_image
    sum=0
    for i in c:
        sum+=depth_image[i[0][1],i[0][0]]
    return sum/len(c)

def isTooclose(l):
    threshold=1.5
    j=0
    while j<len(l):
        i=j+1
        while i<len(l):
            if dist(l[j],l[i])<=threshold:
                temp=[]
                for k in range(len(l[j])):
                    temp.append((l[j][k]+l[i][k])/2)
                # x=(l[j][0]+l[i][0])/2
                # cy=(l[j][1]+l[i][1])/2
                # w=l[j][2]+l[i][2]
                l[j]=temp
                l.pop(i)
            else:
                i+=1
        j+=1

def image_callback(data):
    global bridge,br,t
    rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")
    if 1==1:
        hsv=cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV)
        upper=(10,255,255)
        lower=(0,60,100)

        mask=cv2.inRange(hsv,lower,upper)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        l=[]
        count=1
        for c in contours:
            cx,cy = get_contour_center(c)
            cv2.drawContours(rgb_image,[c],-1,(255,255,255),1)
            # cv2.ellipse(rgb_image,cv2.fitEllipse(c),(0,255,0),1)
            distance = get_distance(c)
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            world_x = (cx - center_x)/focal_length*distance
            world_y = (cy - center_y)/focal_length*distance
            world_z = distance
            l.append([world_x,world_y,world_z,cx,cy])
            # l.append([cx,cy,distance])
            l.sort(key = lambda x: x[2])

        count=0
        less = len(l) if len(l)<3 else 3
        for i in range(less):
            temp = l[i]
            cv2.putText(rgb_image, "obj"+str(count+1), (temp[3]- 20, temp[4] - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "camera_rgb_frame2"
            t.child_frame_id = "obj"+str(count%3+1)
            count+=1
            t.transform.translation.x = temp[0]
            t.transform.translation.y = temp[1]
            t.transform.translation.z = temp[2]
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
            br.sendTransform(t)
        
    cv2.imshow("RGB Image",rgb_image)
    cv2.waitKey(1)


def depth_callback(data):
    global depth_image
    depth_image=bridge.imgmsg_to_cv2(data,"32FC1")


def main(args):
    rospy.init_node('tomato_detector', anonymous=True)
    # rospy.Subscriber('/odom', Odometry, odom_callback)

    depth_sub=rospy.Subscriber("/camera/depth/image_raw2", Image, depth_callback)
    img_sub=rospy.Subscriber("/camera/color/image_raw2", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
