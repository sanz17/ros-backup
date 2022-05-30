#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf import listener
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import tf

cv_image = None

def dist(a,b):
    return ((a[0]-b[0])**2+(a[1]-b[1])**2)**.5

def isTooclose(l):
    threshold=1.5
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
                



def callback(data):
    global cv_image
    focal_length = 554.387
    center_x = 320.5
    center_y = 240.5
    aruco_dimension =0.075
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        upper_red=np.array([10,255,255])
        lower_red=np.array([0,60,100])

        mask=cv.inRange(hsv,lower_red,upper_red)
        result=cv.bitwise_and(frame,frame,mask=mask)

        gray=cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        edged = cv.Canny(gray, 30, 200)
        count=0
        l=[]
        contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) 
        for i in contours:
            
            x,y,w,h = cv.boundingRect(i)
            M = cv.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv.drawContours(frame, [i], -1, (0, 255, 0), 2)
                # print(cx,cy)
                cv.circle(frame,(cx, cy), 1, (0, 0, 255), -1)
                cv.putText(frame, "obj"+str(count), (cx - 20, cy - 20),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                l.append([cx,cy,w])
        print(len(l),end=' ')
        isTooclose(l)
        print(len(l))
        for j in l:
                count+=1
                distance = (focal_length*aruco_dimension)/j[2]
                print(distance)
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                # transforming pixel coordinates to world coordinates
                world_x = (j[0] - center_x)/focal_length*distance
                world_y = (j[1] - center_y)/focal_length*distance
                world_z = distance
                
                # broadcasting TF for each tomato
                br = tf2_ros.TransformBroadcaster()
                listener=tf.TransformListener()
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "camera_rgb_frame2"
                t.child_frame_id = "obj"+str(count)

                # putting world coordinates coordinates as viewed for sjcam frame
                t.transform.translation.x = world_x
                t.transform.translation.y = world_y
                t.transform.translation.z = world_z
                # not extracting any orientation thus orientation is (0, 0, 0)
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                # print(listener.transformPose("odom",t))
                br.sendTransform(t)
                # print(world_x,world_y,world_z,q)
                
           
        cv.imshow("Frame",frame)
        cv.imshow('Canny Edges After Contouring', edged)
        cv.imshow("masked",result)
        cv.waitKey(1)              
    except CvBridgeError as e:
        print(e)



def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw2", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
