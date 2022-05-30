#!/usr/bin/env python3
import rospy
from math import pi
from geometry_msgs.msg import Twist
from rospy.timer import Rate

def move():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    print("Let's move your robot")
    speed=1
    vel_msg.linear.x=speed
    vel_msg.angular.z = -1
    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        rate=rospy.Rate(10)
        d = 0

        #Loop to move the turtle in an specified distance
        while d<2*pi+0.05:
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            d=(rospy.Time.now().to_sec()-t0)*speed
            rospy.loginfo(d)
            rate.sleep()
        vel_msg.angular.z *= -1
        while d<4*pi:
            velocity_publisher.publish(vel_msg)
            d=(rospy.Time.now().to_sec()-t0)*speed
            rospy.loginfo(d)
            rate.sleep()

        #After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        exit()

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
