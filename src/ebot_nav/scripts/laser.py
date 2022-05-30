#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

rospy.init_node('ebot_controller')
sub = rospy.Subscriber('/ebot/laser/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.spin()
velocity_msg.linear.x=0
velocity_msg.angular.z=0
pub.publish(velocity_msg)
velocity_msg = Twist()
rate = rospy.Rate(10) 

def callback(msg):
    if msg.ranges[360]!=0:
        velocity_msg.linear.x=1
        velocity_msg.angular.z=-1
        pub.publish(velocity_msg)
        rate.sleep()
if __name__ == '__main__':
    try:
        #Testing our function
        callback()
    except rospy.ROSInterruptException: pass
 
