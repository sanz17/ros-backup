#! /usr/bin/env python3

import rospy
import sys
import moveit_commander
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

# The time taken in video and bag file might be different than bag file because they are done on different computers

# For this execution the joint speed is set to 10 and acceleration to 0.5 which can be changed for faster execution
# Angle which bot makes with z-axis is initialised
angle=0
# Array for lidar values is initialised
ranges=[0 for _ in range(3)]
# Message object used is initalised
vel_msg=Twist()


def laser_callback(data):
    global ranges
    ranges=[data.ranges[0],data.ranges[360],data.ranges[-1]]

rospy.init_node('pickAndPlace', anonymous=True) 
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
rate=rospy.Rate(20)

def set_vel(angular_vel,linear_x):
    vel_msg.angular.z=angular_vel
    vel_msg.linear.x=linear_x
    pub.publish(vel_msg)

def move(lv):
    pv=[0 for _ in range(5)]
    lc=0
    ctr=1
    while sum(pv)/5<3:
        lc=lc%5
        if ranges[0]>2:
            print(ctr,ranges[0])
            ctr+=1
        pv[lc]=ranges[0]
        lc+=1
        set_vel(0,lv)
        rate.sleep()
    rospy.sleep(0.5)
    brake(lv,0)

def brake(cv,tv):
    v=(cv-tv)/10
    while cv>tv:
        cv-=v
        print(cv)
        set_vel(0,cv)
        rate.sleep()

def radians(l):
    t=[]
    for i in l:
        t.append(math.radians(i))
    return t


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    arm_grp=moveit_commander.MoveGroupCommander("arm")

    # l= radians([90,-20,-70,90,0,0])
    # l2=radians([-90,-20,-70,90,0,0])
    l2=radians([-90,-45,-45,90,0,0])
    l=radians([90,-45,-45,90,0,0])
    
    arm_grp.set_joint_value_target(l2)
    arm_grp.go()
    
    # brake(0,0.5)

    for i in range(80):
        set_vel(0,0.5)
        # print("f",ranges)
        rate.sleep()

    move(0.5)

    arm_grp.set_joint_value_target([0 for _ in range(6)])
    arm_grp.go()

    arm_grp.set_joint_value_target(l)
    arm_grp.go()

    brake(0,-0.5)
    for i in range(20):
        set_vel(0,-0.5)
        rate.sleep()
    move(-0.5)

    set_vel(0,0)
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass