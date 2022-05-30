#!/usr/bin/env python3

# Importing the required modules
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi

# Angle which bot makes with z-axis is initialised
angle=0
# Array for lidar values is initialised

ranges=[0 for _ in range(3)]
# Message object used is initalised
vel_msg=Twist()

# Instead of using the template given in mannual we chose this because
# Using all values would extra for the task at hand
def laser_callback(data):
    global ranges
    ranges=[data.ranges[0],data.ranges[360],data.ranges[-1]]

# From odometery we only take the angle it makes with z-axis
# As the coordinates are not required for the task at hand
def odom_callback(data):
    global angle
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    angle = euler_from_quaternion([x,y,z,w])[2]

# Initializing the node
rospy.init_node('ebot_controller')

# Initializing the publisher and subscribers
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
rospy.Subscriber('/odom', Odometry, odom_callback)

# Setting the rate
rate=rospy.Rate(10)

# Function to publish the velocity
# Angular velocity is in z-direction 
# Linear velocity is in x-direction
def set_vel(angular_vel,linear_x):
    vel_msg.angular.z=angular_vel
    vel_msg.linear.x=linear_x
    pub.publish(vel_msg)



# Function to turn the robot
# Target angle is the angle at which you want to stop 
# Angular velocity is in z-direction 
# Linear velocity is in x-direction
def turn(target_angle,angular_vel,linear_vel):
    
    # Set initial velocities to 0 just in case the values are non-zero
    # to avoid unecessary movement
    set_vel(0,0)
    print(angle)
    # If else to differentiate wrt to clockwise or anticlockwise rotation
    if angular_vel>0:
        while angle<=target_angle:
            set_vel(angular_vel,linear_vel)
            rate.sleep()
    else:
        while angle>=target_angle:
            set_vel(angular_vel,linear_vel)
            rate.sleep()
    
    # Setting velocity as negative of current values 
    # Because even after leaving the while loop there is change in angle
    # This can be due to conservation of momentum(maybe)
    set_vel(-angular_vel,-linear_vel)

# Function to move the robot in straight line
# Sign is to differentiate between 2 differnt kinds of straight motions
def straight(sign):
    # Set initial velocities to 0 just in case the values are non-zero
    # to avoid unecessary movement
    set_vel(0,0)
    print(angle)
    # This is for when we move in the lanes
    # Here due to gaps in the pots we cannot check only the current value of lidar
    # Instead we take an average of n values 
    # n should be greater than 2(Trial and Error)
    # It should not be more than 5-6 as it would move the robot too far away from 
    # the trough increasing distance
    if sign:
        # Array to store most recent 3 values of lidar reading
        prevVal=[0 for _ in range(3)]
        loopCount=0

        # Here 6.5 is taken because sum of the corner lidar ranges first and last elements 
        # comes to vary between 6.6 and 6.7 
        while sum(prevVal)/3<6.5:
            
            ang_vel=0.1
            lin_vel_x=1
            
            # This statement is to correct the bot's 
            # Because if the angle is not close to + or - pi/2 the bot may collide with the trough
            if angle>0:
                # No need to change set angular velocity
                if angle>=1.569 and angle<=1.573:
                    ang_vel=0
                # Decrease angular velocity
                elif angle>1.573:
                    ang_vel=-0.1
                # Increase angular velocity
                else:
                    ang_vel=0.1
            else:
                # No need to change set angular velocity
                if angle>=-1.573 and angle<=-1.569:
                    ang_vel=0
                # Decrease angular velocity
                elif angle>-1.573:
                    ang_vel=-0.1
                # Increase angular velocity
                else:
                    ang_vel=0.1

            set_vel(ang_vel,lin_vel_x)
            # This allows array to have the most recent n values of lidar reading
            prevVal[loopCount%3]=ranges[0]+ranges[-1]
            loopCount+=1
            rate.sleep()
    # This is used when the bot is crossing the left trough
    # Here we are moving closer to the green wall which acts as obstacle
    # Since there are no abnormalities in lidar reading we dont need to 
    # use an array like above
    else:
        while ranges[1]>2.5:
            if angle<0:
                set_vel(-0.1,1)
            else:
                set_vel(0,1)
            print(angle)
            rate.sleep()
            
    # To stop the bot from moving after it has done the task
    set_vel(0,0)

def main():
    while not rospy.is_shutdown():
        # The value of target_angle should theoretically be some multiple of pi
        # But to do that practically the rate should be inf 
        # so we need to add/subtract some value found by trial and error
        # For the first turn
        turn(pi-0.05,0.45,0)
        
        # Move Straight till the bot croses the left trough
        straight(False)
        
        # Turn right into the left lane
        turn(pi/2+0.04,-1,0.3)
        
        # Move straight in the left lane
        straight(True)
        
        # Take a U-turn to get into the middle lane from the left lane
        turn(-pi/2+0.04,-1,0.3)
        
        # Move straight in the middle lane 
        straight(True)
        
        
        # Take a U-turn to get into the right lane from the middle lane
        turn(pi/2-0.05,1,0.38)
        
        # Move straight in the right lane 
        straight(True)

        # The traversal is finished and so the program terminates
        exit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
    