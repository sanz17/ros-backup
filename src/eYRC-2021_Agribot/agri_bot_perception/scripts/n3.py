#! /usr/bin/env python3

import rospy
import sys
import moveit_commander
import math

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pickAndPlace', anonymous=True)

    robot=moveit_commander.RobotCommander()

    arm_grp=moveit_commander.MoveGroupCommander("arm")
    gripper=moveit_commander.MoveGroupCommander("gripper")

    l = [[math.radians(90),math.radians(-45),math.radians(-45),math.radians(90),math.radians(0),math.radians(0)],
        [math.radians(81.0462809098735),math.radians(42.809468227062716),math.radians(-39.874252486951605),math.radians(-0.23456464938979463),math.radians(-6.632704530257954),math.radians(-0.3209795771434106)],
        [math.radians(-25.47383766683283),math.radians(-31.140987381313757),math.radians(-42.87145516073542),math.radians(-104.37613032455783),math.radians(-40.04231350673591),math.radians(178.40643993800404)]]

    arm_grp.set_joint_value_target(l[0])
    arm_grp.go()
    arm_grp.set_joint_value_target(l[1])
    arm_grp.go()
    gripper.set_named_target("close")
    gripper.go()
    arm_grp.set_named_target("home")
    arm_grp.go()
    gripper.set_named_target("open")
    gripper.go()
    
    arm_grp.set_joint_value_target(l[0])
    arm_grp.go()
    arm_grp.set_joint_value_target(l[2])
    arm_grp.go()
    gripper.set_named_target("close")
    gripper.go()
    arm_grp.set_named_target("home")
    arm_grp.go()
    gripper.set_named_target("open")
    gripper.go()

    moveit_commander.roscpp_shutdown()
if __name__ == '__main__':
    main()

