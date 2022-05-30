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

    l = [[math.radians(-14.9463958903425),math.radians(3.932293697040383),math.radians(-112.08881385402887),math.radians(108.14933641810856),math.radians(-94.41013942248325),math.radians(74.52323431120624)],
        [math.radians(81.59525534213837),math.radians(39.429493977834106),math.radians(-36.18511726138622),math.radians(-3.271936635497391),math.radians(-2.954252151180251),math.radians(74.59483453426044)]]

    arm_grp.set_joint_value_target(l[0])
    arm_grp.go()
    gripper.set_named_target("close")
    gripper.go()
    arm_grp.set_named_target("home")
    arm_grp.go()
    gripper.set_named_target("open")
    gripper.go()
    
    arm_grp.set_joint_value_target(l[1])
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

