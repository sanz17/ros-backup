#!usr/bin/env python3

import rospy

def hello()
	rospy.init_node('node_hello_ros',anonymous=True)
	rospy.loginfo("HELLO WORLD")
	rospy.spin()

if __name__='__main__':
	try:
	    hello()
	except rospy.ROSInterruptException:
	    pass
	    
