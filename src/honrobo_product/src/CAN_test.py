#!/usr/bin/env python3

import rospy
from my_msgs.msg import can_msg

def callback(msg):
    if msg.id == 0x10:
	    rospy.loginfo(msg)
	
def param_init():
	rospy.init_node('CAN', anonymous=True)

def main():
	param_init()
	pub = rospy.Publisher('CAN_RX', can_msg, queue_size=8)
	sub = rospy.Subscriber('CAN_TX', can_msg, callback)

	rate = rospy.Rate(500)

	while not rospy.is_shutdown():
		rate.sleep()
		

main()