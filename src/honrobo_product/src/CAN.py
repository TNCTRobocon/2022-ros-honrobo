#!/usr/bin/env python3

import rospy
from my_msgs.msg import can_msg
import can
import os

try:
	os.system('sudo ip link set can0 type can bitrate 500000')
	os.system('sudo ifconfig can0 up')
	can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')
except OSError:
	rospy.loginfo('error OS')
	exit()

def callback(msg):
	rospy.loginfo(msg)
	can_tx_message = can.Message(arbitration_id=msg.id, data=msg.data, extended_id=False)
	can0.send(can_tx_message)
	
def param_init():
	rospy.init_node('CAN', anonymous=True)

def main():
	param_init()
	pub = rospy.Publisher('CAN_RX', can_msg, queue_size=8)
	sub = rospy.Subscriber('CAN_TX', can_msg, callback)

	rate = rospy.Rate(500) #control rate is 500Hz

	while not rospy.is_shutdown():
		CAN_msg = can0.recv(0.01)
		if CAN_msg is not None and 0x00 <= CAN_msg.id <= 0x0f:
			msg = can_msg()
			msg.data = CAN_msg.data.copy()
			msg.id = CAN_msg.arbitration_id
			pub.publish(msg)
		
		rate.sleep()



try:
	main()
except rospy.ROSInterruptException: pass