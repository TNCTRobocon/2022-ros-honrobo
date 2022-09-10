#!/usr/bin/env python3

import rospy
from my_msgs.msg import custom
import can
import os

can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')

def callback(msg):
	can_tx_message = can.Message(arbitration_id=msg.id, data=msg.data, extended_id=False)
	can0.send(can_tx_message)
	
def param_init():
	rospy.init_node('CAN', anonymous=True)
	os.system('sudo ip link set can0 type can bitrate 500000')
	os.system('sudo ifconfig can0 up')

def main():
	param_init()
	pub = rospy.Publisher('CAN_RX', custom, queue_size=8)
	sub = rospy.Subscriber('CAN_TX', custom, callback)

	while not rospy.is_shutdown():
		CAN_msg = can0.recv(0.05)
		if CAN_msg is not None:
			msg = custom()

			for n in range(8):
				msg.data[n] = CAN_msg[n]
			msg.id = CAN_msg.arbitration_id
			pub.publish(msg)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass