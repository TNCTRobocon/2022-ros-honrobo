/**
 * @file data_process.cpp
 * @author naga
 * @brief data process
 *        data convert and relay node
 * @version 0.1
 * @date 2022-09-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <my_msgs/can.h>

my_msgs::can can_msg;
void can_rx_callback(const my_msgs::can& CAN_RX){
	can_msg = CAN_RX;
}


void controller_callback(const std_msgs::Int8MultiArray &f){
    
}

int main(int argc, char**argv){
	ros::init(argc, argv, "data_process");
	ros::NodeHandle nh;
	ros::Publisher can_tx = nh.advertise<my_msgs::can>("CAN_TX", 10);
	ros::Subscriber can_rx = nh.subscribe("CAN_RX", 10, can_rx_callback);
	ros::Subscriber controller_input = nh.subscribe("control_data", 10, controller_callback);
	// need more?

	ros::Rate rate(10);

	while(ros::ok()){
		//TODO
		
		rate.sleep();
	}
}
