/**
 * @file wireless.cpp
 * @author naga
 * @brief 
 * @version 0.1
 * @date 2022-09-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <ros/ros.h>
#include <libserial/SerialStream.h>

#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt8MultiArray.h>

using namespace std;
using namespace LibSerial;

void serial_set(SerialStream& serial);
bool read(SerialStream& serial, vector<uint8_t>& send_data);
bool check_start(SerialStream&serial);
bool check_sum(SerialStream& serial, char data[]);


int main(int argc, char** argv){
	ros::init(argc, argv, "wireless");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("control_data", 10);

	std_msgs::UInt8MultiArray msg;
	msg.data.clear();

	SerialStream serial;
	serial_set(serial);

	ros::Rate rate(100);

	while(ros::ok()){
		vector<uint8_t> send_data(8);
		if(!read(serial, send_data)) continue;
		msg.data.swap(send_data);

		pub.publish(msg);
		rate.sleep();
	}

}

void serial_set(SerialStream& serial){
	serial.Open("/dev/ttyUSB0");
	serial.SetBaudRate(BaudRate::BAUD_115200);
	serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
	serial.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
	serial.SetParity(Parity::PARITY_NONE);
	serial.SetStopBits(StopBits::STOP_BITS_1);
}

bool read(SerialStream& serial, vector<uint8_t>& send_data){
	char str[8];

	if(check_start(serial)){
		serial.read(str, 8);
	} else {
		ROS_WARN("serial could not get header data");
		return false;
	}

	if(!check_sum(serial, str)){
		ROS_WARN("check sum is not good");
		return false;
	}

	for(auto i = 0LU; i < send_data.size(); i++){
		send_data[i] = str[i];
	}
	return true;
}

constexpr char start_byte[] = "aa";//need to decide value

bool check_start(SerialStream& serial){
	char check_start = 0;



	while(serial.good()){
		serial>>check_start;
		if(check_start == start_byte[0]){
			serial>>check_start;
			if(check_start == start_byte[1]){
				return true;
			}
		}
	}

	return false;
}

bool check_sum(SerialStream& serial, char data[]){
	string str(data);
	uint8_t sum = 0, recv = 0;
	
	for(int i = 0LU; i < strlen(start_byte); i++){
		sum += start_byte[i];
	}

	for(auto i: str){
		sum += i;
	}

	serial>>recv;

	if(recv == sum) return true;
	return false;
}