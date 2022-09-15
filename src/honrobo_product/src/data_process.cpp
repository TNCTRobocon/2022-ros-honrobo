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
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <my_msgs/can_msg.h>

#define TEST // need to remove

#ifdef TEST
#define TEST 1
#endif //TEST

#ifdef PRODUCT
#define PRODUCT 1
#endif // PRODUCT

bool is_emergency = false;

// structures
struct input_status{
	bool neatly = false,
		 sense_cycle_update = false,
		 tire_return        = false,
		 robot_return       = false,
		 shot_and_reload    = false,
		 change_cartridge   = false,
		 return_to_start    = false,
		 shot_cycle_update  = false;

	uint8_t spin = 0,
			shot_angle = 0,
			x_velo = 0,
			y_velo = 0;
};

class cycle{
	uint8_t cycle_val = 0;

public:
	enum MODE{
		HI_SENSE,
		MIDDLE_SENSE,
		LOW_SENSE
	};

	cycle(){}
	~cycle(){}

	void operator++(){
		cycle_val++;
		cycle_val = (cycle_val == 3)? 0: cycle_val;
	}

	uint8_t get_cycle_state(){ return cycle_val; }

};

//prototype decl
uint32_t float_to_int(float f);
void convert_32_8(uint32_t i, uint8_t*buf);

//callback functions
bool is_rx_can = false;
my_msgs::can_msg can_msg;
void can_rx_callback(const my_msgs::can_msg& CAN_RX){
	is_rx_can = true;
	if(can_msg.data.size() != 8) can_msg.data.resize(8);
	std::copy(CAN_RX.data.begin(), CAN_RX.data.end(), can_msg.data.begin());
			
#if TEST
	ROS_INFO("receive data: %d, %d, %d, %d, %d, %d, %d, %d", can_msg.data[0], can_msg.data[1], can_msg.data[2], can_msg.data[3], can_msg.data[4], can_msg.data[5], can_msg.data[6], can_msg.data[7]);
#endif // TEST
}

bool is_new_value = false;
std_msgs::UInt8MultiArray rx_array;
void controller_callback(const std_msgs::UInt8MultiArray &f){
	if(rx_array.data.size() != 8) rx_array.data.resize(8);
	std::copy(f.data.begin(), f.data.end(), rx_array.data.begin());
	is_new_value = true;
}

#if TEST

#endif //TEST

int main(int argc, char**argv){
	ros::init(argc, argv, "data_process");
	ros::NodeHandle nh;
	ros::Publisher can_tx = nh.advertise<my_msgs::can_msg>("CAN_TX", 10);
	ros::Subscriber can_rx = nh.subscribe("CAN_RX", 10, can_rx_callback);
	ros::Publisher controller_output = nh.advertise<std_msgs::Float32MultiArray>("lidar data", 10);
	ros::Subscriber controller_input = nh.subscribe("control_data", 10, controller_callback);
	// need more?

	cycle sense;
	cycle shot_spd;

	ros::Rate rate(10);

	bool reload_lock = false;

	while(ros::ok()){
		input_status state;
		bool is_update_value = false;

		if(is_new_value){
			state.neatly             = can_msg.data[0] & 0b10000000;
			state.sense_cycle_update = can_msg.data[0] & 0b01000000;
			state.tire_return        = can_msg.data[0] & 0b00100000;
			state.robot_return       = can_msg.data[0] & 0b00010000;
			state.shot_and_reload    = can_msg.data[0] & 0b00001000;
			state.change_cartridge   = can_msg.data[0] & 0b00000100;
			
			state.return_to_start    = can_msg.data[1] & 0x00000100;

			state.shot_cycle_update  = can_msg.data[6]>50? true: false;

			state.spin       = can_msg.data[2];
			state.shot_angle = can_msg.data[3];
			state.x_velo     = can_msg.data[4];
			state.y_velo     = can_msg.data[5];
			

			is_new_value = false;
			is_update_value = true;
		}

		if(state.neatly){
			//TODO
		}

		if(state.sense_cycle_update){
			++sense;
		}

		if(state.tire_return){
			my_msgs::can_msg msg;
			msg.id = 0x12;
			msg.data.resize(8);
			can_tx.publish(msg);
		}

		if(state.robot_return){
			my_msgs::can_msg msg;
			msg.id = 0x13;
			msg.data.resize(8);
			can_tx.publish(msg);
		}

		if(state.shot_and_reload && !reload_lock){
			my_msgs::can_msg msg_0x2n;
			my_msgs::can_msg msg_0x5n;

			msg_0x2n.id = 0x20;
			msg_0x5n.id = 0x50;
			msg_0x2n.data.resize(8);
			msg_0x5n.data.resize(8);
			can_tx.publish(msg_0x2n);
			can_tx.publish(msg_0x5n);
		}

		if(state.change_cartridge){
			my_msgs::can_msg msg_0x5n;
			my_msgs::can_msg msg_0x6n;

			msg_0x5n.id = 0x51;
			msg_0x6n.id = 0x60;
			msg_0x5n.data.resize(8);
			msg_0x6n.data.resize(8);
			can_tx.publish(msg_0x5n);
			can_tx.publish(msg_0x6n);
			
			reload_lock = true;
		}

		if(state.shot_cycle_update){
			++shot_spd;

			my_msgs::can_msg msg;
			msg.id = 0x21;
			msg.data.resize(8);
			msg.data; //have to decide protocol

			can_tx.publish(msg);
		}

		if(is_update_value){
			///@brief sense coeff 
			///       have to change to controller's opinion
			constexpr float hi_sense_coeff     = 3/128,
							middle_sense_coeff = 1/128,
							low_sense_coeff    = 0.5/128;
			constexpr float spin_hi_sense_coeff     = 2*M_PI/128,
							spin_middle_sense_coeff = 0.5*M_PI/128,
							spin_low_sense_coeff    = 0.25*M_PI/128;

			float pub_x = 0, pub_y = 0, pub_spin = 0;

			switch (sense.get_cycle_state()){
			case sense.HI_SENSE:
				pub_x = state.x_velo * hi_sense_coeff;
				pub_y = state.y_velo * hi_sense_coeff;
				pub_spin = state.spin * spin_hi_sense_coeff;
				break;

			case sense.MIDDLE_SENSE:
				pub_x = state.x_velo * middle_sense_coeff;
				pub_y = state.y_velo * middle_sense_coeff;
				pub_spin = state.spin * spin_middle_sense_coeff;
				break;

			case sense.LOW_SENSE:
				pub_x = state.x_velo * low_sense_coeff;
				pub_y = state.y_velo * low_sense_coeff;
				pub_spin = state.spin * spin_low_sense_coeff;
				break;
			
			default:
				break;
			}

			uint8_t msg1_uint8[8], msg2_uint8[8];

			convert_32_8(float_to_int(pub_x), &(msg1_uint8[0]));
			convert_32_8(float_to_int(pub_y), &(msg1_uint8[4]));
			convert_32_8(float_to_int(pub_spin), &(msg2_uint8[0]));

			my_msgs::can_msg msg;
			msg.id = 0x10;
			msg.data.resize(8);
			for(auto i = 0lu; i < strlen((char*)msg1_uint8); i++){
				msg.data[i] = msg1_uint8[i];
			}
			can_tx.publish(msg);

			msg.id = 0x11;
			for(auto i = 0lu; i < strlen((char*)msg2_uint8); i++){
				msg.data[i] = msg2_uint8[i];
			}
			can_tx.publish(msg);
		}

		if(is_update_value){
			uint8_t angle = state.shot_angle;
			my_msgs::can_msg msg;
			msg.data.resize(8);
			msg.id = 0x22;
			msg.data[7] = angle;

			can_tx.publish(msg);
		}
		//TODO

		if(is_emergency){
			my_msgs::can_msg msg;
			msg.id = 0x0f;
			msg.data.resize(8);

			for(auto i = 1; i < 7; i++){
				msg.id += 0x10;
				can_tx.publish(msg);
			}
		}

		if(is_rx_can){
			if((can_msg.id >= 0x08 && 0x0D <= can_msg.id) || can_msg.id == 0x0F) is_emergency = true;
			if(can_msg.id == 0x0E) is_emergency = false;

			if(can_msg.id == 0x07) reload_lock = false;

			// TODO
			// write odom
		}
		
#if TEST

#endif //TEST

		ros::spinOnce();
		rate.sleep();
	}
}

#if TEST

#endif //TEST

//helper functions
uint32_t float_to_int(float f){
	uint32_t*p = reinterpret_cast<uint32_t*>(&f);
	return *p;
}


void convert_32_8(uint32_t i32, uint8_t buf[]){
	for(int i = 0LU; i < 4; i++){
		buf[i] = i32>>8*(3-i);
	}
}

