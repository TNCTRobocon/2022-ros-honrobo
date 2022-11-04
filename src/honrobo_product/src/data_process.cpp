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
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <my_msgs/can_msg.h>

#define TEST // need to remove

#ifdef TEST
#define TEST 1
#endif //TEST

#ifdef PRODUCT
#define PRODUCT 1
#endif // PRODUCT

constexpr size_t can_data_size = 8;
bool is_emergency = false;
bool is_restoration = false;

enum YUKARI_STATUS{
emergency,    move,         slow,
hispeed,      no_plane,     spin,
logic_start,  restoration,  start,
middle,       shot,         stop,

};

// structures
struct input_status{
	bool drive_mode_update  = false,
		 sense_cycle_update = false,
		 tire_return        = false,
		 robot_return       = false,
		 shot_and_reload    = false,
		 change_cartridge   = false,
		 return_to_start    = false,
		 rise_initialize    = false,
		 shot_cycle_update  = false;

	int8_t spin = 0,
		   x_velo = 0,
		   y_velo = 0;

	uint8_t shot_angle = 0;

	enum {
		MOVE,
		SPIN,
		SHOT
	};
};

class cycle{
	uint8_t cycle_val = 0;

public:
	enum MODE{
		STOP,
		LOW_SENSE,
		MIDDLE_SENSE,
		HI_SENSE,
	};

	cycle(){}
	~cycle(){}

	void operator++(){
		cycle_val++;
		cycle_val = (cycle_val == 4)? 0: cycle_val;
	}

	uint8_t get_cycle_state(){ return cycle_val; }

};

//prototype decl
template<typename T>
T clamp(const T val, const T max, const T min);
template<typename T>
T clamp(const T val, const T diff_val, const T max, const T min);
uint32_t float_to_int(float f);
void convert_32_8(uint32_t i, uint8_t*buf);

//callback functions
bool is_rx_can = false;
my_msgs::can_msg can_msg;
void can_rx_callback(const my_msgs::can_msg& CAN_RX){
	is_rx_can = true;
	if(can_msg.data.size() != can_data_size) can_msg.data.resize(can_data_size);
	can_msg.id = CAN_RX.id;
	std::copy(CAN_RX.data.begin(), CAN_RX.data.end(), can_msg.data.begin());
}

bool is_new_value = false;
std_msgs::UInt8MultiArray rx_array;
void controller_wireless_callback(const std_msgs::UInt8MultiArray &rx_msg){
	if(rx_array.data.size() != can_data_size) rx_array.data.resize(can_data_size);
	std::copy(rx_msg.data.begin(), rx_msg.data.end(), rx_array.data.begin());
	is_new_value = true;
}

void controller_wifi_callback(const std_msgs::UInt8MultiArray& rx_msg){
	if(rx_array.data.size() != can_data_size) rx_array.data.resize(can_data_size);
	std::copy(rx_msg.data.begin(), rx_msg.data.end(), rx_array.data.begin());
	is_new_value = true;
}

bool lidar_rx = false;
float lidar = 0.0f;
void lidar_callback(const std_msgs::Float32& lidar_angle){
	lidar = lidar_angle.data;
	lidar_rx = true;
}

#if TEST

#endif //TEST

int main(int argc, char**argv){
	ros::init(argc, argv, "data_process");
	ros::NodeHandle nh;
	ros::Publisher can_tx = nh.advertise<my_msgs::can_msg>("CAN_TX", 10);
	ros::Subscriber can_rx = nh.subscribe("CAN_RX", 10, can_rx_callback);
	ros::Subscriber lidar_input = nh.subscribe("lidar_val", 10, lidar_callback);
	ros::Publisher controller_output = nh.advertise<std_msgs::Float32>("lidar_data", 10);
	ros::Subscriber controller_wireless_input = nh.subscribe("control_wireless_data", 10, controller_wireless_callback);
	ros::Subscriber controller_wifi_input = 	nh.subscribe("control_data", 10, controller_wifi_callback);

	ros::Publisher yukari = nh.advertise<std_msgs::UInt8>("yukari_voice", 10);
	// need more?

	cycle sense;
	cycle shot_spd;
	uint8_t shot_status = 0;
	bool pre_LT = false;
	bool reload_lock = false;
	bool one_sec_flag = false;

	std_msgs::UInt8 yukari_msg;
	uint8_t shot_counter = 0;

	my_msgs::can_msg start_msg;
	start_msg.data.resize(8);
	start_msg.id = 0x30;
	start_msg.data[0] = 0b100'100;

	can_tx.publish(start_msg);

	while(ros::ok()){
		input_status state;
		bool is_update_value = false;

		if(lidar_rx){
			std_msgs::Float32 msg;
			msg.data = lidar;
			controller_output.publish(msg);
			lidar_rx = false;
		}

		if(is_new_value){
			state.drive_mode_update  = rx_array.data[0] & 0b10000000;
			state.sense_cycle_update = rx_array.data[0] & 0b01000000;
			state.tire_return        = rx_array.data[0] & 0b00100000;
			state.robot_return       = rx_array.data[0] & 0b00010000;
			state.shot_and_reload    = rx_array.data[0] & 0b00001000;
			state.change_cartridge   = rx_array.data[0] & 0b00000100;
			
			state.rise_initialize    = rx_array.data[1] & 0b10000000;
			state.return_to_start    = rx_array.data[1] & 0b00000100;
			bool now_LT = (rx_array.data[6]>50)? true: false;
			state.shot_cycle_update  = (pre_LT^now_LT) & now_LT;
			pre_LT = now_LT;


			int16_t tmp_spin       = -(rx_array.data[2] - 128 -4) & 0xfff8;
			int16_t tmp_shot_angle = (rx_array.data[3] - 128 + 4) & 0xfff8;
			int16_t tmp_x_velo     = -(rx_array.data[4] - 128 -4) & 0xfff8;
			int16_t tmp_y_velo     = (rx_array.data[5] - 128 + 4) & 0xfff8;

			constexpr int16_t max = 127, min = -128;
			state.spin = clamp<int16_t>(tmp_spin, max, min);
			state.shot_angle = clamp(tmp_shot_angle, max, min);
			state.x_velo = (int8_t)clamp<int16_t>(tmp_x_velo, max, min);
			state.y_velo = clamp(tmp_y_velo, max, min);

			is_new_value = false;
			is_update_value = true;
		}

		if(state.drive_mode_update){
			shot_status++;
			if(shot_status%3 == 0) shot_status = 0;
			my_msgs::can_msg msg;
			msg.id = 0x14;
			msg.data.resize(8);
			msg.data[0] = shot_status;
			can_tx.publish(msg);

			msg.id = 0x23;
			
			can_tx.publish(msg);

			enum {
				R,
				G,
				B,
			};
			
			switch(shot_status){
				case input_status::MOVE:
					// decide val
					yukari_msg.data = YUKARI_STATUS::move;
					yukari.publish(yukari_msg);
					msg.id = 0x30;
					msg.data[0] = 0b100'100;
					break;
				
				case input_status::SPIN:
					// decide val
					yukari_msg.data = YUKARI_STATUS::spin;
					yukari.publish(yukari_msg);
					msg.id = 0x30;
					msg.data[0] = 0b010'010;
					break;
				
				case input_status::SHOT:
					// decide val
					yukari_msg.data = YUKARI_STATUS::shot;
					yukari.publish(yukari_msg);
					msg.id = 0x30;
					msg.data[0] = 0b001'001;
					break; 
				
				default:
					//error status
					// decide val
					msg.id = 0x30;
					msg.data[0] = 0;
					msg.data[1] = 0;
					msg.data[2] = 0;
					break;
			}
			can_tx.publish(msg);
		}

		if(state.sense_cycle_update){
			++sense;
			
			switch(sense.get_cycle_state()){
				case sense.HI_SENSE:
					yukari_msg.data = YUKARI_STATUS::hispeed;
					yukari.publish(yukari_msg);
					break;
				case sense.MIDDLE_SENSE:
					yukari_msg.data = YUKARI_STATUS::middle;
					yukari.publish(yukari_msg);
					break;
				case sense.LOW_SENSE:
					yukari_msg.data = YUKARI_STATUS::slow;
					yukari.publish(yukari_msg);
					break;
				case sense.STOP:
					yukari_msg.data = YUKARI_STATUS::stop;
					yukari.publish(yukari_msg);
					break;
				default:
					break;
				
			}
		}

		if(state.tire_return){
			my_msgs::can_msg msg;
			msg.id = 0x12;
			msg.data.resize(can_data_size);
			can_tx.publish(msg);
		}

		if(state.robot_return){
			my_msgs::can_msg msg;
			msg.id = 0x13;
			msg.data.resize(can_data_size);
			can_tx.publish(msg);
		}

		if(state.shot_and_reload && state.SHOT){
			my_msgs::can_msg msg_0x2n;
			my_msgs::can_msg msg_0x5n;

			if(shot_counter < 8){
				msg_0x2n.id = 0x20;
				msg_0x5n.id = 0x50;
				msg_0x2n.data.resize(can_data_size);
				msg_0x5n.data.resize(can_data_size);
				can_tx.publish(msg_0x2n);
				can_tx.publish(msg_0x5n);
			}else{
				yukari_msg.data = YUKARI_STATUS::no_plane;
				yukari.publish(yukari_msg);
			}

			++shot_counter;
		}

		if(state.change_cartridge){

			my_msgs::can_msg msg;
			msg.id = 0x25;
			msg.data.resize(8);
			can_tx.publish(msg);
			shot_counter = 0;
		}

		if(state.shot_cycle_update){
			++shot_spd;

			my_msgs::can_msg msg;
			msg.id = 0x21;
			msg.data.resize(can_data_size);
			msg.data; //have to decide protocol

			can_tx.publish(msg);
		}

		if(is_update_value && shot_status == state.MOVE){
			///@brief sense coeff 
			///       have to change to controller's opinion
			enum{
				X, Y, SPIN
			};
			constexpr float hi_sense_coeff     = 3.0f/128.0f,
							middle_sense_coeff = 1.5f/128.0f,
							low_sense_coeff    = 0.5/128.0f;
			constexpr float spin_hi_sense_coeff     = 1.0f*M_PI/128.0f,
							spin_middle_sense_coeff = 0.5*M_PI/128.0f,
							spin_low_sense_coeff    = 0.25*M_PI/128.0f;

			float pub_x = 0, pub_y = 0, pub_spin = 0;
			static std::array<float, 3> pre_v = {0, 0, 0};

			switch (sense.get_cycle_state()){
			case sense.STOP:
				pub_x = 0;
				pub_y = 0;
				pub_spin = 0;
				break;
							
			case sense.LOW_SENSE:
				pub_x = state.x_velo * low_sense_coeff;
				pub_y = state.y_velo * low_sense_coeff;
				pub_spin = state.spin * spin_low_sense_coeff;
				break;

			case sense.MIDDLE_SENSE:
				pub_x = state.x_velo * middle_sense_coeff;
				pub_y = state.y_velo * middle_sense_coeff;
				pub_spin = state.spin * spin_middle_sense_coeff;
				break;

			case sense.HI_SENSE:
				pub_x = state.x_velo * hi_sense_coeff;
				pub_y = state.y_velo * hi_sense_coeff;
				pub_spin = state.spin * spin_hi_sense_coeff;
				break;
			
			default:
				break;
			}

			pub_x = clamp(pub_x, pre_v[X], 0.1f, -0.1f);
			pub_y = clamp(pub_y, pre_v[Y], 0.1f, -0.1f);
			pub_spin = clamp(pub_spin, pre_v[SPIN], 0.05f, -0.05f);
			ROS_INFO("%f", pub_x);

			uint8_t msg1_uint8[can_data_size], msg2_uint8[can_data_size];

			convert_32_8(float_to_int(pub_x), &(msg1_uint8[0]));
			convert_32_8(float_to_int(pub_y), &(msg1_uint8[4]));
			convert_32_8(float_to_int(pub_spin), &(msg2_uint8[0]));
			convert_32_8(0, &(msg2_uint8[4]));

			my_msgs::can_msg msg;
			msg.id = 0x10;
			msg.data.resize(can_data_size);
			for(auto i = 0lu; i < 8; i++){
				msg.data[i] = msg1_uint8[i];
			}
			can_tx.publish(msg);

			msg.id = 0x11;
			for(auto i = 0lu; i < 8; i++){
				msg.data[i] = msg2_uint8[i];
			}
			can_tx.publish(msg);
			pre_v[X] = pub_x;
			pre_v[Y] = pub_y;
			pre_v[SPIN] = pub_spin;
		}

		if(is_update_value && shot_status == state.SPIN){
			///@brief sense coeff 
			///       have to change by controller's opinion
			constexpr float hi_sense_coeff     = 2.0f/128.0f,
							middle_sense_coeff = 1.0f/128.0f,
							low_sense_coeff    = 0.5/128.0f;
			constexpr float spin_hi_sense_coeff     = 1.0f*M_PI/128.0f,
							spin_middle_sense_coeff = 0.5f*M_PI/128.0f,
							spin_low_sense_coeff    = 0.25*M_PI/128.0f;

			float pub_x = 0, pub_y = 0, pub_spin = 0;

			switch (sense.get_cycle_state()){
			case sense.STOP:
				pub_x = 0;
				break;
							
			case sense.LOW_SENSE:
				pub_x = state.x_velo * low_sense_coeff;
				break;

			case sense.MIDDLE_SENSE:
				pub_x = state.x_velo * middle_sense_coeff;
				break;

			case sense.HI_SENSE:
				pub_x = state.x_velo * hi_sense_coeff;
				break;
			
			default:
				break;
			}

			uint8_t msg1_uint8[can_data_size];

			convert_32_8(float_to_int(pub_x), &(msg1_uint8[0]));
			convert_32_8(0.0f, &(msg1_uint8[4]));
			
			my_msgs::can_msg msg;
			msg.id = 0x15;
			msg.data.resize(can_data_size);
			for(auto i = 0lu; i < 8; i++){
				msg.data[i] = msg1_uint8[i];
			}
			can_tx.publish(msg);
		}

		if(is_update_value){
			my_msgs::can_msg msg;
			msg.data.resize(can_data_size);
			msg.id = 0x22;
			msg.data[7] = state.shot_angle;

			can_tx.publish(msg);
		}
		if(state.rise_initialize){
			my_msgs::can_msg msg;
			msg.data.resize(can_data_size);
			msg.id = 0x24;
			can_tx.publish(msg);
		}

		if(is_restoration){
			my_msgs::can_msg msg;
			yukari_msg.data = YUKARI_STATUS::restoration;
			yukari.publish(yukari_msg);
			msg.id = 0x0E;
			msg.data.resize(8);

			for(auto i = 1; i <= 3; i++){
				msg.id += 0x10;
				can_tx.publish(msg);
			}

			msg.id = 0x30;
			msg.data[0] = 0b001001;
			can_tx.publish(msg);

			is_restoration = false;
		}

		if(is_emergency){
			yukari_msg.data = YUKARI_STATUS::emergency;
			yukari.publish(yukari_msg);
			my_msgs::can_msg msg;
			msg.id = 0x0f;
			msg.data.resize(can_data_size);

			for(auto i = 1; i <= 3; i++){
				msg.id += 0x10;
				can_tx.publish(msg);
			}
			shot_status = 0;
			while(!(sense.get_cycle_state() == sense.STOP)) ++sense;
			while(!(shot_spd.get_cycle_state() == shot_spd.STOP)) ++shot_spd;
			is_emergency = false;
		}

		if(is_rx_can && !one_sec_flag){
			if(can_msg.id == 0x0F) {
				is_emergency = true;
				one_sec_flag = true;
			}
			if(can_msg.id == 0x0E) is_restoration = true;

			if(can_msg.id == 0x07) reload_lock = false;

			if(can_msg.id == 0x04) {
				ROS_INFO("%09f", (float)can_msg.data[0]*2.0*M_PI/255.0);
			}

			is_rx_can = false;

			static uint8_t logic_ok = 0b11;
			if(can_msg.data[0] == 0 && can_msg.data[7] == 0){
				if(can_msg.id == 0x09) logic_ok &= 0b01;
				if(can_msg.id == 0x0A) logic_ok &= 0b10;
			}

			static bool is_pass = false;
			if(!logic_ok){
				is_pass = true;
				my_msgs::can_msg LED_msg;
				LED_msg.id = 0x30;
				LED_msg.data.resize(8);
				LED_msg.data[0] = 0b001001;
				can_tx.publish(LED_msg);

				yukari_msg.data = YUKARI_STATUS::logic_start;
				yukari.publish(yukari_msg);
				logic_ok = 0b11;
			}
			// write odom?
		}

		if(one_sec_flag){
			static bool pre_one_sec = false;

			static ros::Time start;
			static ros::Time end;

			if((one_sec_flag^pre_one_sec)&one_sec_flag){
				start = ros::Time::now();
			}

			end = ros::Time::now();
			auto time = end-start;

			pre_one_sec = one_sec_flag;

			if(time.nsec > 100'000'000ULL) one_sec_flag = false;
		}
		
#if TEST

#endif //TEST
		ros::spinOnce();
	}
}

#if TEST

#endif //TEST

template<typename T>
T clamp(const T val, const T max, const T min){
	if(val > max) return max;
	if(val < min) return min;
	return val;
}

template<typename T>
T clamp(const T val, const T diff_val, const T max, const T min){
	if((val-diff_val) > max) return diff_val+max;
	if((val-diff_val) < min) return diff_val+min;
	return val;
}

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

