#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

#define angle(i) 2.0*M_PI-2.0*M_PI/505*(float)i

/**
 * @brief need to decide angle range that lidar measured data
 * 
 */
sensor_msgs::LaserScan laser_scan;
void lidar_rx_callback(const sensor_msgs::LaserScan& msg){
	constexpr uint angle1=256, angle2 = 250;
	if(!(msg.ranges[angle1] == 0.0f || msg.ranges[angle2] == 0.0f)){
		auto angler1 = atan2(msg.ranges[angle1]*sin(angle(angle1)) - msg.ranges[angle2]*sin(angle(angle2)), msg.ranges[angle1]*cos(angle(angle1)) - msg.ranges[angle2]*cos(angle(angle2))) /2.0/M_PI*360;
		ROS_INFO("range1: %f, range2: %f, angler: %f", msg.ranges[angle1],msg.ranges[angle2], fmod(angler1, 90.0f));
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "lidar_process");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("scan", 10, lidar_rx_callback);
	ros::Publisher pub = nh.advertise<std_msgs::Float32>("lidar_val", 10);

	ros::Rate rate(100);

	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
}
