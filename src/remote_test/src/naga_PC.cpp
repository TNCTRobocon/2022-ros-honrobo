#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String& str){
    ROS_INFO("listen: %s", str.data.c_str());
}

int main(int argc, char** argv){
    ros::init(argc, argv, "naga_PC");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter1", 10);
    ros::Subscriber sub = nh.subscribe("chatter2", 10, callback);

    ros::Rate rate(10);

    while(ros::ok()){
        std_msgs::String msg;
        msg.data = "Hello remote";
        pub.publish(msg);

        ROS_INFO("hello world naga's PC");

        ros::spinOnce();
        rate.sleep();
    }
}