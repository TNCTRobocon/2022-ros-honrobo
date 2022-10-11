#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

sensor_msgs::Joy joy;
bool is_rx = false;
void callback(const sensor_msgs::Joy& joy_){
    if(joy.axes.size() != 8) joy.axes.resize(8);
    if(joy.buttons.size() != 8) joy.buttons.resize(8);
    std::copy(joy_.axes.begin(), joy_.axes.end(), joy.axes.begin());
    std::copy(joy_.buttons.begin(), joy_.buttons.end(), joy.buttons.begin());
    is_rx = true;
}

int main(int argc, char**argv){
    joy.buttons.resize(8);
    joy.axes.resize(8);
    
    ros::init(argc, argv, "joy_encode");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("joy", 10, callback);
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("control_data", 10);
    ros::Rate rate(100);
    std_msgs::UInt8MultiArray msg;
    msg.data.resize(8);
    while (ros::ok()){
        msg.data[0] = 0;
        if(is_rx){
            msg.data[0] = 
                (joy.buttons[0] << 7) +
                (joy.buttons[1] << 6) +
                (joy.buttons[2] << 5) +
                (joy.buttons[3] << 4) +
                (joy.buttons[5] << 3) +
                (joy.buttons[4] << 2);
                ROS_INFO("%d, %d", msg.data[0], msg.data[2]);
                is_rx = false;

            msg.data[1] = 
                0;//TODO
        }
        
        msg.data[2] = joy.axes[3]*127 - 128;
        msg.data[3] = joy.axes[4]*127 - 128;
        msg.data[4] = joy.axes[0]*127 - 128;
        msg.data[5] = joy.axes[1]*127 - 128;
        msg.data[6] = joy.axes[5]*127 - 128;
        msg.data[7] = joy.axes[2]*127 - 128;
        pub.publish(msg);
    
        ros::spinOnce();
        rate.sleep();
    }
    
}
