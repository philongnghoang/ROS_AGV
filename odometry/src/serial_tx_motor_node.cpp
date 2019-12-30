#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <sstream>
#include "tf/transform_datatypes.h"

serial::Serial ser;
using namespace std;
void read_velocity(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM(msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_tx_motor_node");
    string serial_port;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
 //   nh_private.getParam("serial_port", serial_port); 
    ros::Subscriber read_sub = nh.subscribe("velocity", 1000, read_velocity);
 //   ros::Publisher enc_l_pub = nh.advertise<std_msgs::Int16>("lwheel", 1000);
 //   ros::Publisher enc_r_pub = nh.advertise<std_msgs::Int16>("rwheel", 1000);

    try
    {
        ser.setPort(serial_port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port! ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(50);
    ROS_INFO_STREAM("Reading from serial port");
    float tam=0;
    int16_t data[2];
    while(ros::ok()){
        std_msgs::Int16  enc_l_msg;
        std_msgs::Int16  enc_r_msg;

        if(ser.available())
            {
//                std::string line;
//                line = ser.readline(1024, "\n");
//                ROS_INFO_STREAM(line);
//                char *token = std::strtok(&line[0]," ");
//            
//                int cnt=0;
//                string number;
//                while(token!= NULL){
//                number = token;
//                // standalize data from imu
//                data[cnt] = atoi(number.c_str());
//                token = std::strtok(NULL, " ");
//                cnt++;
//                }
            }

//        enc_l_msg.data = data[0];
//        enc_r_msg.data = data[1];
//        enc_l_pub.publish(enc_l_msg);
//        enc_r_pub.publish(enc_r_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}