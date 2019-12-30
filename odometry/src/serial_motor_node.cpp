#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include "tf/transform_datatypes.h"
#include <stdio.h>
#include <math.h>
serial::Serial ser;

using namespace std;
void read_velocity(const std_msgs::String::ConstPtr& msg){
//    ROS_INFO_STREAM(msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_motor_node");
    string serial_port;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    //nh_private.getParam("serial_port", serial_port);
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
     
    ros::Subscriber read_sub = nh.subscribe("velocity", 1000, read_velocity);
    //ros::Publisher pub_left = nh.advertise<std_msgs::Float32>("l_vel", 1000);
    //ros::Publisher pub_right = nh.advertise<std_msgs::Float32>("r_vel", 1000);
    ros::Publisher pub_left = nh.advertise<std_msgs::Int64>("lwheel", 1000);
    ros::Publisher pub_right = nh.advertise<std_msgs::Int64>("rwheel", 1000);
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

    ros::Rate loop_rate(100);
    ROS_INFO_STREAM("Reading from serial port");
    float tam=0;
    while(ros::ok()){
        // doc xung encoder
        std_msgs::Int64 vel_left;
        std_msgs::Int64 vel_right;
        // doc van toc
        //std_msgs::Float32 vel_left;
        //std_msgs::Float32 vel_right;
        //std_msgs::Float32 vel_right
        int32_t data[3];
        if(ser.available())
        {
            if(ser.available())
            {
            
            std::string line;
            line = ser.readline(1024, "\n");
            ROS_INFO_STREAM(line);
            char *token = std::strtok(&line[0]," ");
            
            int cnt=0;
            string number;
            while(token!= NULL){
                number = token;
                data[cnt] = atof(number.c_str());
                token = std::strtok(NULL, " ");
                cnt++;      
            }
            }
        }
        //double a = data[1];
        //float vel_right=(float)data[2]; 
        vel_left.data = (int)data[0];
        vel_right.data = (int)data[1];
        pub_left.publish(vel_left);
        pub_right.publish(vel_right);
        //read_pub.publish(vel_right);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}