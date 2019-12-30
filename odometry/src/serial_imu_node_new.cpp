/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include "tf/transform_datatypes.h"
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <math.h>

#define pi 3.14159265358979323846 

serial::Serial ser;
tf::Quaternion q;
using namespace std;
void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_imu_node");
    string serial_port;
    int serial_baudrate = 926100;
    string frame_id;
    string imu_node;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB3"); 
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
    nh_private.param<std::string>("frame_id", frame_id, "map");
    nh_private.param<string>("imu_node", imu_node, "/imu_quat");
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<sensor_msgs::Imu>(imu_node, 1000);

    try
    {
        ser.setPort(serial_port);
      //  ser.setBaudrate(921600);
      //  ser.setBaudrate(460800);
        ser.setBaudrate(serial_baudrate);
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

    ros::Rate loop_rate(1000);
    ROS_INFO_STREAM("Reading from serial port");
    float tam=0;
    while(ros::ok()){
        sensor_msgs::Imu imu_msg;
        
        // Turn on frame_id when run only IMU
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id;

       
        double data[10];
        if(ser.available()){
            
            std::string line;
            line = ser.readline(1024, "\n");
        ROS_INFO_STREAM(line);
            char *token = std::strtok(&line[0]," ");
            
            int cnt=0;
            string number;
            while(token!= NULL){
                number = token;
                // standalize data from imu
                data[cnt] = atof(number.c_str())/1000   ;
                token = std::strtok(NULL, " ");
                cnt++;
                
            }
        }
        
        double Roll=data[0];  double Pitch=data[1]; double Yaw=data[2];
        // Convert from Roll Pitch Yaw to Quaternion 
        q=tf::createQuaternionFromRPY(-Roll*pi/180,Pitch*pi/180,-Yaw*pi/180);
       
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        imu_msg.angular_velocity.x = 0;
        imu_msg.angular_velocity.y = 0;
        imu_msg.angular_velocity.z = 0;
        imu_msg.linear_acceleration.x = 0;
        imu_msg.linear_acceleration.y = 0;
        imu_msg.linear_acceleration.z = 0;

        read_pub.publish(imu_msg);
        ros::spinOnce();
      //  loop_rate.sleep();

    }
    return 0;
}

