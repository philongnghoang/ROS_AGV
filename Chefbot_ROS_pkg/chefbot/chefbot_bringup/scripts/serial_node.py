#!/usr/bin/env python

'''
launchpad_node.py - Receive sensor values from Launchpad board and publish as topics

Created September 2014

Copyright(c) 2014 Lentin Joseph

Some portion borrowed from  Rainer Hessmer blog
http://www.hessmer.org/blog/
'''

#Python client library for ROS
import rospy
import sys
import time
import math

#This module helps to receive values from serial port
from SerialDataGateway import SerialDataGateway
#Importing ROS data types
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64
#Importing ROS data type for IMU
from sensor_msgs.msg import Imu

#Class to handle serial data from Launchpad and converted to ROS topics
class Launchpad_Class(object):
	
	def __init__(self):
		print ("Initializing Launchpad Class")

		#######################################################################################################################
		#Sensor variables
		self._Counter = 0

		self._left_encoder_value = 0
		self._right_encoder_value = 0

		self._left_vel_value = 0
		self._right_vel_value = 0


		self._left_wheel_speed_ = 0
		self._right_wheel_speed_ = 0

		self._LastUpdate_Microsec = 0
		self._Second_Since_Last_Update = 0

		self.robot_heading = 0
		#######################################################################################################################
		#Get serial port and baud rate of Tiva C Launchpad
		port = rospy.get_param("~port", "/dev/ttyUSB0")
		baudRate = int(rospy.get_param("~baudRate", 115200))

		#######################################################################################################################
		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
		#Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
		rospy.loginfo("Started serial communication")


		######################################################################################################################		
		#Publisher for left and right wheel velocity values
		self._Left_Vel = rospy.Publisher('vellwheel',Int64,queue_size = 10)		
		self._Right_Vel = rospy.Publisher('velrRwheel',Int64,queue_size = 10)

		#Publisher for entire serial data
		self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)

	#######################################################################################################################
	#Subscribers and Publishers of IMU data topic


	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))


		if(len(line) > 0):

			lineParts = line.split('\t')
			try:

				if(lineParts[0] == 'v'):
					self._left_vel_value = float(lineParts[1])
					self._right_vel_value = float(lineParts[2])
					self._Left_Vel.publish(self._left_vel_value)
					self._Right_Vel.publish(self._right_vel_value)
					print("hello")

##########################################################################
			except:
				rospy.logwarn("Error in Sensor values")
				rospy.logwarn(lineParts)
				pass		

	def _WriteSerial(self, message):
		self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)

#######################################################################################################################


	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

#######################################################################################################################

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()

#######################################################################################################################


	def Subscribe_Speed(self):
		a = 1
#		print "Subscribe speed"

#######################################################################################################################


	def Reset_Launchpad(self):
		print ("Reset")
		reset = 'r\r'
		self._WriteSerial(reset)
		time.sleep(1)
		self._WriteSerial(reset)
		time.sleep(2)


#######################################################################################################################

	def Send_Speed(self):
		#		print "Set speed"
		a = 3


if __name__ =='__main__':
	rospy.init_node('launchpad_ros',anonymous=True)
	launchpad = Launchpad_Class()
	try:

		launchpad.Start()	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Error in main function")


	#launchpad.Reset_Launchpad()
	#pilaunchpad.Stop()

#######################################################################################################################


