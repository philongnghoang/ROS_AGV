#!/usr/bin/env python

import rospy
#import roslib
#roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16, Int64,Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
import math

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',20.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 572.95))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.4)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','laser') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
#        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
#        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
#        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
#        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.pre_d_left = 0
        self.pre_d_right = 0

        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0

        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        
        self.yaw = 0
        self.pre_yaw = 0
        self.orientation_q = 0
        self.orien_z = 0
        self.orien_w = 0
        # subscriptions
        
        rospy.Subscriber("lwheel", Int16, self.lwheelCallback)
        rospy.Subscriber("rwheel", Int16, self.rwheelCallback)
        rospy.Subscriber("/imu_quat", Imu, self.imuCallback)

        self.odomPub = rospy.Publisher("odom", Odometry,queue_size=100)
        self.odomBroadcaster = TransformBroadcaster()
        
        self.d = 0
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then       
            self.then = now
            elapsed = elapsed.to_sec()      # elapsed = 0.1s
            
            # calculate odometry
            if self.enc_left == None:
                self.d_left = 0
                self.d_right = 0
            else:
                denta_left = self.left - self.enc_left
                if denta_left>1000:
                    denta_left = denta_left - 2000
                elif denta_left < -1000:
                    denta_left = denta_left + 2000

                denta_right = self.right - self.enc_right
                if denta_right>1000:
                    denta_right = denta_right - 2000
                elif denta_right < -1000:
                    denta_right = denta_right + 2000    
                self.d_left = denta_left / self.ticks_meter
                self.d_right = denta_right / self.ticks_meter

            self.enc_left = self.left
            self.enc_right = self.right
            
#            print(self.d_left)
            # distance traveled is the average of the two wheels 
            d = ( self.d_left + self.d_right ) / 2
            self.d = self.d + d
            #print(self.d)

            # this approximation works (in radians) for small angles
#            th = ( self.d_right - self.d_left ) / self.base_width 
            
            th = self.yaw - self.pre_yaw
            #if th >= math.pi:
            #    th = th - 2*math.pi
            #elif th <= -math.pi:
            #    th = th + 2*math.pi
            self.pre_yaw = self.yaw 

            # calculate velocities
            self.dx = d / elapsed   
            self.dr = th / elapsed      # rad/s
           
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
            #print(self.th)    
 #           print(self.x,self.y)
            
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th/2 )#self.orien_z       # or using orientation_q.z
            quaternion.w = cos( self.th/2 )#self.orien_w       # or using orientation_q.w 
                
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion

            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            
    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        self.left = enc    
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        self.right = enc

    #############################################################################
    def imuCallback(self, msg):
        global roll, pitch, yaw
        self.orientation_q = msg.orientation
        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion( orientation_list)
        self.orien_z = self.orientation_q.z
        self.orien_w = self.orientation_q.w
        self.yaw = yaw


############################################################################
if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.spin()
    
    
   
