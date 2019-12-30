#!/usr/bin/env python

import rospy
#import roslib
#roslib.load_manifest('differential_drive')
from math import sin, cos, pi
import geometry_msgs.msg
import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
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
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 572.95))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.4)) # The wheel base width in meters
        
        #self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        #self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
#        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
#        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
#        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
#        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.pi_ = 3.14159265358979323846
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
        self.th_imu = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        self.flag = False
        self.yaw = 0
        self.yaw_0 = 0
        self.pre_yaw = 0
        self.orientation_q = 0
        self.orien_z = 0
        self.orien_w = 0
        #self.k = 0
        # subscriptions
        
        #rospy.Subscriber("lwheel", Int16, self.lwheelCallback)
        #rospy.Subscriber("rwheel", Int16, self.rwheelCallback)
        rospy.Subscriber("/imu_quat", Imu, self.imuCallback)
        #print("0k")
        #self.odomPub = rospy.Publisher("odom", Odometry,queue_size=100)
        self.odomBroadcaster = tf.TransformBroadcaster()
        
        self.d = 0
    #############################################################################
    def spin(self):
    #############################################################################
        """
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            print(self.flag)
            if self.flag == False :
                self.yaw_0 = self.yaw
                self.pre_yaw = self.yaw
            if self.yaw != 0:
                self.flag = True
                print("pre_yaw",self.pre_yaw)
                print("yaw0",self.yaw_0)
                if self.flag == True :
                    print("ok")
                    self.update()
                    r.sleep()
            else:
                pass
        """
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
            elapsed = elapsed.to_sec() 
            
            
            if( self.pre_yaw != 0):
                th_imu = self.yaw - self.pre_yaw
            else:
                th_imu = 0

            if th_imu > math.pi : 
                th_imu = th_imu - 2*math.pi
            elif th_imu < -math.pi:
                th_imu = th_imu + 2*math.pi
            self.pre_yaw = self.yaw 
            if(th_imu!=0):
     #          th_x = (th+th_imu)/2
                self.th_imu = self.th_imu + th_imu            
                #self.th = self.th + th
                print("Theta imu:",self.th_imu)
                #print("Theta enc:", self.th)
                print("Yaw",self.yaw)
                print("-------------------------") 
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
        #self.orien_z = self.orientation_q.z
        #self.orien_w = self.orientation_q.w
        self.yaw = yaw
        
############################################################################
if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.spin()
    
    
   
