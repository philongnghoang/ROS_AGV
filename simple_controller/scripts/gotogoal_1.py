#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi
from tf.transformations import euler_from_quaternion
from leg_tracker.msg import Person, PersonArray, Leg, LegArray
import yaml 
class My_Robot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/odom',
                                                Odometry, self.update_pose)

        self.sub_tracker = rospy.Subscriber("/people_tracked", 
                                                PersonArray, self.person  )

        self.pose = Odometry()
        self.person = PersonArray()
        self.rate = rospy.Rate(10)
        self.x = 0
        self.y = 0
        self.theta = 0

        self.person_x = None
        self.person_y = None
        self.person_id = None
        self.follow_id = None
        self.person_cnt = -1
        self.collect_data = []
        self.collect_per = []
        self.count = 0
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        global x,y,theta
        self.pose = data
        self.x = round(self.pose.pose.pose.position.x, 4)
        self.y = round(self.pose.pose.pose.position.y, 4)
        rot_q = self.pose.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        if self.theta<0:
            self.theta = self.theta + 2*pi
        
    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.person_x - self.x), 2) +
                    pow((self.person_y - self.y), 2))

    def linear_vel(self, constant=0.25):
        return constant * self.euclidean_distance()

    def steering_angle(self):
        angular = atan2(self.person_y - self.y, self.person_x - self.x)
        if angular < 0:
            angular = angular + 2*pi
        return angular

    def angular_vel(self, constant=0.4):
        return constant * (self.steering_angle() - self.theta)

    def person(self, msg):

        self.person = msg
    #    print(len(msg.people))
        if len(self.person.people) != 0:
            for index in range(len(self.person.people)):
                if self.person.people[index].id == self.follow_id:
                    self.person_cnt = index
                    break
                else:
                    self.person_cnt = -1
        else:
            self.person_cnt = -1   

        if self.person_cnt != -1:
            self.person_x = self.person.people[self.person_cnt].pose.position.x
            self.person_y = self.person.people[self.person_cnt].pose.position.y
            
        
    def move2goal(self):
        """Moves the turtle to the goal."""
        

        self.follow_id = input("Set follow_id: ")
        vel_msg = Twist()

        while not rospy.is_shutdown():
            if self.person_cnt !=-1:
                while self.euclidean_distance() >= 0.6 or self.euclidean_distance() <= 0.5 :
                    
                    # Porportional controller.
                    # https://en.wikipedia.org/wiki/Proportional_control
                    if self.person_cnt != -1:
                    # Linear velocity in the x-axis.
                        #print(self.steering_angle(),self.theta)
                        if self.euclidean_distance()>0.6:
                            vel_msg.linear.x = self.linear_vel()
                        elif self.euclidean_distance() < 0.5:
                            vel_msg.linear.x = -self.linear_vel()-0.1
                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0

                        # Angular velocity in the z-axis.
                        vel_msg.angular.x = 0
                        vel_msg.angular.y = 0
                        vel_msg.angular.z = self.angular_vel()
                        self.count += 1
                        
                        # Publishing our vel_msg
                        self.velocity_publisher.publish(vel_msg)
                        self.collect_data.append(["pose",self.x ,self.y]) 
                        self.collect_per.append(["person",self.person_x ,self.person_y])   
                        # Publish at the desired rate.
                        if self.count ==10:
                            print(["pose",self.x ,self.y],["person",self.person_x ,self.person_y])
                            self.count = 0
                        self.rate.sleep()
                        
                    else:
                        break
                with open('/home/philong/my_project/data_follow.yaml', 'w') as yaml_file:            
                            yaml.dump([[self.collect_data,self.collect_per]], yaml_file, default_flow_style=False)
                # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
            
            

if __name__ == '__main__':
    try:
        x = My_Robot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
