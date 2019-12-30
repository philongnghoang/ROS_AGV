import rospy 
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf
rospy.init_node('odom_pub')

odom_pub = rospy.Publisher('/my_odom',Odometry)

odom_broadcaster = tf.TransformBroadcaster()