import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped , PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
from models.image.imagenet import classify_image
import yaml 

class RosTensorFlow():
    def __init__(self):
        classify_image.maybe_download_and_extract()
        self._session = tf.Session()
        classify_image.create_graph()
        self._cv_bridge = CvBridge()

        self._sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('result', String, queue_size=1)
        self.score_threshold = rospy.get_param('~score_threshold', 0.1)
        self.use_top_k = rospy.get_param('~use_top_k', 5)

        self.rate = rospy.get_param('~rate',10)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.ori_x = 0.0
        self.ori_y = 0.0
        self.ori_z = 0.0
        self.ori_w = 0.0
        self.result = []
        self.sub_pos = rospy.Subscriber('/poseupdate',PoseWithCovarianceStamped, self.callback_pos, queue_size=1)
        self.list_obj = ["water bottle","mouse"]
    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # copy from
        # https://github.com/tensorflow/tensorflow/blob/master/tensorflow/models/image/imagenet/classify_image.py
        image_data = cv2.imencode('.jpg', cv_image)[1].tostring()
        
        # Creates graph from saved GraphDef.
        softmax_tensor = self._session.graph.get_tensor_by_name('softmax:0')
        predictions = self._session.run(softmax_tensor, {'DecodeJpeg/contents:0': image_data})
        predictions = np.squeeze(predictions)
        # Creates node ID --> English string lookup.
        node_lookup = classify_image.NodeLookup()
        top_k = predictions.argsort()[-self.use_top_k:][::-1]

        node_id = top_k[0]
        human_string = node_lookup.id_to_string(node_id)
        score = predictions[node_id]
        
        if score > self.score_threshold:
            rospy.loginfo('%s (score = %.5f)' % (human_string, score))
            
            if human_string.split(',')[0] in self.list_obj:
                #self._pub.publish(human_string)
                #self.sub_pos = rospy.Subscriber('/poseupdate',PoseWithCovarianceStamped, self.callback_pos, queue_size=1)
                #print("Position x :",self.pos_x , ",Position y :" ,self.pos_y)
                res = self.get_dict(str(human_string.split(',')[0]))
                print(self.dict_obj_pos)
                print(len(res)) 
                with open('param_yaml/data_3.yaml', 'w') as yaml_file:            
                    yaml.dump(res, yaml_file, default_flow_style=False)

    def get_dict(self,name_obj):
        self.dict_obj_pos = dict()
        positon_obj = dict()
        quaternion_obj = dict()
        #Name object 
        self.dict_obj_pos['name_object'] = name_obj
        #Position 
        positon_obj['x'] = self.pos_x
        positon_obj['y'] = self.pos_y
        self.dict_obj_pos['position'] = positon_obj 
        #Quaternion
        quaternion_obj['r1'] = 0
        quaternion_obj['r2'] = 0
        quaternion_obj['r3'] = self.ori_w
        quaternion_obj['r4'] = self.ori_z

        self.dict_obj_pos['quaternion'] = quaternion_obj
        self.result.append(self.dict_obj_pos)
        return self.result
    def get_result(self):
        return self.result
    def callback_pos(self,msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.ori_x = msg.pose.pose.orientation.x
        self.ori_y = msg.pose.pose.orientation.y
        self.ori_z = msg.pose.pose.orientation.z
        self.ori_w = msg.pose.pose.orientation.w
        #print("Position x :",self.pos_x , ",Position y :" ,self.pos_y)
        #print("Qua",self.ori_x,self.ori_y,self.ori_w,self.ori_z)
        euler = euler_from_quaternion([self.ori_x, self.ori_y,self.ori_w,self.ori_z])
        #print("Euler",euler)   
    def main(self):
        rospy.spin()

class SubscribePosition(object):
    def __init__(self):
        self.rate = rospy.get_param('~rate',10)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.ori_z = 0.0
        self.ori_w = 0.0
        self.sub_pos = rospy.Subscriber('/poseupdate',PoseWithCovarianceStamped, self.callback_pos, queue_size=1)
        rospy.spin()
    def callback_pos(self,msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.ori_z = msg.pose.pose.orientation.z
        self.ori_w = msg.pose.pose.orientation.w
        print("Position x :",self.pos_x , ",Position y :" ,self.pos_y)
    # def spin(self):
    #     #############################################################################
    #     r = rospy.Rate(self.rate)
    #     while not rospy.is_shutdown():
            
    #         #r.sleep()
    #         rospy.spin()
if __name__ == '__main__':
    rospy.init_node('rostensorflow')
    tensor = RosTensorFlow()
    tensor.main()
    #my_goal = SubscribePosition()
    #my_goal.spin()