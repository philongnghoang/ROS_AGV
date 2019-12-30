#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int16, Int64,Float32, String
import yaml
class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("-----------------Start--------------- ")
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        
    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	    # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

class Nav_goal():
    def __init__(self):
        self.obj_detect = [' ']
        # Read information from yaml file
        with open("/home/philong/my_project/src/obj_detect/src/param_yaml/final_goal.yaml", 'r') as stream:
            self.dataMap = yaml.load(stream)
    
        rospy.Subscriber('result', String, self.result_callback)
        rospy.init_node('test', anonymous=False)
        self.result = []
        while not rospy.is_shutdown():
            
            for idex,ob in enumerate(self.dataMap):
                if rospy.is_shutdown():
                    break
                
                if ob['name_object'].split('_')[0] == self.obj_detect[0]: 
                    self.get_dict(ob['name_object'],ob['position'],ob['quaternion'])
                    #print(self.get_result())
                    #main(self.get_result())
                    #rospy.loginfo("-----------------Finish--------------- ")
                    #rospy.loginfo("Please show me new object")
                    #break
                else: 
                    #self.get_result() = []
                    pass    
        
            if self.result != []:
                main(self.get_result())
                rospy.loginfo("-----------------Finish--------------- ")
                rospy.loginfo("Please show me new object")  
                self.result = []  
                
    def get_dict(self,name_obj,positon_obj,quaternion_obj):
        self.dict_obj_pos = dict()
        self.dict_obj_pos['name_object'] = name_obj
        self.dict_obj_pos['position'] = positon_obj 
        self.dict_obj_pos['quaternion'] = quaternion_obj
        self.result.append(self.dict_obj_pos)
        
    def get_result(self): 
        return self.result

    def result_callback(self,msg):
        obj = msg.data
        self.obj_detect = obj.split(',')
        #print(self.obj_detect[0])
def main(dataMap):
    try:
        #rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        for obj in dataMap:    
            if rospy.is_shutdown():
                break

            name = obj['name_object']
            # Customize the following values so they are appropriate for your location
            position = obj['position']
            quaternion = obj['quaternion']
            rospy.loginfo("Go to %s pose", name)
            rospy.loginfo("Goal: (%s, %s) ", position['x'], position['y'])
            success = navigator.goto(position, quaternion)

            if success:
                rospy.loginfo("Hooray, reached the desired pose")
            else:
                rospy.loginfo("The base failed to reach the desired pose")
            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)
            #break
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")     
if __name__ == '__main__':
    my_obj = Nav_goal()
    #print(my_obj.get_result())
    #with open('/home/philong/my_project/src/obj_detect/src/param_yaml/final.yaml', 'w') as yaml_file:            
    #    yaml.dump(my_obj.get_result(), yaml_file, default_flow_style=False)
    