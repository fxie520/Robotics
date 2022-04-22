#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
# from sensor_msgs.msg import Joy
import numpy as np

class obj2_control:

    def __init__(self):
        self.target_dist = 0.6
        self.response_factor_ = rospy.get_param('~response_factor')
        self.exp_response_factor_ = rospy.get_param('~exp_response_factor')
        
        rospy.Subscriber("~min_z", Float32, self.distance_callback, queue_size=1)
        rospy.Subscriber("~twist", Twist, self.cmd_callback, queue_size=1)
        
        self.twist_pub_ = rospy.Publisher("~twist", Twist, queue_size=1)
        self.cmd_x = Twist().linear.x
        self.cmd_y = Twist().linear.y

    # Receive the original twist command
    def cmd_callback(self, orig_cmd):
    	self.cmd_x = orig_cmd.linear.x
    	self.cmd_y = orig_cmd.linear.y
    
    # Guideline for a responsive control: large exp response factor, low (linear) response factor
    def distance_callback(self, min_distance_msg):
    	min_distance = min_distance_msg.data
    	out = Twist()
    	out.linear.x = self.cmd_x
    	out.linear.y = self.cmd_y
    	# Sensor/tool too far away from ground <-> lower arm tip, and vice versa 
    	out.linear.z = self.response_factor_*(np.exp(self.exp_response_factor_*(self.target_dist-min_distance))-1)  
    	self.twist_pub_.publish(out)
    	
    	# print(f"Distance diff = {round(self.target_dist-min_distance, 2)}")
    	# print(f"Distance diff = {round(self.target_dist-min_distance, 2)}, exp distance diff = {round(np.exp(self.exp_response_factor_*(self.target_dist-min_distance))-1, 2)}, arm tip lin upward vel = {round(out.linear.z, 2)}")

if __name__ == '__main__':
        rospy.init_node('obj2_control', anonymous=True)
        fp = obj2_control()
        rospy.spin()

