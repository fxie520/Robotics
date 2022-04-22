#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
# from sensor_msgs.msg import Joy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
import numpy as np
import cv2 as cv

class obj1_control:

    def __init__(self):
        self.target_green_ratio = 0.5
        self.response_factor_ = rospy.get_param('~response_factor')
        self.exp_response_factor_ = rospy.get_param('~exp_response_factor')
        
        rospy.Subscriber("~twist", Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("~detector", Float32, self.detector_callback, queue_size=1)
        
        self.bridge = CvBridge()
        self.twist_pub_ = rospy.Publisher("~twist", Twist, queue_size=1)
        self.cmd_y = Twist().linear.y
        self.cmd_z = Twist().linear.z

    # Receive the original twist command
    def cmd_callback(self, orig_cmd):
    	self.cmd_y = orig_cmd.linear.y
    	self.cmd_z = orig_cmd.linear.z
    	
    def detector_callback(self, value):
    	pass
    	# print(f"Metal detector value = {round(value.data, 2)}")
    
    # Guideline for a responsive control: large exp response factor, low (linear) response factor
    def image_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        img = cv.medianBlur(img, ksize=5)
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, (25,0,0), (80,255,255)) > 0
        green_pixel_ratio = np.count_nonzero(mask)/img.shape[0]/img.shape[1]
        # print(f"Green pixel ratio: {green_pixel_ratio}")

        out = Twist()
        out.linear.x = self.response_factor_*(np.exp(self.exp_response_factor_*(self.target_green_ratio-green_pixel_ratio))-1)
        # out.linear.x = 0.5 if green_pixel_ratio < self.target_green_ratio else -0.5
        out.linear.y = self.cmd_y
        out.linear.z = self.cmd_z
        self.twist_pub_.publish(out)
        
        print(f"Green pixel ratio diff = {round(self.target_green_ratio-green_pixel_ratio, 2)}, exp diff = {round(np.exp(self.exp_response_factor_*(self.target_green_ratio-green_pixel_ratio))-1, 2)}, arm tip lin outward vel = {round(out.linear.x, 2)}")
        
if __name__ == '__main__':
        rospy.init_node('obj1_control', anonymous=True)
        fp = obj1_control()
        rospy.spin()

