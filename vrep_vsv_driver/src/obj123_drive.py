#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState
import tensorflow as tf
import numpy as np
from std_msgs.msg import Float32
import cv2 as cv

class Obj123_Drive:

    def __init__(self):
        self.response_factor_1 = rospy.get_param('~response_factor_1')
        self.exp_response_factor_1 = rospy.get_param('~exp_response_factor_1')
        self.response_factor_2 = rospy.get_param('~response_factor_2')
        self.exp_response_factor_2 = rospy.get_param('~exp_response_factor_2')
        self.model_dir_ = rospy.get_param("~model_dir")
        self.linear_vel_ = rospy.get_param('~linear_vel')
        self.twist_factor_ = rospy.get_param('~twist_factor')
        self.twist_factor_correction = rospy.get_param('~twist_factor_correction')
        self.target_dist = rospy.get_param('~target_dist')
        
        self.target_green_ratio = 0.5
        self.green_pixel_ratio = 0.5
        self.bridge = CvBridge()
        self.sess_ = None
        self.load_model()
        self.bridge = CvBridge()
        self.vsv_twist = Twist()
        self.arm_twist = Twist()
        self.armfold = 0.5

        self.vsv_twist_pub_ = rospy.Publisher("~vsv_twist", Twist, queue_size=1)
        self.arm_twist_pub_ = rospy.Publisher("~arm_twist", Twist, queue_size=1)
        rospy.Subscriber("~vimage", Image, self.vimage_callback, queue_size=1)
        rospy.Subscriber("~kimage", Image, self.kimage_callback, queue_size=1)
        rospy.Subscriber("~detector", Float32, self.detector_callback, queue_size=1)
        rospy.Subscriber("~min_z", Float32, self.distance_callback, queue_size=1)
        rospy.Subscriber("~armfold", JointState, self.armfold_callback, queue_size=1)
        
    # Loads the model
    def load_model(self):
        self.model = tf.keras.models.load_model(self.model_dir_, compile=False)  # Only run the model -> no need to compile
        
    def armfold_callback(self, state):
        self.armfold = state.position[0]

    # Image call back and resize to proper shape (32*32). Regularization is done directly inside the model so we don't have to do it.
    def vimage_callback(self, data):
        raw = self.bridge.imgmsg_to_cv2(data,"bgr8")
        processed_ = np.expand_dims(cv.resize(raw, (0,0), fx=32.0/data.height, fy=32.0/data.width, interpolation=cv.INTER_AREA), axis=0).astype(np.single)
        
        res = self.model(processed_, training=False)[0].numpy()  # Runs the network
        # print("Left prob = %5.2f, straight = %5.2f, right = %5.2f" %(res[0],res[1],res[2]))
        
        self.vsv_twist.angular.z = (res[0] - res[2])*self.twist_factor_
        self.vsv_twist.linear.x = self.linear_vel_  # Go straight
        # self.pub()
        
    def kimage_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        img = cv.medianBlur(img, ksize=5)
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, (25,0,0), (80,255,255)) > 0
        self.green_pixel_ratio = np.count_nonzero(mask)/img.shape[0]/img.shape[1]

        if self.armfold < 1.0:
             self.arm_twist.linear.x = self.response_factor_1*(np.exp(self.exp_response_factor_1*(self.target_green_ratio-self.green_pixel_ratio))-1)
             # print(f"Green pixel ratio diff = {round(self.target_green_ratio-self.green_pixel_ratio, 2)}, exp diff = {round(np.exp(self.exp_response_factor_1*(self.target_green_ratio-self.green_pixel_ratio))-1, 2)}, arm tip lin outward vel = {round(self.arm_twist.linear.x, 2)}")
        else:  # Avoid truck flipping
             self.arm_twist.linear.x = 0.5
             # print("Armfold too large is dangerous, resetting arm extanding velocity to 0.5.")
             
        self.vsv_twist.angular.z -= (self.target_green_ratio-self.green_pixel_ratio)*self.twist_factor_correction   # Add correction term
        self.pub()
        
    def distance_callback(self, min_distance_msg):
        min_distance = min_distance_msg.data
        # Sensor/tool too far away from ground <-> lower arm tip, and vice versa 
        self.arm_twist.linear.z = self.response_factor_2*(np.exp(self.exp_response_factor_2*(self.target_dist-min_distance))-1)  

        # print(f"Distance diff = {round(self.target_dist-min_distance, 2)}, exp distance diff = {round(np.exp(self.exp_response_factor_2*(self.target_dist-min_distance))-1, 2)}, arm tip lin upward vel = {round(self.arm_twist.linear.z, 2)}")
        self.pub()
    
    def detector_callback(self, value):
        if value.data > 0.5:
             print(f"Treasure detected: metal detector value = {round(value.data, 2)}")
    
    def pub(self):
        self.vsv_twist_pub_.publish(self.vsv_twist)
        self.arm_twist_pub_.publish(self.arm_twist)

if __name__ == '__main__':
    rospy.init_node('obj123_drive', anonymous=True)
    fp = Obj123_Drive()
    rospy.spin()

