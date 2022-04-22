#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
import numpy as np

class Obj3_Drive:
    def __init__(self):
        self.model_dir_ = rospy.get_param("~model_dir")
        self.linear_vel_ = rospy.get_param('~linear_vel')
        self.twist_factor_ = rospy.get_param('~twist_factor')
        
        self.sess_ = None
        self.load_model()
        self.bridge = CvBridge()
        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        self.twist_pub_ = rospy.Publisher("~twist", Twist, queue_size=1)

    def load_model(self):
        # Loads the model
        self.model = tf.keras.models.load_model(self.model_dir_, compile=False)  # Only run the model -> no need to compile

    def image_callback(self, data):
        # Image call back and resize to proper shape (32*32). 
        # Regularization is done directly inside the model so we don't have to do it.
        raw = self.bridge.imgmsg_to_cv2(data,"bgr8")
        processed_ = np.expand_dims(cv2.resize(raw, (0,0), fx=32.0/data.height, fy=32.0/data.width, interpolation=cv2.INTER_AREA), axis=0).astype(np.single)
        self.twist_pub_.publish(self.image_to_rot(processed_))

    def image_to_rot(self, img):
        # Reads the image, feed it to the network, get the predictions and act on it.
        out = Twist()
        # Runs the network
        res = self.model(img, training=False)[0].numpy()
        # assert(res.shape[0] == 3)  # Makes sure that the shape of the network matches the required shape
        # print("Left prob = %5.2f, straight = %5.2f, right = %5.2f" %(res[0],res[1],res[2]))
        
        if res[0] > res[2] + 0.1:
           out.angular.z = self.twist_factor_  # Turn left
        elif res[2] > res[0] + 0.1:
           out.angular.z = -self.twist_factor_  # Turn right

        out.linear.x = 2*self.linear_vel_  # Go straight
        return out

if __name__ == '__main__':
        rospy.init_node('obj3_drive', anonymous=True)
        fp = Obj3_Drive()
        rospy.spin()

