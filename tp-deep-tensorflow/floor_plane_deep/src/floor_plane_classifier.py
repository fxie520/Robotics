#!/usr/bin/env python3

import rospy
import tensorflow as tf
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class FloorPlaneClassifier:
    def __init__(self):
        self.model_dir_ = rospy.get_param("~model_dir")
        self.ts_ = rospy.get_param("~thumb_size")
        self.load_model()
        self.bridge = CvBridge()

        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        self.image_pub_ = rospy.Publisher("~image_label", Image, queue_size=1)

    def load_model(self):  # Loads the model
        self.model = tf.keras.models.load_model(self.model_dir_, compile=False)  # Only run the model -> no need to compile

    def image_callback(self, data):
        # Gets the image, check that it is square and divisible by the thumbnail size
        assert(data.height == data.width)
        assert(data.height%self.ts_ == 0)
        # Convert to np array
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        # Reshape the image as a batch of thumbnails (faster processing when using batches)
        # batch = np.reshape(np.array(np.split(np.array(np.split(img,self.ts_)),self.ts_)), [-1,self.ts_,self.ts_,3]).astype(np.single)
        batch = np.array(np.split(img, 4, axis=0))
        batch = np.array(np.split(batch, 4, axis=2))
        batch = np.transpose(batch, axes=(1,0,2,3,4)).reshape((16,32,32,3)).astype(np.single)
        # Calls the network
        checked = self.check_thumb(batch)
        # Transforms the array into low resolution image (on pixel per thumbnail)
        # low_res = np.reshape(checked,[data.height//self.ts_,data.width//self.ts_,3])
        low_res = checked.reshape((4,4,3))
        # Upsamples the predictions so they have the same size as the input image
        # classified = cv2.resize(low_res,(0,0),fx=self.ts_,fy=self.ts_, interpolation=cv2.INTER_NEAREST).astype(np.uint8)
        classified = cv2.resize(low_res,dsize=(128, 128), interpolation=cv2.INTER_NEAREST).astype(np.uint8)
        overlay = cv2.addWeighted(img, 0.5, classified, 0.5, 0)
        # Publish the result
        enc = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        self.image_pub_.publish(enc)

    def check_thumb(self, batch):
        # Run the network
        res = self.model(batch, training=False)  # res (output) shape: (batch size, num_classes)
        # Makes sure that the output has the proper shape
        assert(res.shape[1] == 2)
        # TODO : Use network output to determine traversability (idealy levaraging vector-wise operation in numpy)
        # Returns a numpy array of dimension [res.shape[0],3] (3 for b, g, r)
        argm = np.argmax(res, axis=1)
        array = np.zeros((res.shape[0],3))
        # Map traversability to the g channel, untraversability to the r channel, the b channel stays zero
        array[np.arange(res.shape[0]), 2-argm] = 255
        return array
           
if __name__ == '__main__':
    rospy.init_node('floor_plane_classifier')
    fp = FloorPlaneClassifier()
    rospy.spin()
