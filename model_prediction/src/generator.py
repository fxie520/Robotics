#!/usr/bin/env python3
import roslib; roslib.load_manifest('model_prediction')
import rospy
from math import sin,cos
from std_msgs.msg import Float64

rospy.init_node('generator')
spub = rospy.Publisher("/state",Float64,queue_size=1)
cpub = rospy.Publisher("/command",Float64,queue_size=1)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    spub.publish(Float64(sin(rospy.Time.now().to_sec())));
    cpub.publish(Float64(cos(rospy.Time.now().to_sec())));
    rate.sleep()

