#!/usr/bin/env python3
import roslib; roslib.load_manifest('model_prediction')
import rospy
from std_msgs.msg import Float64

class ModelPrediction:
    def __init__(self):
        rospy.init_node('model_prediction')
        self.rate_param = rospy.get_param("~rate",10.0)
        self.command_field = rospy.get_param("~command_field","")
        self.command_coef = [float(x) for x in rospy.get_param("~command_coef_csv","").split(",") if len(x)>0]
        self.command_type = rospy.get_param("~command_type","").split("/")
        if len(self.command_type)!=2:
            rospy.log_fatal("Invalid command type. Use the pkg/msg syntax")
        self.state_type = rospy.get_param("~state_type","").split("/")
        if len(self.state_type)!=2:
            rospy.log_fatal("Invalid state type. Use the pkg/msg syntax")
        exec ("from %s.msg import %s" % (self.command_type[0],self.command_type[1]))
        exec ("from %s.msg import %s" % (self.state_type[0],self.state_type[1]))
        
        self.state_field = rospy.get_param("~state_field","")
        self.state_coef = [float(x) for x in rospy.get_param("~state_coef_csv","").split(",") if len(x)>0]

        if len(self.command_coef) == 0:
            self.command_coef=[0.0]
        if len(self.state_coef) == 0:
            self.state_coef=[0.0]
        self.command = None
        self.state = None

        # exec ("self.state_sub = rospy.Subscriber('~state', %s , self.state_cb, queue_size=1)" % self.state_type[1])
        exec ("self.state_sub = rospy.Subscriber('~/sim_ros_interface/drone/out_vel', %s , self.state_cb, queue_size=1)" % self.state_type[1])
        # exec ("self.command_sub = rospy.Subscriber('~command', %s, self.command_cb,queue_size=1)" % self.command_type[1])
        exec ("self.command_sub = rospy.Subscriber('~/sim_ros_interface/drone/cmd_vel', %s, self.command_cb, queue_size=1)" % self.command_type[1])
        self.pub = rospy.Publisher("~prediction", Float64, queue_size=1)
        self.pub2 = rospy.Publisher("~reality", Float64, queue_size=1)


    def command_cb(self,msg):
        if len(self.command_field):
            value = eval("float(msg.%s)" % self.command_field) 
            #exec ("value = float(msg.%s)" % self.command_field)  # Does not work in Python 3
        else:
            value = float(msg)
        self.command = value

    def state_cb(self,msg):
        if len(self.state_field):
            value = eval("float(msg.%s)" % self.state_field)
            # exec ("value = float(msg.%s)" % self.state_field)  # Does not work in Python 3
        else:
            value = float(msg)
        self.state = value

    def run(self):
        rate = rospy.Rate(self.rate_param)
        x = []
        u = []
        while not rospy.is_shutdown():
            if not self.state is None:
                x.append(self.state)
                if len(x) > len(self.state_coef):
                    x = x[-len(self.state_coef):]
            if not self.command is None:
                u.append(self.command)
                if len(u) > len(self.command_coef):
                    u = u[-len(self.command_coef):]
            if len(x)==len(self.state_coef) and len(u)==len(self.command_coef):
                # print zip(self.state_coef,x)
                # print zip(self.command_coef,u)
            
                pred = sum([-a*xi for a,xi in zip(self.state_coef,x)]) + sum([b*ui for b,ui in zip(self.command_coef,u)])
                self.pub.publish(Float64(pred))
                self.pub2.publish(self.state)
            rate.sleep()


if __name__ == '__main__':
    try:
        P = ModelPrediction()
        P.run()
    except rospy.ROSInterruptException:
        pass
