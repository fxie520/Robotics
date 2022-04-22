#!/usr/bin/env python3
import roslib; roslib.load_manifest('model_prediction')
import rospy
from std_msgs.msg import Float64

class ModelPrediction:
    def __init__(self):
        rospy.init_node('model_prediction')
        self.trigger = rospy.get_param("~trigger","command")
        self.command_field = rospy.get_param("~command_field","")
        self.command_coef = [float(x) for x in str(rospy.get_param("~command_coef_csv","")).split(",") if len(x)>0]
        self.command_type = rospy.get_param("~command_type","").split("/")
        if len(self.command_type)!=2:
            rospy.log_fatal("Invalid command type. Use the pkg/msg syntax")
        self.state_type = rospy.get_param("~state_type","").split("/")
        if len(self.state_type)!=2:
            rospy.log_fatal("Invalid state type. Use the pkg/msg syntax")
        exec ("from %s.msg import %s" % (self.command_type[0],self.command_type[1]))
        exec ("from %s.msg import %s" % (self.state_type[0],self.state_type[1]))
        
        self.state_field = rospy.get_param("~state_field","")
        self.state_coef = [float(x) for x in str(rospy.get_param("~state_coef_csv","")).split(",") if len(x)>0]

        if len(self.command_coef) == 0:
            self.command_coef=[0.0]
        if len(self.state_coef) == 0:
            self.state_coef=[0.0]
	self.state_coef.reverse()
	self.command_coef.reverse()
        self.command = []
        self.state = []

        exec ("self.state_sub = rospy.Subscriber('~state', %s , self.state_cb,queue_size=1)" % self.state_type[1])
        exec ("self.command_sub = rospy.Subscriber('~command', %s, self.command_cb,queue_size=1)" % self.command_type[1])
        self.pub = rospy.Publisher("~prediction",Float64,queue_size=1)


    def command_cb(self,msg):
        #print self.command
        if (self.trigger == "command") :
		if len(self.command) > len(self.command_coef):
		    self.command = self.command[-len(self.command_coef):]
		if len(self.state) > len(self.state_coef):
		    self.state = self.state[-len(self.state_coef):]
        if (self.trigger == "command") \
            and len(self.state)==len(self.state_coef) \
            and len(self.command)==len(self.command_coef):
            pred = sum([-a*x for a,x in zip(self.state_coef,self.state)],0.0) + sum([b*u for b,u in zip(self.command_coef,self.command)],0.0)
            self.pub.publish(Float64(pred))
        if len(self.command_field):
            exec ("value = float(msg.%s)" % self.command_field)
        else:
            value = float(msg)
        self.command.append(value)
        #print "command value %f len %d/%d " % (value,len(self.command),len(self.command_coef))

    def state_cb(self,msg):
        if (self.trigger == "state") :
		if len(self.command) > len(self.command_coef):
		    self.command = self.command[-len(self.command_coef):]
		if len(self.state) > len(self.state_coef):
		    self.state = self.state[-len(self.state_coef):]
        if (self.trigger == "state") \
            and len(self.state)==len(self.state_coef) \
            and len(self.command)==len(self.command_coef):
            pred = sum([-a*x for a,x in zip(self.state_coef,self.state)]) + sum([b*u for b,u in zip(self.command_coef,self.command)])
            self.pub.publish(Float64(pred))
        if len(self.state_field):
            exec ("value = float(msg.%s)" % self.state_field)
        else:
            value = float(msg)
        self.state.append(value)
        # print "state value %f len %d/%d" % (value,len(self.state),len(self.state_coef))


if __name__ == '__main__':
    try:
        P = ModelPrediction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
