#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // Includes the twist msg so that we can publish twist commands to the turtle 
#include <sensor_msgs/Joy.h>  // Includes the joystick msg so that we can listen to the joy topic 

class TeleopRobot
{
public:
  TeleopRobot();  // Constructor

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;  // Used to define which axes of the joystick will control our robot
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

// Write the constructor
TeleopRobot::TeleopRobot():  // The 2 lines below: initialization list used to initialize member variables before the body of the constructor executes
  linear_(1),
  angular_(2),
  a_scale_(1),
  l_scale_(1)
{
  // Assign value from parameter server, with default. 
  nh_.param("axis_linear", linear_, linear_);  
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("vrep/twistCommand", 1);  // A publisher that will advertise on the command_velocity topic of the robot
  // vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/sim_ros_interface/drone/cmd_vel", 1);  // To control the drone

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRobot::joyCallback, this);  // Subscribe to the joystick topic for the input to drive the robot
}

void TeleopRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_robot");
  TeleopRobot teleop_robot;

  ros::spin();
}
