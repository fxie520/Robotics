#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <math.h> // For the "floor" function
#include "std_msgs/Float64MultiArray.h"
#include <vector>


class FloorPlaneMapping {
    protected:
        ros::Subscriber floor_param_sub_;
        ros::Publisher map_img_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        cv::Mat_<uint8_t> map_img;
        cv_bridge::CvImage map_img_cv;
		sensor_msgs::Image map_img_msg;
		std_msgs::Header header;

        int n_x, n_y;
        double x_bin_size, y_bin_size;
        double x_min, x_max, y_min, y_max;
        double angle_threshold;

    protected: // ROS Callback

        void param_callback(const std_msgs::Float64MultiArray msg) {
			std::vector<double> plane_param = msg.data; 
			// ROS_INFO("Plane parameters: a=%.2f, b=%.2f, c=%.2f", plane_param[0], plane_param[1], plane_param[2]);
			// ROS_INFO("Plane position: x=%.2f, y=%.2f", plane_param[3], plane_param[4]);
			
			// Quantization (which square the plane belongs to):
			int pos_i = std::floor((y_max - plane_param[4])/y_bin_size);
			int pos_j = std::floor((plane_param[3] - x_min)/x_bin_size);
			// ROS_INFO("Pos i = %d, j = %d", pos_i, pos_j);
			
			// Traversability (angle < angle_threshold degrees):
			if ((pos_i <= n_y - 1) and (pos_i >= 0) and (pos_j <= n_x - 1) and (pos_j >= 0)) {
				double angle_with_ground = acos(1/sqrt(plane_param[0]*plane_param[0] + plane_param[1]*plane_param[1] + 1)); // Angle computed in robot frame, in radian
				ROS_INFO("Angle from horizontal=%.2f", angle_with_ground*180/2/M_PI);
				if (angle_with_ground*180/2/M_PI < angle_threshold) {  // Traversable
					map_img(pos_i, pos_j) = 128;  
				}
				else {  // Non traversable
					map_img(pos_i, pos_j) = 255;
				}
				
				map_img_cv = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, map_img);
				map_img_cv.toImageMsg(map_img_msg);
				map_img_pub_.publish(map_img_msg);
			}	
        }

    public:
        FloorPlaneMapping() : nh_("~") {  // Constructor
            nh_.param("n_x",n_x,10);
            nh_.param("x_min",x_min,-5.0);
            nh_.param("x_max",x_max,+5.0);
            nh_.param("n_y",n_y,10);
            nh_.param("y_min",y_min,-5.0);
            nh_.param("y_max",y_max,+5.0);
            nh_.param("angle_threshold",angle_threshold,6.0);
            
            x_bin_size = (x_max - x_min)/n_x;
            y_bin_size = (y_max - y_min)/n_y;

			// Initialize map image to zeros (i.e. unknown/unobserved)
            map_img = cv::Mat::zeros(n_y, n_x, CV_8U);  

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            floor_param_sub_ = nh_.subscribe("floor_param",1,&FloorPlaneMapping::param_callback,this);
            map_img_pub_ = nh_.advertise<sensor_msgs::Image>("map_img",1);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_mapping");
    FloorPlaneMapping fpm;

    ros::spin();
    return 0;
}


