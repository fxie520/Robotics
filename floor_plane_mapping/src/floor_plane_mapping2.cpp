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


class FloorPlaneMapping2 {
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
        double R; // Constant variance of the measurements
        
        // state_tensor: dim 1&2 is space x, y dimensions, dim 3 is (k, Pk, Xk)
        std::vector<std::vector<std::vector<double>>> state_tensor;

    protected: // ROS Callback

        void param_callback(const std_msgs::Float64MultiArray msg) {
			std::vector<double> plane_param = msg.data; 
			
			// Quantization (which square the plane belongs to):
			int pos_i = std::floor((y_max - plane_param[4])/y_bin_size);
			int pos_j = std::floor((plane_param[3] - x_min)/x_bin_size);
			
			// If detected area is in the area to consider
			if ((pos_i <= n_y - 1) and (pos_i >= 0) and (pos_j <= n_x - 1) and (pos_j >= 0)) {\
				// Compute angle_with_ground
				double angle_with_ground = acos(1/sqrt(plane_param[0]*plane_param[0] + plane_param[1]*plane_param[1] + 1)); // Angle computed in robot frame, in radian
				angle_with_ground *= 180/2/M_PI; // Transform from radian to degrees
				ROS_INFO("Angle from horizontal=%.2f", angle_with_ground);
				
				// Update state_tensor
				// ROS_INFO("state_tensor[pos_i][pos_j][1]=%.3f, R=%.3f", state_tensor[pos_i][pos_j][1], R);
				
				double K = std::max(state_tensor[pos_i][pos_j][1]/(state_tensor[pos_i][pos_j][1] + R), 1e-9);
				double Pk = std::max((1 - K)*state_tensor[pos_i][pos_j][1], 1e-9);
				double Xk = state_tensor[pos_i][pos_j][2] + K*(angle_with_ground - state_tensor[pos_i][pos_j][2]);
				ROS_INFO("K=%.2f, Pk=%f, Xk=%.2f", K, Pk, Xk);
				std::vector<double> new_state_vector{K, Pk, Xk};
				state_tensor[pos_i][pos_j] = new_state_vector;
				
				// Update map_img
				map_img(pos_i, pos_j) = (int)std::round(Xk*255/45);
				
				map_img_cv = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, map_img);
				map_img_cv.toImageMsg(map_img_msg);
				map_img_pub_.publish(map_img_msg);
			}	
        }

    public:
        FloorPlaneMapping2() : nh_("~") {  // Constructor
            nh_.param("n_x",n_x,10);
            nh_.param("x_min",x_min,-4.5);
            nh_.param("x_max",x_max,+4.5);
            nh_.param("n_y",n_y,10);
            nh_.param("y_min",y_min,-4.5);
            nh_.param("y_max",y_max,+4.5);
            
            x_bin_size = (x_max - x_min)/n_x;
            y_bin_size = (y_max - y_min)/n_y;

			// Initialize map image to zeros (i.e. unknown/unobserved)
            map_img = cv::Mat::zeros(n_y, n_x, CV_8U);  

            // Make sure TF is ready
            ros::Duration(0.5).sleep();
            
            // Initialize state_tensor
            state_tensor.resize(n_x);
            for (int ix=0;ix<n_x;ix++) {
				state_tensor[ix].resize(n_y);
				for (int iy=0;iy<n_y;iy++) {
					state_tensor[ix][iy].resize(3);
					state_tensor[ix][iy][0] = 0;  // Gain k
					state_tensor[ix][iy][1] = 1;  // Variance of initial belief of angle (between ground plane and horizontal plane)
					state_tensor[ix][iy][2] = 0;  // Mean of initial belief of angle
				}
			}
			
			R = 0.01;

            floor_param_sub_ = nh_.subscribe("floor_param",1,&FloorPlaneMapping2::param_callback,this);
            map_img_pub_ = nh_.advertise<sensor_msgs::Image>("map_img",1);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_mapping2");
    FloorPlaneMapping2 fpm;

    ros::spin();
    return 0;
}


