#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <math.h> // For the "floor" function
#include "std_msgs/Float64MultiArray.h"
#include <vector>


class FloorPlaneHough {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        ros::Publisher plane_param_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        cv::Mat_<uint32_t> accumulator;

        int n_a, n_b, n_c;
        double a_min, a_max, b_min, b_max, c_min, c_max;
        
        double a_bin_size; 
        double b_bin_size;
        double c_bin_size;
        double *a_quantized, *b_quantized, *c_quantized;
        
        bool verbose;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
			// Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);  // Gives lastpc_ values

            double min_d = 100;
            unsigned int n = temp.size();
            std::vector<size_t> pidx;  // std::size_t is the unsigned integer type of the result of the sizeof operator
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);  // hypot(x,y)=sqrt(x2+y2)
                // In the sensor frame, this point would be inside the camera
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                d = hypot(x,y);
                if (d < min_d) {
					min_d = d;
				}
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }
            if (verbose) {
				ROS_INFO("Mininum d after removing bogus points: %f", min_d);
			}
			
			// Approximated mean distance of useful points in front of the robot (in the robot frame)
			double detected_distance_x = (max_range_ + min_d)/2;
			if (verbose) {
				ROS_INFO("Detected distance in x direction: %f", detected_distance_x);
			}
			tf::Vector3 pt_pos_robot_frame(detected_distance_x, 0, -0.31);  // 0.31 is the height of the sensor
			// Tranfrom mean position of useful points to "world" frame
			tf::StampedTransform transform;
			listener_.lookupTransform("world", "bubbleRob", ros::Time(0), transform);
			tf::Vector3 pt_pos_world_frame = transform*pt_pos_robot_frame;	
			if (verbose) {
				ROS_INFO("Mean position of useful points in the world frame: [%f, %f, %f]", pt_pos_world_frame[0], pt_pos_world_frame[1], pt_pos_world_frame[2]);
			}
            
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c using the hough transform
            // Remember to use the a_min,a_max,n_a variables (resp. b, c).
            n = pidx.size();
            if (verbose) {
				ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
			}
            // fill the accumulator with zeros
            accumulator = 0;
            for (unsigned int i=0;i<n;i++) {
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;
                double z = lastpc_[pidx[i]].z;
                // Update the accumulator based on current point here
                // individual cells in the accumulator can be accessed as follows
                for (int i_a=0; i_a<n_a; i_a++) {
					for (int i_b=0; i_b<n_b; i_b++) {
						double c = z - a_quantized[i_a]*x - b_quantized[i_b]*y;
						if ((c >= c_min) && (c < c_max)) {
							int i_c = std::floor((c-c_min)/c_bin_size);
							assert(i_c >= 0);
							assert(i_c <= n_c);
							accumulator(i_a,i_b,i_c) += 1;
						}
					}
				}
            }

            double X[3] = {0,0,0};
            // Use the accumulator to find the best plane parameters and store
            // them in X (this will be used for display later)
            // X = {a,b,c}
            
            unsigned int current_peak = 0;
            for (int i_a=0; i_a<n_a; i_a++) {
				for (int i_b=0; i_b<n_b; i_b++) {
					for (int i_c=0; i_c<n_c; i_c++) {
						if (accumulator(i_a,i_b,i_c) > current_peak) {
							current_peak = accumulator(i_a,i_b,i_c);
							X[0] = a_quantized[i_a];
							X[1] = b_quantized[i_b];
							X[2] = c_quantized[i_c];
						}
					}
				}
			}

            // END OF TODO
            if (verbose) {
				ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f", X[0],X[1],X[2]);
			}
            std::vector<double> X_vector{X[0],X[1],X[2],pt_pos_world_frame[0],pt_pos_world_frame[1]};
            std_msgs::Float64MultiArray msg1;
            msg1.data = X_vector;
            plane_param_pub_.publish(msg1);

            Eigen::Vector3f O,u,v,w;
            w << X[0], X[1], -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf::Quaternion Q;
            R.getRotation(Q);
            
            visualization_msgs::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = base_frame_;
            m.ns = "floor_plane";
            m.id = 1;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = O(0);
            m.pose.position.y = O(1);
            m.pose.position.z = O(2);
            tf::quaternionTFToMsg(Q,m.pose.orientation);
            m.scale.x = 1.0;
            m.scale.y = 1.0;
            m.scale.z = 0.01;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;
            marker_pub_.publish(m);  
            
            // Check the position of the center of detected area in the world frame by plotting it
            visualization_msgs::Marker p;
            p.header.stamp = msg->header.stamp;
            p.header.frame_id = "world";
            p.ns = "Detect_area_center";
            p.id = 2;
            p.type = visualization_msgs::Marker::CYLINDER;
            p.action = visualization_msgs::Marker::ADD;
            p.pose.position.x = pt_pos_world_frame[0];
            p.pose.position.y = pt_pos_world_frame[1];
            p.pose.position.z = pt_pos_world_frame[2];
            tf::quaternionTFToMsg(Q,p.pose.orientation);
            p.scale.x = 0.2;
            p.scale.y = 0.2;
            p.scale.z = 0.1;
            p.color.a = 1.0;
            p.color.r = 0.0;
            p.color.g = 1.0;
            p.color.b = 0.0;
            marker_pub_.publish(p);  
        }

    public:
        FloorPlaneHough() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_a",n_a,10);
            nh_.param("a_min",a_min,-1.0);
            nh_.param("a_max",a_max,+1.0);
            nh_.param("n_b",n_b,10);
            nh_.param("b_min",b_min,-1.0);
            nh_.param("b_max",b_max,+1.0);
            nh_.param("n_c",n_c,10);
            nh_.param("c_min",c_min,-1.0);
            nh_.param("c_max",c_max,+1.0);
            nh_.param("verbose",verbose,false);

            assert(n_a > 0);
            assert(n_b > 0);
            assert(n_c > 0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("a: %d value in [%f, %f]",n_a,a_min,a_max);
            ROS_INFO("b: %d value in [%f, %f]",n_b,b_min,b_max);
            ROS_INFO("c: %d value in [%f, %f]",n_c,c_min,c_max);

            // the accumulator is created here as a 3D matrix of size n_a x n_b x n_c
            int dims[3] = {n_a,n_b,n_c};
            accumulator = cv::Mat_<uint32_t>(3,dims);
            
            // BEGIN TODO

            // You might want to add some pre-computation here to help working on the accumulator
            a_bin_size = (a_max - a_min)/n_a;
            b_bin_size = (b_max - b_min)/n_b;
            c_bin_size = (c_max - c_min)/n_c;
            ROS_INFO("Bin size of parameter a: %f", a_bin_size);
            ROS_INFO("Bin size of parameter b: %f", b_bin_size);
            ROS_INFO("Bin size of parameter c: %f", c_bin_size);
            a_quantized = new double[n_a];
            b_quantized = new double[n_b];
            c_quantized = new double[n_c];
            for (int i=0;i<n_a;i++) {
				a_quantized[i] = a_min + a_bin_size/2 + a_bin_size*i;
				// printf("%f ", a_quantized[i]);
			}
			for (int i=0;i<n_b;i++) {
				b_quantized[i] = b_min + b_bin_size/2 + b_bin_size*i;
			}
			for (int i=0;i<n_c;i++) {
				c_quantized[i] = c_min + c_bin_size/2 + c_bin_size*i;
			}

            // END TODO

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneHough::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);
			plane_param_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("floor_plane_param",1);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Hough");
    FloorPlaneHough fp;

    ros::spin();
    return 0;
}


