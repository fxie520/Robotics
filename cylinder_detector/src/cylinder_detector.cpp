#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <cmath>  // For abs function


class CylinderDetector {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        double tolerance;
        double min_percent_inliers;
        int n_samples; 

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        
        std::vector<std::vector<double>> cylinder_array;
        
        tf::Matrix3x3 R;
        tf::Quaternion Q;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
			visualization_msgs::MarkerArray ma; 
			for (int i = 0; i < (int)cylinder_array.size(); i++) {
				visualization_msgs::Marker m;
				m.header.stamp = msg->header.stamp;
				m.header.frame_id = "world";
				m.ns = "cylinder_extracted";
				m.id = i;
				m.type = visualization_msgs::Marker::CYLINDER;
				m.action = visualization_msgs::Marker::ADD;
				m.pose.position.x = cylinder_array[i][0];
				m.pose.position.y = cylinder_array[i][1];
				m.pose.position.z = 0;
				tf::quaternionTFToMsg(Q,m.pose.orientation);
				m.scale.x = 2*cylinder_array[i][2];
				m.scale.y = 2*cylinder_array[i][2];
				m.scale.z = 2;
				m.color.a = 0.5;
				m.color.r = 1.0;
				m.color.g = 0.0;
				m.color.b = 1.0;
				ma.markers.push_back(m);
			}
			marker_pub_.publish(ma);
			
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);  // Input pc: temp; output pc: lastpc_ (which is in the base frame)

            unsigned int n = temp.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
				float z = lastpc_[i].z;
				if (z < 0.2 - 0.30) {  // 0.30 is the height of the robot
					// Potential ground point, ignore
					continue;
				}
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }
            n = pidx.size();
            // ROS_INFO("n = %d",n);
            
            // Finding cylinders in the x,y plane described by X[3] = (xc, yc, r) using RANSAC
            std::vector<double> X = {0,0,0};
            unsigned int max_nb_inliers = 0;
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {           
                if (n <= 100) {ROS_INFO("Only %d useful points, pass",n); return;}
                
				// Select 3 different random numbers in [0, n-1]
				size_t idx1 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);  // size_t: long unsigned int
				size_t idx2 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				while (idx2 == idx1) {
					idx2 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				}
				size_t idx3 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				while ((idx3 == idx1) or (idx3 == idx2)) {
					idx3 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				}
				// ROS_INFO("Indices of the 3 different points selected: %d, %d, %d", (int)idx1, (int)idx2, (int)idx3);
			    
			    // Calculate cylinder with the 3 points selected
			    double x1 = lastpc_[pidx[idx1]].x, y1 = lastpc_[pidx[idx1]].y;
			    double x2 = lastpc_[pidx[idx2]].x, y2 = lastpc_[pidx[idx2]].y;
			    double x3 = lastpc_[pidx[idx3]].x, y3 = lastpc_[pidx[idx3]].y;
			    double A = x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2;
			    // A = 0 (EPS for double: 2.22045e-16) means colinear points (cannot identify cylinder) 
			    if (fabs(A) < std::numeric_limits<double>::epsilon()) {continue;}
			    double B = (x1*x1+y1*y1)*(y3-y2) + (x2*x2+y2*y2)*(y1-y3) + (x3*x3+y3*y3)*(y2-y1);
			    double C = (x1*x1+y1*y1)*(x2-x3) + (x2*x2+y2*y2)*(x3-x1) + (x3*x3+y3*y3)*(x1-x2);
			    double D = (x1*x1+y1*y1)*(x3*y2-x2*y3) + (x2*x2+y2*y2)*(x1*y3-x3*y1) + (x3*x3+y3*y3)*(x2*y1-x1*y2);
			    double xc = -B/2/A;
			    if ((xc < 0) or (xc > 5)) {continue;}
			    double yc = -C/2/A;
			    if ((yc < -3) or (yc > 3)) {continue;}
			    double r = sqrt((B*B+C*C-4*A*D)/4/A/A);
			    if ((r > 0.25) or (r < 0.05)) {continue;}
			    
			    // Calculate number of inliers
			    unsigned int nb_inliers = 0;
			    for (unsigned int j=0; j<n; j++) {  // For each useful point
					double x = lastpc_[pidx[j]].x;
					double y = lastpc_[pidx[j]].y;
					if (fabs(hypot(x-xc,y-yc)-r) < tolerance) {
						nb_inliers += 1;
					}
				}
				
				if (nb_inliers > max_nb_inliers) {
					X[0] = xc;
					X[1] = yc;
					X[2] = r;
					max_nb_inliers = nb_inliers;
					// ROS_INFO("i = %d, max_nb_inliers = %d",i,max_nb_inliers);
				}
            }
            
            if ((X[0] < 0) or (X[0] > 5)) {return;}
			if ((X[1] < -3) or (X[1] > 3)) {return;}
			if ((X[2] > 0.14) or (X[2] < 0.08)) {return;}
            
            // Minimun percentage of inliers for cylinder extraction to be considered successful
            if (max_nb_inliers/(double)n < min_percent_inliers) {ROS_INFO("Best percentage of inliers is only %.2f, no cylinder detected",max_nb_inliers/(double)n); return;}
			
			// Transform cylinder to world frame
			tf::Vector3 cylinder_robot_frame(X[0], X[1], 0);
			tf::StampedTransform transform;
			listener_.lookupTransform("world", "bubbleRob", ros::Time(0), transform);
			tf::Vector3 cylinder_world_frame = transform*cylinder_robot_frame;	
			X[0] = cylinder_world_frame[0];
			X[1] = cylinder_world_frame[1];
			
			// Check if cylinder detected is a repetition
			for (int i = 0; i < (int)cylinder_array.size(); i++) {
				double dx = cylinder_array[i][0] - X[0];
				double dy = cylinder_array[i][1] - X[1];
				if (hypot(dx, dy) < 0.5) {ROS_INFO("hypot(dx, dy) = %.2f repetition!",hypot(dx, dy));return;}
			}
			
			// At this point we are sure a new cylinder was just detected
			cylinder_array.push_back(X);
			
			for (int i = 0; i < (int)cylinder_array.size(); i++)
			{
				for (int j = 0; j < (int)cylinder_array[i].size(); j++)
				{
					std::cout << cylinder_array[i][j] << " ";
				}   
				std::cout << std::endl;
			}
        }

    public:
        CylinderDetector() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,2.0);
            nh_.param("n_samples",n_samples,100);
            nh_.param("tolerance",tolerance,0.001);
            nh_.param("min_percent_inliers",min_percent_inliers,0.5);

            ROS_INFO("Searching for cylinder parameters xc, yc, r");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);
            
            tf::Matrix3x3 R(1,0,0,
							0,1,0,
							0,0,1);
            R.getRotation(Q);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&CylinderDetector::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cylinder_array",1);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"cylinder_detector");
    CylinderDetector cd;

    ros::spin();
    return 0;
}


