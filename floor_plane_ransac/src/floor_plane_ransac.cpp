#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <cmath>  // For abs function


class FloorPlaneRansac {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        double tolerance;
        int n_samples; 

        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            //
            unsigned int n = temp.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
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
            
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c
            // Remember to use the n_samples and the tolerance variable
            n = pidx.size();
            // size_t best = 0;
            double X[3] = {0,0,0};
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            unsigned int min_error = n;
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {
                // Implement RANSAC here. Useful commands:
                // Select a random number in [0,i-1]
                // size_t j = std::min((rand() / (double)RAND_MAX) * i,(double)i-1);
                // Create a 3D point:
                // Eigen::Vector3f P; P << x,y,z;
                // Eigen::Vector3f P; P << lastpc_[pidx[0]].x, lastpc_[pidx[0]].y, lastpc_[pidx[0]].z; 
                // Dot product
                // double x = P.dot(P);
                // Cross product
                // Eigen::Vector3f Q = P.cross(P); 
                // Vector norm
                // double norm = P.norm();
                
                // RANSAC 
                
                if (n == 0) {
					continue;  // Pass if no useful point
				}
                
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

				/*
				// Calculate the plane y = a*x + b*y + c identified by the 3 points selected (1st method)
				Eigen::MatrixXf A(3,3);
				A << lastpc_[pidx[idx1]].x, lastpc_[pidx[idx1]].y, 1,
					 lastpc_[pidx[idx2]].x, lastpc_[pidx[idx2]].y, 1,
					 lastpc_[pidx[idx3]].x, lastpc_[pidx[idx3]].y, 1;
				Eigen::MatrixXf B(3,1);
				B << lastpc_[pidx[idx1]].z,
					 lastpc_[pidx[idx2]].z,
					 lastpc_[pidx[idx3]].z;
			    Eigen::MatrixXf X_iter = A.colPivHouseholderQr().solve(B);  // Solve A*X_iter = B, where X_iter = [a; b; c]. Accuracy of linear solver is important when using this method
			    double a = X_iter(0,0);
			    double b = X_iter(1,0);
			    double c = X_iter(2,0);
			    */
			    
			    // Calculate the plane y = a*x + b*y + c identified by the 3 points selected (2nd method)
			    Eigen::Vector3d P1; P1 << lastpc_[pidx[idx1]].x, lastpc_[pidx[idx1]].y, lastpc_[pidx[idx1]].z;
			    Eigen::Vector3d P2; P2 << lastpc_[pidx[idx2]].x, lastpc_[pidx[idx2]].y, lastpc_[pidx[idx2]].z;
			    Eigen::Vector3d P3; P3 << lastpc_[pidx[idx3]].x, lastpc_[pidx[idx3]].y, lastpc_[pidx[idx3]].z;
			    Eigen::Vector3d v12 = P2 - P1, v13 = P3 - P1;
			    Eigen::Vector3d Normal = v12.cross(v13); 
			    if (Normal(2) == 0) {  // For an almost vertical plane
					Normal(2) += std::numeric_limits<double>::epsilon(); // EPS for double: 2.22045e-16
				}
			    Normal = -Normal/Normal(2);  // Scale the normal s.t. its last coeff = -1
			    double a = Normal(0), b = Normal(1);
			    double c = P1(2) - a*P1(0) - b*P1(1);
			    
			    // Calculate error
			    unsigned int error = 0;
			    double denominator = std::sqrt(a*a+b*b+1);
			    for (unsigned int j=0; j<n; j++) {  // For each useful point
					double x = lastpc_[pidx[j]].x;
					double y = lastpc_[pidx[j]].y;
					double z = lastpc_[pidx[j]].z;
					double d = std::abs(-a*x-b*y+z-c)/denominator;  // Vertical distance from a useful point to the plane just identified
					if (d > tolerance) {
						error += 1;
					}
				}
				
				// Compare error with minimum error up to now
				if (error < min_error) {
					X[0] = a;
					X[1] = b;
					X[2] = c;
					min_error = error;
					// ROS_INFO("Temporary minimun error: %.2f", min_error);
				}
            }
            ROS_INFO("Minimun error: %d", min_error);
            // At the end, make sure to store the best plane estimate in X
            // X = {a,b,c}. This will be used for display

            // END OF TODO
            ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X[0],X[1],X[2]);

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
        }

    public:
        FloorPlaneRansac() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRansac::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Ransac");
    FloorPlaneRansac fp;

    ros::spin();
    return 0;
}


