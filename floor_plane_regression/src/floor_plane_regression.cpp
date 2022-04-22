#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include "std_msgs/Float64MultiArray.h"
#include <vector>

class FloorPlaneRegression {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        ros::Publisher plane_param_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            // Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);  // Gives lastpc_ values

            unsigned int n = temp.size();
            double min_d = 100;
            std::vector<size_t> pidx;  // std::size_t is the unsigned integer type of the result of the sizeof operator
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);  // hypot(x,y)=sqrt(x2+y2)
                // In the sensor frame, this point would be inside the camera
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;  // The continue statement breaks one iteration (in the loop), if a specified condition occurs, and continues with the next iteration in the loop.
                }
                // Measure the point distance in the base frame
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
                // If we reach this stage, we have an acceptable point, so let's store it
                pidx.push_back(i);
            }
            n = pidx.size();  // n is the number of useful point in the point cloud
            
            // TODO START 

			// ros::WallTime begin = ros::WallTime::now();
            // Eigen is a matrix library. The line below create a 3x3 matrix A, and a 3x1 vector B
            // MatrixXf: Matrix<float,Dynamic nb of row,Dynamic nb of col>
            Eigen::MatrixXf A(3,3);  // A = phi^T*phi
            Eigen::MatrixXf B(3,1);  // B = phi^T*Z
            Eigen::MatrixXf phi(n,3);
            Eigen::MatrixXf Z(n,1);
            for (unsigned int i=0;i<n;i++) {
                // Assign x,y,z to the coordinates of the point we are considering.
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;
                double z = lastpc_[pidx[i]].z;

				phi(i,0) = x;
				phi(i,1) = y;
				phi(i,2) = 1;
				Z(i,0) = z;
            }
            A = phi.transpose() * phi;
            B = phi.transpose() * Z;
            Eigen::MatrixXf X = A.ldlt().solve(B);  // Solve Ax = B
            
            // Assuming the result is computed in vector X
            // ROS_INFO("Extracted floor plane: z = %fx + %fy + %f", X(0),X(1),X(2));
            // ros::WallDuration duration = ros::WallTime::now() - begin;
            // double secs = duration.toSec();
            // ROS_INFO("Time consumed: %f seconds", secs);
            
            double detected_distance_x = (max_range_ + min_d)/2;
            tf::Vector3 pt_pos_robot_frame(detected_distance_x, 0, -0.31);  // 0.31 is the height of the sensor
			// Tranfrom mean position of useful points to "world" frame
			tf::StampedTransform transform;
			listener_.lookupTransform("world", "bubbleRob", ros::Time(0), transform);
			tf::Vector3 pt_pos_world_frame = transform*pt_pos_robot_frame;	
			// ROS_INFO("Mean position of useful points in the world frame: [%f, %f, %f]", pt_pos_world_frame[0], pt_pos_world_frame[1], pt_pos_world_frame[2]);
			
            std::vector<double> X_vector{X(0),X(1),X(2),pt_pos_world_frame[0],pt_pos_world_frame[1]};
            std_msgs::Float64MultiArray msg1;
            msg1.data = X_vector;
            plane_param_pub_.publish(msg1);

            // END OF TODO

            // Now build an orientation vector to display a marker in rviz
            // First we build a basis of the plane normal to its normal vector
            Eigen::Vector3f O,u,v,w;
            w << X(0), X(1), -1.0;  // Normal vector of the plane
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X(0)+0.0*X(1)+X(2);
            u << 2.0, 0.0, 2.0*X(0)+0.0*X(1)+X(2);
            u -= O;
            u /= u.norm();
            v = w.cross(u);  // Cross product
            // Then we build a rotation matrix out of it
            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            // And convert it to a quaternion
            tf::Quaternion Q;
            R.getRotation(Q);  // Get the pose of the floor plane relative to the base frame?
            
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
            // Finally publish the marker
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
        FloorPlaneRegression() : nh_("~") {  // Constructor
            // TODO START
            // The parameter below described the frame in which the point cloud
            // must be projected to be estimated. You need to understand TF
            // enough to find the correct value to update in the launch file
            nh_.param("base_frame",base_frame_,std::string("/body"));
            // This parameter defines the maximum range at which we want to
            // consider points. Experiment with the value in the launch file to
            // find something relevant.
            nh_.param("max_range",max_range_,1.2);
            // END OF TODO

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Subscribe to the point cloud and prepare the marker publisher
            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRegression::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);
            plane_param_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("floor_plane_param",1);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_regression");
    FloorPlaneRegression fp;

    ros::spin();
    return 0;
}


