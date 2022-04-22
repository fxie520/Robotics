#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "std_msgs/Float32.h"

class Obj2SensorController {
    protected:
        ros::Subscriber scan_sub_;
        // ros::Publisher marker_pub_;
        ros::Publisher min_distance_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        // std::string base_frame_;
        // double max_range_;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            // Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // No need to transform frame as the default frame for point cloud is the kision sensor frame

            unsigned int n = temp.size();
            float min_z = 10; 
            for (unsigned int i=0;i<n;i++) {
				float z = temp[i].z;
				if (z < min_z) {
					min_z = z;
				}
            }
            // ROS_INFO("Min point distance: %f", min_z);
            std_msgs::Float32 msg1;
            msg1.data = min_z;
            min_distance_pub_.publish(msg1);

			/*
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
            */
        }

    public:
        Obj2SensorController() : nh_("~") {  // Constructor
            // nh_.param("base_frame",base_frame_,std::string("/body"));
            // nh_.param("max_range",max_range_,1.2);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Subscribe to the point cloud and prepare the marker publisher
            scan_sub_ = nh_.subscribe("scans",1,&Obj2SensorController::pc_callback,this);
            // marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);
            min_distance_pub_ = nh_.advertise<std_msgs::Float32>("min_distance",1);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"obj2_sensor_controller");
    Obj2SensorController fp;

    ros::spin();
    return 0;
}


