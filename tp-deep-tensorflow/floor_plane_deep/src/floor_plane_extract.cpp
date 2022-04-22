#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include "boost/filesystem/operations.hpp" 
#include "boost/filesystem/fstream.hpp"
#include <iostream>                        
#include <string>  // int to string

class FloorPlaneExtract {
    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

        ros::Subscriber info_sub_;
        ros::Subscriber joy_sub_;
        ros::Subscriber scan_sub_;
        tf::TransformListener listener_;

		// boost::shared_pt is a versatile tool for managing shared ownership of an object or array.
		// message_filters takes in messages and may output those messages at a later time, based on the conditions that filter needs to met.
        boost::shared_ptr< message_filters::Subscriber<sensor_msgs::Image> > imgSub;
        boost::shared_ptr< message_filters::Subscriber<sensor_msgs::PointCloud2> > pclSub;
        // typedef is used to create an additional name (alias) for another data type, but does not create a new type. It is often used to simplify the syntax of declaring complex data structures consisting of struct and union types.
        typedef message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,sensor_msgs::Image> TS;
        boost::shared_ptr<TS> sync;


        std::string base_frame_;
        std::string world_frame_;
        std::string outdir_;
        double max_range_;
        double height_threshold_;
        double min_displacement_;
        double min_rotation_;
        int thumb_size_;
        int max_image_per_type_;
        unsigned long image_counter_;
        unsigned long traversable_counter_;
        unsigned long untraversable_counter_;
        int joystick_button_;
        bool learning_;
        ros::Time last_joy_time_;

        bool has_info;
        double fx,fy,cx,cy;
        geometry_msgs::Pose2D last_pose;

		// Enumeration is a user-defined data type which can be assigned some limited values.
        typedef enum {
            UNUSABLE,
            UNTRAVERSABLE,
            TRAVERSABLE
        } ThumbType;

        ThumbType check_thumb(const cv::Mat_<cv::Vec3b> & thumb,
                const cv::Mat_<float> & thumb_z) {
            // TODO: Modify this function to take a decision on the traversability of the patch of ground corresponding to this image.
            int useful_pt_count = 0;
            for (int r=0;r<thumb_z.rows;r++) {
                for (int c=0;c<thumb_z.cols;c++) {
                    if (std::isnan(thumb_z(r,c))) {
                        // ignore this point, it has not been observed from the kinect
                        continue;
                    }
                    else if (thumb_z(r,c) > -0.25) {
						return UNTRAVERSABLE;
					}
                    else {
						useful_pt_count += 1;
						//z_total += thumb_z(r,c);
					}
                }
            }
            if (useful_pt_count/32.0/32.0 > 0.9) {
				// ROS_INFO("Useful pt percentage = %f", useful_pt_count/32.0/32.0);
				return TRAVERSABLE;
			}
			// ROS_INFO("Useful pt percentage = %f too little", useful_pt_count/32.0/32.0);
            return UNUSABLE;
        }

    protected: // ROS Callbacks

		// Toggle between learning and not learning each time a button is pressed
        void joy_callback(const sensor_msgs::JoyConstPtr msg) {  
            if ((joystick_button_<msg->buttons.size()) && (msg->buttons[joystick_button_])) {
                if ((ros::Time::now()-last_joy_time_).toSec()<0.5) {
                    // This is a bounce, ignore
                    return;
                }
                last_joy_time_ = ros::Time::now();
                learning_ = !learning_;
                if (learning_) {
                    ROS_INFO("Learning started, recording images and labels");
                } else {
                    ROS_INFO("Learning interruped");
                }
            }
        }

		// Get camera calibration parameters when they are available
        void calibration_callback(const sensor_msgs::CameraInfoConstPtr msg) {
            fx=msg->K[0]; cx=msg->K[2];
            fy=msg->K[4]; cy=msg->K[5];
            has_info = true;
        }

        void sync_callback(const sensor_msgs::PointCloud2ConstPtr pcl_msg, const sensor_msgs::ImageConstPtr img_msg) {
            if (!learning_) {
                // If we're not learning, we don't care about this image
                return;
            }
            if (!has_info) return;  // If no calibration parameters, we don't care about this image

            tf::StampedTransform tfw;
            listener_.waitForTransform(base_frame_,world_frame_,pcl_msg->header.stamp,ros::Duration(1.0));
            listener_.lookupTransform(base_frame_,world_frame_,pcl_msg->header.stamp,tfw);  // Source frame: world_frame_ ; target frame: base_frame_
            // Check if we moved. If not moved much, return. tfw.getOrigin().x(): x coordinate of bubbleRob (base_frame_)
            if ((hypot(last_pose.x-tfw.getOrigin().x(),last_pose.y-tfw.getOrigin().y())<min_displacement_) &&
                    (fabs(remainder(last_pose.theta-tf::getYaw(tfw.getRotation()),2*M_PI))<min_rotation_)) {
                return;
            }
            // Update last_pose of bubbleRob
            last_pose.x=tfw.getOrigin().x();
            last_pose.y=tfw.getOrigin().y();
            last_pose.theta=tf::getYaw(tfw.getRotation());
            

			// Create OpenCV image using message img_msg
            cv::Mat img(cv_bridge::toCvShare(img_msg,"bgr8")->image);
            // Create point clouds in the sensor and base frames using message pcl_msg
            pcl::PointCloud<pcl::PointXYZ> pc_sensor, pc_base;
            pcl::fromROSMsg(*pcl_msg, pc_sensor);
            // Make sure the point cloud is in the base_frame
            listener_.waitForTransform(base_frame_,pcl_msg->header.frame_id,pcl_msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,pcl_msg->header.stamp, pc_sensor, pcl_msg->header.frame_id, pc_base, listener_);  // Input pc: pc_sensor; output pc: pc_base (which is in the base frame)


			// First build an image of ground height
            cv::Mat_<float> img_z(img.size(),NAN);  // Initialize each position of img_z to NAN
            unsigned int n = pc_sensor.size();  // Number of points in the point cloud
            for (unsigned int i=0;i<n;i++) {
                float x = pc_sensor[i].x;
                float y = pc_sensor[i].y;
                float z = pc_sensor[i].z;
                if (z > max_range_) {  // Point cloud projection is up to 3.50 meters in front of the sensor
                    // Ignoring points too far out.
                    continue;
                }
                int ix = round(cx - x*fx/z);
                int iy = round(cy - y*fy/z);
                if ((ix < 0) || (ix >= img_z.cols) || (iy < 0) || (iy >= img_z.rows)) {
                    // Outside of the image. This is not possible, but may happen due to numerical uncertainties.
                    continue;
                }
                img_z(iy,ix) = pc_base[i].z;
            }
            cv::flip(img_z, img_z, -1);  // depth_registered/points data should be flipped !!!

			// Sampling/cropping 32*32 thumbs from each original image
            for (int r=0;r<img.rows;r+=thumb_size_) {  // thumb_size_ = 32 by default
                if (r+thumb_size_>img.rows) { continue; }
                for (int c=0;c<img.cols;c+=thumb_size_) {
                    // printf("r %d c %d\n",r,c);
                    if (c+thumb_size_>img.cols) { continue; }
                    cv::Rect roi(c,r,thumb_size_,thumb_size_);  // Template class for 2D rectangles
                    cv::Mat thumb = img(roi);  // Crop a 32*32 rectangle from image
                    cv::Mat_<float> thumb_z = img_z(roi); // Crop a 32*32 rectangle from height image
                    ThumbType type = check_thumb(thumb,thumb_z);
                    // printf("type : %d\n",type);
                    if ((type == TRAVERSABLE) && (traversable_counter_>=(unsigned)max_image_per_type_)) {
                        // We have enough images of this type
                        type = UNUSABLE;
                    }
                    if ((type == UNTRAVERSABLE) && (untraversable_counter_>=(unsigned)max_image_per_type_)) {
                        // We have enough images of this type
                        type = UNUSABLE;
                    }
                    // Save usable images and corresponding labels
                    if (type != UNUSABLE) { 
                        // Old fashion formatting
                        char dirname[1024],filename[1024],labelname[1024];
                        sprintf(dirname,"%s/%04ld",outdir_.c_str(),image_counter_/1000);  // sprintf: write formatted data to string (first argument)
                        mkdir(dirname,0700);// may already exist but it is OK, each number of "700" refers to the permissions (read (r), write (w), execute (x) etc.) of a user (owner/group/others) to the directory created
                        sprintf(filename,"%s/%04ld/%04ld.png",outdir_.c_str(),image_counter_/1000,image_counter_%1000);
                        // Save the new image
                        cv::imwrite(filename,thumb);
                        sprintf(labelname,"%s/labels.txt",outdir_.c_str());
                        FILE * fp = fopen(labelname,"a");
                        // Save the new label
                        fprintf(fp,"%04ld/%04ld.png %d\n",image_counter_/1000,image_counter_%1000,(type==TRAVERSABLE)?1:0);  // label = 1 if traversable, 0 otherwise
                        fclose(fp);
                        image_counter_ ++ ;
                        if (type == TRAVERSABLE) {
                            traversable_counter_ ++;
                        } else {
                            untraversable_counter_ ++;
                        }
                    }
                    // Now for display (could be disabled to save CPU)
                    int mark = 0;
                    switch (type) {
                        case UNTRAVERSABLE: 
                            mark = 2; break;  // Untraversable: red
                        case TRAVERSABLE: 
                            mark = 1; break;  // Traversable: green
                        case UNUSABLE: 
                        default:
                            mark = 0; break;  // Unusable: blue
                    }
                    for (int tr=0;tr<thumb.rows;tr++) {
                        for (int tc=0;tc<thumb.cols;tc++) {
                            thumb.at<cv::Vec3b>(tr,tc)[mark] = 255;  // Vec3b: vector with 3 byte entries (for b, g, r channels)
                        }
                    }
                    // getchar();  // Get character from stdin (user keyboard input for ex.)
                }
            }
            printf("Image counter at %ld (%ld / %ld)\n",image_counter_,traversable_counter_,untraversable_counter_);
            
            cv_bridge::CvImage br(img_msg->header,"bgr8",img);
            image_pub_.publish(br.toImageMsg());
        }

    public:
        FloorPlaneExtract() : nh_("~"), it_(nh_) {
            learning_ = true;
            has_info = false;
            nh_.param("world_frame",world_frame_,std::string("/world"));
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,3.0);
            nh_.param("thumb_size",thumb_size_,32);
            nh_.param("out_dir",outdir_,std::string("."));
            nh_.param("height_threshold",height_threshold_,0.02);
            nh_.param("min_displacement",min_displacement_,0.1);
            nh_.param("min_rotation",min_rotation_,0.1);
            nh_.param("max_image_per_type",max_image_per_type_,1000);
            nh_.param("joystick_button",joystick_button_,3);
            std::string transport = "raw";
            nh_.param("transport",transport,transport);

            // Reset label file
            char labelname[1024];
            sprintf(labelname,"%s/labels.txt",outdir_.c_str());
            FILE * fp = fopen(labelname,"w");
            assert(fp);
            fclose(fp);
            
            // Delete existing image folders
            for (int i=0;i<(int)round(2*max_image_per_type_/1000);i++) {
				char dirname[1024];
				std::string s = std::to_string(i);
				char const *i_char = s.c_str();
				sprintf(dirname,"%s/000%s",outdir_.c_str(),i_char);
				boost::uintmax_t nb_files_removed = boost::filesystem::remove_all(dirname);
				if (nb_files_removed) {ROS_INFO("Img folder %s removed.", i_char);}
			}

            image_counter_ = 0;
            traversable_counter_ = 0;
            untraversable_counter_ = 0;

            // Make sure TF is ready
            ros::Duration(0.5).sleep();
            tf::StampedTransform tfw;
            listener_.lookupTransform(base_frame_,world_frame_,ros::Time(0),tfw);  // Source frame: world_frame_ ; target frame: base_frame_
            last_pose.x=tfw.getOrigin().x()-1.0;
            last_pose.y=tfw.getOrigin().y()-1.0;
            last_pose.theta=tf::getYaw(tfw.getRotation());

            image_pub_ = it_.advertise("image_label",1);  // Publish image labels (the img variable)
            joy_sub_ = nh_.subscribe("joy",1,&FloorPlaneExtract::joy_callback,this);
            info_sub_ = nh_.subscribe("info",1,&FloorPlaneExtract::calibration_callback,this);
            // scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneExtract::pc_callback,this);
            // image_sub_ = it_.subscribe<FloorPlaneExtract>("image",1, &FloorPlaneExtract::image_callback,this,transport);
            
            // Created a synchronized subscriber for point cloud and image. The need for pointer is suspicious, but it works this way.
            imgSub.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_,"image",1));
            pclSub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,"pointcloud",1));
            sync.reset(new TS(*pclSub,*imgSub,50));
            sync->registerCallback(boost::bind(&FloorPlaneExtract::sync_callback,this,_1,_2));
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Extract");
    FloorPlaneExtract fp;

    ros::spin();
    return 0;
}


