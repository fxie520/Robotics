#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>

#include "boost/filesystem/operations.hpp" 
#include "boost/filesystem/fstream.hpp"
#include <iostream>                        
#include <string>  // int to string

class InterfaceObserve {
    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;

        ros::Subscriber twist_sub_;
        ros::Subscriber joy_sub_;
        tf::TransformListener listener_;

        std::string base_frame_;
        std::string world_frame_;
        std::string outdir_;
        double twist_threshold_;
        double min_displacement_;
        double min_rotation_;
        int max_image_per_type_;
        unsigned long image_counter_;
        unsigned long type_counter_[3]; // left, no movement, right
        int joystick_button_;
        bool learning_;

        geometry_msgs::Pose2D last_pose_;
        geometry_msgs::Twist last_command_;
        ros::Time last_command_time_;
        ros::Time last_joy_time_;

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

        void twist_callback(const geometry_msgs::TwistConstPtr msg) {
            last_command_time_ = ros::Time::now();
            last_command_ = *msg;
        }

        void image_callback(const sensor_msgs::ImageConstPtr & img_msg) {
            if (!learning_) {
                // If we're not learning, we don't care about this image
                return;
            }

            if ((img_msg->header.stamp-last_command_time_).toSec()>0.1) {
                // We can't accept an old command
                return;
            }

            tf::StampedTransform tfw;
            listener_.waitForTransform(base_frame_,world_frame_,img_msg->header.stamp,ros::Duration(1.0));
            listener_.lookupTransform(base_frame_,world_frame_,img_msg->header.stamp,tfw);  // Source frame: world_frame_ ; target frame: base_frame_
            // Check if we moved since last time, no point saving twice the same image
            if ((hypot(last_pose_.x-tfw.getOrigin().x(),last_pose_.y-tfw.getOrigin().y())<min_displacement_) &&
                    (fabs(remainder(last_pose_.theta-tf::getYaw(tfw.getRotation()),2*M_PI))<min_rotation_)) {
                return;
            }
            last_pose_.x=tfw.getOrigin().x();
            last_pose_.y=tfw.getOrigin().y();
            last_pose_.theta=tf::getYaw(tfw.getRotation());
            

            cv::Mat img(cv_bridge::toCvShare(img_msg,"bgr8")->image);
            bool save_it = true;
            int label = 1;  // No movement
            if (last_command_.linear.x<-twist_threshold_) {
                label = 0;  // Turned left
            } else if (last_command_.linear.x>twist_threshold_) {
                label = 2;  // Turned right (away from the truck)
            }
            save_it = (type_counter_[label] < (unsigned)max_image_per_type_);
            if (save_it) {
                char dirname[1024],filename[1024],labelname[1024];
                sprintf(dirname,"%s/%04ld",outdir_.c_str(),image_counter_/1000);
                mkdir(dirname,0700); // may already exist but it is OK
                sprintf(filename,"%s/%04ld/%04ld.png",outdir_.c_str(),image_counter_/1000,image_counter_%1000);
                cv::imwrite(filename,img);
                sprintf(labelname,"%s/labels.txt",outdir_.c_str());
                FILE * fp = fopen(labelname,"a");
                fprintf(fp,"%04ld/%04ld.png %d\n",image_counter_/1000,image_counter_%1000,label);
                fclose(fp);
                type_counter_[label]++;
                image_counter_++;
            }
            printf("Image counter at %ld (%ld %ld %ld)\n",image_counter_,type_counter_[0],type_counter_[1],type_counter_[2]);
        }

    public:
        InterfaceObserve() : nh_("~"), it_(nh_) {
            learning_ = false;
            nh_.param("world_frame",world_frame_,std::string("/world"));
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("out_dir",outdir_,std::string("."));
            nh_.param("twist_threshold",twist_threshold_,0.05);
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
            for (int i=0;i<(int)round(3*max_image_per_type_/1000);i++) {
				char dirname[1024];
				std::string s = std::to_string(i);
				char const *i_char = s.c_str();
				sprintf(dirname,"%s/000%s",outdir_.c_str(),i_char);
				boost::uintmax_t nb_files_removed = boost::filesystem::remove_all(dirname);
				if (nb_files_removed) {ROS_INFO("Img folder %s removed.", i_char);}
			}

            image_counter_ = 0;
            type_counter_[0] = type_counter_[1] = type_counter_[2] = 0;

            // Make sure TF is ready
            last_command_time_ = ros::Time::now();
            last_joy_time_ = ros::Time::now();
            ros::Duration(0.5).sleep();
            tf::StampedTransform tfw;
            listener_.lookupTransform(base_frame_,world_frame_,ros::Time(0),tfw);  // Source frame: world_frame_ ; target frame: base_frame_
            last_pose_.x=tfw.getOrigin().x()-1.0;  // -1.0 so that the first image is saved
            last_pose_.y=tfw.getOrigin().y()-1.0;
            last_pose_.theta=tf::getYaw(tfw.getRotation());

            twist_sub_ = nh_.subscribe("twist",1,&InterfaceObserve::twist_callback,this);
            joy_sub_ = nh_.subscribe("joy",1,&InterfaceObserve::joy_callback,this);
            image_sub_ = it_.subscribe<InterfaceObserve>("image",1, &InterfaceObserve::image_callback,this,transport);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"interface_observe");
    InterfaceObserve fp;

    ros::spin();
    return 0;
}


