
#include "include/ImuTypes.h"
#include "include/System.h"

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <ros/time.h>
#include <ros/ros.h>

#include <iostream>
#include <fstream>

#ifndef ORB_SLAM3_INTERFACE_H
#define ORB_SLAM3_INTERFACE_H

#define ORB_SLAM3_ROS_WRAPPER_POSE_TOPIC  "/orb_slam3_ros_wrapper/pose"
#define ORB_SLAM3_ROS_WRAPPER_DEPTH_TOPIC "/orb_slam3_ros_wrapper/depth"
#define ORB_SLAM3_ROS_WRAPPER_RGB_TOPIC   "/orb_slam3_ros_wrapper/rgb"
#define ORB_SLAM3_ROS_WRAPPER_PCD_TOPIC   "/orb_slam3_ros_wrapper/pcd"
#define DEBUG                             1
#define DEBUG_LOCALE_PCD_LOG              "/home/user/locale_pcd.dat"
#define TRANSLATION_X                     .3093
#define TRANSLATION_Y                     .24

class ORB_SLAM3_interface {
   
private:
    ORB_SLAM3::System* mpSLAM;

    ros::NodeHandle* node_handle;

    ros::Publisher _publisher;
    ros::Publisher __publisher;
    ros::Publisher ___publisher;
    ros::Publisher ____publisher;

    std::string map_frame_id;
    std::string pose_frame_id;

    ros::Time prev_sample_time;

#ifdef DEBUG
    std::ofstream log_fd;
#endif

public:
    ORB_SLAM3_interface(ORB_SLAM3::System* pSLAM, ros::NodeHandle* node_handle);
    ~ORB_SLAM3_interface(void);

    // void rgb_callback(const sensor_msgs::ImageConstPtr& msgRGB);
    // void rgb_imu_callback(const sensor_msgs::ImageConstPtr& msgRGB);
    // void stereo_callback(const sensor_msgs::ImageConstPtr& msgRGB);
    
    void rgbd_callback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

    geometry_msgs::PoseStamped SE3toPoseMsg(Sophus::SE3f tf);

    void image_to_pointcloud(const sensor_msgs::ImageConstPtr&);

    void publish_frame(Sophus::SE3f Tcw, sensor_msgs::Image msgRGB, 
    
    sensor_msgs::Image msgD, ORB_SLAM3::System::eSensor sensor_type);
};

#endif


