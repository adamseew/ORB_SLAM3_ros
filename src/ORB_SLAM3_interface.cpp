
#include "orb_slam3_ros_wrapper/ORB_SLAM3_interface.h"
#include "sophus/geometry.hpp"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>

#include <vector>


ORB_SLAM3_interface::ORB_SLAM3_interface(ORB_SLAM3::System* pSLAM, 
    ros::NodeHandle* node_handle) : mpSLAM(pSLAM), node_handle(node_handle) {

    _publisher = node_handle->advertise<geometry_msgs::Pose>(ORB_SLAM3_ROS_WRAPPER_POSE_TOPIC, 1);
    __publisher = node_handle->advertise<sensor_msgs::Image>(ORB_SLAM3_ROS_WRAPPER_DEPTH_TOPIC, 1);
    ___publisher = node_handle->advertise<sensor_msgs::Image>(ORB_SLAM3_ROS_WRAPPER_RGB_TOPIC, 1);
    ____publisher = node_handle->advertise<sensor_msgs::PointCloud2>(ORB_SLAM3_ROS_WRAPPER_PCD_TOPIC, 1);

    std::string node_name = ros::this_node::getName();
    node_handle->param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handle->param<std::string>(node_name + "/pose_frame_id", pose_frame_id, "pose");

    prev_sample_time = ros::Time::now();

    ROS_INFO("Running the wrapper for ORB SLAM3 in iSDF ROS1 workspace");
}

ORB_SLAM3_interface::~ORB_SLAM3_interface(void) { }

void ORB_SLAM3_interface::rgbd_callback(const sensor_msgs::ImageConstPtr& msgRGB,
    const sensor_msgs::ImageConstPtr& msgD) {

    // copy the ros image message to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Main algorithm runs here
    Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, 
                                         cv_ptrD->image, 
                                         cv_ptrRGB->header.stamp.toSec()
                                        );

    image_to_pointcloud(msgD);     
    publish_frame(Tcw, *msgRGB, *msgD, ORB_SLAM3::System::STEREO);
    // publish_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), cv_ptrRGB->header.stamp);
}

geometry_msgs::PoseStamped ORB_SLAM3_interface::SE3toPoseMsg(Sophus::SE3f tf) {
    
    Eigen::Isometry3d camera_tf;
    camera_tf.matrix() = tf.matrix().cast<double>();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_id;

    pose_msg.pose.position.x = camera_tf.translation().x();
    pose_msg.pose.position.y = camera_tf.translation().y();
    pose_msg.pose.position.z = camera_tf.translation().z();

    Eigen::Quaterniond q(camera_tf.linear());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    return pose_msg;
}


void ORB_SLAM3_interface::image_to_pointcloud(const sensor_msgs::ImageConstPtr& _image) {

    static int    _count;
    int           i, j, n_points;
    float         d, depth_scale = 1e-3;

    const ushort* row_ptr;
    vector<float> points;

    cv_bridge::CvImageConstPtr image =  cv_bridge::toCvShare(_image);
    Eigen::Matrix3f K;
    K <<  607.199951171875, 0.0,               320.0849609375, 
          0.0,              606.1614990234375, 244.21034240722656, 
          0.0,              0.0,               1.0;
    Eigen::Matrix3f invK; 
    invK = K.inverse();
    points.clear();

    for (i = 0; i < image->image.rows; i++) {

        row_ptr = image->image.ptr<ushort>(i);

        for (j = 0; j < image->image.cols; j++) {

            ushort id = row_ptr[j];

            if (id != 0) {
                d = depth_scale*id;
                
		Eigen::Vector3f image_point(j*d, i*d, d);
                Eigen::Vector3f camera_point = invK*image_point;

                points.push_back(camera_point.x());
                points.push_back(camera_point.y());
                points.push_back(camera_point.z());
            }
        }
    }

    n_points = points.size();

    // Create a PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(n_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    cloud_msg.height = 1;
    cloud_msg.width = n_points;
    cloud_msg.header.frame_id = _image->header.frame_id;
    cloud_msg.header.seq = _image->header.seq;
    cloud_msg.header.stamp = _image->header.stamp;

#ifdef DEBUG
    if (_count == 0)
    	log_fd.open(DEBUG_LOCALE_PCD_LOG, std::ios::out|std::ios::trunc);
#endif

    for(i = 0; i < n_points/3; ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = points[3*i+0]-TRANSLATION_X; // tranlsations
        *iter_y = points[3*i+1]-TRANSLATION_Y;
        *iter_z = points[3*i+2];
	
#ifdef DEBUG
        if (_count ==  0) {
            log_fd << points[3*i+0] << "," << points[3*i+1] << "," << points[3*i+2];

	    if (i != n_points/3-1)
                log_fd << ",";
        }
#endif
    }
    
#ifdef DEBUG
    if (_count++ == 0)
        log_fd.close();
#endif

    ____publisher.publish(cloud_msg);
}

void ORB_SLAM3_interface::publish_frame(Sophus::SE3f Tcw, 
    sensor_msgs::Image msgRGB, 
    sensor_msgs::Image msgD,
    ORB_SLAM3::System::eSensor sensor_type) {

    _publisher.publish(SE3toPoseMsg(Tcw).pose);
    __publisher.publish(msgD);
    ___publisher.publish(msgRGB);
}
