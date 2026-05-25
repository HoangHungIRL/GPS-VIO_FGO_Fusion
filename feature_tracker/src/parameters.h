#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS2 message types (.hpp and msg/ namespace)
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>

// OpenCV (modernized)
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL (unchanged)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

// TF2 (replaces tf)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// Standard C++ (unchanged)
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <cassert>
#include <iostream>


#define _VAL(x) #x
#define _STR(x) _VAL(x)
#define AT __FILE__ ":" _STR(__LINE__) "]"

using namespace std;

// ✅ Define PointType
typedef pcl::PointXYZI PointType;

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;

extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

// DUY ADD
extern std::string POINT_CLOUD_TOPIC;

extern int USE_LIDAR;
extern int LIDAR_SKIP;
extern double L_C_TX;
extern double L_C_TY;
extern double L_C_TZ;
extern double L_C_RX;
extern double L_C_RY;
extern double L_C_RZ;
extern std::string POINT_CLOUD_TOPIC;

extern cv::Mat CAMERA_IMU_ROTATION;      // 3x3 rotation matrix
extern cv::Mat CAMERA_IMU_TRANSLATION;   // 3x1 translation vector
extern double C_I_TX, C_I_TY, C_I_TZ;  // Translation: Camera → IMU
extern double C_I_RX, C_I_RY, C_I_RZ;  // Rotation (RPY): Camera → IMU

//

void readParameters(const rclcpp::Node::SharedPtr &n);

// ✅ Helper functions
inline float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <typename T>
void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr* publisher, 
                  const T& cloud, 
                  const rclcpp::Time& stamp, 
                  const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = frame_id;
    (*publisher)->publish(cloud_msg);
}
