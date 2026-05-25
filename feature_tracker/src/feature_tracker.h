#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;


// ✅ Define PointType for PCL
typedef pcl::PointXYZI PointType;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
  FeatureTracker();

  void readImage(const cv::Mat &_img, double _cur_time);

  void setMask();

  void addPoints();

  bool updateID(unsigned int i);

  void readIntrinsicParameter(const string &calib_file);

  void showUndistortion(const string &name);

  void rejectWithF();

  void undistortedPoints();

  cv::Mat mask;
  cv::Mat fisheye_mask;
  cv::Mat prev_img, cur_img, forw_img;
  vector<cv::Point2f> n_pts;
  vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
  vector<cv::Point2f> prev_un_pts, cur_un_pts;
  vector<cv::Point2f> pts_velocity;
  vector<int> ids;
  vector<int> track_cnt;
  map<int, cv::Point2f> cur_un_pts_map;
  map<int, cv::Point2f> prev_un_pts_map;
  camodocal::CameraPtr m_camera;
  double cur_time;
  double prev_time;

  static int n_id;
};


// DUY ADD
class DepthRegister
{
public:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers for visualization
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_feature;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_image;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_cloud;

    // TF2 listener and buffer
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped transform;

    const int num_bins = 360;
    vector<vector<PointType>> pointsArray;

    DepthRegister(rclcpp::Node::SharedPtr node_in) : node_(node_in)
    {
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Messages for RVIZ visualization
        pub_depth_feature = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/depth/depth_feature", 5);
        pub_depth_image = node_->create_publisher<sensor_msgs::msg::Image>("/depth/depth_image", 5);
        pub_depth_cloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/depth/depth_cloud", 5);

        pointsArray.resize(num_bins);
        for (int i = 0; i < num_bins; ++i)
            pointsArray[i].resize(num_bins);
    }

    sensor_msgs::msg::ChannelFloat32 get_depth(const rclcpp::Time& stamp_cur, const cv::Mat& imageCur, const pcl::PointCloud<PointType>::Ptr& depthCloud,const camodocal::CameraPtr& camera_model,const vector<geometry_msgs::msg::Point32>& features_2d)
    {
        // RCLCPP_INFO(node_->get_logger(), "========== GET_DEPTH START ==========");
        
        // 0.1 Initialize depth for return
        sensor_msgs::msg::ChannelFloat32 depth_of_point;
        depth_of_point.name = "depth";
        depth_of_point.values.resize(features_2d.size(), -1);
        // RCLCPP_INFO(node_->get_logger(), "[0.1] Init depth for %zu features", features_2d.size());

        // 0.2 Check if depthCloud available
        if (depthCloud->size() == 0)
        {
            RCLCPP_WARN(node_->get_logger(), "[0.2] EMPTY depth cloud! Returning...");
            return depth_of_point;
        }
        // RCLCPP_INFO(node_->get_logger(), "[0.2] Input depth cloud size: %zu points", depthCloud->size());

        // 0.3 Look up transform at current image time
        try {
            transform = tf_buffer_->lookupTransform("world", "camera", tf2::TimePointZero);
            // RCLCPP_INFO(node_->get_logger(), "[0.3] TF lookup SUCCESS");
        } 
        catch (tf2::TransformException& ex) {
            // RCLCPP_WARN(node_->get_logger(), "[0.3] TF lookup FAILED: %s", ex.what());
            return depth_of_point;
        }

        // Extract translation and rotation
        double xCur = transform.transform.translation.x;
        double yCur = transform.transform.translation.y;
        double zCur = transform.transform.translation.z;
        // RCLCPP_INFO(node_->get_logger(), "[0.3] Translation: [%.3f, %.3f, %.3f]", xCur, yCur, zCur);
        
        // Convert quaternion to RPY
        tf2::Quaternion q(transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double rollCur, pitchCur, yawCur;
        m.getRPY(rollCur, pitchCur, yawCur);
        // RCLCPP_INFO(node_->get_logger(), "[0.3] RPY: [%.2f, %.2f, %.2f] deg", rollCur*180/M_PI, pitchCur*180/M_PI, yawCur*180/M_PI);
        
        Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

        // 0.4 Transform cloud from global frame to camera frame
        pcl::PointCloud<PointType>::Ptr depth_cloud_local(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*depthCloud, *depth_cloud_local, transNow.inverse());         // dung inverse // Dùng NGHỊCH: World → Camera ✅
        // RCLCPP_INFO(node_->get_logger(), "[0.4] Transformed to local frame: %zu points", depth_cloud_local->size());  // chuyen ve frame cua camera

        // 0.5 Project undistorted normalized (z) 2d features onto a unit sphere
        pcl::PointCloud<PointType>::Ptr features_3d_sphere(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)features_2d.size(); ++i)
        {
            Eigen::Vector3f feature_cur(features_2d[i].x, features_2d[i].y, features_2d[i].z);
            feature_cur.normalize();   // dua ve unit sphere.
            PointType p;
            p.x =  feature_cur(2);
            p.y = -feature_cur(0);
            p.z = -feature_cur(1);
            p.intensity = -1;
            features_3d_sphere->push_back(p);
        }
        // RCLCPP_INFO(node_->get_logger(), "[0.5] Projected %zu features to unit sphere", features_3d_sphere->size());

        // 3. Project depth cloud on a range image
        float bin_res = 180.0 / (float)num_bins;
        cv::Mat rangeImage = cv::Mat(num_bins, num_bins, CV_32F, cv::Scalar::all(FLT_MAX));

        int points_in_view = 0;
        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            PointType p = depth_cloud_local->points[i];  // depth
            if (p.x < 0 || abs(p.y / p.x) > 10 || abs(p.z / p.x) > 10)
                continue;
            
            float row_angle = atan2(p.z, sqrt(p.x * p.x + p.y * p.y)) * 180.0 / M_PI + 90.0;
            int row_id = round(row_angle / bin_res);
            float col_angle = atan2(p.x, p.y) * 180.0 / M_PI;
            int col_id = round(col_angle / bin_res);        // tinh id tren range image
            
            if (row_id < 0 || row_id >= num_bins || col_id < 0 || col_id >= num_bins)
                continue;
            
            float dist = pointDistance(p);
            if (dist < rangeImage.at<float>(row_id, col_id))
            {
                rangeImage.at<float>(row_id, col_id) = dist;
                pointsArray[row_id][col_id] = p;
                points_in_view++;
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "[3] Points in camera FOV: %d", points_in_view);

        // 4. Extract downsampled depth cloud from range image
        pcl::PointCloud<PointType>::Ptr depth_cloud_local_filter2(new pcl::PointCloud<PointType>());
        for (int i = 0; i < num_bins; ++i)
        {
            for (int j = 0; j < num_bins; ++j)
            {
                if (rangeImage.at<float>(i, j) != FLT_MAX)
                    depth_cloud_local_filter2->push_back(pointsArray[i][j]);
            }
        }
        *depth_cloud_local = *depth_cloud_local_filter2;
        // RCLCPP_INFO(node_->get_logger(), "[4] After range image filtering: %zu points", depth_cloud_local->size());
        
        publishCloud(&pub_depth_cloud, depth_cloud_local, stamp_cur, "camera");      //body 

        // 5. Project depth cloud onto a unit sphere
        pcl::PointCloud<PointType>::Ptr depth_cloud_unit_sphere(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            PointType p = depth_cloud_local->points[i];
            float range = pointDistance(p);
            p.x /= range;
            p.y /= range;
            p.z /= range;
            p.intensity = range;
            depth_cloud_unit_sphere->push_back(p);
        }
        
        if (depth_cloud_unit_sphere->size() < 10)
        {
            RCLCPP_WARN(node_->get_logger(), "[5] Too few points on unit sphere: %zu (need >= 10)", 
                        depth_cloud_unit_sphere->size());
            return depth_of_point;
        }
        // RCLCPP_INFO(node_->get_logger(), "[5] Unit sphere cloud: %zu points", depth_cloud_unit_sphere->size());

        // 6. Create a kd-tree using the spherical depth cloud
        pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
        kdtree->setInputCloud(depth_cloud_unit_sphere);
        // RCLCPP_INFO(node_->get_logger(), "[6] KD-tree created");

        // 7. Find the feature depth using kd-tree
        vector<int> pointSearchInd;
        vector<float> pointSearchSqDis;
        float dist_sq_threshold = pow(sin(bin_res / 180.0 * M_PI) * 5.0, 2);
        
        int features_with_depth = 0;
        int features_rejected = 0;
        
        for (int i = 0; i < (int)features_3d_sphere->size(); ++i)
        {
            kdtree->nearestKSearch(features_3d_sphere->points[i], 3, pointSearchInd, pointSearchSqDis);
            if (pointSearchInd.size() == 3 && pointSearchSqDis[2] < dist_sq_threshold)
            {
                float r1 = depth_cloud_unit_sphere->points[pointSearchInd[0]].intensity;
                Eigen::Vector3f A(depth_cloud_unit_sphere->points[pointSearchInd[0]].x * r1,
                                depth_cloud_unit_sphere->points[pointSearchInd[0]].y * r1,
                                depth_cloud_unit_sphere->points[pointSearchInd[0]].z * r1);

                float r2 = depth_cloud_unit_sphere->points[pointSearchInd[1]].intensity;
                Eigen::Vector3f B(depth_cloud_unit_sphere->points[pointSearchInd[1]].x * r2,
                                depth_cloud_unit_sphere->points[pointSearchInd[1]].y * r2,
                                depth_cloud_unit_sphere->points[pointSearchInd[1]].z * r2);

                float r3 = depth_cloud_unit_sphere->points[pointSearchInd[2]].intensity;
                Eigen::Vector3f C(depth_cloud_unit_sphere->points[pointSearchInd[2]].x * r3,
                                depth_cloud_unit_sphere->points[pointSearchInd[2]].y * r3,
                                depth_cloud_unit_sphere->points[pointSearchInd[2]].z * r3);

                Eigen::Vector3f V(features_3d_sphere->points[i].x,
                                features_3d_sphere->points[i].y,
                                features_3d_sphere->points[i].z);

                Eigen::Vector3f N = (A - B).cross(B - C);
                float s = (N(0) * A(0) + N(1) * A(1) + N(2) * A(2)) 
                        / (N(0) * V(0) + N(1) * V(1) + N(2) * V(2));

                float min_depth = min(r1, min(r2, r3));
                float max_depth = max(r1, max(r2, r3));
                
                if (max_depth - min_depth > 2 || s <= 0.5)
                {
                    features_rejected++;
                    continue;
                } 
                else if (s - max_depth > 0) {
                    s = max_depth;
                } else if (s - min_depth < 0) {
                    s = min_depth;
                }
                
                features_3d_sphere->points[i].x *= s;
                features_3d_sphere->points[i].y *= s;
                features_3d_sphere->points[i].z *= s;
                features_3d_sphere->points[i].intensity = features_3d_sphere->points[i].x;
                features_with_depth++;
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "[7] KD-tree search: %d features with depth, %d rejected", features_with_depth, features_rejected);

        // Visualize features
        publishCloud(&pub_depth_feature, features_3d_sphere, stamp_cur, "camera");    //camera
        
        // Update depth value for return
        int valid_depths = 0;
        for (int i = 0; i < (int)features_3d_sphere->size(); ++i)
        {
            if (features_3d_sphere->points[i].intensity > 3.0)
            {
                depth_of_point.values[i] = features_3d_sphere->points[i].intensity;
                valid_depths++;
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "[8] Valid depths (>3m): %d / %zu features", valid_depths, features_2d.size());

        // Visualization
        if (pub_depth_image->get_subscription_count() != 0)
        {
            RCLCPP_INFO(node_->get_logger(), "[9] Generating depth visualization image...");
            
            vector<cv::Point2f> points_2d;
            vector<float> points_distance;

            for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
            {
                Eigen::Vector3d p_3d(-depth_cloud_local->points[i].y,
                                    -depth_cloud_local->points[i].z,
                                    depth_cloud_local->points[i].x);
                Eigen::Vector2d p_2d;
                camera_model->spaceToPlane(p_3d, p_2d);
                
                points_2d.push_back(cv::Point2f(p_2d(0), p_2d(1)));
                points_distance.push_back(pointDistance(depth_cloud_local->points[i]));
            }

            cv::Mat showImage, circleImage;
            cv::cvtColor(imageCur, showImage, cv::COLOR_GRAY2RGB);
            circleImage = showImage.clone();
            for (int i = 0; i < (int)points_2d.size(); ++i)
            {
                float r, g, b;
                getColor(points_distance[i], 50.0, r, g, b);
                cv::circle(circleImage, points_2d[i], 0, cv::Scalar(r, g, b), 5);
            }
            cv::addWeighted(showImage, 1.0, circleImage, 0.7, 0, showImage);

            cv_bridge::CvImage bridge;
            bridge.image = showImage;
            bridge.encoding = "rgb8";
            auto imageShowPointer = bridge.toImageMsg();
            imageShowPointer->header.stamp = stamp_cur;
            pub_depth_image->publish(*imageShowPointer);
            
            // RCLCPP_INFO(node_->get_logger(), "[9] Depth image published with %zu projected points", points_2d.size());
        }

        // RCLCPP_INFO(node_->get_logger(), "========== GET_DEPTH END ==========\n");
        return depth_of_point;
    }

    void getColor(float p, float np, float&r, float&g, float&b) 
    {
        float inc = 6.0 / np;
        float x = p * inc;
        r = 0.0f; g = 0.0f; b = 0.0f;
        if ((0 <= x && x <= 1) || (5 <= x && x <= 6)) r = 1.0f;
        else if (4 <= x && x <= 5) r = x - 4;
        else if (1 <= x && x <= 2) r = 1.0f - (x - 1);

        if (1 <= x && x <= 3) g = 1.0f;
        else if (0 <= x && x <= 1) g = x - 0;
        else if (3 <= x && x <= 4) g = 1.0f - (x - 3);

        if (3 <= x && x <= 5) b = 1.0f;
        else if (2 <= x && x <= 3) b = x - 2;
        else if (5 <= x && x <= 6) b = 1.0f - (x - 5);
        r *= 255.0;
        g *= 255.0;
        b *= 255.0;
    }
};
