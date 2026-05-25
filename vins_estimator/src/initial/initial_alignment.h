#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include "../feature_manager.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <deque>
#include <vector>

using namespace Eigen;
using namespace std;

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>& _points, const vector<float> &_lidar_initialization_info, double _t):t{_t},is_key_frame{false}, reset_id{-1}, gravity{9.805}       // DUY ADD
        {
            points = _points;

            // reset id in case lidar odometry relocate
            reset_id = (int)round(_lidar_initialization_info[0]);
            // Pose
            T.x() = _lidar_initialization_info[1];
            T.y() = _lidar_initialization_info[2];
            T.z() = _lidar_initialization_info[3];
            // Rotation
            Eigen::Quaterniond Q = Eigen::Quaterniond(_lidar_initialization_info[7],
                                                      _lidar_initialization_info[4],
                                                      _lidar_initialization_info[5],
                                                      _lidar_initialization_info[6]);
            R = Q.normalized().toRotationMatrix();
            // Velocity
            V.x() = _lidar_initialization_info[8];
            V.y() = _lidar_initialization_info[9];
            V.z() = _lidar_initialization_info[10];
            // Acceleration bias
            Ba.x() = _lidar_initialization_info[11];
            Ba.y() = _lidar_initialization_info[12];
            Ba.z() = _lidar_initialization_info[13];
            // Gyroscope bias
            Bg.x() = _lidar_initialization_info[14];
            Bg.y() = _lidar_initialization_info[15];
            Bg.z() = _lidar_initialization_info[16];
            // Gravity
            gravity = _lidar_initialization_info[17];
        };
        map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>> > > points; // DUY ADD
        double t;
        // Matrix3d R;
        // Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;

        // Lidar odometry info
        int reset_id;
        Vector3d T;
        Matrix3d R;
        Vector3d V;
        Vector3d Ba;
        Vector3d Bg;
        double gravity;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);


class odometryRegister
{
public:
    rclcpp::Node::SharedPtr node_;
    tf2::Quaternion q_lidar_to_cam;
    Eigen::Quaterniond q_lidar_to_cam_eigen;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_latest_odometry;

    odometryRegister(rclcpp::Node::SharedPtr node_in)
    : node_(node_in)
    {
        q_lidar_to_cam = tf2::Quaternion(0, 1, 0, 0); // rotate orientation // mark: camera - lidar
        q_lidar_to_cam_eigen = Eigen::Quaterniond(0, 0, 0, 1); // rotate position by pi, (w, x, y, z) // mark: camera - lidar
        // pub_latest_odometry = node_->create_publisher<nav_msgs::msg::Odometry>("odometry/test", 1000);
    }

    // convert odometry from ROS Lidar frame to VINS camera frame
    vector<float> getOdometry(deque<nav_msgs::msg::Odometry>& odomQueue, double img_time)
    {
        RCLCPP_INFO(node_->get_logger(), "========== GET_ODOMETRY START ==========");
        RCLCPP_INFO(node_->get_logger(), "[0] Request time: %.6f", img_time);
        
        vector<float> odometry_channel;
        odometry_channel.resize(18, -1); // reset id(1), P(3), Q(4), V(3), Ba(3), Bg(3), gravity(1)

        nav_msgs::msg::Odometry odomCur;
        
        // pop old odometry msg
        int removed_count = 0;
        while (!odomQueue.empty()) 
        {
            double front_time = Utility::toSec(odomQueue.front().header.stamp);
            if (front_time < img_time - 0.05)
            {
                odomQueue.pop_front();
                removed_count++;
            }
            else
                break;
        }
        RCLCPP_INFO(node_->get_logger(), "[1] Removed %d old odom msgs. Queue size: %zu", removed_count, odomQueue.size());

        if (odomQueue.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "[2] Odom queue EMPTY! Returning...");
            return odometry_channel;
        }

        // find the odometry time that is the closest to image time
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            odomCur = odomQueue[i];
            double odom_time = Utility::toSec(odomCur.header.stamp);

            if (odom_time < img_time - 0.002) // 500Hz imu
                continue;
            else
            {
                RCLCPP_INFO(node_->get_logger(), "[2] Found closest odom at index %d, time: %.6f", i, odom_time);
                break;
            }
        }

        // time stamp difference still too large
        double time_diff = abs(Utility::toSec(odomCur.header.stamp) - img_time);
        if (time_diff > 0.05)
        {
            RCLCPP_WARN(node_->get_logger(), "[3] Time diff too large: %.6f > 0.05s. Returning...", time_diff);
            return odometry_channel;
        }
        RCLCPP_INFO(node_->get_logger(), "[3] Time diff OK: %.6f s", time_diff);

        // Print original odometry (LiDAR frame)
        RCLCPP_INFO(node_->get_logger(), "[4] Original Odom (LiDAR frame):");
        RCLCPP_INFO(node_->get_logger(), "    Position: [%.3f, %.3f, %.3f]", 
                    odomCur.pose.pose.position.x, 
                    odomCur.pose.pose.position.y, 
                    odomCur.pose.pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "    Orientation: [%.3f, %.3f, %.3f, %.3f]",
                    odomCur.pose.pose.orientation.x,
                    odomCur.pose.pose.orientation.y,
                    odomCur.pose.pose.orientation.z,
                    odomCur.pose.pose.orientation.w);
        RCLCPP_INFO(node_->get_logger(), "    Velocity: [%.3f, %.3f, %.3f]",
                    odomCur.twist.twist.linear.x,
                    odomCur.twist.twist.linear.y,
                    odomCur.twist.twist.linear.z);

        // convert odometry rotation from lidar ROS frame to VINS camera frame
        tf2::Quaternion q_odom_lidar;
        tf2::convert(odomCur.pose.pose.orientation, q_odom_lidar);

        tf2::Quaternion q_global_rot;
        q_global_rot.setRPY(0, 0, M_PI);
        tf2::Quaternion q_odom_cam = q_global_rot * (q_odom_lidar * q_lidar_to_cam);
        
        odomCur.pose.pose.orientation = tf2::toMsg(q_odom_cam);

        RCLCPP_INFO(node_->get_logger(), "[5] After rotation conversion:");
        RCLCPP_INFO(node_->get_logger(), "    Orientation (Camera): [%.3f, %.3f, %.3f, %.3f]",
                    odomCur.pose.pose.orientation.x,
                    odomCur.pose.pose.orientation.y,
                    odomCur.pose.pose.orientation.z,
                    odomCur.pose.pose.orientation.w);

        // convert odometry position from lidar ROS frame to VINS camera frame
        Eigen::Vector3d p_eigen(odomCur.pose.pose.position.x, 
                                odomCur.pose.pose.position.y, 
                                odomCur.pose.pose.position.z);
        Eigen::Vector3d v_eigen(odomCur.twist.twist.linear.x, 
                                odomCur.twist.twist.linear.y, 
                                odomCur.twist.twist.linear.z);
        Eigen::Vector3d p_eigen_new = q_lidar_to_cam_eigen * p_eigen;
        Eigen::Vector3d v_eigen_new = q_lidar_to_cam_eigen * v_eigen;

        odomCur.pose.pose.position.x = p_eigen_new.x();
        odomCur.pose.pose.position.y = p_eigen_new.y();
        odomCur.pose.pose.position.z = p_eigen_new.z();

        odomCur.twist.twist.linear.x = v_eigen_new.x();
        odomCur.twist.twist.linear.y = v_eigen_new.y();
        odomCur.twist.twist.linear.z = v_eigen_new.z();

        RCLCPP_INFO(node_->get_logger(), "[6] After position/velocity conversion:");
        RCLCPP_INFO(node_->get_logger(), "    Position (Camera): [%.3f, %.3f, %.3f]", 
                    odomCur.pose.pose.position.x, 
                    odomCur.pose.pose.position.y, 
                    odomCur.pose.pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "    Velocity (Camera): [%.3f, %.3f, %.3f]",
                    odomCur.twist.twist.linear.x,
                    odomCur.twist.twist.linear.y,
                    odomCur.twist.twist.linear.z);

        // Pack into channel
        odometry_channel[0] = odomCur.pose.covariance[0];
        odometry_channel[1] = odomCur.pose.pose.position.x;
        odometry_channel[2] = odomCur.pose.pose.position.y;
        odometry_channel[3] = odomCur.pose.pose.position.z;
        odometry_channel[4] = odomCur.pose.pose.orientation.x;
        odometry_channel[5] = odomCur.pose.pose.orientation.y;
        odometry_channel[6] = odomCur.pose.pose.orientation.z;
        odometry_channel[7] = odomCur.pose.pose.orientation.w;
        odometry_channel[8]  = odomCur.twist.twist.linear.x;
        odometry_channel[9]  = odomCur.twist.twist.linear.y;
        odometry_channel[10] = odomCur.twist.twist.linear.z;
        odometry_channel[11] = odomCur.pose.covariance[1];
        odometry_channel[12] = odomCur.pose.covariance[2];
        odometry_channel[13] = odomCur.pose.covariance[3];
        odometry_channel[14] = odomCur.pose.covariance[4];
        odometry_channel[15] = odomCur.pose.covariance[5];
        odometry_channel[16] = odomCur.pose.covariance[6];
        odometry_channel[17] = odomCur.pose.covariance[7];

        RCLCPP_INFO(node_->get_logger(), "[7] Odometry channel packed:");
        RCLCPP_INFO(node_->get_logger(), "    reset_id: %.0f", odometry_channel[0]);
        RCLCPP_INFO(node_->get_logger(), "    P: [%.3f, %.3f, %.3f]", odometry_channel[1], odometry_channel[2], odometry_channel[3]);
        RCLCPP_INFO(node_->get_logger(), "    Q: [%.3f, %.3f, %.3f, %.3f]", odometry_channel[4], odometry_channel[5], odometry_channel[6], odometry_channel[7]);
        RCLCPP_INFO(node_->get_logger(), "    V: [%.3f, %.3f, %.3f]", odometry_channel[8], odometry_channel[9], odometry_channel[10]);
        RCLCPP_INFO(node_->get_logger(), "    Ba: [%.3f, %.3f, %.3f]", odometry_channel[11], odometry_channel[12], odometry_channel[13]);
        RCLCPP_INFO(node_->get_logger(), "    Bg: [%.3f, %.3f, %.3f]", odometry_channel[14], odometry_channel[15], odometry_channel[16]);
        RCLCPP_INFO(node_->get_logger(), "    gravity: %.3f", odometry_channel[17]);
        
        RCLCPP_INFO(node_->get_logger(), "========== GET_ODOMETRY END ==========\n");

        return odometry_channel;
    }


    // // convert odometry from ROS Lidar frame to VINS camera frame
    // vector<float> getOdometry(deque<nav_msgs::msg::Odometry>& odomQueue, double img_time)
    // {
    //     vector<float> odometry_channel;
    //     odometry_channel.resize(18, -1); // reset id(1), P(3), Q(4), V(3), Ba(3), Bg(3), gravity(1)

    //     nav_msgs::msg::Odometry odomCur;
        
    //     // pop old odometry msg
    //     while (!odomQueue.empty()) 
    //     {
    //         if (Utility::toSec(odomQueue.front().header.stamp) < img_time - 0.05)
    //             odomQueue.pop_front();
    //         else
    //             break;
    //     }

    //     if (odomQueue.empty())
    //     {
    //         return odometry_channel;
    //     }

    //     // find the odometry time that is the closest to image time
    //     for (int i = 0; i < (int)odomQueue.size(); ++i)
    //     {
    //         odomCur = odomQueue[i];

    //         if (Utility::toSec(odomCur.header.stamp) < img_time - 0.002) // 500Hz imu
    //             continue;
    //         else
    //             break;
    //     }

    //     // time stamp difference still too large
    //     if (abs(Utility::toSec(odomCur.header.stamp) - img_time) > 0.05)
    //     {
    //         return odometry_channel;
    //     }

    //     // convert odometry rotation from lidar ROS frame to VINS camera frame
    //     tf2::Quaternion q_odom_lidar;
    //     // tf2::fromMsg(odomCur.pose.pose.orientation, q_odom_lidar);
    //     tf2::convert(odomCur.pose.pose.orientation, q_odom_lidar);

    //     tf2::Quaternion q_global_rot;
    //     q_global_rot.setRPY(0, 0, M_PI);
    //     tf2::Quaternion q_odom_cam = q_global_rot * (q_odom_lidar * q_lidar_to_cam);
        
    //     odomCur.pose.pose.orientation = tf2::toMsg(q_odom_cam);

    //     // convert odometry position from lidar ROS frame to VINS camera frame
    //     Eigen::Vector3d p_eigen(odomCur.pose.pose.position.x, 
    //                             odomCur.pose.pose.position.y, 
    //                             odomCur.pose.pose.position.z);
    //     Eigen::Vector3d v_eigen(odomCur.twist.twist.linear.x, 
    //                             odomCur.twist.twist.linear.y, 
    //                             odomCur.twist.twist.linear.z);
    //     Eigen::Vector3d p_eigen_new = q_lidar_to_cam_eigen * p_eigen;
    //     Eigen::Vector3d v_eigen_new = q_lidar_to_cam_eigen * v_eigen;

    //     odomCur.pose.pose.position.x = p_eigen_new.x();
    //     odomCur.pose.pose.position.y = p_eigen_new.y();
    //     odomCur.pose.pose.position.z = p_eigen_new.z();

    //     odomCur.twist.twist.linear.x = v_eigen_new.x();
    //     odomCur.twist.twist.linear.y = v_eigen_new.y();
    //     odomCur.twist.twist.linear.z = v_eigen_new.z();

    //     // odomCur.header.stamp = rclcpp::Time(img_time);
    //     // odomCur.header.frame_id = "vins_world";
    //     // odomCur.child_frame_id = "vins_body";
    //     // pub_latest_odometry->publish(odomCur);

    //     odometry_channel[0] = odomCur.pose.covariance[0];
    //     odometry_channel[1] = odomCur.pose.pose.position.x;
    //     odometry_channel[2] = odomCur.pose.pose.position.y;
    //     odometry_channel[3] = odomCur.pose.pose.position.z;
    //     odometry_channel[4] = odomCur.pose.pose.orientation.x;
    //     odometry_channel[5] = odomCur.pose.pose.orientation.y;
    //     odometry_channel[6] = odomCur.pose.pose.orientation.z;
    //     odometry_channel[7] = odomCur.pose.pose.orientation.w;
    //     odometry_channel[8]  = odomCur.twist.twist.linear.x;
    //     odometry_channel[9]  = odomCur.twist.twist.linear.y;
    //     odometry_channel[10] = odomCur.twist.twist.linear.z;
    //     odometry_channel[11] = odomCur.pose.covariance[1];
    //     odometry_channel[12] = odomCur.pose.covariance[2];
    //     odometry_channel[13] = odomCur.pose.covariance[3];
    //     odometry_channel[14] = odomCur.pose.covariance[4];
    //     odometry_channel[15] = odomCur.pose.covariance[5];
    //     odometry_channel[16] = odomCur.pose.covariance[6];
    //     odometry_channel[17] = odomCur.pose.covariance[7];

    //     return odometry_channel;
    // }
};