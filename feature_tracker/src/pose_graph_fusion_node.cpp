#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/LocalCartesian.hpp>

#include <queue>
#include <mutex>
#include <thread>
#include <algorithm>
#include <cmath>

using namespace gtsam;

enum InitState {
    WAIT_FOR_ORIGIN, 
    WAIT_FOR_YAW,    
    RUNNING          
};

class PoseGraphFusionNode : public rclcpp::Node {
public:
    PoseGraphFusionNode() : Node("pose_graph_fusion_node") {
        // ========================================================
        // KHAI BÁO PARAMETERS
        // ========================================================
        
        // Topics
        this->declare_parameter<std::string>("odom_sub_topic", "/vins_estimator/odometry");
        this->declare_parameter<std::string>("gps_sub_topic", "/fix");
        this->declare_parameter<std::string>("odom_pub_topic", "/aft_pgo_odom");
        this->declare_parameter<std::string>("path_pub_topic", "/aft_pgo_path");
        
        // [THÊM MỚI] Topic xuất tọa độ GPS đã được làm mượt để hiện lên bản đồ RViz
        this->declare_parameter<std::string>("gps_pub_topic", "/aft_pgo_fix");
        
        // Frames
        this->declare_parameter<std::string>("global_frame_id", "world");
        this->declare_parameter<std::string>("base_frame_id", "base_link");

        // Logic GTSAM & GPS
        this->declare_parameter<double>("keyframe_meter_gap", 0.3);
        this->declare_parameter<double>("keyframe_deg_gap", 10.0);
        this->declare_parameter<double>("gps_cov_threshold", 5.0);
        this->declare_parameter<std::string>("gps_frame_type", "enu"); 
        this->declare_parameter<double>("output_yaw_offset", 180.0); 

        // Noise Models
        this->declare_parameter<double>("noise_prior_rot", 1e-4);
        this->declare_parameter<double>("noise_prior_trans", 1e-4);
        this->declare_parameter<double>("noise_odom_rot", 1e-4);
        this->declare_parameter<double>("noise_odom_trans", 1e-1);

        // ========================================================
        // ĐỌC PARAMETERS VÀO BIẾN
        // ========================================================
        odom_sub_topic_  = this->get_parameter("odom_sub_topic").as_string();
        gps_sub_topic_   = this->get_parameter("gps_sub_topic").as_string();
        odom_pub_topic_  = this->get_parameter("odom_pub_topic").as_string();
        path_pub_topic_  = this->get_parameter("path_pub_topic").as_string();
        gps_pub_topic_   = this->get_parameter("gps_pub_topic").as_string(); // [THÊM MỚI]
        
        global_frame_id_ = this->get_parameter("global_frame_id").as_string();
        base_frame_id_   = this->get_parameter("base_frame_id").as_string();

        kf_meter_gap_ = this->get_parameter("keyframe_meter_gap").as_double();
        kf_rad_gap_ = this->get_parameter("keyframe_deg_gap").as_double() * M_PI / 180.0;
        gps_cov_threshold_ = this->get_parameter("gps_cov_threshold").as_double();
        gps_frame_type_ = this->get_parameter("gps_frame_type").as_string();
        output_yaw_offset_rad_ = this->get_parameter("output_yaw_offset").as_double() * M_PI / 180.0;

        double np_r = this->get_parameter("noise_prior_rot").as_double();
        double np_t = this->get_parameter("noise_prior_trans").as_double();
        double no_r = this->get_parameter("noise_odom_rot").as_double();
        double no_t = this->get_parameter("noise_odom_trans").as_double();

        // ========================================================
        // KHỞI TẠO SUBSCRIBERS / PUBLISHERS & GTSAM
        // ========================================================
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_sub_topic_, 100, std::bind(&PoseGraphFusionNode::odom_callback, this, std::placeholders::_1));
        
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_sub_topic_, 100, std::bind(&PoseGraphFusionNode::gps_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic_, 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic_, 10);
        
        // [THÊM MỚI] Publisher cho GPS Fix
        fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_pub_topic_, 10);

        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam_ = std::make_unique<ISAM2>(parameters);

        prior_noise_ = noiseModel::Diagonal::Variances((Vector(6) << np_r, np_r, np_r, np_t, np_t, np_t).finished());
        odom_noise_  = noiseModel::Diagonal::Variances((Vector(6) << no_r, no_r, no_r, no_t, no_t, no_t).finished()); 
        
        latest_correction_ = Pose3::identity();
        T_global_odom_ = Pose3::identity();
        state_ = WAIT_FOR_ORIGIN;

        process_thread_ = std::thread(&PoseGraphFusionNode::process_graph_loop, this);
        RCLCPP_INFO(this->get_logger(), "[PGO] Khởi động thành công! Đã load toàn bộ tham số.");
    }

    ~PoseGraphFusionNode() {
        if (process_thread_.joinable()) {
            process_thread_.join();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(m_buf_);
        odom_buf_.push(msg);
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) return;
        std::lock_guard<std::mutex> lock(m_buf_);
        gps_buf_.push(msg);
    }

    Pose3 OdomMsgToGtsam(const nav_msgs::msg::Odometry::SharedPtr& msg) {
        return Pose3(
            Rot3(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
            Point3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)
        );
    }

    void convert_gps_to_metric(double lat, double lon, double alt, double& x, double& y, double& z) {
        double enu_x, enu_y, enu_z;
        geo_converter_.Forward(lat, lon, alt, enu_x, enu_y, enu_z);
        if (gps_frame_type_ == "ned") {
            x = enu_y;  
            y = enu_x;  
            z = -enu_z; 
        } else {
            x = enu_x;  
            y = enu_y;  
            z = enu_z;  
        }
    }

    // [THÊM MỚI] Hàm dịch ngược Metric (X,Y,Z) về lại GPS (Lat, Lon, Alt)
    void convert_metric_to_gps(double x, double y, double z, double& lat, double& lon, double& alt) {
        double enu_x, enu_y, enu_z;
        if (gps_frame_type_ == "ned") {
            enu_x = y;  
            enu_y = x;  
            enu_z = -z; 
        } else {
            enu_x = x;  
            enu_y = y;  
            enu_z = z;  
        }
        geo_converter_.Reverse(enu_x, enu_y, enu_z, lat, lon, alt);
    }

    void process_graph_loop() {
        rclcpp::Rate rate(100);
        while (rclcpp::ok()) {
            std::queue<nav_msgs::msg::Odometry::SharedPtr> local_odom_buf;
            {
                std::lock_guard<std::mutex> lock(m_buf_);
                while (!odom_buf_.empty()) {
                    local_odom_buf.push(odom_buf_.front());
                    odom_buf_.pop();
                }
            }

            if (local_odom_buf.empty()) {
                rate.sleep();
                continue;
            }

            while (!local_odom_buf.empty()) {
                auto curr_odom = local_odom_buf.front();
                local_odom_buf.pop();

                sensor_msgs::msg::NavSatFix::SharedPtr matched_gps = nullptr;
                {
                    std::lock_guard<std::mutex> lock(m_buf_);
                    double t_odom = rclcpp::Time(curr_odom->header.stamp).seconds();
                    while (!gps_buf_.empty()) {
                        double t_gps = rclcpp::Time(gps_buf_.front()->header.stamp).seconds();
                        if (t_gps < t_odom - 1.0) {
                            gps_buf_.pop(); 
                        } else if (t_gps <= t_odom + 0.1) {
                            matched_gps = gps_buf_.front(); 
                            gps_buf_.pop();
                        } else {
                            break; 
                        }
                    }
                }
                
                if (matched_gps) {
                    last_valid_gps_ = matched_gps;
                }

                Pose3 raw_pose = OdomMsgToGtsam(curr_odom);

                if (state_ == WAIT_FOR_ORIGIN) {
                    if (last_valid_gps_) {
                        geo_converter_.Reset(last_valid_gps_->latitude, last_valid_gps_->longitude, last_valid_gps_->altitude);
                        odom_start_ = raw_pose;
                        state_ = WAIT_FOR_YAW;
                        RCLCPP_INFO(this->get_logger(), "[PGO] Đã thiết lập mốc gốc GPS. HÃY DI CHUYỂN ROBOT > 1.5 MÉT...");
                    }
                    continue;
                }
                else if (state_ == WAIT_FOR_YAW) {
                    init_odom_buf_.push_back(curr_odom);
                    double dx = raw_pose.x() - odom_start_.x();
                    double dy = raw_pose.y() - odom_start_.y();
                    double dist = std::hypot(dx, dy);

                    if (dist > 1.5 && last_valid_gps_) {
                        double t_odom = rclcpp::Time(curr_odom->header.stamp).seconds();
                        double t_gps = rclcpp::Time(last_valid_gps_->header.stamp).seconds();
                        
                        if (std::abs(t_odom - t_gps) < 1.0) { 
                            double gps_x, gps_y, gps_z;
                            convert_gps_to_metric(last_valid_gps_->latitude, last_valid_gps_->longitude, last_valid_gps_->altitude, gps_x, gps_y, gps_z);
                            
                            double yaw_odom = std::atan2(dy, dx);
                            double yaw_gps = std::atan2(gps_y, gps_x);
                            
                            double yaw_offset = yaw_gps - yaw_odom;
                            yaw_offset = std::atan2(std::sin(yaw_offset), std::cos(yaw_offset));

                            Rot3 R_align = Rot3::Ypr(yaw_offset, 0, 0);
                            Point3 t_align = -(R_align * odom_start_.translation());
                            T_global_odom_ = Pose3(R_align, t_align);

                            state_ = RUNNING;
                            RCLCPP_INFO(this->get_logger(), "[PGO] AUTO-ALIGN HOÀN TẤT! Đã xoay tự động %.2f độ để khớp hệ GPS.", yaw_offset * 180.0 / M_PI);

                            for (auto& old_odom : init_odom_buf_) {
                                process_single_node(old_odom);
                            }
                            init_odom_buf_.clear();
                            continue;
                        }
                    }
                    continue;
                }
                else if (state_ == RUNNING) {
                    process_single_node(curr_odom);
                }
            }
            rate.sleep();
        }
    }

    void process_single_node(const nav_msgs::msg::Odometry::SharedPtr& curr_odom) {
        Pose3 raw_pose = OdomMsgToGtsam(curr_odom);
        Pose3 aligned_pose = T_global_odom_ * raw_pose;

        bool is_keyframe = false;
        if (keyframe_poses_.empty()) {
            is_keyframe = true;
        } else {
            Pose3 pose_prev = keyframe_poses_.back();
            Pose3 delta = pose_prev.between(aligned_pose);
            double dx = delta.translation().norm();
            double droll = delta.rotation().roll();
            double dpitch = delta.rotation().pitch();
            double dyaw = delta.rotation().yaw();
            
            if (dx > kf_meter_gap_ || (abs(droll) + abs(dpitch) + abs(dyaw)) > kf_rad_gap_) {
                is_keyframe = true;
            }
        }

        if (is_keyframe) {
            keyframe_poses_.push_back(aligned_pose);
            int curr_node_idx = keyframe_poses_.size() - 1;

            if (curr_node_idx == 0) {
                gtSAMgraph_.add(PriorFactor<Pose3>(0, aligned_pose, prior_noise_));
                initialEstimate_.insert(0, aligned_pose);
            } else {
                Pose3 pose_prev = keyframe_poses_[curr_node_idx - 1];
                gtSAMgraph_.add(BetweenFactor<Pose3>(curr_node_idx - 1, curr_node_idx, pose_prev.between(aligned_pose), odom_noise_));
                initialEstimate_.insert(curr_node_idx, aligned_pose);

                if (last_valid_gps_) {
                    double t_gps = rclcpp::Time(last_valid_gps_->header.stamp).seconds();
                    double t_odom = rclcpp::Time(curr_odom->header.stamp).seconds();

                    if (std::abs(t_odom - t_gps) < 1.0 && t_gps != last_used_gps_time_) {
                        last_used_gps_time_ = t_gps; 

                        double gps_x, gps_y, gps_z;
                        convert_gps_to_metric(last_valid_gps_->latitude, last_valid_gps_->longitude, last_valid_gps_->altitude, gps_x, gps_y, gps_z);
                        
                        double cov_x = last_valid_gps_->position_covariance[0];
                        double cov_y = last_valid_gps_->position_covariance[4];
                        if (cov_x < gps_cov_threshold_) {
                            Vector3 gps_noise_vec(std::max(cov_x, 0.001), std::max(cov_y, 0.001), 2.0);
                            auto gps_noise = noiseModel::Diagonal::Variances(gps_noise_vec);
                            gtSAMgraph_.add(GPSFactor(curr_node_idx, Point3(gps_x, gps_y, gps_z), gps_noise));
                        }
                    }
                }
            }

            isam_->update(gtSAMgraph_, initialEstimate_);
            isam_->update();
            gtSAMgraph_.resize(0);
            initialEstimate_.clear();
            
            isamCurrentEstimate_ = isam_->calculateEstimate();

            if (isamCurrentEstimate_.size() > 0) {
                Pose3 last_optimized_kf = isamCurrentEstimate_.at<Pose3>(isamCurrentEstimate_.size() - 1);
                Pose3 last_raw_kf = keyframe_poses_.back();
                latest_correction_ = last_optimized_kf * last_raw_kf.inverse();
            }

            publish_path(curr_odom->header.stamp);
        }

        publish_high_freq_odom(curr_odom->header.stamp, aligned_pose);
    }

    void publish_high_freq_odom(rclcpp::Time stamp, const Pose3& aligned_pose) {
        Pose3 optimized_pose = latest_correction_ * aligned_pose;

        Rot3 R_offset = Rot3::Ypr(output_yaw_offset_rad_, 0, 0);
        Rot3 adjusted_rot = optimized_pose.rotation() * R_offset;

        // 1. Publish Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = global_frame_id_; 
        odom_msg.child_frame_id = base_frame_id_;

        odom_msg.pose.pose.position.x = optimized_pose.translation().x();
        odom_msg.pose.pose.position.y = optimized_pose.translation().y();
        odom_msg.pose.pose.position.z = optimized_pose.translation().z();
        
        odom_msg.pose.pose.orientation.w = adjusted_rot.toQuaternion().w();
        odom_msg.pose.pose.orientation.x = adjusted_rot.toQuaternion().x();
        odom_msg.pose.pose.orientation.y = adjusted_rot.toQuaternion().y();
        odom_msg.pose.pose.orientation.z = adjusted_rot.toQuaternion().z();
        
        odom_pub_->publish(odom_msg);

        // ========================================================
        // [THÊM MỚI] 2. Dịch X,Y,Z về Lat,Lon để vẽ bản đồ vệ tinh
        // ========================================================
        if (state_ == RUNNING) {
            double opt_lat, opt_lon, opt_alt;
            convert_metric_to_gps(optimized_pose.translation().x(), 
                                  optimized_pose.translation().y(), 
                                  optimized_pose.translation().z(), 
                                  opt_lat, opt_lon, opt_alt);

            sensor_msgs::msg::NavSatFix fix_msg;
            fix_msg.header.stamp = stamp;
            fix_msg.header.frame_id = base_frame_id_;
            fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            fix_msg.latitude = opt_lat;
            fix_msg.longitude = opt_lon;
            fix_msg.altitude = opt_alt;
            
            fix_pub_->publish(fix_msg);
        }
    }

    void publish_path(rclcpp::Time stamp) {
        if (isamCurrentEstimate_.empty()) return;

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = global_frame_id_;

        Rot3 R_offset = Rot3::Ypr(output_yaw_offset_rad_, 0, 0);

        for (size_t i = 0; i < isamCurrentEstimate_.size(); ++i) {
            Pose3 p = isamCurrentEstimate_.at<Pose3>(i);
            Rot3 adjusted_rot = p.rotation() * R_offset;
            
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = p.translation().x();
            ps.pose.position.y = p.translation().y();
            ps.pose.position.z = p.translation().z();
            
            ps.pose.orientation.w = adjusted_rot.toQuaternion().w();
            ps.pose.orientation.x = adjusted_rot.toQuaternion().x();
            ps.pose.orientation.y = adjusted_rot.toQuaternion().y();
            ps.pose.orientation.z = adjusted_rot.toQuaternion().z();
            path_msg.poses.push_back(ps);
        }
        
        path_pub_->publish(path_msg);
    }

    // Các biến string cấu hình
    std::string odom_sub_topic_;
    std::string gps_sub_topic_;
    std::string odom_pub_topic_;
    std::string path_pub_topic_;
    std::string gps_pub_topic_; // [THÊM MỚI]
    std::string global_frame_id_;
    std::string base_frame_id_;
    std::string gps_frame_type_;

    // Các biến thuật toán
    double kf_meter_gap_, kf_rad_gap_, gps_cov_threshold_;
    double output_yaw_offset_rad_; 
    
    InitState state_;
    Pose3 odom_start_;
    Pose3 T_global_odom_; 
    Pose3 latest_correction_;

    std::vector<nav_msgs::msg::Odometry::SharedPtr> init_odom_buf_;
    sensor_msgs::msg::NavSatFix::SharedPtr last_valid_gps_ = nullptr;
    double last_used_gps_time_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_; // [THÊM MỚI]

    std::queue<nav_msgs::msg::Odometry::SharedPtr> odom_buf_;
    std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gps_buf_;
    std::mutex m_buf_;
    std::thread process_thread_;

    std::unique_ptr<ISAM2> isam_;
    NonlinearFactorGraph gtSAMgraph_;
    Values initialEstimate_;
    Values isamCurrentEstimate_;
    noiseModel::Diagonal::shared_ptr prior_noise_;
    noiseModel::Diagonal::shared_ptr odom_noise_;

    std::vector<Pose3> keyframe_poses_;
    GeographicLib::LocalCartesian geo_converter_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseGraphFusionNode>());
    rclcpp::shutdown();
    return 0;
}