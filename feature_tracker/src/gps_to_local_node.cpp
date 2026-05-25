#include <memory>
#include <cmath>
#include <vector>
#include <deque>
#include <mutex>
#include <string>
#include <atomic>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <Eigen/Geometry>

const double WGS84_A = 6378137.0;           
const double WGS84_F = 1.0 / 298.257223563; 
const double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

class TGINSLocalConverter : public rclcpp::Node {
public:
    TGINSLocalConverter() : Node("tgins_local_converter") {
        // ========================================================
        // KHAI BÁO PARAMETERS
        // ========================================================
        
        // 1. Topics
        this->declare_parameter<std::string>("gps_sub_topic", "/fix");
        this->declare_parameter<std::string>("imu_sub_topic", "/imu/data_newIMU");
        this->declare_parameter<std::string>("gps_odom_pub_topic", "/gps/odom_local");
        this->declare_parameter<std::string>("imu_odom_pub_topic", "/imu_eskf/odom");
        this->declare_parameter<std::string>("gps_path_pub_topic", "/gps/path");
        this->declare_parameter<std::string>("gps_raw_path_pub_topic", "/gps/raw_path");
        this->declare_parameter<std::string>("base_link_gps_pub_topic", "/gps/fix_base_link");

        // 2. Frames
        this->declare_parameter<std::string>("global_frame_id", "world");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<std::string>("gps_frame_type", "ned"); 
        this->declare_parameter<std::string>("imu_frame_type", "ned");

        // 3. Lever Arm (Offset từ base_link tới Antenna GPS)
        this->declare_parameter<double>("lever_arm_x", -3.3);
        this->declare_parameter<double>("lever_arm_y", 0.7);
        this->declare_parameter<double>("lever_arm_z", 0.0);

        // 4. Thuật toán Alignment & Update
        this->declare_parameter<double>("min_align_dist", 1.5); 
        this->declare_parameter<bool>("enable_yaw_alignment", true);
        this->declare_parameter<bool>("planar_mode", true); 
        this->declare_parameter<bool>("continuous_yaw_update", true);
        this->declare_parameter<double>("update_distance", 3.0); 
        this->declare_parameter<double>("yaw_update_rate", 0.05); 

        // 5. Phát hiện trạng thái tĩnh (Static Detection)
        this->declare_parameter<bool>("enable_static_detection", true);
        this->declare_parameter<double>("imu_accel_var_thresh", 0.05); 
        this->declare_parameter<double>("imu_gyro_thresh", 0.02);      

        // ========================================================
        // ĐỌC PARAMETERS VÀO BIẾN
        // ========================================================
        
        std::string gps_sub_topic = this->get_parameter("gps_sub_topic").as_string();
        std::string imu_sub_topic = this->get_parameter("imu_sub_topic").as_string();
        std::string gps_odom_pub_topic = this->get_parameter("gps_odom_pub_topic").as_string();
        std::string imu_odom_pub_topic = this->get_parameter("imu_odom_pub_topic").as_string();
        std::string gps_path_pub_topic = this->get_parameter("gps_path_pub_topic").as_string();
        std::string gps_raw_path_pub_topic = this->get_parameter("gps_raw_path_pub_topic").as_string();
        std::string base_link_gps_pub_topic = this->get_parameter("base_link_gps_pub_topic").as_string();

        global_frame_id_ = this->get_parameter("global_frame_id").as_string();
        base_frame_id_   = this->get_parameter("base_frame_id").as_string();
        gps_frame_type_  = this->get_parameter("gps_frame_type").as_string();
        imu_frame_type_  = this->get_parameter("imu_frame_type").as_string();

        lever_arm_x_ = this->get_parameter("lever_arm_x").as_double();
        lever_arm_y_ = this->get_parameter("lever_arm_y").as_double();
        lever_arm_z_ = this->get_parameter("lever_arm_z").as_double();

        min_align_dist_ = this->get_parameter("min_align_dist").as_double();
        enable_yaw_alignment_ = this->get_parameter("enable_yaw_alignment").as_bool();
        planar_mode_ = this->get_parameter("planar_mode").as_bool();
        continuous_yaw_update_ = this->get_parameter("continuous_yaw_update").as_bool();
        update_distance_ = this->get_parameter("update_distance").as_double();
        yaw_update_rate_ = this->get_parameter("yaw_update_rate").as_double();
        
        enable_static_detection_ = this->get_parameter("enable_static_detection").as_bool();
        imu_accel_var_thresh_ = this->get_parameter("imu_accel_var_thresh").as_double();
        imu_gyro_thresh_ = this->get_parameter("imu_gyro_thresh").as_double();

        // Cấu hình khởi tạo frame cho Path msg
        raw_path_msg_.header.frame_id = global_frame_id_;
        path_msg_.header.frame_id = global_frame_id_;

        // ========================================================
        // KHỞI TẠO SUBSCRIBERS / PUBLISHERS
        // ========================================================
        
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_sub_topic, 10, std::bind(&TGINSLocalConverter::gps_callback, this, std::placeholders::_1));

        eskf_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_sub_topic, 100, std::bind(&TGINSLocalConverter::eskf_callback, this, std::placeholders::_1));

        gps_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(gps_odom_pub_topic, 100); 
        imu_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(imu_odom_pub_topic, 100);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(gps_path_pub_topic, 10);
        raw_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(gps_raw_path_pub_topic, 10);
        base_link_gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(base_link_gps_pub_topic, 10);
    }

private:
    void eskf_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(mtx_eskf_);
            latest_raw_imu_quat_ = msg->orientation;
            
            if (enable_static_detection_) {
                static_imu_buffer_.push_back(*msg);
                if (static_imu_buffer_.size() > 50) { 
                    static_imu_buffer_.pop_front();
                }

                if (static_imu_buffer_.size() == 50) {
                    double sum_ax = 0, sum_ay = 0, sum_az = 0;
                    double max_g = 0;

                    for (const auto& imu_data : static_imu_buffer_) {
                        sum_ax += imu_data.linear_acceleration.x;
                        sum_ay += imu_data.linear_acceleration.y;
                        sum_az += imu_data.linear_acceleration.z;
                        
                        double g_mag = std::sqrt(std::pow(imu_data.angular_velocity.x, 2) + 
                                                 std::pow(imu_data.angular_velocity.y, 2) + 
                                                 std::pow(imu_data.angular_velocity.z, 2));
                        if (g_mag > max_g) max_g = g_mag;
                    }

                    double mean_ax = sum_ax / 50.0;
                    double mean_ay = sum_ay / 50.0;
                    double mean_az = sum_az / 50.0;

                    double var_ax = 0, var_ay = 0, var_az = 0;
                    for (const auto& imu_data : static_imu_buffer_) {
                        var_ax += std::pow(imu_data.linear_acceleration.x - mean_ax, 2);
                        var_ay += std::pow(imu_data.linear_acceleration.y - mean_ay, 2);
                        var_az += std::pow(imu_data.linear_acceleration.z - mean_az, 2);
                    }
                    var_ax /= 50.0; var_ay /= 50.0; var_az /= 50.0;

                    double total_accel_var = var_ax + var_ay + var_az;

                    if (total_accel_var < imu_accel_var_thresh_ && max_g < imu_gyro_thresh_) {
                        is_imu_static_ = true;
                    } else {
                        is_imu_static_ = false;
                    }
                }
            }
        }

        if (!is_yaw_aligned_) {
            std::lock_guard<std::mutex> lock(mtx_eskf_);
            eskf_queue_.push_back(*msg);
            if (eskf_queue_.size() > 500) eskf_queue_.pop_front();
            return; 
        }

        Eigen::Quaterniond q_imu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        Eigen::Quaterniond q_offset(Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ()));
        
        Eigen::Quaterniond q_aligned = q_offset * q_imu;
        q_aligned.normalize();

        Eigen::Quaterniond q_imu_out = q_aligned;

        if (gps_frame_type_ != imu_frame_type_) {
            Eigen::Matrix3d M_trans;
            M_trans << 0, 1, 0,  
                       1, 0, 0,  
                       0, 0, -1; 
            Eigen::Quaterniond q_trans(M_trans);
            q_imu_out = q_trans * q_aligned;
        }
        q_imu_out.normalize();
        
        nav_msgs::msg::Odometry imu_odom;
        imu_odom.header = msg->header;
        imu_odom.header.frame_id = global_frame_id_; 
        imu_odom.child_frame_id = base_frame_id_;
        imu_odom.pose.pose.position.x = 0.0;
        imu_odom.pose.pose.position.y = 0.0;
        imu_odom.pose.pose.position.z = 0.0;
        imu_odom.pose.pose.orientation.w = q_imu_out.w();
        imu_odom.pose.pose.orientation.x = q_imu_out.x();
        imu_odom.pose.pose.orientation.y = q_imu_out.y();
        imu_odom.pose.pose.orientation.z = q_imu_out.z();

        imu_odom_pub_->publish(imu_odom);

        if (has_first_gps_) {
            double imu_time = rclcpp::Time(msg->header.stamp).seconds();
            double ext_dt = imu_time - last_gps_time_;
            double ext_vx = gps_vx_;
            double ext_vy = gps_vy_;
            double ext_vz = gps_vz_;

            if (ext_dt < 0.0) {
                ext_dt = 0.0; 
            } else if (ext_dt > 1.0) {
                ext_vx = 0.0;
                ext_vy = 0.0;
                ext_vz = 0.0;
            }

            double extrapolated_x = extrapolate_base_x_ + (ext_vx * ext_dt);
            double extrapolated_y = extrapolate_base_y_ + (ext_vy * ext_dt);
            double extrapolated_z = planar_mode_ ? 0.0 : (extrapolate_base_z_ + (ext_vz * ext_dt));

            // SỬ DỤNG LEVER ARM TỪ PARAMETER
            Eigen::Vector3d lever_arm(lever_arm_x_, lever_arm_y_, lever_arm_z_); 
            Eigen::Matrix3d R = q_aligned.toRotationMatrix(); 
            Eigen::Vector3d pos_gps(extrapolated_x, extrapolated_y, extrapolated_z);
            Eigen::Vector3d pos_imu = pos_gps - (R * lever_arm);

            nav_msgs::msg::Odometry gps_odom;
            gps_odom.header.stamp = msg->header.stamp; 
            gps_odom.header.frame_id = global_frame_id_;
            gps_odom.child_frame_id = base_frame_id_;
            
            gps_odom.pose.pose.position.x = pos_imu.x();
            gps_odom.pose.pose.position.y = pos_imu.y();
            gps_odom.pose.pose.position.z = planar_mode_ ? 0.0 : pos_imu.z(); 
            
            gps_odom.pose.pose.orientation.w = 1.0;
            gps_odom.pose.pose.orientation.x = 0.0;
            gps_odom.pose.pose.orientation.y = 0.0;
            gps_odom.pose.pose.orientation.z = 0.0;
            
            gps_odom_pub_->publish(gps_odom);

            double new_lat, new_lon, new_alt;
            convert_local_to_wgs84(pos_imu.x(), pos_imu.y(), pos_imu.z(), new_lat, new_lon, new_alt);

            sensor_msgs::msg::NavSatFix base_link_fix;
            base_link_fix.header.stamp = msg->header.stamp;
            base_link_fix.header.frame_id = base_frame_id_;
            base_link_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; 
            base_link_fix.latitude = new_lat;
            base_link_fix.longitude = new_lon;
            base_link_fix.altitude = new_alt;
            base_link_gps_pub_->publish(base_link_fix);

            static int path_counter = 0;
            if (path_counter++ % 10 == 0) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = gps_odom.header.stamp;
                pose_msg.header.frame_id = global_frame_id_;
                pose_msg.pose = gps_odom.pose.pose;
                
                path_msg_.header.stamp = gps_odom.header.stamp;
                path_msg_.header.frame_id = global_frame_id_;
                path_msg_.poses.push_back(pose_msg);
                
                if (path_msg_.poses.size() > 2000) path_msg_.poses.erase(path_msg_.poses.begin());
                path_pub_->publish(path_msg_);
            }
        }
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) return;

        if (!is_datum_set_) {
            datum_lat_ = msg->latitude;
            datum_lon_ = msg->longitude;
            datum_alt_ = msg->altitude;
            
            start_x_ = 0.0;
            start_y_ = 0.0;
            last_update_x_ = 0.0;
            last_update_y_ = 0.0;
            
            last_raw_x_ = 0.0;
            last_raw_y_ = 0.0;
            last_raw_z_ = 0.0;
            extrapolate_base_x_ = 0.0;
            extrapolate_base_y_ = 0.0;
            extrapolate_base_z_ = 0.0;
            
            is_datum_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Datum set at: Lat: %f, Lon: %f, Alt: %f", datum_lat_, datum_lon_, datum_alt_);
        }

        double x, y, z;
        convert_wgs84_to_local(msg->latitude, msg->longitude, msg->altitude, x, y, z);
        double gps_time = rclcpp::Time(msg->header.stamp).seconds();

        if (enable_static_detection_) {
            if (is_imu_static_) {
                if (!was_static_) {
                    static_locked_x_ = extrapolate_base_x_;
                    static_locked_y_ = extrapolate_base_y_;
                    static_locked_z_ = extrapolate_base_z_;
                    was_static_ = true;
                }
                x = static_locked_x_;
                y = static_locked_y_;
                z = static_locked_z_;
                gps_vx_ = 0.0;
                gps_vy_ = 0.0;
                gps_vz_ = 0.0;
                extrapolate_base_x_ = x;
                extrapolate_base_y_ = y;
                extrapolate_base_z_ = z;
            } else {
                if (was_static_) {
                    was_static_ = false;
                    last_gps_time_ = gps_time; 
                    last_raw_x_ = x;
                    last_raw_y_ = y;
                    last_raw_z_ = z;
                }
                if (has_first_gps_ && last_gps_time_ > 0.0) {
                    double dt = gps_time - last_gps_time_;
                    if (dt > 0.001) {
                        gps_vx_ = (x - last_raw_x_) / dt;
                        gps_vy_ = (y - last_raw_y_) / dt;
                        gps_vz_ = planar_mode_ ? 0.0 : ((z - last_raw_z_) / dt);
                    }
                }
                extrapolate_base_x_ = x;
                extrapolate_base_y_ = y;
                extrapolate_base_z_ = z;
            }
        } else {
            if (has_first_gps_ && last_gps_time_ > 0.0) {
                double dt = gps_time - last_gps_time_;
                if (dt > 0.001) {
                    gps_vx_ = (x - last_raw_x_) / dt;
                    gps_vy_ = (y - last_raw_y_) / dt;
                    gps_vz_ = planar_mode_ ? 0.0 : ((z - last_raw_z_) / dt);
                }
            }
            extrapolate_base_x_ = x;
            extrapolate_base_y_ = y;
            extrapolate_base_z_ = z;
        }

        last_raw_x_ = x;
        last_raw_y_ = y;
        last_raw_z_ = z;
        last_gps_time_ = gps_time;
        has_first_gps_ = true;

        geometry_msgs::msg::PoseStamped raw_pose_msg;
        raw_pose_msg.header.stamp = msg->header.stamp;
        raw_pose_msg.header.frame_id = global_frame_id_; 
        raw_pose_msg.pose.position.x = x;
        raw_pose_msg.pose.position.y = y;
        raw_pose_msg.pose.position.z = z;
        
        raw_pose_msg.pose.orientation.x = 0.0;
        raw_pose_msg.pose.orientation.y = 0.0;
        raw_pose_msg.pose.orientation.z = 0.0;
        raw_pose_msg.pose.orientation.w = 1.0; 

        raw_path_msg_.header.stamp = msg->header.stamp;
        raw_path_msg_.header.frame_id = global_frame_id_;
        raw_path_msg_.poses.push_back(raw_pose_msg);
        if (raw_path_msg_.poses.size() > 2000) raw_path_msg_.poses.erase(raw_path_msg_.poses.begin());
        raw_path_pub_->publish(raw_path_msg_);

        if (!is_yaw_aligned_) {
            if (!enable_yaw_alignment_) {
                yaw_offset_ = 0.0; 
                is_yaw_aligned_ = true; 
                std::lock_guard<std::mutex> lock(mtx_eskf_);
                eskf_queue_.clear();
            } else {
                double dist = std::hypot(x - start_x_, y - start_y_);
                if (dist >= min_align_dist_) {
                    geometry_msgs::msg::Quaternion synced_quat;
                    bool got_eskf = false;
                    {
                        std::lock_guard<std::mutex> lock(mtx_eskf_);
                        if (!eskf_queue_.empty()) {
                            while (eskf_queue_.size() > 1 && rclcpp::Time(eskf_queue_[1].header.stamp).seconds() <= gps_time) {
                                eskf_queue_.pop_front();
                            }
                            if (rclcpp::Time(eskf_queue_.front().header.stamp).seconds() >= gps_time || eskf_queue_.size() == 1) {
                                synced_quat = eskf_queue_.front().orientation;
                            } else {
                                double t0 = rclcpp::Time(eskf_queue_[0].header.stamp).seconds();
                                double t1 = rclcpp::Time(eskf_queue_[1].header.stamp).seconds();
                                double ratio = (gps_time - t0) / (t1 - t0);
                                Eigen::Quaterniond q0(eskf_queue_[0].orientation.w, eskf_queue_[0].orientation.x, eskf_queue_[0].orientation.y, eskf_queue_[0].orientation.z);
                                Eigen::Quaterniond q1(eskf_queue_[1].orientation.w, eskf_queue_[1].orientation.x, eskf_queue_[1].orientation.y, eskf_queue_[1].orientation.z);
                                Eigen::Quaterniond q_interp = q0.slerp(ratio, q1);
                                q_interp.normalize();
                                synced_quat.w = q_interp.w();
                                synced_quat.x = q_interp.x();
                                synced_quat.y = q_interp.y();
                                synced_quat.z = q_interp.z();
                            }
                            got_eskf = true;
                        }
                    }

                    if (got_eskf) {
                        double gps_yaw = std::atan2(y - start_y_, x - start_x_);
                        double imu_yaw = std::atan2(2.0 * (synced_quat.w * synced_quat.z + synced_quat.x * synced_quat.y), 
                                                    1.0 - 2.0 * (synced_quat.y * synced_quat.y + synced_quat.z * synced_quat.z));
                        yaw_offset_ = gps_yaw - imu_yaw;
                        while (yaw_offset_ > M_PI)  yaw_offset_ -= 2.0 * M_PI;
                        while (yaw_offset_ < -M_PI) yaw_offset_ += 2.0 * M_PI;
                        is_yaw_aligned_ = true;
                        
                        RCLCPP_INFO(this->get_logger(), 
                            "\033[1;32m[KINEMATIC ALIGNMENT] Success! GPS_Yaw: %.2f deg | IMU_Yaw: %.2f deg | Offset: %.2f deg\033[0m", 
                            gps_yaw * 180.0/M_PI, imu_yaw * 180.0/M_PI, yaw_offset_ * 180.0/M_PI);

                        last_update_x_ = x;
                        last_update_y_ = y;
                        
                        {
                            std::lock_guard<std::mutex> lock(mtx_eskf_);
                            eskf_queue_.clear();
                        }
                    }
                }
            }
        }

        if (enable_static_detection_ && is_imu_static_) {
            last_update_x_ = x;
            last_update_y_ = y;
        } 
        else if (continuous_yaw_update_ && is_yaw_aligned_) {
            double update_dist = std::hypot(x - last_update_x_, y - last_update_y_);
            
            if (update_dist >= update_distance_) {
                double gps_segment_yaw = std::atan2(y - last_update_y_, x - last_update_x_);
                
                geometry_msgs::msg::Quaternion current_imu_quat;
                {
                    std::lock_guard<std::mutex> lock(mtx_eskf_);
                    current_imu_quat = latest_raw_imu_quat_; 
                }

                double imu_current_yaw = std::atan2(2.0 * (current_imu_quat.w * current_imu_quat.z + current_imu_quat.x * current_imu_quat.y), 
                                                    1.0 - 2.0 * (current_imu_quat.y * current_imu_quat.y + current_imu_quat.z * current_imu_quat.z));
                
                double new_raw_offset = gps_segment_yaw - imu_current_yaw;
                while (new_raw_offset > M_PI)  new_raw_offset -= 2.0 * M_PI;
                while (new_raw_offset < -M_PI) new_raw_offset += 2.0 * M_PI;

                double offset_error = new_raw_offset - yaw_offset_;
                while (offset_error > M_PI)  offset_error -= 2.0 * M_PI;
                while (offset_error < -M_PI) offset_error += 2.0 * M_PI;

                yaw_offset_ = yaw_offset_ + (offset_error * yaw_update_rate_);
                
                while (yaw_offset_ > M_PI)  yaw_offset_ -= 2.0 * M_PI;
                while (yaw_offset_ < -M_PI) yaw_offset_ += 2.0 * M_PI;

                last_update_x_ = x;
                last_update_y_ = y;
            }
        }
    }

    void convert_wgs84_to_local(double lat, double lon, double alt, double &x, double &y, double &z) {
        double d_x, d_y, d_z;
        wgs84_to_ecef(datum_lat_, datum_lon_, datum_alt_, d_x, d_y, d_z);
        double c_x, c_y, c_z;
        wgs84_to_ecef(lat, lon, alt, c_x, c_y, c_z);

        double dx = c_x - d_x;
        double dy = c_y - d_y;
        double dz = c_z - d_z;

        double phi = datum_lat_ * M_PI / 180.0;
        double lambda = datum_lon_ * M_PI / 180.0;

        double enu_x = -sin(lambda) * dx + cos(lambda) * dy;
        double enu_y = -sin(phi) * cos(lambda) * dx - sin(phi) * sin(lambda) * dy + cos(phi) * dz;
        double enu_z = cos(phi) * cos(lambda) * dx + cos(phi) * sin(lambda) * dy + sin(phi) * dz;

        if (gps_frame_type_ == "ned") {
            x = enu_y;
            y = enu_x;
            z = planar_mode_ ? 0.0 : -enu_z; 
        } else { 
            x = enu_x;
            y = enu_y;
            z = planar_mode_ ? 0.0 : enu_z;  
        }
    }
    
    void convert_local_to_wgs84(double x, double y, double z, double &lat, double &lon, double &alt) {
        double enu_x, enu_y, enu_z;
        if (gps_frame_type_ == "ned") {
            enu_x = y;
            enu_y = x;
            enu_z = planar_mode_ ? 0.0 : -z;
        } else {
            enu_x = x;
            enu_y = y;
            enu_z = planar_mode_ ? 0.0 : z;
        }

        double phi = datum_lat_ * M_PI / 180.0;
        double lambda = datum_lon_ * M_PI / 180.0;

        double sin_phi = std::sin(phi);
        double cos_phi = std::cos(phi);
        double sin_lam = std::sin(lambda);
        double cos_lam = std::cos(lambda);

        double dx = -sin_lam * enu_x - sin_phi * cos_lam * enu_y + cos_phi * cos_lam * enu_z;
        double dy =  cos_lam * enu_x - sin_phi * sin_lam * enu_y + cos_phi * sin_lam * enu_z;
        double dz =                    cos_phi * enu_y           + sin_phi * enu_z;

        double d_x, d_y, d_z;
        wgs84_to_ecef(datum_lat_, datum_lon_, datum_alt_, d_x, d_y, d_z);

        double ecef_x = d_x + dx;
        double ecef_y = d_y + dy;
        double ecef_z = d_z + dz;

        ecef_to_wgs84(ecef_x, ecef_y, ecef_z, lat, lon, alt);
    }

    void wgs84_to_ecef(double lat, double lon, double alt, double &ecef_x, double &ecef_y, double &ecef_z) {
        double phi = lat * M_PI / 180.0;
        double lambda = lon * M_PI / 180.0;
        double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(phi) * std::sin(phi));

        ecef_x = (N + alt) * std::cos(phi) * std::cos(lambda);
        ecef_y = (N + alt) * std::cos(phi) * std::sin(lambda);
        ecef_z = (N * (1.0 - WGS84_E2) + alt) * std::sin(phi);
    }

    void ecef_to_wgs84(double x, double y, double z, double &lat, double &lon, double &alt) {
        double a = WGS84_A;
        double e2 = WGS84_E2;
        double b = std::sqrt(a * a * (1.0 - e2));
        double ep2 = (a * a - b * b) / (b * b);
        double p = std::sqrt(x * x + y * y);
        
        double th = std::atan2(a * z, b * p);
        double sth = std::sin(th);
        double cth = std::cos(th);
        
        double lat_rad = std::atan2(z + ep2 * b * sth * sth * sth,
                                    p - e2 * a * cth * cth * cth);
        double lon_rad = std::atan2(y, x);
        
        double slat = std::sin(lat_rad);
        double N = a / std::sqrt(1.0 - e2 * slat * slat);
        alt = p / std::cos(lat_rad) - N;
        
        lat = lat_rad * 180.0 / M_PI;
        lon = lon_rad * 180.0 / M_PI;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr eskf_sub_; 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr base_link_gps_pub_;

    std::mutex mtx_eskf_;
    std::deque<sensor_msgs::msg::Imu> eskf_queue_;
    std::deque<sensor_msgs::msg::Imu> static_imu_buffer_; 
    geometry_msgs::msg::Quaternion latest_raw_imu_quat_; 
    
    bool has_first_gps_ = false;
    double last_raw_x_ = 0.0;
    double last_raw_y_ = 0.0;
    double last_raw_z_ = 0.0;
    
    double extrapolate_base_x_ = 0.0;
    double extrapolate_base_y_ = 0.0;
    double extrapolate_base_z_ = 0.0;
    
    double last_gps_time_ = -1.0;
    double gps_vx_ = 0.0;
    double gps_vy_ = 0.0;
    double gps_vz_ = 0.0;

    // Các biến parameter được khai báo ra class
    std::string global_frame_id_;
    std::string base_frame_id_;
    std::string gps_frame_type_;
    std::string imu_frame_type_;
    
    double lever_arm_x_;
    double lever_arm_y_;
    double lever_arm_z_;

    bool planar_mode_ = true;
    bool enable_static_detection_ = true;
    double imu_accel_var_thresh_ = 0.05; 
    double imu_gyro_thresh_ = 0.02;      
    std::atomic<bool> is_imu_static_{false};
    bool was_static_ = false;
    double static_locked_x_ = 0.0;
    double static_locked_y_ = 0.0;
    double static_locked_z_ = 0.0;

    bool enable_yaw_alignment_ = true;

    bool is_datum_set_ = false;
    double datum_lat_, datum_lon_, datum_alt_;
    double start_x_ = 0.0, start_y_ = 0.0;
    
    bool is_yaw_aligned_ = false;
    double yaw_offset_ = 0.0;
    double min_align_dist_ = 1.5; 

    bool continuous_yaw_update_ = true;
    double update_distance_ = 3.0;
    double yaw_update_rate_ = 0.05; 
    double last_update_x_ = 0.0, last_update_y_ = 0.0;

    nav_msgs::msg::Path path_msg_;
    nav_msgs::msg::Path raw_path_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TGINSLocalConverter>());
    rclcpp::shutdown();
    return 0;
}