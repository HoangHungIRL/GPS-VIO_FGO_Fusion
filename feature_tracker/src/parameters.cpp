#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

// DUY ADD
std::string POINT_CLOUD_TOPIC;
double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;
int USE_LIDAR;
int LIDAR_SKIP;


// ✅ Define global variables
cv::Mat CAMERA_IMU_ROTATION;
cv::Mat CAMERA_IMU_TRANSLATION;
double C_I_TX, C_I_TY, C_I_TZ;  // Translation: Camera → IMU
double C_I_RX, C_I_RY, C_I_RZ;  // Rotation (RPY): Camera → IMU
//



template <typename T>
T readParam(const rclcpp::Node::SharedPtr &n, const std::string &name)
{
    T ans;
    if (n->get_parameter(name, ans))
    {
        RCLCPP_INFO_STREAM(n->get_logger(), AT << " Loaded " << name << ": " << ans);
    }
    else
    {
        RCLCPP_ERROR_STREAM(n->get_logger(), AT << " Failed to load " << name);
        rclcpp::shutdown();
    }
    return ans;
}

void readParameters(const rclcpp::Node::SharedPtr &n)
{
    n->declare_parameter<std::string>("config_file");
    n->declare_parameter<std::string>("vins_folder");

    std::string config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;

    // DUY ADD
    fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;  // ✅ Now defined
    // lidar configurations
    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["lidar_skip"] >> LIDAR_SKIP;

    L_C_TX = fsSettings["lidar_to_cam_tx"];
    L_C_TY = fsSettings["lidar_to_cam_ty"];
    L_C_TZ = fsSettings["lidar_to_cam_tz"];
    L_C_RX = fsSettings["lidar_to_cam_rx"];
    L_C_RY = fsSettings["lidar_to_cam_ry"];
    L_C_RZ = fsSettings["lidar_to_cam_rz"];


    // ✅ Read Camera-IMU extrinsic
    fsSettings["extrinsicRotation"] >> CAMERA_IMU_ROTATION;
    fsSettings["extrinsicTranslation"] >> CAMERA_IMU_TRANSLATION;
    // Convert rotation matrix to RPY for easier use
    if (!CAMERA_IMU_ROTATION.empty() && !CAMERA_IMU_TRANSLATION.empty())
    {
        // Extract translation
        C_I_TX = CAMERA_IMU_TRANSLATION.at<double>(0, 0);
        C_I_TY = CAMERA_IMU_TRANSLATION.at<double>(1, 0);
        C_I_TZ = CAMERA_IMU_TRANSLATION.at<double>(2, 0);
        
        // Convert rotation matrix to Euler angles (RPY)
        // Using Eigen for conversion
        Eigen::Matrix3d rot_eigen;
        rot_eigen << CAMERA_IMU_ROTATION.at<double>(0, 0), CAMERA_IMU_ROTATION.at<double>(0, 1), CAMERA_IMU_ROTATION.at<double>(0, 2),
                     CAMERA_IMU_ROTATION.at<double>(1, 0), CAMERA_IMU_ROTATION.at<double>(1, 1), CAMERA_IMU_ROTATION.at<double>(1, 2),
                     CAMERA_IMU_ROTATION.at<double>(2, 0), CAMERA_IMU_ROTATION.at<double>(2, 1), CAMERA_IMU_ROTATION.at<double>(2, 2);
        
        // Convert to RPY (ZYX Euler angles)
        Eigen::Vector3d euler = rot_eigen.eulerAngles(2, 1, 0);  // Yaw, Pitch, Roll
        C_I_RZ = euler(0);  // Yaw (around Z)
        C_I_RY = euler(1);  // Pitch (around Y)
        C_I_RX = euler(2);  // Roll (around X)
        
        cout << "Camera-IMU Extrinsic loaded:" << endl;
        cout << "  Translation: [" << C_I_TX << ", " << C_I_TY << ", " << C_I_TZ << "]" << endl;
        cout << "  Rotation (RPY): [" << C_I_RX << ", " << C_I_RY << ", " << C_I_RZ << "] rad" << endl;
        cout << "  Rotation (RPY): [" << C_I_RX*180/M_PI << ", " << C_I_RY*180/M_PI << ", " << C_I_RZ*180/M_PI << "] deg" << endl;
    }
    else
    {
        cout << "Camera-IMU extrinsic not found in config!" << endl;
        // Set to identity
        C_I_TX = C_I_TY = C_I_TZ = 0.0;
        C_I_RX = C_I_RY = C_I_RZ = 0.0;
    }
    // 


    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
    {
        FISHEYE_MASK = "/home/duy/VINS_PL_ws/src/vins-mono-ros2/vins_estimator/config/fisheye_mask_848_480.png";
    }
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
