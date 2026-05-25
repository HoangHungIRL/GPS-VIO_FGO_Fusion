import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ------------------------------------------------------------------
    # 1. Đường dẫn thư mục share và tham số cấu hình
    # ------------------------------------------------------------------
    # Đổi sang tên package mới của bạn
    share_dir = get_package_share_directory('feature_tracker')
    config_file = os.path.join(pkg_dir, 'config', 'fusion_params.yaml')

    # Định nghĩa file cấu hình tham số (mặc định lấy file params.yaml trong thư mục config)
    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
        description='Full path to the ROS 2 parameters file to use.'
    )

    # ------------------------------------------------------------------
    # 2. Khai báo Node imageProjection độc lập
    # ------------------------------------------------------------------
    image_projection_node = Node(
        package='feature_tracker',                      # Thử mục package mới
        executable='feature_tracker_imageProjection',    # Tên executable đã định nghĩa trong CMakeLists.txt
        name='feature_tracker_imageProjection',
        parameters=[params_file],
        output='screen'
    )

    gps_to_local = Node(
        package='gps_odom_fusion',
        executable='gps_to_local_node',
        name='gps_to_local_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/gps/fix', '/fix')  # Lắng nghe GPS thô từ topic /fix
        ]
    )

    # =========================================================================
    # CẬP NHẬT: Pose Graph Fusion (GTSAM)
    # =========================================================================
    pose_graph_fusion = Node(
        package='gps_odom_fusion',
        executable='pose_graph_fusion_node',
        name='pose_graph_fusion_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/external/odom', '/vins_estimator/odometry'), 
            
            # Thay vì đọc '/fix' thô, giờ PGO sẽ đọc GPS đã được dời tâm về base_link
            ('/fix', '/gps/fix_base_link')                 
        ]
    )

    # Thêm các thành phần cần thực thi vào LaunchDescription
    return LaunchDescription([
        declare_params,
        image_projection_node,
        gps_to_local,
        pose_graph_fusion
    ])
