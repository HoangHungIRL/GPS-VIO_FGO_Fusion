import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ------------------------------------------------------------------
    # 1. Định nghĩa đường dẫn và cấu hình tham số
    # ------------------------------------------------------------------
    vins_share_dir = get_package_share_directory('vins_estimator')
    ft_share_dir = get_package_share_directory('feature_tracker')

    # Tham số từ file XML cũ
    config_path = LaunchConfiguration('config_path')
    declare_config_path = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(vins_share_dir, 'config', 'AGRI', 'AGRI.yaml'),
        description='Full path to VINS configuration file'
    )

    # [ĐÃ SỬA] Thêm LaunchConfiguration để sửa lỗi NameError
    config_gps_fusion_path = LaunchConfiguration('config_gps_fusion_path')
    declare_config_gps_fusion_path = DeclareLaunchArgument(
        'config_gps_fusion_path',
        # Lưu ý: Ở bước trước bạn đã gộp tham số vào chung file AGRI.yaml, 
        # nếu bạn vẫn tách ra file riêng thì giữ nguyên đường dẫn này.
        default_value=os.path.join(vins_share_dir, 'config', 'AGRI', 'gps_fusion_params.yaml'),
        description='Full path to GPS fusion configuration file'
    )

    vins_path = LaunchConfiguration('vins_path')
    declare_vins_path = DeclareLaunchArgument(
        'vins_path',
        default_value=vins_share_dir,
        description='Path to vins_estimator share directory'
    )

    # Tham số params_file từ file Python cũ
    params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ft_share_dir, 'config', 'params.yaml'),
        description='Full path to the ROS 2 parameters file for image projection'
    )

    # ------------------------------------------------------------------
    # 2. Khai báo các Node
    # ------------------------------------------------------------------
    
    # Node từ file Python cũ
    image_projection_node = Node(
        package='feature_tracker',
        executable='feature_tracker_imageProjection',
        name='feature_tracker_imageProjection',
        parameters=[params_file],
        output='screen'
    )

    # Các Node từ file XML cũ
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker_node',
        name='feature_tracker',
        parameters=[
            {'config_file': config_path},
            {'vins_folder': vins_path}
        ],
        output='log'
    )

    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        parameters=[
            {'config_file': config_path},
            {'vins_folder': vins_path}
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        arguments=['-d', os.path.join(vins_share_dir, 'config', 'vins_rviz_config.rviz')],
        output='log'
    )

    # =========================================================================
    # [ĐÃ SỬA] CẬP NHẬT CÁCH TRUYỀN PARAMETER CHO NODE MỚI
    # Truyền trực tiếp file YAML vào thay vì biến config_file
    # =========================================================================
    gps_to_local = Node(
        package='feature_tracker',
        executable='gps_to_local_node',
        name='gps_to_local_node',
        parameters=[config_gps_fusion_path],
        output='screen',     
    )

    pose_graph_fusion = Node(
        package='feature_tracker',
        executable='pose_graph_fusion_node',
        name='pose_graph_fusion_node',
        parameters=[config_gps_fusion_path],
        output='screen',              
    )

    # ------------------------------------------------------------------
    # 3. Trả về LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription([
        # Khai báo Arguments trước
        declare_config_path,
        declare_config_gps_fusion_path, # [ĐÃ SỬA] Phải add dòng này vào LaunchDescription
        declare_vins_path,
        declare_params_file,

        # Khai báo các Node thực thi
        image_projection_node,
        feature_tracker_node,
        vins_estimator_node,
        rviz_node,
        gps_to_local,
        pose_graph_fusion
    ])