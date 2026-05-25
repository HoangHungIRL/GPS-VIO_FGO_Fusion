#!/usr/bin/env python3

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的share目录
    package_share_directory = get_package_share_directory('vins_estimator')

    # 读取配置文件
    config_file_path = os.path.join(package_share_directory, 'config', 'euroc', 'euroc_config.yaml')

    # 定义节点
    feature_tracker = launch_ros.actions.Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        output='log',
        parameters=[{"vins_folder": package_share_directory}, {"config_file": config_file_path}]
    )

    vins_estimator = launch_ros.actions.Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        output='screen',
        parameters=[{"vins_folder": package_share_directory}, {"config_file": config_file_path}]
    )

    pose_graph = launch_ros.actions.Node(
        package='pose_graph',
        executable='pose_graph',
        name='pose_graph',
        output='screen',
        parameters=[
            {"config_file": config_file_path},
            {"visualization_shift_x": 0},
            {"visualization_shift_y": 0},
            {"skip_cnt": 0},
            {"skip_dis": 0.0}
        ]
    )

    # 定义启动描述
    ld = launch.LaunchDescription([
        feature_tracker,
        vins_estimator,
        pose_graph,
    ])

    return ld
