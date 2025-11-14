from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # パラメータ定義
    scan_period = LaunchConfiguration('scan_period', default='0.1')
    max_dis = LaunchConfiguration('max_dis', default='3.0')
    min_dis = LaunchConfiguration('min_dis', default='0.05')
    map_resolution = LaunchConfiguration('map_resolution', default='0.05')
    
    return LaunchDescription([
        # 特徴抽出ノード
        Node(
            package='ssl_slam_ros2',
            executable='ssl_feature_extraction_node',
            name='ssl_feature_extraction_node',
            output='screen'
        ),
        
        # オドメトリ推定ノード
        Node(
            package='ssl_slam_ros2',
            executable='odom_estimation_node_ros2',
            name='odom_estimation_node',
            output='screen',
            parameters=[{
                'scan_period': scan_period,
                'max_dis': max_dis,
                'min_dis': min_dis,
                'map_resolution': map_resolution
            }]
        ),
        
        # RViz2起動（オプション）
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', '/path/to/config.rviz']
        # )
    ])