import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    realsense_prefix = os.getcwd()
    slam_cfg = os.path.join(realsense_prefix, 'config', 'params.yaml')
    #realsense_prefix = get_package_share_directory('realsense_examples')
    cartographer_config_dir = LaunchConfiguration('file_path', default=os.path.join(realsense_prefix, 'config', 'params.yaml'))
    print(slam_cfg)
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(realsense_prefix, 'config', 'rs_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'file_path',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='word2map_tf',
            output='screen',
            arguments=['0.00', '0.0', '0.0', '1.57', '3.14', '1.57', '/world', '/map']
            ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='bob2',
            output='screen',
            arguments=['0.00', '0.0', '0.0', '0', '0', '0', '/odom', '/camera_depth_optical_frame']
            ),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='ssl_slam3',
            node_executable='ssl_slam3_laser_processing_node',
            node_name='ssl_slam3_laser_processing_node',
            parameters=[{'file_path': slam_cfg}],
            #prefix=['gdbserver localhost:3000'],
            output='screen',
        ),
        Node(
            package='ssl_slam3',
            node_executable='ssl_slam3_odom_estimation_node',
            node_name='ssl_slam3_odom_estimation_node',
            parameters=[{'file_path': slam_cfg}],
            #prefix=['gdbserver localhost:3000'],
            output='screen',
        ),
        Node(
            package='ssl_slam3',
            node_executable='ssl_slam3_laser_mapping_node',
            node_name='ssl_slam3_laser_mapping_node',
            output='screen',
        ),
        Node(
            package='realsense2_camera',
            node_executable='realsense2_camera_node',
            node_name='realsense2_camera_node',
            #namespace='camera',
            parameters=[{'enable_gyro': True, 'enable_pose': True, 'enable_accel': True, 'unite_imu_method': 2, 'pointcloud.enable': True, 'pointcloud.stream_filter': 2}],
        ),
    ])
