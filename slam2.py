import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rtabmap_ros'),
                    'launch/rtabmap.launch.py'
                ]),
            ]),
            launch_arguments={
               'args': '--delete_db_on_start',
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'depth_camera_info_topic': '/camera/depth/camera_info',
                'rtabmapviz': 'False',
                'rviz': 'True',
                'frame_id': 'camera_link',
            }.items()
        ),
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[os.path.join(realsense_prefix, 'config', 'ukf.yaml')],
        ),
        Node(
            package='imu_filter_madgwick',
            node_executable='imu_filter_madgwick_node',
            node_name='ImuFilter',
            parameters=[{'use_mag': False, '_publish_tf': False, '_world_frame': 'enu'}],
            remappings=[
                ('/imu/data_raw', '/camera/imu')
            ],
            output='screen',
        ),
        Node(
            package='realsense2_camera',
            node_executable='realsense2_camera_node',
            node_name='realsense2_camera_node',
            namespace='camera',
            parameters=[{'enable_gyro': True, 'enable_pose': True, 'enable_accel': True, 'unite_imu_method': 2, 'pointcloud.enable': True, 'pointcloud.stream_filter': 2, 'align_depth.enable': True, 'linear_accel_cov': 1.0,
                'rgb_camera.enable_auto_exposure': False,
                'rgb_camera.exposure': 100,
                'rgb_camera.gain': 4096,
                'depth_module.min_distance': 50,
                'depth_module.laser_power': 100}],
        ),

        SetParameter(name='/ukf_se/frequency', value=300),
        SetParameter(name='/ukf_se/base_link_frame', value='camera_link'),
        SetParameter(name='/ukf_se/odom0', value='rtabmap/odom'),
        SetParameter(name='/ukf_se/odom0_relative', value=True),
        SetParameter(name='/ukf_se/odom0_pose_rejection_threshold', value=10000000),
        SetParameter(name='/ukf_se/odom0_twist_rejection_threshold', value=10000000),
        SetParameter(name='/ukf_se/imu0', value='/imu/data'),
        SetParameter(name='/ukf_se/imu0_differential', value=True),
        SetParameter(name='/ukf_se/imu0_relative', value=False),
        SetParameter(name='/ukf_se/use_control', value=False),
    ])
