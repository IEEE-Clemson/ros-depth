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
    #realsense_prefix = get_package_share_directory('realsense_examples')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',                                                     default=os.path.join(realsense_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rs_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(realsense_prefix, 'config', 'rs_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_frame']
            ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='bob',
            output='screen',
            arguments=['0', '0', '0.00', '0', '0', '0', 'odom_frame', 'camera_link']
            ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='bob2',
            output='screen',
            arguments=['0.001', '0.012', '-0.016', '0', '0', '0', 'camera_link', 'tracking']
            ),
        Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'camera_link'}],
            remappings=[('depth','/depth/image_rect_raw'),
                        ('depth_camera_info', '/depth/camera_info')],
            ),

        Node(
            package='cartographer_ros',
           node_executable='cartographer_node',
           output='log',
           parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename]),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            node_name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
        Node(
            package='realsense2_camera',
            node_executable='realsense2_camera_node',
            node_name='realsense2_camera_node',
            #namespace='camera',
            parameters=[{'enable_gyro': True, 'enable_pose': True, 'enable_accel': True, 'unite_imu_method': 2}],
        ),
    ])
