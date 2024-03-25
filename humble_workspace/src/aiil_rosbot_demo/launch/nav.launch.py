import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    aiil_rosbot_demo_dir = get_package_share_directory('aiil_rosbot_demo')
    nav2_bringup_launch_file_dir = os.path.join(
        # get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(aiil_rosbot_demo_dir, 'config', 'navigation.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation (Gazebo) clock if true'
    )

    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam', default_value='True', description='Use SLAM mapping instead of a fixed map'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'params_file': params_file,
            'slam': use_slam,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_slam_cmd)

    ld.add_action(nav2_bringup_launch)

    return ld