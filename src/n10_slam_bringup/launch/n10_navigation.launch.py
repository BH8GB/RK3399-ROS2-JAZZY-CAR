import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('n10_slam_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'n10_slam.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_base': 'True',
            'base_port': LaunchConfiguration('base_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': os.path.join(package_share, 'config', 'nav2_params.yaml'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('base_port', default_value='/dev/robot_base'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/n10_lidar'),
        mapping,
        navigation,
    ])
