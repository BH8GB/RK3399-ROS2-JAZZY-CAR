import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('n10_slam_bringup')
    lidar_share = get_package_share_directory('lslidar_driver')
    nav2_share = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(package_share, 'config', 'nav2_params.yaml')

    lidar = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[
            os.path.join(lidar_share, 'params', 'lidar_uart_ros2', 'lsn10.yaml'),
            {'serial_port_': LaunchConfiguration('lidar_port')},
        ],
    )

    base = Node(
        package='n10_slam_bringup',
        executable='base_serial_node',
        name='base_serial',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'base_serial.yaml'),
            {'port': LaunchConfiguration('base_port')},
        ],
    )

    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--x', '0.110', '--y', '0', '--z', '0.160',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser',
        ],
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
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
            'params_file': params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_share, 'rviz', 'n10_slam.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/root/my_robot_ws/maps/n10_map.yaml',
            description='Saved occupancy-grid YAML map.',
        ),
        DeclareLaunchArgument('base_port', default_value='/dev/robot_base'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/n10_lidar'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        lidar,
        base,
        base_to_laser,
        localization,
        navigation,
        rviz,
    ])
