import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    package_share = get_package_share_directory('n10_slam_bringup')
    lidar_share = get_package_share_directory('lslidar_driver')

    # 厂商驱动是普通节点，覆盖串口参数，避免两个 USB 串口设备混淆。
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
        condition=IfCondition(LaunchConfiguration('use_base')),
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

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'slam_toolbox.yaml'),
            {
                'use_sim_time': False,
                'resolution': ParameterValue(
                    LaunchConfiguration('resolution'), value_type=float
                ),
                'map_update_interval': ParameterValue(
                    LaunchConfiguration('map_update_interval'), value_type=float
                ),
                'minimum_time_interval': ParameterValue(
                    LaunchConfiguration('minimum_time_interval'), value_type=float
                ),
                'minimum_travel_distance': ParameterValue(
                    LaunchConfiguration('minimum_travel_distance'), value_type=float
                ),
                'minimum_travel_heading': ParameterValue(
                    LaunchConfiguration('minimum_travel_heading'), value_type=float
                ),
            },
        ],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
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
            'use_rviz',
            default_value='true',
            description='Start RViz2 together with the lidar and SLAM nodes.',
        ),
        DeclareLaunchArgument(
            'use_base',
            default_value='true',
            description='Start the UART8 base controller and wheel odometry.',
        ),
        DeclareLaunchArgument(
            'base_port',
            default_value='/dev/robot_base',
            description='Linux serial device for the STM32 UART8 adapter.',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/n10_lidar',
            description='Stable udev alias for the N10 lidar.',
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Map cell size in meters, e.g. 0.02 for 2 cm/pixel.',
        ),
        DeclareLaunchArgument('map_update_interval', default_value='0.5'),
        DeclareLaunchArgument('minimum_time_interval', default_value='0.02'),
        DeclareLaunchArgument('minimum_travel_distance', default_value='0.0'),
        DeclareLaunchArgument('minimum_travel_heading', default_value='0.0'),
        lidar,
        base,
        base_to_laser,
        slam,
        configure_slam,
        activate_slam,
        rviz,
    ])
