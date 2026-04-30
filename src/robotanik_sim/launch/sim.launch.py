import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robotanik_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robotanik.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'robotanik_sera.world')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    map_yaml_path = os.path.join(pkg_share, 'maps', 'robotanik_sera.yaml')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )

    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'robotanik',
                    '-file', urdf_path,
                    '-x', '10.45',
                    '-y', '1.0',
                    '-z', '0.2',
                    '-Y', '1.5708'
                ],
                output='screen'
            )
        ]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_node = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_yaml_path,
                    'params_file': nav2_params_path,
                    'use_sim_time': 'true',
                    'use_composition': 'False'
                }.items(),
            )
        ]
    )

    row_fsm_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='robotanik_sim',
                executable='row_fsm.py',
                name='robotanik_row_fsm',
                output='screen',
                parameters=[{
                    'use_sim_time': True
                }]
            )
        ]
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        static_tf_node,
        spawn_entity,
        rviz_node,
        nav2_node,
        row_fsm_node,
    ])
