#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- Common args ----
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui          = LaunchConfiguration('gui', default='true')
    verbose      = LaunchConfiguration('verbose', default='false')

    # ---- Nav2 args ----
    map_file = LaunchConfiguration('map')  # 実機と同様に外から指定
    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([
            FindPackageShare('fwdsrover_xna_navigation'),
            'config', 'nav2_params.yaml'
        ])
    )

    # ---- Gazebo + robot bringup (odometry bridge 内蔵) ----
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fwdsrover_xna_bringup'),
                'launch', 'gazebo_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'verbose': verbose,
            # gazebo_bringup 内で gazebo_odom_bridge を起動し /odom を出力
        }.items()
    )

    # ---- Navigation (Nav2) ----
    # 既にお持ちの navigation.launch.py を再利用（map_server, amcl, nav2_bringup, rviz を内包）
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fwdsrover_xna_navigation'),
                'launch', 'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Gazebo 側が立ち上がって /odom・/scan・TF が安定してから Nav2 を起動
    nav_after = TimerAction(period=6.0, actions=[navigation])

    return LaunchDescription([
        # ---- Args ----
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('map', description='Full path to map YAML'),
        DeclareLaunchArgument('params_file', default_value=params_file),

        # ---- Sequence ----
        gazebo_bringup,
        nav_after,
    ])

