#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ------- common args -------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui          = LaunchConfiguration('gui', default='true')
    verbose      = LaunchConfiguration('verbose', default='false')

    # frames / topics
    odom_frame   = LaunchConfiguration('odom_frame', default='odom')
    base_frame   = LaunchConfiguration('base_frame', default='base_footprint')
    map_frame    = LaunchConfiguration('map_frame', default='map')
    scan_topic   = LaunchConfiguration('scan_topic', default='/scan')

    # params (pkg 既定の YAML をデフォルトに)
    slam_params = LaunchConfiguration(
        'slam_params',
        default=PathJoinSubstitution([
            FindPackageShare('fwdsrover_xna_navigation'),
            'config', 'mapper_params_online_async.yaml'
        ])
    )

    # ------- Gazebo + robot bringup -------
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
        }.items()
    )

    # （必要な場合だけ使う想定。通常は不要）
    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser_sim',
        arguments=['0','0','0','0','0','0', base_frame, 'front_lrf_link'],
        output='screen'
    )

    # ------- SLAM include (slam_toolbox) -------
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fwdsrover_xna_navigation'),
                'launch', 'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,   # ここは slam.launch.py の引数名に合わせる
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'map_frame': map_frame,
            'scan_topic': scan_topic,          # ← 追加：scan を確実に渡す
        }.items()
    )

    # Gazebo 側が立ち上がって /scan /tf が流れ始めてから SLAM を起動
    slam_after = TimerAction(period=6.0, actions=[slam_include])

    return LaunchDescription([
        # args
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('slam_params', default_value=PathJoinSubstitution([
            FindPackageShare('fwdsrover_xna_navigation'),
            'config', 'mapper_params_online_async.yaml'
        ])),

        # sequence
        gazebo_bringup,
        # static_laser_tf,  # 通常はコメントアウトでOK（必要なら有効化）
        slam_after,
    ])

