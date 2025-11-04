#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cmd_topic    = LaunchConfiguration('cmd_topic',    default='/rover_twist')
    model_name   = LaunchConfiguration('model_name',   default='x40a')
    scan_topic   = LaunchConfiguration('scan_topic',   default='/scan')
    odom_frame   = LaunchConfiguration('odom_frame',   default='odom')
    base_frame   = LaunchConfiguration('base_frame',   default='base_footprint')
    rviz_config  = LaunchConfiguration(
        'rvizconfig',
        default=PathJoinSubstitution([get_package_share_directory('fwdsrover_xna_navigation'), 'rviz', 'slam.rviz'])
    )

    pkg_desc  = get_package_share_directory('fwdsrover_description')
    pkg_bring = get_package_share_directory('fwdsrover_xna_bringup')
    pkg_nav   = get_package_share_directory('fwdsrover_xna_navigation')
    pkg_gzb   = get_package_share_directory('fwdsrover_xna_gazebo')

    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gzb, 'launch', 'gazebo_bringup.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_wall = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gzb, 'launch', 'spawn_wall.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rover_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_bring, 'launch', 'rover_sim.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'mode': 'normal',
            'cmd_topic': cmd_topic,
            'odo_topic': '/__unused_rover_odo',
            'joint_states_topic': '/joint_states',
            'steer_cmd_topic': '/steer_position_controller/commands',
            'wheel_cmd_topic': '/wheel_velocity_controller/commands',
        }.items()
    )

    gazebo_odom_bridge = Node(
        package='fwdsrover_xna_bringup',
        executable='gazebo_odom_bridge',
        name='gazebo_odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_name': model_name,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'publish_tf': True,
            'flatten_to_2d': True
        }],
    )

    slam_and_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_nav, 'launch', 'slam.launch.py'])),
        launch_arguments={'rvizconfig': rviz_config}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_topic',    default_value='/rover_twist'),
        DeclareLaunchArgument('model_name',   default_value='x40a'),
        DeclareLaunchArgument('scan_topic',   default_value='/scan'),
        DeclareLaunchArgument('odom_frame',   default_value='odom'),
        DeclareLaunchArgument('base_frame',   default_value='base_footprint'),
        DeclareLaunchArgument('rvizconfig',   default_value=PathJoinSubstitution([pkg_nav, 'rviz', 'slam.rviz'])),

        gazebo_bringup,
        spawn_wall,
        rover_sim,
        gazebo_odom_bridge,
        slam_and_rviz,
    ])