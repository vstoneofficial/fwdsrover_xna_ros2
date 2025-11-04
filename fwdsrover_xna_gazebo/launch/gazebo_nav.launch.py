#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------------------
    # Common args (can be overridden at launch)
    # ---------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')   # Use Gazebo /clock
    gui          = LaunchConfiguration('gui',          default='true')   # Show Gazebo client
    verbose      = LaunchConfiguration('verbose',      default='false')  # Gazebo verbosity
    gazebo_pkg   = FindPackageShare('fwdsrover_xna_gazebo')

    # ---------------------------
    # Navigation-related args
    #  - default_map points to a packaged map file; replace if your map differs.
    # ---------------------------
    default_map = PathJoinSubstitution([gazebo_pkg, 'maps', 'test.yaml'])
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([
            FindPackageShare('fwdsrover_xna_navigation'),
            'config', 'nav2_params.yaml'
        ])
    )

    # ---------------------------
    # Gazebo bringup (spawns robot, controllers, and odom bridge)
    # ---------------------------
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fwdsrover_xna_gazebo'),
                'launch', 'gazebo_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'verbose': verbose,
        }.items()
    )

    # ---------------------------
    # Wait for TF(odom -> base_footprint) exactly once, then exit
    # Ensures Nav2 starts only after the odom bridge publishes TF.
    # ---------------------------
    wait_tf = Node(
        package='tf2_ros',
        executable='tf2_echo',
        arguments=['--once', 'odom', 'base_footprint'],
        output='screen'
    )

    # ---------------------------
    # Nav2 stack (map_server, AMCL, planners, controller, RViz via your file)
    # Expects a static occupancy map YAML (map:=...) and params YAML.
    # ---------------------------
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
    
    # ---------------------------
    # wall spawn
    # ---------------------------
    spawn_wall = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('fwdsrover_xna_gazebo'),
             'launch', 'spawn_wall.launch.py'
             ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # ---------------------------
    # rover_sim_node
    # ---------------------------
    rover_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('fwdsrover_xna_bringup'),
             'launch', 'rover_sim.launch.py'
             ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'cmd_topic': '/rover_twist',
            'joint_states_topic': '/joint_states',
            'steer_cmd_topic': '/steer_position_controller/commands',
            'wheel_cmd_topic': '/wheel_velocity_controller/commands',
        }.items()
    )

    # ---------------------------
    # Final description (ordering matters)
    #   Gazebo → TF wait → Nav2
    # ---------------------------
    return LaunchDescription([
        # User arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',  description='Use /clock from Gazebo'),
        DeclareLaunchArgument('gui',          default_value='true',  description='Launch Gazebo client'),
        DeclareLaunchArgument('verbose',      default_value='false', description='Gazebo verbose logging'),
        DeclareLaunchArgument('map',          default_value=default_map, description='Full path to map YAML'),
        DeclareLaunchArgument('params_file',  default_value=params_file,  description='Nav2 params YAML'),

        # Sequence
        gazebo_bringup,
        spawn_wall,
        rover_sim,
        wait_tf,
        navigation,
    ])

