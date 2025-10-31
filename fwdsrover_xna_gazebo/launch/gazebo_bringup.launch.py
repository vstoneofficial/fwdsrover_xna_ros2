#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
    LogInfo,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
    FindExecutable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _make_world_with_state(context, *args, **kwargs):
    """
    Create a minimal SDF world file in /tmp and expose its path via
    LaunchConfiguration('world_path').

    - Adds `gazebo_ros_state` plugin to publish /model_states and /link_states.
    - Keeping the world here avoids shipping static .world files and lets us
      tweak the state publisher in one place.
    """
    world_xml = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <!-- Publish /model_states and /link_states for tooling and bridges -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <publish_model_state>true</publish_model_state>
      <publish_link_state>true</publish_link_state>
      <update_rate>30</update_rate>
      <model_states_topic>/model_states</model_states_topic>
      <link_states_topic>/link_states</link_states_topic>
    </plugin>
  </world>
</sdf>
"""
    # NOTE: mkstemp gives us a unique filename. We keep it for the Gazebo session lifetime.
    fd, path = tempfile.mkstemp(prefix="fwds_world_with_state_", suffix=".world")
    with os.fdopen(fd, "w") as f:
        f.write(world_xml)

    return [
        LogInfo(msg=f"[gazebo_bringup] generated temp world: {path}"),
        SetLaunchConfiguration('world_path', path),
    ]


def generate_launch_description():
    # ---------------------------
    # User-facing launch arguments
    # ---------------------------
    use_sim_time      = LaunchConfiguration('use_sim_time',      default='true')   # Use Gazebo /clock
    gui               = LaunchConfiguration('gui',               default='true')   # Show Gazebo client
    verbose           = LaunchConfiguration('verbose',           default='false')  # Gazebo verbosity
    enable_odom_bridge= LaunchConfiguration('enable_odom_bridge',default='true')   # Start gazebo_odom_bridge
    model_name        = LaunchConfiguration('model_name',        default='x40a')   # Entity name in Gazebo
    odom_frame        = LaunchConfiguration('odom_frame',        default='odom')   # Odometry frame id
    base_frame        = LaunchConfiguration('base_frame',        default='base_footprint')  # Base child frame
    publish_tf        = LaunchConfiguration('publish_tf',        default='true')   # Whether bridge publishes TF
    flatten_2d        = LaunchConfiguration('flatten_to_2d',     default='true')   # Drop Z/yaw drift in sim

    # ---------------------------
    # Robot description (Xacro -> URDF)
    # ---------------------------
    urdf_path = PathJoinSubstitution([
        FindPackageShare('fwdsrover_description'), 'urdf', 'x40a.xacro'
    ])
    xacro_cmd = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        TextSubstitution(text=' '),
        urdf_path,
        # Add Xacro args here if needed, e.g. ' scale:=0.001'
    ])
    robot_description = {'robot_description': ParameterValue(xacro_cmd, value_type=str)}

    # Publish robot_description (and TF from URDF fixed joints)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
        output='screen',
    )

    # ---------------------------
    # World generation + Gazebo server/client
    # ---------------------------
    make_world = OpaqueFunction(function=_make_world_with_state)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'gui': gui,
            'verbose': verbose,
            'world': LaunchConfiguration('world_path'),  # from _make_world_with_state()
        }.items()
    )

    # ---------------------------
    # Spawn robot from /robot_description
    # ---------------------------
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'x40a', '-topic', 'robot_description'],
        output='screen',
    )

    # ---------------------------
    # ros2_control controllers
    #   - joint_state_broadcaster: publishes joint states for RViz and others
    #   - steer_position_controller: steering joints (position interface)
    #   - wheel_velocity_controller: wheel joints (velocity interface)
    # Order matters: bring up JSB first, then steering, then wheels.
    # ---------------------------
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawner_steer = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steer_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawner_wheel = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ---------------------------
    # Gazebo → /odom & TF bridge
    #  - Listens /model_states (from gazebo_ros_state) and publishes /odom, TF(odom->base)
    #  - Remap legacy topic names just in case
    #  - Delayed to ensure Gazebo & controllers are alive
    # ---------------------------
    bridge_node = Node(
        package='fwdsrover_xna_bringup',
        executable='gazebo_odom_bridge',
        name='gazebo_odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_name': model_name,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'publish_tf': publish_tf,
            'flatten_to_2d': flatten_2d,
        }],
        remappings=[
            ('/gazebo/model_states', '/model_states'),  # defensive remap
            ('gazebo/model_states', '/model_states'),
        ],
        condition=IfCondition(enable_odom_bridge),
    )

    # ---------------------------
    # Bring-up sequencing (coarse but robust)
    # ---------------------------
    jsb_after    = TimerAction(period=3.0,  actions=[spawner_jsb])
    steer_after  = TimerAction(period=5.0,  actions=[spawner_steer])
    wheel_after  = TimerAction(period=6.5,  actions=[spawner_wheel])
    bridge_after = TimerAction(period=8.0,  actions=[bridge_node])

    # ---------------------------
    # Final LaunchDescription
    # ---------------------------
    return LaunchDescription([
        # User args (override at launch)
        DeclareLaunchArgument('use_sim_time',      default_value='true',  description='Use /clock from Gazebo'),
        DeclareLaunchArgument('gui',               default_value='true',  description='Launch Gazebo client'),
        DeclareLaunchArgument('verbose',           default_value='false', description='Gazebo verbose logging'),
        DeclareLaunchArgument('enable_odom_bridge',default_value='true',  description='Run gazebo_odom_bridge'),
        DeclareLaunchArgument('model_name',        default_value='x40a',  description='Gazebo entity name'),
        DeclareLaunchArgument('odom_frame',        default_value='odom',  description='Odometry frame id'),
        DeclareLaunchArgument('base_frame',        default_value='base_footprint', description='Base child frame'),
        DeclareLaunchArgument('publish_tf',        default_value='true',  description='Bridge publishes TF'),
        DeclareLaunchArgument('flatten_to_2d',     default_value='true',  description='Project motion to 2D'),

        # Bringup sequence
        rsp,             # 1) Publish /robot_description
        make_world,      # 2) Create temp world with gazebo_ros_state
        gazebo_launch,   # 3) Start Gazebo (server/client) with that world
        spawn,           # 4) Spawn robot from /robot_description
        jsb_after,       # 5) Controllers: JSB
        steer_after,     # 6) Controllers: steering
        wheel_after,     # 7) Controllers: wheels
        bridge_after,    # 8) Start odom bridge (optional)
    ])

